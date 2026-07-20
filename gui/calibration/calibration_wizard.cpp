// gui/calibration/calibration_wizard.cpp

#include "calibration_wizard.h"

#include <QApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScreen>
#include <QScrollArea>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

// OpenCV 5.0 moved the chessboard pattern detection functions
// (findChessboardCorners, CALIB_CB_*) from calib3d to objdetect. The
// calib3d.hpp shim shipped by some 5.x packagings (e.g. Homebrew) does not
// pull in the full objdetect declarations, so findChessboardCorners goes
// missing. Include <opencv2/core.hpp> first so CV_MAJOR_VERSION is defined,
// then pick the correct header per OpenCV major version.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#if CV_MAJOR_VERSION >= 5
#include <opencv2/objdetect.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

#include "app/camera_controller.h"
#include "chessboard_display.h"
#include "display/event_display_widget.h"

namespace gui {

namespace {

// 1 ms accumulation window, 50 ms capture cycle → 50 candidate windows per
// cycle. The chessboard flips at 20 Hz (one flip per cycle), so the
// max-event window is the one aligned with the flip burst.
constexpr Metavision::timestamp kWindowUs = 1000;
constexpr Metavision::timestamp kSpanUs   = 50000;
constexpr int kCaptureTimerMs = 50;

// Default MSE below which two frames are considered duplicates (same pose).
// 0-255 grayscale scale; 50 is conservative — different poses typically
// produce MSE > 100.
constexpr double kDefaultMseThreshold = 50.0;

} // namespace

CalibrationWizard::CalibrationWizard(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Calibration Wizard"));
    setMinimumSize(720, 560);

    intrinsic_ = std::make_unique<gui_algo::IntrinsicCalibration>();

    tabs_ = new QTabWidget(this);
    build_intrinsic_tab();

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(tabs_);

    auto* close_box = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(close_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(close_box);

    capture_timer_ = new QTimer(this);
    capture_timer_->setTimerType(Qt::PreciseTimer);
    capture_timer_->setInterval(kCaptureTimerMs);
    connect(capture_timer_, &QTimer::timeout, this, &CalibrationWizard::on_capture_tick);
}

CalibrationWizard::~CalibrationWizard() {
    if (capturing_) on_stop_capture();
}

void CalibrationWizard::set_camera(CameraController* controller) {
    camera_ = controller;
    tap_.attach(controller);
    const bool has_camera = (camera_ != nullptr && camera_->is_connected());
    in_start_btn_->setEnabled(has_camera);
}

void CalibrationWizard::set_display(EventDisplayWidget* display) {
    display_ = display;
}

void CalibrationWizard::show_intrinsic() {
    tabs_->setCurrentIndex(0);
    rebuild_screen_combo();
    show();
    raise();
    activateWindow();
}

// ---------------------------------------------------------------------------
// Intrinsic tab
// ---------------------------------------------------------------------------

void CalibrationWizard::build_intrinsic_tab() {
    auto* page = new QWidget(this);
    auto* outer = new QVBoxLayout(page);

    // Configuration form.
    auto* form = new QFormLayout();

    in_screen_ = new QComboBox(page);
    form->addRow(tr("Target screen"), in_screen_);

    in_cols_ = new QSpinBox(page);
    in_cols_->setRange(2, 30);
    in_cols_->setValue(9);
    in_rows_ = new QSpinBox(page);
    in_rows_->setRange(2, 30);
    in_rows_->setValue(6);
    auto* dims_row = new QWidget(page);
    auto* dims_layout = new QHBoxLayout(dims_row);
    dims_layout->setContentsMargins(0, 0, 0, 0);
    dims_layout->addWidget(new QLabel(tr("Cols:"), dims_row));
    dims_layout->addWidget(in_cols_);
    dims_layout->addWidget(new QLabel(tr("Rows:"), dims_row));
    dims_layout->addWidget(in_rows_);
    dims_layout->addStretch();
    form->addRow(tr("Inner corners"), dims_row);

    in_target_frames_ = new QSpinBox(page);
    in_target_frames_->setRange(5, 100);
    in_target_frames_->setValue(30);
    form->addRow(tr("Target frames"), in_target_frames_);

    in_mse_threshold_ = new QDoubleSpinBox(page);
    in_mse_threshold_->setRange(0.0, 10000.0);
    in_mse_threshold_->setDecimals(1);
    in_mse_threshold_->setValue(kDefaultMseThreshold);
    in_mse_threshold_->setToolTip(tr("Frames with MSE below this vs. the last "
        "accepted frame are rejected as duplicates (same pose)."));
    form->addRow(tr("Duplicate MSE"), in_mse_threshold_);

    outer->addLayout(form);

    // Buttons.
    auto* btn_row = new QHBoxLayout();
    in_show_board_btn_ = new QPushButton(tr("Show Chessboard"), page);
    in_start_btn_      = new QPushButton(tr("Start Auto-Capture"), page);
    in_stop_btn_       = new QPushButton(tr("Stop"), page);
    in_reset_btn_      = new QPushButton(tr("Reset"), page);
    in_save_btn_       = new QPushButton(tr("Export..."), page);
    in_stop_btn_->setEnabled(false);
    in_save_btn_->setEnabled(false);
    btn_row->addWidget(in_show_board_btn_);
    btn_row->addWidget(in_start_btn_);
    btn_row->addWidget(in_stop_btn_);
    btn_row->addWidget(in_reset_btn_);
    btn_row->addWidget(in_save_btn_);
    btn_row->addStretch();
    outer->addLayout(btn_row);

    // Progress bar.
    in_progress_ = new QProgressBar(page);
    in_progress_->setRange(0, in_target_frames_->value());
    in_progress_->setValue(0);
    outer->addWidget(in_progress_);

    // Preview area.
    in_preview_area_ = new QScrollArea(page);
    in_preview_area_->setAlignment(Qt::AlignCenter);
    in_preview_label_ = new QLabel(in_preview_area_);
    in_preview_label_->setText(tr("No frames captured yet."));
    in_preview_area_->setWidget(in_preview_label_);
    in_preview_area_->setMinimumHeight(220);
    outer->addWidget(in_preview_area_, 1);

    in_status_ = new QLabel(tr("Idle. Click 'Show Chessboard' then 'Start Auto-Capture'."), page);
    in_status_->setWordWrap(true);
    outer->addWidget(in_status_);

    connect(in_show_board_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_show_chessboard);
    connect(in_start_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_start_capture);
    connect(in_stop_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_stop_capture);
    connect(in_reset_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_intrinsic_reset);
    connect(in_save_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_intrinsic_save);
    connect(in_target_frames_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int v) { in_progress_->setRange(0, v); });

    // Apply initial configuration.
    on_intrinsic_reset();

    tabs_->addTab(page, tr("Intrinsic"));
}

void CalibrationWizard::rebuild_screen_combo() {
    if (!in_screen_) return;
    in_screen_->clear();
    in_screen_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    const auto screens = QGuiApplication::screens();
    for (int i = 0; i < screens.size(); ++i) {
        QScreen* s = screens[i];
        const QRect g = s->geometry();
        const QString label = tr("Screen %1: %2×%3")
            .arg(i + 1).arg(g.width()).arg(g.height());
        in_screen_->addItem(label, QVariant::fromValue(s));
    }
}

void CalibrationWizard::on_show_chessboard() {
    QScreen* target = in_screen_->currentData().value<QScreen*>();
    if (!target) target = QGuiApplication::primaryScreen();
    if (!chessboard_) {
        chessboard_ = new ChessboardDisplay(this);
        // WA_DeleteOnClose would null chessboard_ on close; we want to reuse
        // the same instance across Show/Hide clicks, so leave it owned by
        // 'this' and just show/hide.
        connect(chessboard_, &ChessboardDisplay::destroyed, this, [this]() {
            chessboard_ = nullptr;
        });
    }
    chessboard_->set_pattern(in_cols_->value(), in_rows_->value());
    chessboard_->attach_to_screen(target);
    // Fullscreen by default — the board must fill the screen so the camera
    // can see the whole pattern. User can press F to toggle windowed mode.
    chessboard_->showFullScreen();
    chessboard_->raise();
    chessboard_->start_flicker();
}

void CalibrationWizard::on_start_capture() {
    if (!camera_ || !camera_->is_connected()) {
        QMessageBox::warning(this, tr("Calibration"), tr("No camera connected."));
        return;
    }
    if (capturing_) return;
    // Auto-show the chessboard if the user hasn't yet — without it,
    // square_size_mm is unknown (no DPI geometry) and no flip bursts are
    // generated, so capture would produce garbage.
    if (!chessboard_) {
        on_show_chessboard();
        if (!chessboard_) {
            QMessageBox::warning(this, tr("Calibration"),
                tr("Could not display the chessboard. Cannot start capture."));
            return;
        }
    }
    // Refresh detector config from current controls.
    intrinsic_->set_pattern(gui_algo::CalibrationPattern::Chessboard,
                            in_cols_->value(), in_rows_->value(),
                            chessboard_->square_size_mm());
    intrinsic_->reset();
    intrinsic_result_ = {};
    last_accepted_gray_ = cv::Mat();
    in_last_preview_ = QImage();
    in_preview_label_->clear();
    in_preview_label_->setText(tr("Capturing..."));
    in_progress_->setRange(0, in_target_frames_->value());
    in_progress_->setValue(0);

    // Enable CD broadcast so the tap receives batches.
    tap_.clear();
    camera_->set_cd_broadcast(true);
    capturing_ = true;
    capture_timer_->start();
    in_start_btn_->setEnabled(false);
    in_stop_btn_->setEnabled(true);
    in_save_btn_->setEnabled(false);
    set_status(tr("Capturing — point the camera at the chessboard from multiple angles..."));
}

void CalibrationWizard::on_stop_capture() {
    if (!capturing_) return;
    capture_timer_->stop();
    capturing_ = false;
    if (camera_) camera_->set_cd_broadcast(false);
    in_start_btn_->setEnabled(true);
    in_stop_btn_->setEnabled(false);
    set_status(tr("Capture stopped. Captured %1 frames.")
        .arg(intrinsic_->frame_count()));
}

void CalibrationWizard::on_capture_tick() {
    if (!capturing_) return;
    std::vector<Metavision::EventCD> best;
    const std::size_t n = tap_.drain_and_pick_max_window(kWindowUs, kSpanUs, best);
    if (n == 0 || best.empty()) {
        set_status(tr("No events in this cycle. Point the camera at the chessboard."));
        return;
    }

    // Determine sensor dimensions: prefer the camera's sensor_info, fall
    // back to the display's current frame size.
    int sensor_w = 0, sensor_h = 0;
    if (camera_) {
        sensor_w = camera_->sensor_info().width;
        sensor_h = camera_->sensor_info().height;
    }
    if (sensor_w <= 0 || sensor_h <= 0) {
        set_status(tr("Sensor geometry unknown."));
        return;
    }

    // Render the 1 ms window to a grayscale frame.
    cv::Mat gray = render_event_window(best, sensor_w, sensor_h);
    auto res = intrinsic_->add_frame(gray, true);

    if (!res.found) {
        set_status(tr("Rejected — chessboard not detected (%1 events).").arg(n));
        return;
    }

    // Duplicate check via MSE vs last accepted frame.
    if (!last_accepted_gray_.empty()) {
        const double m = mse_gray(gray, last_accepted_gray_);
        if (m < in_mse_threshold_->value()) {
            set_status(tr("Rejected — duplicate pose (MSE=%1 < %2). Move the camera.")
                .arg(m, 0, 'f', 1).arg(in_mse_threshold_->value(), 0, 'f', 1));
            return;
        }
    }

    // Accept: store the grayscale for future MSE comparison, update preview.
    last_accepted_gray_ = gray.clone();
    in_last_preview_ = cv_to_qimage(res.image);
    update_intrinsic_preview(in_last_preview_);
    const std::size_t got = intrinsic_->frame_count();
    in_progress_->setValue(static_cast<int>(got));
    set_status(tr("Captured %1 / %2 frames (last: %3 events).")
        .arg(got).arg(in_target_frames_->value()).arg(n));

    if (static_cast<int>(got) >= in_target_frames_->value()) {
        // Auto-end: stop capture and run calibration.
        on_stop_capture();
        set_status(tr("Capture complete. Running calibration..."));
        QApplication::processEvents();
        intrinsic_result_ = intrinsic_->run();
        if (intrinsic_result_.ok) {
            set_status(tr("Calibration OK. RMS = %1 px (%2 frames). Click Export to save.")
                .arg(intrinsic_result_.rms, 0, 'f', 3)
                .arg(intrinsic_result_.frames_used));
            in_save_btn_->setEnabled(true);
        } else {
            QMessageBox::warning(this, tr("Calibration failed"),
                QString::fromStdString(intrinsic_result_.error));
            set_status(tr("Calibration failed: %1")
                .arg(QString::fromStdString(intrinsic_result_.error)));
        }
    }
}

void CalibrationWizard::on_intrinsic_reset() {
    if (capturing_) on_stop_capture();
    intrinsic_->set_pattern(gui_algo::CalibrationPattern::Chessboard,
                            in_cols_->value(), in_rows_->value(),
                            chessboard_ ? chessboard_->square_size_mm() : 25.0f);
    intrinsic_->reset();
    intrinsic_result_ = {};
    last_accepted_gray_ = cv::Mat();
    in_last_preview_ = QImage();
    in_preview_label_->clear();
    in_preview_label_->setText(tr("No frames captured yet."));
    in_progress_->setRange(0, in_target_frames_->value());
    in_progress_->setValue(0);
    in_save_btn_->setEnabled(false);
    set_status(tr("Idle. Click 'Show Chessboard' then 'Start Auto-Capture'."));
}

QString CalibrationWizard::default_export_path() const {
    const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString base = docs.isEmpty() ? QDir::homePath() : docs;
    // Stable filename (no timestamp) so the default matches the Preprocessor
    // undistort path exactly — the user can rely on both defaults pointing at
    // the same file. QFileDialog prompts for overwrite if it already exists.
    return base + "/EBplus/calibration/intrinsic.yml";
}

void CalibrationWizard::on_intrinsic_save() {
    if (!intrinsic_result_.ok) return;
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save Intrinsic Calibration"),
        default_export_path(),
        tr("YAML (*.yml *.yaml);;All Files (*)"));
    if (path.isEmpty()) return;
    try {
        cv::FileStorage fs(path.toStdString(), cv::FileStorage::WRITE);
        if (!fs.isOpened()) throw std::runtime_error("Cannot open file");
        fs << "image_width"  << intrinsic_->image_size().width;
        fs << "image_height" << intrinsic_->image_size().height;
        fs << "camera_matrix" << intrinsic_result_.K;
        fs << "distortion_coefficients" << intrinsic_result_.dist_coeffs;
        fs << "rms" << intrinsic_result_.rms;
        fs.release();
        set_status(tr("Saved to %1").arg(path));
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("Save failed"), e.what());
    }
}

void CalibrationWizard::update_intrinsic_preview(const QImage& img) {
    if (img.isNull()) return;
    in_preview_label_->setPixmap(QPixmap::fromImage(img).scaledToWidth(
        in_preview_area_->viewport()->width(), Qt::SmoothTransformation));
    in_preview_label_->resize(in_preview_label_->pixmap().size());
}

void CalibrationWizard::set_status(const QString& text) {
    if (in_status_) in_status_->setText(text);
}

QImage CalibrationWizard::cv_to_qimage(const cv::Mat& mat) {
    if (mat.empty()) return QImage();
    cv::Mat rgb;
    if (mat.channels() == 1) {
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
    } else {
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    }
    return QImage(rgb.data, rgb.cols, rgb.rows,
                  static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
}

cv::Mat CalibrationWizard::render_event_window(
    const std::vector<Metavision::EventCD>& evs, int sensor_w, int sensor_h) {
    cv::Mat gray(sensor_h, sensor_w, CV_8UC1, cv::Scalar(128));
    // ON (p=1) → white (255), OFF (p=0) → black (0). Background grey (128)
    // so the chessboard pattern stands out for cv::findChessboardCorners.
    for (const auto& ev : evs) {
        if (ev.x < 0 || ev.x >= sensor_w || ev.y < 0 || ev.y >= sensor_h) continue;
        gray.ptr<uchar>(ev.y)[ev.x] = ev.p ? 255 : 0;
    }
    return gray;
}

double CalibrationWizard::mse_gray(const cv::Mat& a, const cv::Mat& b) {
    if (a.size() != b.size() || a.type() != b.type()) return 1e9;
    cv::Mat diff;
    cv::absdiff(a, b, diff);
    diff.convertTo(diff, CV_64F);
    diff = diff.mul(diff);
    cv::Scalar s = cv::sum(diff);
    return s[0] / static_cast<double>(a.total());
}

} // namespace gui
