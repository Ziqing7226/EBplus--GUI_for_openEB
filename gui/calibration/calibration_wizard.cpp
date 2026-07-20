// gui/calibration/calibration_wizard.cpp

#include "calibration_wizard.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
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

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>

#include "app/camera_controller.h"
#include "chessboard_display.h"
#include "display/event_display_widget.h"

namespace gui {

namespace {

// 1 ms accumulation window, 50 ms capture cycle → the tap searches the best
// 1 ms sliding window inside each 50 ms span (audit §9.2-F). The chessboard
// flips at 20 Hz (one flip per cycle), so the max-event window is the one
// aligned with the flip burst.
constexpr Metavision::timestamp kWindowUs = 1000;
constexpr Metavision::timestamp kSpanUs   = 50000;
constexpr int kCaptureTimerMs = 50;

// Default corner-displacement threshold (audit §9.2-D): if the mean
// displacement of corresponding corners vs. the last accepted frame is below
// this, the pose is considered unchanged and the frame is rejected as a
// duplicate. Immune to the 20 Hz inversion (corner positions are identical
// in both phases), unlike the old MSE check.
constexpr double kDefaultMinMovePx = 2.0;

// Max queued capture jobs. If the worker falls behind (slow detection on a
// noisy 1280×720 frame), whole cycles are dropped rather than letting the
// queue grow unbounded — the flip burst repeats every 50 ms anyway.
constexpr std::size_t kMaxQueuedJobs = 3;

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
    stop_and_join_worker();  // covers the "calibration running" phase too
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

    // §9.2-D: corner-displacement duplicate check (replaces the MSE check,
    // which was defeated by the 20 Hz board inversion).
    in_min_move_px_ = new QDoubleSpinBox(page);
    in_min_move_px_->setRange(0.0, 100.0);
    in_min_move_px_->setDecimals(1);
    in_min_move_px_->setValue(kDefaultMinMovePx);
    in_min_move_px_->setToolTip(tr("Frames whose mean corner displacement vs. the "
        "last accepted frame is below this are rejected as duplicates (same pose)."));
    form->addRow(tr("Min corner movement (px)"), in_min_move_px_);

    // §9.2-G: editable physical square size. The default is the DPI-derived
    // value from the chessboard window (refreshed each time the board is
    // shown); the user can measure one square with a ruler and correct it.
    // Only the metric scale of the translation vectors depends on it.
    in_square_mm_ = new QDoubleSpinBox(page);
    in_square_mm_->setRange(0.1, 1000.0);
    in_square_mm_->setDecimals(2);
    in_square_mm_->setValue(25.0);
    in_square_mm_->setToolTip(tr("Physical side length of one chessboard square. "
        "Defaults to the screen-DPI estimate — measure a square with a ruler "
        "and correct it for accurate metric translations."));
    form->addRow(tr("Square size (mm)"), in_square_mm_);

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
        [this](int v) {
            in_progress_->setRange(0, v);
            if (chessboard_) chessboard_->set_progress(in_progress_->value(), v);
        });

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
            if (capturing_) {
                // §9.4 收尾: don't auto-stop — the user may just reopen the
                // board — but make it obvious why no frames are accepted.
                set_status(tr("Chessboard closed — capture paused. "
                              "Click 'Show Chessboard' to resume."));
            }
        });
        // HUD controls drive the same slots as the dialog buttons, so the
        // full workflow is usable in fullscreen (audit §9.2-C).
        connect(chessboard_, &ChessboardDisplay::startRequested,
                this, &CalibrationWizard::on_start_capture);
        connect(chessboard_, &ChessboardDisplay::stopRequested,
                this, &CalibrationWizard::on_stop_capture);
    }
    chessboard_->set_pattern(in_cols_->value(), in_rows_->value());
    chessboard_->attach_to_screen(target);
    // Fullscreen by default — the board must fill the screen so the camera
    // can see the whole pattern. User can press F to toggle windowed mode.
    chessboard_->showFullScreen();
    chessboard_->raise();
    chessboard_->start_flicker();
    // §9.2-G: refresh the editable square-size default from the board's DPI
    // estimate. Note this overwrites a manually entered value each time the
    // board is (re)shown.
    in_square_mm_->setValue(static_cast<double>(chessboard_->square_size_mm()));
    // Sync HUD with current session state.
    chessboard_->set_capturing(capturing_);
    chessboard_->set_progress(in_progress_->value(), in_target_frames_->value());
    if (in_status_) chessboard_->set_status(in_status_->text());
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

    // Sensor geometry: prefer the camera's sensor_info.
    int sensor_w = 0, sensor_h = 0;
    sensor_w = camera_->sensor_info().width;
    sensor_h = camera_->sensor_info().height;
    if (sensor_w <= 0 || sensor_h <= 0) {
        QMessageBox::warning(this, tr("Calibration"), tr("Sensor geometry unknown."));
        return;
    }

    // Reap any previous worker FIRST, so the config snapshot below can't be
    // read concurrently by a still-running worker.
    join_worker();
    {
        std::lock_guard<std::mutex> lk(work_mutex_);
        work_queue_.clear();
    }

    // Snapshot the configuration for the worker thread (plain data, handed
    // over before std::thread construction → no locking needed).
    capture_cfg_.cols          = in_cols_->value();
    capture_cfg_.rows          = in_rows_->value();
    capture_cfg_.square_mm     = static_cast<float>(in_square_mm_->value());
    capture_cfg_.target_frames = in_target_frames_->value();
    capture_cfg_.min_move_px   = in_min_move_px_->value();
    capture_cfg_.sensor_w      = sensor_w;
    capture_cfg_.sensor_h      = sensor_h;

    worker_stop_ = false;
    worker_ = std::thread([this]() { worker_loop(); });

    // Enable CD broadcast so the tap receives batches.
    tap_.clear();
    camera_->set_cd_broadcast(true);
    capturing_ = true;
    capture_timer_->start();

    in_last_preview_ = QImage();
    in_preview_label_->clear();
    in_preview_label_->setText(tr("Capturing..."));
    in_progress_->setRange(0, in_target_frames_->value());
    in_progress_->setValue(0);
    in_start_btn_->setEnabled(false);
    in_stop_btn_->setEnabled(true);
    in_reset_btn_->setEnabled(true);
    in_save_btn_->setEnabled(false);
    if (chessboard_) {
        chessboard_->set_capturing(true);
        chessboard_->set_progress(0, in_target_frames_->value());
    }
    set_status(tr("Capturing — point the camera at the chessboard from multiple angles..."));
}

void CalibrationWizard::on_stop_capture() {
    if (!capturing_) return;
    capture_timer_->stop();
    capturing_ = false;
    if (camera_) camera_->set_cd_broadcast(false);
    stop_and_join_worker();
    in_start_btn_->setEnabled(camera_ && camera_->is_connected());
    in_stop_btn_->setEnabled(false);
    in_reset_btn_->setEnabled(true);
    if (chessboard_) chessboard_->set_capturing(false);
    // intrinsic_ is safe to read here: the worker is joined.
    set_status(tr("Capture stopped. Captured %1 frames.")
        .arg(intrinsic_->frame_count()));
}

void CalibrationWizard::on_capture_tick() {
    if (!capturing_) return;
    // GUI thread does only the cheap part: drain the tap and hand the 1 ms
    // window to the worker. Rendering + detection run off-thread (§9.2-B).
    auto best = std::make_shared<std::vector<Metavision::EventCD>>();
    const std::size_t n = tap_.drain_and_pick_max_window(kWindowUs, kSpanUs, *best);
    if (n == 0 || best->empty()) {
        if (!chessboard_) {
            set_status(tr("Chessboard closed — capture paused. "
                          "Click 'Show Chessboard' to resume."));
        } else {
            set_status(tr("No events in this cycle. Point the camera at the chessboard."));
        }
        return;
    }
    {
        std::lock_guard<std::mutex> lk(work_mutex_);
        if (work_queue_.size() >= kMaxQueuedJobs) return;  // worker behind: drop cycle
        work_queue_.push_back(std::move(best));
    }
    work_cv_.notify_one();
}

// ---------------------------------------------------------------------------
// Worker thread (audit §9.2-B / §六-B3)
// ---------------------------------------------------------------------------

void CalibrationWizard::worker_loop() {
    // intrinsic_ is touched ONLY on this thread (and by on_intrinsic_save()
    // after the worker is joined). set_pattern + reset here replace the old
    // GUI-thread configuration in on_start_capture().
    intrinsic_->set_pattern(gui_algo::CalibrationPattern::Chessboard,
                            capture_cfg_.cols, capture_cfg_.rows,
                            capture_cfg_.square_mm);
    intrinsic_->reset();

    // Board size for the duplicate pre-detection. IntrinsicCalibration no
    // longer exposes board_size() publicly, so mirror its chessboard
    // convention here: (cols, rows) IS the OpenCV inner-corner count. Must
    // match set_pattern() above, otherwise the pre-check and add_frame()
    // would look for different patterns.
    const cv::Size board(std::max(capture_cfg_.cols, 1),
                         std::max(capture_cfg_.rows, 1));
    const int detect_flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

    // Corners of the last accepted frame, for the §9.2-D displacement check.
    std::vector<cv::Point2f> last_corners;

    for (;;) {
        std::shared_ptr<std::vector<Metavision::EventCD>> job;
        {
            std::unique_lock<std::mutex> lk(work_mutex_);
            work_cv_.wait(lk, [this]() {
                return worker_stop_.load() || !work_queue_.empty();
            });
            if (worker_stop_.load()) return;
            job = std::move(work_queue_.front());
            work_queue_.pop_front();
        }

        const std::size_t n_events = job->size();
        cv::Mat gray = render_event_window(*job, capture_cfg_.sensor_w,
                                           capture_cfg_.sensor_h);
        // §9.2-E: denoise before detection — hot pixels / BA noise punch
        // black/white holes into the rendered squares. 3×3 median is cheap.
        cv::medianBlur(gray, gray, 3);

        // §9.2-D duplicate check (corner displacement). add_frame() already
        // accumulates on success, so a rejected duplicate must be caught
        // BEFORE calling it — IntrinsicCalibration has no "un-add" API, so
        // we pre-detect corners here (worker thread, cost acceptable) and
        // only forward non-duplicates to add_frame(). Skipped for the very
        // first frame (no reference yet).
        if (!last_corners.empty()) {
            std::vector<cv::Point2f> corners;
            const bool found = cv::findChessboardCorners(gray, board, corners,
                                                         detect_flags);
            if (!found) {
                post_to_gui([this, n_events]() {
                    set_status(tr("Rejected — chessboard not detected (%1 events).")
                        .arg(n_events));
                });
                continue;
            }
            cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                 30, 0.01));
            if (corners.size() == last_corners.size()) {
                double mean_disp = 0.0;
                for (std::size_t i = 0; i < corners.size(); ++i)
                    mean_disp += cv::norm(corners[i] - last_corners[i]);
                mean_disp /= static_cast<double>(corners.size());
                if (mean_disp < capture_cfg_.min_move_px) {
                    const double thr = capture_cfg_.min_move_px;
                    post_to_gui([this, mean_disp, thr]() {
                        set_status(tr("Rejected — duplicate pose (corner shift %1 px < %2 px). Move the camera.")
                            .arg(mean_disp, 0, 'f', 2).arg(thr, 0, 'f', 1));
                    });
                    continue;
                }
            }
        }

        auto res = intrinsic_->add_frame(gray, true);
        if (!res.found) {
            post_to_gui([this, n_events]() {
                set_status(tr("Rejected — chessboard not detected (%1 events).")
                    .arg(n_events));
            });
            continue;
        }

        // Accept: reuse the detected corners from the result as the new
        // duplicate reference (OpenCV returns a stable corner order for the
        // same board, so index-wise comparison is valid).
        last_corners = res.points;
        const int got = static_cast<int>(intrinsic_->frame_count());
        const int target = capture_cfg_.target_frames;
        const QImage preview = cv_to_qimage(res.image);
        post_to_gui([this, got, target, n_events, preview]() {
            in_last_preview_ = preview;
            update_intrinsic_preview(preview);
            in_progress_->setValue(got);
            if (chessboard_) chessboard_->set_progress(got, target);
            set_status(tr("Captured %1 / %2 frames (last: %3 events).")
                .arg(got).arg(target).arg(n_events));
        });

        if (got >= target) {
            // Auto-end: run calibration on the worker thread (§六-B3 — the
            // old code ran cv::calibrateCamera on the GUI thread and even
            // called processEvents(), §六-B4). Freeze the capture controls
            // while it runs.
            post_to_gui([this]() {
                capture_timer_->stop();
                in_start_btn_->setEnabled(false);
                in_stop_btn_->setEnabled(false);
                in_reset_btn_->setEnabled(false);
                in_save_btn_->setEnabled(false);
                set_status(tr("Capture complete. Running calibration..."));
            });
            auto result = std::make_shared<gui_algo::IntrinsicResult>(intrinsic_->run());
            post_to_gui([this, result]() {
                join_worker();  // reap this worker (it is about to return)
                capturing_ = false;
                if (camera_) camera_->set_cd_broadcast(false);
                if (chessboard_) chessboard_->set_capturing(false);
                intrinsic_result_ = *result;
                in_start_btn_->setEnabled(camera_ && camera_->is_connected());
                in_stop_btn_->setEnabled(false);
                in_reset_btn_->setEnabled(true);
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
            });
            return;
        }
    }
}

void CalibrationWizard::stop_and_join_worker() {
    worker_stop_ = true;
    work_cv_.notify_all();
    join_worker();
    std::lock_guard<std::mutex> lk(work_mutex_);
    work_queue_.clear();
}

void CalibrationWizard::join_worker() {
    if (worker_.joinable()) worker_.join();
}

// ---------------------------------------------------------------------------
// Reset / export
// ---------------------------------------------------------------------------

void CalibrationWizard::on_intrinsic_reset() {
    if (capturing_) on_stop_capture();
    join_worker();  // intrinsic_ must not be mid-use; the next capture
                    // reconfigures it in the worker anyway.
    intrinsic_result_ = {};
    in_last_preview_ = QImage();
    in_preview_label_->clear();
    in_preview_label_->setText(tr("No frames captured yet."));
    in_progress_->setRange(0, in_target_frames_->value());
    in_progress_->setValue(0);
    in_save_btn_->setEnabled(false);
    if (chessboard_) chessboard_->set_progress(0, in_target_frames_->value());
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
    join_worker();  // intrinsic_ (image_size) is read below.
    if (!intrinsic_result_.ok) return;
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save Intrinsic Calibration"),
        default_export_path(),
        tr("YAML (*.yml *.yaml);;All Files (*)"));
    if (path.isEmpty()) return;
    // §六-B2: the default path's directory doesn't exist on first use —
    // create it, otherwise the export always fails.
    QDir().mkpath(QFileInfo(path).absolutePath());
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

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void CalibrationWizard::update_intrinsic_preview(const QImage& img) {
    if (img.isNull()) return;
    in_preview_label_->setPixmap(QPixmap::fromImage(img).scaledToWidth(
        in_preview_area_->viewport()->width(), Qt::SmoothTransformation));
    in_preview_label_->resize(in_preview_label_->pixmap().size());
}

void CalibrationWizard::set_status(const QString& text) {
    if (in_status_) in_status_->setText(text);
    // Mirror onto the chessboard HUD so the status is visible in fullscreen
    // (audit §9.2-C). HUD updates are state-change driven, never per flip.
    if (chessboard_) chessboard_->set_status(text);
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
    // Polarity-separated rendering is load-bearing (audit §9.2-A): the board
    // only exists as "which pixels went ON vs OFF" during one flip.
    for (const auto& ev : evs) {
        if (ev.x < 0 || ev.x >= sensor_w || ev.y < 0 || ev.y >= sensor_h) continue;
        gray.ptr<uchar>(ev.y)[ev.x] = ev.p ? 255 : 0;
    }
    return gray;
}

} // namespace gui
