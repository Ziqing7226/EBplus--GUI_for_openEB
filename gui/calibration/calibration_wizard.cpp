// gui/calibration/calibration_wizard.cpp — see header (Phase 4).
//
// Wires the event tap (drain_last_window) + CalibrationWorker (async
// circle-grid detection + duplicate/coverage rejection, on-worker
// cv::calibrateCamera, auto-mkdir YAML export) to the Space-key capture.

#include "calibration_wizard.h"

#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QMessageBox>
#include <QPaintEvent>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QScreen>
#include <QSizePolicy>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QShowEvent>
#include <QStandardPaths>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QMetaType>

#include <metavision/sdk/base/utils/timestamp.h>

#include "app/camera_controller.h"
#include "circle_grid_display.h"
#include "calibration_worker.h"
#include "display/event_display_widget.h"

namespace gui {

namespace {

// Aim-feedback poll rate. 30 Hz matches the FocusAssistant.
constexpr int kCameraPollMs = 33;

// Default asymmetric circle-grid board: 6×6 (36 circles). A square grid is
// orientation-agnostic (the user need not rotate the camera) and gives a
// 11×5-cell footprint that conditions calibrateCamera well.
constexpr int kDefaultCols = 6;
constexpr int kDefaultRows = 6;
constexpr double kDefaultSquareMm = 5.0;
constexpr int kDefaultTargetFrames = 15;

// Space-capture window: the most recent 500 µs of CD events (polarity
// ignored). Per the Phase 4 plan this may need widening if static-pattern
// detection rate is too low (decided at field test per §0).
constexpr Metavision::timestamp kCaptureWindowUs = 500;

// Register cross-thread metatypes used by the worker signals/slots.
Q_DECL_UNUSED static const int kRegCvMat = qRegisterMetaType<cv::Mat>("cv::Mat");
Q_DECL_UNUSED static const int kRegSizeT = qRegisterMetaType<std::size_t>("std::size_t");

} // namespace

// ---------------------------------------------------------------------------
// CalibrationCameraView — lightweight camera preview (same approach as
// FocusCameraView: QPainter::drawImage in paintEvent)
// ---------------------------------------------------------------------------

CalibrationCameraView::CalibrationCameraView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(240, 180);
    setAutoFillBackground(false);
    setAttribute(Qt::WA_OpaquePaintEvent);
}

void CalibrationCameraView::set_frame(const QImage& frame) {
    frame_ = frame;
    message_.clear();
    update();
}

void CalibrationCameraView::set_message(const QString& msg) {
    frame_ = QImage();
    message_ = msg;
    update();
}

void CalibrationCameraView::paintEvent(QPaintEvent* /*event*/) {
    QPainter p(this);
    p.fillRect(rect(), QColor(20, 20, 20));
    if (!frame_.isNull()) {
        const QImage scaled = frame_.scaled(size(), Qt::KeepAspectRatio,
                                            Qt::SmoothTransformation);
        p.drawImage((width() - scaled.width()) / 2,
                    (height() - scaled.height()) / 2, scaled);
    } else if (!message_.isEmpty()) {
        p.setPen(QColor(160, 160, 160));
        p.drawText(rect(), Qt::AlignCenter, message_);
    }
}

// ---------------------------------------------------------------------------
// CalibrationWizard
// ---------------------------------------------------------------------------

CalibrationWizard::CalibrationWizard(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Intrinsic Calibration"));
    // Qt::Window (instead of the default Qt::Dialog) gives a full top-level
    // window with working minimize/maximize/close buttons. The default
    // Qt::Dialog on X11 shows a maximize button that is visually present but
    // non-functional — switching to Qt::Window makes it actually work.
    // WindowMinMaxButtonsHint ensures both min and max buttons are present.
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowSystemMenuHint |
                   Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
    setMinimumSize(900, 560);
    build_ui();

    camera_timer_ = new QTimer(this);
    camera_timer_->setInterval(kCameraPollMs);
    connect(camera_timer_, &QTimer::timeout, this, &CalibrationWizard::on_camera_tick);

    // Worker on a dedicated thread: detection (and, in 4-3, calibrateCamera)
    // run off the GUI thread so OpenCV blocking does not freeze the UI.
    worker_thread_ = new QThread(this);
    worker_ = new CalibrationWorker;  // no parent — moved to worker_thread_
    worker_->moveToThread(worker_thread_);
    worker_thread_->start();

    connect(this, &CalibrationWizard::configure_requested,
            worker_, &CalibrationWorker::configure);
    connect(this, &CalibrationWizard::submit_frame,
            worker_, &CalibrationWorker::process_frame);
    connect(this, &CalibrationWizard::run_calibration_requested,
            worker_, &CalibrationWorker::run_calibration);
    connect(this, &CalibrationWizard::export_requested,
            worker_, &CalibrationWorker::export_to);
    connect(worker_, &CalibrationWorker::frame_accepted,
            this, &CalibrationWizard::on_frame_accepted);
    connect(worker_, &CalibrationWorker::frame_rejected,
            this, &CalibrationWizard::on_frame_rejected);
    connect(worker_, &CalibrationWorker::capture_complete,
            this, &CalibrationWizard::on_capture_complete);
    connect(worker_, &CalibrationWorker::calibration_done,
            this, &CalibrationWizard::on_calibration_done);
    connect(worker_, &CalibrationWorker::export_done,
            this, &CalibrationWizard::on_export_done);

    configure_worker();
}

CalibrationWizard::~CalibrationWizard() {
    if (camera_timer_) camera_timer_->stop();
    if (camera_) camera_->set_cd_broadcast(false);
    teardown_worker();
}

void CalibrationWizard::set_camera(CameraController* controller) {
    camera_ = controller;
    tap_.attach(controller);
    enable_capture(controller && controller->is_connected() && !capture_done_);
}

void CalibrationWizard::set_display(EventDisplayWidget* display) {
    display_ = display;
}

void CalibrationWizard::show_intrinsic() {
    // Open full-screen via setGeometry() (NOT showMaximized()). changeEvent()
    // below likewise intercepts any later maximize (button / title-bar
    // double-click / WM shortcut) and converts it to a normal-state
    // full-screen geometry. Keeping the window out of Qt::WindowMaximized is
    // the key to smooth preview: on X11 compositors (Mutter/KWin) the
    // maximized state takes a different frame-sync/scanout path that stutters
    // with this app's render pipeline (QOpenGLWidget main display streaming +
    // 30 Hz aim-view), while the normal state — the same path manual resize
    // uses — stays smooth. Confirmed by the user: dragging the edges to full
    // size is smooth, only the maximize button lagged.
    if (QScreen* screen = QGuiApplication::primaryScreen()) {
        setGeometry(screen->availableGeometry());
    }
    show();
    raise();
    activateWindow();
    camera_timer_->start();
    // Start streaming CD events into the tap so a Space press has a recent
    // 500 µs window to grab.
    if (camera_ && camera_->is_connected()) {
        tap_.clear();
        camera_->set_cd_broadcast(true);
    }
}

void CalibrationWizard::changeEvent(QEvent* event) {
    QDialog::changeEvent(event);

    // Intercept ANY transition into Qt::WindowMaximized (maximize button,
    // title-bar double-click, WM keyboard shortcut, …) and replace it with a
    // normal-state full-screen geometry. See show_intrinsic() for why the
    // maximized state must be avoided. We can't stop the WM from applying the
    // maximize, but we can immediately undo it and re-apply a full-screen
    // geometry, so the window LOOKS maximized yet stays on the smooth normal
    // compositing path (the same path manual edge-resize uses).
    //
    // Deferred via QueuedConnection so the current state-change event finishes
    // first — calling setWindowState re-entrantly from inside changeEvent is
    // fragile. Our own setWindowState fires a second changeEvent in which the
    // window is no longer maximized, so the guard below falls through (no
    // loop). The WM processes un-maximize then our setGeometry in order; a
    // brief transition flicker is possible but the end state is smooth.
    if (event->type() == QEvent::WindowStateChange &&
        (windowState() & Qt::WindowMaximized)) {
        QMetaObject::invokeMethod(this, [this] {
            setWindowState(windowState() & ~Qt::WindowMaximized);
            if (QScreen* screen = QGuiApplication::primaryScreen()) {
                setGeometry(screen->availableGeometry());
            }
        }, Qt::QueuedConnection);
    }
}

void CalibrationWizard::hideEvent(QHideEvent* event) {
    if (camera_timer_) camera_timer_->stop();
    if (camera_) camera_->set_cd_broadcast(false);
    QDialog::hideEvent(event);
}

void CalibrationWizard::showEvent(QShowEvent* event) {
    QDialog::showEvent(event);
    // Mirror hideEvent(): whenever the window becomes visible again, restart
    // the aim-view poll and CD broadcast. Without this, any hide→show cycle
    // (minimize→restore, workspace switch, WM state transition) leaves the
    // preview frozen on its last message — e.g. a stale "No camera connected"
    // from before the camera was started in the main window, even though the
    // main display is now streaming fine. QTimer::start() is harmless if the
    // timer is already running (it just resets the interval); if no camera is
    // connected, on_camera_tick reports that state on the next tick.
    if (camera_timer_) camera_timer_->start();
    if (camera_ && camera_->is_connected()) {
        tap_.clear();
        camera_->set_cd_broadcast(true);
    }
}

void CalibrationWizard::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
        event->accept();
        on_capture_pressed();
        return;
    }
    QDialog::keyPressEvent(event);
}

void CalibrationWizard::on_camera_tick() {
    if (!display_ || !camera_) return;
    // Three-state aim-view driven by the real camera state, NOT by
    // current_frame().isNull() — that is also null for a connected-but-not-
    // started camera and during the first-frame race, which previously showed
    // a misleading "No camera connected" (Phase 4 Debug D3).
    if (!camera_->is_connected()) {
        camera_view_->set_message(tr("No camera connected"));
        return;
    }
    if (!camera_->is_running()) {
        camera_view_->set_message(tr("Camera connected — press Start to stream"));
        return;
    }
    const QImage frame = display_->current_frame();
    if (frame.isNull()) {
        camera_view_->set_message(tr("Waiting for first frame…"));
        return;
    }
    camera_view_->set_frame(frame);
}

void CalibrationWizard::on_capture_pressed() {
    // Note: do NOT guard on !capture_btn_->isEnabled() — the button is disabled
    // when no camera is connected, but Space should still surface the popup so
    // the user learns the prerequisite. capture_in_flight_/capture_done_ guard
    // the actual one-at-a-time / target-reached cases.
    if (capture_in_flight_ || capture_done_) return;
    if (!camera_ || !camera_->is_connected()) {
        // Modal popup (not just a status line) so the user knows the
        // prerequisite (Phase 4 Debug D5). GUI is all-English.
        QMessageBox::information(this, tr("Camera not connected"),
            tr("No camera is connected. Please connect a camera in the main window first."));
        return;
    }
    if (!camera_->is_running()) {
        QMessageBox::information(this, tr("Camera not running"),
            tr("Camera is not running. Please start the camera in the main window first."));
        return;
    }

    std::vector<Metavision::EventCD> evs;
    const std::size_t n = tap_.drain_last_window(kCaptureWindowUs, evs);
    if (n == 0 || evs.empty()) {
        set_status(tr("No events in the last %1 µs — move the camera or wait.")
            .arg(kCaptureWindowUs));
        return;
    }

    const int sw = camera_->sensor_info().width;
    const int sh = camera_->sensor_info().height;
    if (sw <= 0 || sh <= 0) {
        set_status(tr("Sensor geometry unknown."));
        return;
    }

    cv::Mat frame = render_event_frame(evs, sw, sh);
    capture_in_flight_ = true;
    // Serialize captures (Phase 4 Debug D7): disable the button while the
    // worker judges this frame; on_frame_accepted/rejected re-enable it. This
    // makes the one-at-a-time flow visible and prevents rapid clicks piling up.
    capture_btn_->setEnabled(false);
    set_status(tr("Detecting (%1 events)…").arg(n));
    emit submit_frame(frame);
}

void CalibrationWizard::on_frame_accepted(QImage annotated,
                                          std::size_t accepted, std::size_t target) {
    capture_in_flight_ = false;
    progress_->setRange(0, static_cast<int>(target));
    progress_->setValue(static_cast<int>(accepted));
    if (!annotated.isNull()) {
        preview_label_->setPixmap(QPixmap::fromImage(annotated).scaledToWidth(
            preview_area_->viewport()->width(), Qt::SmoothTransformation));
        preview_label_->resize(preview_label_->pixmap().size());
    }
    set_status(tr("Captured %1 / %2 frames.").arg(accepted).arg(target));
    // Judgment done — re-enable capture for the next shot (capture_complete
    // disables it for good once the target is reached).
    if (!capture_done_) enable_capture(camera_ && camera_->is_connected());
}

void CalibrationWizard::on_frame_rejected(QString reason) {
    capture_in_flight_ = false;
    set_status(tr("Rejected — %1").arg(reason));
    if (!capture_done_) enable_capture(camera_ && camera_->is_connected());
}

void CalibrationWizard::on_capture_complete(std::size_t accepted) {
    capture_in_flight_ = false;
    capture_done_ = true;
    enable_capture(false);
    // Run cv::calibrateCamera on the worker thread (Phase 4-3).
    set_status(tr("Captured %1 frames. Running calibration…").arg(accepted));
    emit run_calibration_requested();
}

void CalibrationWizard::on_calibration_done(bool ok, double rms,
                                            int frames_used, QString error) {
    if (ok) {
        set_status(tr("Calibration OK. RMS = %1 px (%2 frames). Click Export to save.")
            .arg(rms, 0, 'f', 3).arg(frames_used));
        export_btn_->setEnabled(true);
    } else {
        QMessageBox::warning(this, tr("Calibration failed"), error);
        set_status(tr("Calibration failed: %1").arg(error));
    }
}

void CalibrationWizard::on_export_pressed() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save Intrinsic Calibration"),
        default_export_path(),
        tr("YAML (*.yml *.yaml);;All Files (*)"));
    if (path.isEmpty()) return;
    set_status(tr("Exporting to %1…").arg(path));
    emit export_requested(path);
}

void CalibrationWizard::on_export_done(bool ok, QString message) {
    if (ok) {
        set_status(tr("Saved to %1").arg(message));
    } else {
        QMessageBox::warning(this, tr("Save failed"), message);
        set_status(tr("Export failed: %1").arg(message));
    }
}

void CalibrationWizard::on_intrinsic_reset() {
    capture_in_flight_ = false;
    capture_done_ = false;
    progress_->setValue(0);
    preview_label_->clear();
    preview_label_->setText(tr("No frames captured yet."));
    export_btn_->setEnabled(false);
    // Reset the worker's accumulated observations (queued).
    QMetaObject::invokeMethod(worker_, "reset", Qt::QueuedConnection);
    enable_capture(camera_ && camera_->is_connected());
    set_status(tr("Point the camera at the pattern and press Space to capture."));
}

void CalibrationWizard::on_config_changed() {
    apply_pattern_to_display();
    configure_worker();
    // Geometry change invalidates accumulated observations — reset the view.
    capture_done_ = false;
    progress_->setValue(0);
    preview_label_->clear();
    preview_label_->setText(tr("No frames captured yet."));
    export_btn_->setEnabled(false);
    QMetaObject::invokeMethod(worker_, "reset", Qt::QueuedConnection);
    enable_capture(camera_ && camera_->is_connected());
}

void CalibrationWizard::set_status(const QString& text) {
    if (status_) status_->setText(text);
}

void CalibrationWizard::enable_capture(bool on) {
    capture_btn_->setEnabled(on);
    capture_btn_->setToolTip(on ? QString() :
        tr("Connect a camera first, then press Space to capture."));
}

void CalibrationWizard::apply_pattern_to_display() {
    if (pattern_) {
        pattern_->set_pattern(cols_->value(), rows_->value());
        pattern_->set_square_size_mm(static_cast<float>(square_mm_->value()));
    }
}

void CalibrationWizard::configure_worker() {
    emit configure_requested(cols_->value(), rows_->value(),
                             square_mm_->value(), target_frames_->value());
}

QString CalibrationWizard::default_export_path() const {
    const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString base = docs.isEmpty() ? QDir::homePath() : docs;
    // Stable filename (no timestamp) so the default matches the Preprocessor
    // undistort path exactly — the user can rely on both defaults pointing at
    // the same file. QFileDialog prompts for overwrite if it already exists.
    // The parent directory is created on export (auto-mkdir).
    return base + "/EBplus/calibration/intrinsic.yml";
}

cv::Mat CalibrationWizard::render_event_frame(
    const std::vector<Metavision::EventCD>& evs, int sensor_w, int sensor_h) {
    // Full-resolution binary frame: WHITE background, any event (ON or OFF)
    // → BLACK. Polarity is IGNORED per the Phase 4 design; the capture frame
    // is NOT downsampled. Dark-on-light is the polarity cv::findCirclesGrid's
    // default blob detector expects (dark blobs on a light field) — the
    // on-screen CircleGridDisplay is the inverse (white-on-black) for camera
    // contrast; the rendered capture frame is dark-on-light for detection.
    cv::Mat frame(sensor_h, sensor_w, CV_8UC1, cv::Scalar(255));
    for (const auto& ev : evs) {
        if (ev.x < 0 || ev.x >= sensor_w || ev.y < 0 || ev.y >= sensor_h) continue;
        frame.ptr<uchar>(ev.y)[ev.x] = 0;
    }
    return frame;
}

void CalibrationWizard::teardown_worker() {
    if (worker_thread_) {
        worker_thread_->quit();
        worker_thread_->wait();
        // worker_ has no parent and the QThread::finished → deleteLater
        // connection is intentionally NOT used: after quit()+wait() the
        // worker thread's event loop has already exited, so a queued
        // deleteLater would never be processed and worker_ would leak.
        // Delete it explicitly here (safe — its thread is already joined).
        delete worker_;
        delete worker_thread_;
        worker_thread_ = nullptr;
        worker_ = nullptr;
    }
}

void CalibrationWizard::build_ui() {
    auto* outer = new QVBoxLayout(this);

    // ---- Top row: parameters | live camera aim-view | captured preview ----
    // Three equal-width columns. The aim-view reuses the main display's frame
    // (QImage implicit sharing — no deep copy); the preview holds one transient
    // annotated image. Both stay memory-light (Phase 4 Debug memory note).
    auto* top_row = new QHBoxLayout();

    // Parameters column.
    auto* params_widget = new QWidget(this);
    auto* form = new QFormLayout(params_widget);
    form->setContentsMargins(0, 0, 0, 0);

    cols_ = new QSpinBox(params_widget);
    cols_->setRange(2, 30);
    cols_->setValue(kDefaultCols);
    rows_ = new QSpinBox(params_widget);
    rows_->setRange(2, 30);
    rows_->setValue(kDefaultRows);
    auto* dims_row = new QWidget(params_widget);
    auto* dims_lay = new QHBoxLayout(dims_row);
    dims_lay->setContentsMargins(0, 0, 0, 0);
    dims_lay->addWidget(new QLabel(tr("Cols:"), dims_row));
    dims_lay->addWidget(cols_);
    dims_lay->addWidget(new QLabel(tr("Rows:"), dims_row));
    dims_lay->addWidget(rows_);
    dims_lay->addStretch();
    form->addRow(tr("Grid (circles)"), dims_row);

    square_mm_ = new QDoubleSpinBox(params_widget);
    square_mm_->setRange(0.1, 500.0);
    square_mm_->setDecimals(2);
    square_mm_->setValue(kDefaultSquareMm);
    square_mm_->setSuffix(tr(" mm"));
    square_mm_->setToolTip(tr("Physical spacing between adjacent circle centers "
        "(mm). This value sets the real-world scale for the calibration "
        "algorithm only; it does NOT change the on-screen pattern size (screen "
        "DPI is deliberately not used)."));
    form->addRow(tr("Circle spacing"), square_mm_);

    // Measurement instruction directly below the spinbox so the user knows
    // the correct flow: measure on-screen → enter mm → calibration uses it.
    spacing_note_ = new QLabel(
        tr("Measure the on-screen spacing between adjacent circle centers "
           "with a ruler, then enter that value in mm here. This sets the "
           "real-world scale for calibration."), params_widget);
    spacing_note_->setWordWrap(true);
    spacing_note_->setStyleSheet("font-size:11px; color:#888;");
    form->addRow(spacing_note_);

    target_frames_ = new QSpinBox(params_widget);
    target_frames_->setRange(3, 100);
    target_frames_->setValue(kDefaultTargetFrames);
    form->addRow(tr("Target frames"), target_frames_);

    top_row->addWidget(params_widget, 1);

    // Live camera aim-view column. CalibrationCameraView uses the same
    // approach as FocusCameraView (QPainter::drawImage in paintEvent).
    camera_view_ = new CalibrationCameraView(this);
    camera_view_->set_message(tr("No camera connected"));
    top_row->addWidget(camera_view_, 1);

    // Captured-frame preview column.
    preview_area_ = new QScrollArea(this);
    preview_area_->setAlignment(Qt::AlignCenter);
    preview_area_->setMinimumSize(240, 180);
    preview_label_ = new QLabel(preview_area_);
    preview_label_->setAlignment(Qt::AlignCenter);
    preview_label_->setText(tr("No frames captured yet."));
    preview_area_->setWidget(preview_label_);
    top_row->addWidget(preview_area_, 1);

    outer->addLayout(top_row);

    // ---- Large circle-grid pattern (full width, dominant) ----
    pattern_ = new CircleGridDisplay(this);
    pattern_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pattern_->setMinimumSize(400, 200);
    apply_pattern_to_display();
    outer->addWidget(pattern_, 1);

    // ---- Capture hint ----
    hint_ = new QLabel(tr(
        "Press <b>Space</b> to capture a frame (500 µs event window, polarity "
        "ignored). Move the camera between captures for varied angles."), this);
    hint_->setWordWrap(true);
    hint_->setProperty("class", "hint");
    outer->addWidget(hint_);

    // ---- Controls row: buttons + progress ----
    auto* btns = new QHBoxLayout();
    capture_btn_ = new QPushButton(tr("Capture"), this);
    reset_btn_   = new QPushButton(tr("Reset"), this);
    export_btn_  = new QPushButton(tr("Export..."), this);
    enable_capture(false);
    export_btn_->setEnabled(false);
    btns->addWidget(capture_btn_);
    btns->addWidget(reset_btn_);
    btns->addWidget(export_btn_);
    btns->addStretch();
    progress_ = new QProgressBar(this);
    progress_->setRange(0, target_frames_->value());
    btns->addWidget(progress_);
    outer->addLayout(btns);

    // ---- Status ----
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    outer->addWidget(status_);

    connect(cols_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int) { on_config_changed(); });
    connect(rows_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int) { on_config_changed(); });
    connect(square_mm_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
        [this](double) { on_config_changed(); });
    connect(target_frames_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int v) { progress_->setRange(0, v); on_config_changed(); });
    connect(capture_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_capture_pressed);
    connect(reset_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_intrinsic_reset);
    connect(export_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_export_pressed);

    on_intrinsic_reset();
}

} // namespace gui
