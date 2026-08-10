// gui/calibration/calibration_wizard.cpp — see header (Zhou's Method).
//
// Wires the event tap (drain_last_window) + CalibrationWorker (async screw-head
// grid detection + duplicate/coverage rejection, on-worker cv::calibrateCamera,
// auto-mkdir YAML export) to the Space-key capture.

#include "calibration_wizard.h"

#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QComboBox>
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
#include <QSignalBlocker>
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

// Fixed asymmetric grid: 6×5 (30 markers). The row offset (odd rows shifted by
// one half-cell) gives 8-fold orientation disambiguation, so there is no
// cols/rows spinbox and no square-grid rejection.
constexpr int kGridCols = 6;
constexpr int kGridRows = 5;
constexpr double kDefaultSquareMm = 5.0;
constexpr int kDefaultTargetFrames = 15;

// Dashed-cross dot gap (Zhou's screw-head grid). 1/2/3 are the supported
// values; default 2. The cross dots are 1px, spaced (1+dot_gap) px apart, and
// the solid ring thickness equals dot_gap.
constexpr int kDefaultDotGap = 2;

// Space-capture event window (µs), user-tunable 100–1000 in 100 µs steps.
// The screw-head detector uses event polarity (gold/white) for ring
// verification, so the window must be short enough to capture one motion
// direction (hand micro-tremor) — a long window blurs the leading/trailing
// half-circle structure. Default 500 µs.
constexpr int kCaptureWindowMinUs = 100;
constexpr int kCaptureWindowMaxUs = 1000;
constexpr int kCaptureWindowStepUs = 100;
constexpr int kDefaultCaptureWindowUs = 500;

// Register cross-thread metatypes used by the worker signals/slots.
Q_DECL_UNUSED static const int kRegCvMat = qRegisterMetaType<cv::Mat>("cv::Mat");
Q_DECL_UNUSED static const int kRegSizeT = qRegisterMetaType<std::size_t>("std::size_t");

// Inset (px per side) applied to the screen workarea when sizing the wizard.
// The window must stay BELOW the compositor's full-screen threshold: on Mutter
// (GNOME-50, XWayland, 200% scale), a window that covers most of the output is
// put on the unredirect / direct-scanout path, which throttles the 30 Hz camera
// preview → stutter. A/B testing proved the trigger is size/coverage-driven,
// NOT content-driven (solid-disc and waffle patterns stutter identically at
// near-full-screen, both smooth when the window is dragged clearly smaller).
//
// A 5 px/side inset (10 px total) was NOT enough — the physical surface still
// covered ~95% of the 2880×1920 output and still stuttered. The coverage
// threshold is coarser than a few px, so a substantial inset is needed. 50 px
// per side (100 px total per dimension) drops the physical coverage to ~88%,
// clearly below the scanout threshold on this display. This is intentionally
// generous: if it still stutters, the lever is NOT window size and the fix must
// instead isolate the camera preview on its own small surface (see the
// separate-window fallback noted in show_intrinsic). Tunable — reduce toward
// the minimum that stays smooth once the threshold is empirically pinned down.
constexpr int kFullscreenGuardInset = 50;

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
    setWindowTitle(tr("Intrinsic Calibration (Zhou's Screw-Head Grid Method)"));
    // Qt::Window (instead of the default Qt::Dialog) gives a full top-level
    // window with working minimize/close buttons. The maximize button is
    // deliberately OMITTED: a maximized window covers the full workarea, which
    // on Mutter (GNOME) triggers the unredirect / maximized compositing path
    // and makes the 30 Hz camera preview stutter (see show_intrinsic). The
    // changeEvent override below also blocks maximize from other paths (title-
    // bar double-click, Super+Up, GNOME drag-to-top edge-tiling) since those
    // bypass the button. Qt::WindowMaximizeButtonHint is intentionally absent.
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowSystemMenuHint |
                   Qt::WindowMinimizeButtonHint | Qt::WindowCloseButtonHint);
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
    // Size the window to (nearly) the full workarea but INSET by a few px so it
    // does NOT exactly match the workarea. This is the root-cause fix for the
    // calibration preview stutter.
    //
    // What happens without the inset: a window whose geometry matches the
    // workarea is treated by Mutter (GNOME) as maximized — it is unredirected
    // (direct-scanout) and/or put on the maximized compositing frame-sync path.
    // That path throttles the 30 Hz camera preview → visible stutter. A/B
    // testing proved the trigger is purely geometric (size/coverage-driven,
    // NOT content-driven): solid-disc and waffle patterns stutter IDENTICALLY
    // at near-full-workarea size, and BOTH become smooth when the window is
    // dragged clearly smaller. The coverage threshold is coarser than a few px
    // (a 5 px/side inset was NOT enough — see kFullscreenGuardInset). The GUI
    // thread itself is fine (it paints at a steady 30 Hz); the stall is in the
    // compositor's presentation of the window.
    //
    // Why not _NET_WM_BYPASS_COMPOSITOR=0? GNOME-50 Mutter no longer honors
    // that X11 hint (it was tried and had zero effect). Why not showMaximized()?
    // That sets Qt::WindowMaximized, which is exactly the state to avoid. The
    // geometric inset is compositor-independent: a normal-state window that
    // doesn't match the workarea gets normal compositing on Mutter, KWin, Xorg,
    // and XWayland alike. The changeEvent below keeps it that way even if the
    // user triggers a maximize via the keyboard or drag-to-top edge-tiling.
    if (QScreen* screen = QGuiApplication::primaryScreen()) {
        QRect g = screen->availableGeometry();
        g.adjust(kFullscreenGuardInset, kFullscreenGuardInset,
                 -kFullscreenGuardInset, -kFullscreenGuardInset);
        setGeometry(g);
    }
    show();
    raise();
    activateWindow();
    camera_timer_->start();
    // Start streaming CD events into the tap so a Space press has a recent
    // 5000 µs window to grab.
    if (camera_ && camera_->is_connected()) {
        tap_.clear();
        camera_->set_cd_broadcast(true);
    }
}

void CalibrationWizard::changeEvent(QEvent* event) {
    QDialog::changeEvent(event);
    // Block ANY transition into Qt::WindowMaximized (title-bar double-click,
    // Super+Up, GNOME drag-to-top edge-tiling — the maximize button itself is
    // already removed via window flags, but those other paths bypass it). A
    // maximized window covers the full workarea → Mutter unredirects it →
    // camera preview stutters (see show_intrinsic). Immediately undo the
    // maximize and re-apply the inset workarea geometry, so the window LOOKS
    // maximized yet stays on the smooth normal compositing path.
    //
    // Deferred via QueuedConnection so the current state-change event finishes
    // first — calling setWindowState re-entrantly inside changeEvent is fragile.
    // Our own setWindowState fires a second changeEvent in which the window is
    // no longer maximized, so the guard below falls through (no loop). The WM
    // processes the un-maximize then our setGeometry in order.
    if (event->type() == QEvent::WindowStateChange &&
        (windowState() & Qt::WindowMaximized)) {
        QMetaObject::invokeMethod(this, [this] {
            setWindowState(windowState() & ~Qt::WindowMaximized);
            if (QScreen* screen = QGuiApplication::primaryScreen()) {
                QRect g = screen->availableGeometry();
                g.adjust(kFullscreenGuardInset, kFullscreenGuardInset,
                         -kFullscreenGuardInset, -kFullscreenGuardInset);
                setGeometry(g);
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
    const Metavision::timestamp window_us = capture_window_->value();
    const std::size_t n = tap_.drain_last_window(window_us, evs);
    if (n == 0 || evs.empty()) {
        set_status(tr("No events in the last %1 µs — move the camera or wait.")
            .arg(window_us));
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
        pattern_->set_pattern(kGridCols, kGridRows);
        pattern_->set_dot_gap(dot_gap_->currentData().toInt());
        pattern_->set_square_size_mm(static_cast<float>(square_mm_->value()));
    }
}

void CalibrationWizard::configure_worker() {
    emit configure_requested(square_mm_->value(),
                             target_frames_->value(),
                             dot_gap_->currentData().toInt());
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
    // Three-valued colour frame (BGR): black background, ON (polarity != 0) =
    // gold, OFF (polarity == 0) = white. A pixel hit by BOTH polarities takes
    // the simple average of gold and white (no event-count weighting). The
    // screw-head detector reads polarity from the colour (R-B channel) for ring
    // verification; the averaged blend is a legitimate colour and does not need
    // exclusion. The frame is NOT downsampled.
    static const cv::Vec3b kGold(0, 215, 255);
    static const cv::Vec3b kWhite(255, 255, 255);
    static const cv::Vec3b kBlend = (kGold + kWhite) * 0.5;  // (127, 235, 255)

    cv::Mat frame(sensor_h, sensor_w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat has_on(sensor_h, sensor_w, CV_8U, cv::Scalar(0));
    cv::Mat has_off(sensor_h, sensor_w, CV_8U, cv::Scalar(0));
    for (const auto& ev : evs) {
        if (ev.x < 0 || ev.x >= sensor_w || ev.y < 0 || ev.y >= sensor_h) continue;
        if (ev.p != 0) has_on.at<uchar>(ev.y, ev.x) = 1;
        else           has_off.at<uchar>(ev.y, ev.x) = 1;
    }
    for (int y = 0; y < sensor_h; ++y) {
        const uchar* on = has_on.ptr<uchar>(y);
        const uchar* off = has_off.ptr<uchar>(y);
        cv::Vec3b* f = frame.ptr<cv::Vec3b>(y);
        for (int x = 0; x < sensor_w; ++x) {
            if (on[x] && off[x])      f[x] = kBlend;
            else if (on[x])           f[x] = kGold;
            else if (off[x])          f[x] = kWhite;
            // else: stays black (background)
        }
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

    // Zhou's screw-head grid — dashed-cross dot gap (1/2/3, default 2). The
    // cross is 1px white dots spaced (1+dot_gap) px apart; the solid ring
    // thickness equals dot_gap. Smaller gaps give a denser cross.
    dot_gap_ = new QComboBox(params_widget);
    dot_gap_->addItem(QString::number(1), 1);
    dot_gap_->addItem(QString::number(2), 2);
    dot_gap_->addItem(QString::number(3), 3);
    dot_gap_->setCurrentIndex(dot_gap_->findData(kDefaultDotGap));
    dot_gap_->setToolTip(tr("Dashed-cross dot gap in pixels (1/2/3). The cross "
        "is 1px white dots spaced (1+gap) px apart; the solid ring thickness "
        "equals the gap. Smaller gaps give a denser cross."));
    form->addRow(tr("Dot gap"), dot_gap_);

    // Capture window (µs), 100–1000 in 100 µs steps. Short enough to capture
    // one micro-tremor direction so the ring's leading/trailing half-circles
    // keep opposite polarities.
    capture_window_ = new QSpinBox(params_widget);
    capture_window_->setRange(kCaptureWindowMinUs, kCaptureWindowMaxUs);
    capture_window_->setSingleStep(kCaptureWindowStepUs);
    capture_window_->setValue(kDefaultCaptureWindowUs);
    capture_window_->setSuffix(tr(" µs"));
    capture_window_->setToolTip(tr("Event capture window in microseconds "
        "(100–1000). Shorter windows keep a cleaner polarity signal; longer "
        "windows gather more events. Hold the camera steady and rely on hand "
        "micro-tremor to trigger events."));
    form->addRow(tr("Capture window"), capture_window_);

    square_mm_ = new QDoubleSpinBox(params_widget);
    square_mm_->setRange(0.1, 500.0);
    square_mm_->setDecimals(2);
    square_mm_->setValue(kDefaultSquareMm);
    square_mm_->setSuffix(tr(" mm"));
    square_mm_->setToolTip(tr("Physical distance (mm) between two adjacent "
        "marker centers in the same row or column. Sets the real-world scale "
        "for calibration; does NOT change the on-screen pattern size (screen "
        "DPI is deliberately not used)."));
    form->addRow(tr("Marker spacing"), square_mm_);

    // Measurement instruction: one short sentence. The user measures the
    // same-row (or same-column) adjacent marker distance; the algorithm uses
    // it directly (d = measured/2, no √2).
    spacing_note_ = new QLabel(
        tr("Measure the center-to-center distance between two adjacent markers "
           "in the same row or column, then enter it in mm."),
        params_widget);
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
        "Hold the camera steady; rely on hand micro-tremor to trigger events. "
        "Press <b>Space</b> to capture. Move the camera between captures for "
        "varied angles."), this);
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

    // Config-change handlers. The grid is fixed at 6×5 (no cols/rows spinbox),
    // so there is no square-grid rejection. dot_gap/capture_window/square_mm/
    // target all trigger a re-configure + display refresh.
    connect(dot_gap_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int) { on_config_changed(); });
    connect(capture_window_, QOverload<int>::of(&QSpinBox::valueChanged), this,
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
