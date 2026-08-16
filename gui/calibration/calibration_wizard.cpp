// gui/calibration/calibration_wizard.cpp — see header (blinking chessboard).
//
// Wires the event tap (drain_last_window) + CalibrationWorker (async blink-frame
// chessboard detection + coverage/duplicate rejection, on-worker two-pass
// cv::calibrateCamera, auto-mkdir YAML export) to the Space-key capture. On
// Space the wizard accumulates the last capture-window of CD events into
// per-polarity masks and hands them to the worker, which builds the binary
// blink frame and runs chessboard detection. There is NO capture-review dialog:
// a successful detection is committed directly (coverage + duplicate checks,
// accumulation, progress) and a failure is reported on the status line.

#include "calib_defaults.h"
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
#include <QPolygonF>
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
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QMetaType>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/calibration/blinking_detect.h"
#include "app/camera_controller.h"
#include "blinking_chessboard_display.h"
#include "calibration_worker.h"
#include "display/event_display_widget.h"

namespace gui {

namespace {

// Aim-feedback poll rate. 30 Hz matches the FocusAssistant.
constexpr int kCameraPollMs = 33;

// Fixed asymmetric chessboard: 9×6 INNER corners (10×7 squares). The
// asymmetric board gives the checkerboard a unique orientation, so there is no
// cols/rows spinbox and no square-grid rejection.
constexpr int kGridCols = 9;
constexpr int kGridRows = 6;
constexpr double kDefaultSquareMm = 20.0;
constexpr int kDefaultTargetFrames = 20;

// Space-capture event window (µs), user-tunable 200–200000 in 1 µs steps.
// The chessboard blinks at 10 ms (pattern ↔ blank, a 20 ms full cycle); the
// window must cover at least one full cycle so every toggling square fires both
// polarities. The 100000 µs default covers ~5 blink cycles.
constexpr int kCaptureWindowMinUs = 200;
constexpr int kCaptureWindowMaxUs = 200000;
constexpr int kCaptureWindowStepUs = 1;
constexpr int kDefaultCaptureWindowUs = 100000;

// Register cross-thread metatypes used by the worker signals/slots.
Q_DECL_UNUSED static const int kRegCvMat = qRegisterMetaType<cv::Mat>("cv::Mat");
Q_DECL_UNUSED static const int kRegSizeT = qRegisterMetaType<std::size_t>("std::size_t");
Q_DECL_UNUSED static const int kRegPoints =
    qRegisterMetaType<QVector<QPointF>>("QVector<QPointF>");
Q_DECL_UNUSED static const int kRegBlinkCapture =
    qRegisterMetaType<gui::BlinkCapture>("gui::BlinkCapture");

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

void CalibrationCameraView::set_coverage_overlay(
    const CoverageOverlay& overlay,
    const std::vector<std::vector<cv::Point2f>>& views) {
    overlay_ = overlay;
    overlay_views_ = views;
    update();
}

void CalibrationCameraView::paintEvent(QPaintEvent* /*event*/) {
    QPainter p(this);
    p.fillRect(rect(), QColor(20, 20, 20));
    if (!frame_.isNull()) {
        const QImage scaled = frame_.scaled(size(), Qt::KeepAspectRatio,
                                            Qt::SmoothTransformation);
        const double s = static_cast<double>(scaled.width()) / frame_.width();
        const double ox = (width() - scaled.width()) / 2.0;
        const double oy = (height() - scaled.height()) / 2.0;
        p.drawImage(static_cast<int>(ox), static_cast<int>(oy), scaled);

        // Coverage overlay: accepted corners as dots + convex hull of all
        // accepted corners + area coverage %. Sensor px → widget px uses the
        // same scale/offset as the frame blit above.
        if (!overlay_views_.empty()) {
            p.setRenderHint(QPainter::Antialiasing, true);
            const auto map = [&](const cv::Point2f& pt) {
                return QPointF(ox + s * pt.x, oy + s * pt.y);
            };
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(90, 170, 255, 170));
            for (const auto& view : overlay_views_) {
                for (const auto& pt : view) {
                    p.drawEllipse(map(pt), 2.0, 2.0);
                }
            }
            if (overlay_.hull.size() >= 3) {
                QPolygonF poly;
                poly.reserve(static_cast<qsizetype>(overlay_.hull.size()));
                for (const auto& pt : overlay_.hull) poly << map(pt);
                p.setPen(QPen(QColor(90, 170, 255, 220), 2));
                p.setBrush(QColor(90, 170, 255, 45));
                p.drawPolygon(poly);
            }
            // Coverage percentage, top-left, on a translucent backing.
            const QString text =
                QStringLiteral("Coverage: %1%").arg(qRound(overlay_.coverage * 100));
            const QRect tr = p.fontMetrics().boundingRect(text);
            const QRect box(6, 4, tr.width() + 12, tr.height() + 8);
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(0, 0, 0, 150));
            p.drawRect(box);
            p.setPen(QColor(235, 235, 235));
            p.drawText(box, Qt::AlignCenter, text);
        }
    } else if (!message_.isEmpty()) {
        p.setPen(QColor(160, 160, 160));
        p.drawText(rect(), Qt::AlignCenter, message_);
    }
}

// ---------------------------------------------------------------------------
// CalibrationWizard
// ---------------------------------------------------------------------------

CalibrationWizard::CalibrationWizard(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Intrinsic Calibration (Blinking Chessboard)"));
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

    // Worker on a dedicated thread: detection (and the two-pass calibrateCamera)
    // run off the GUI thread so OpenCV blocking does not freeze the UI.
    worker_thread_ = new QThread(this);
    worker_ = new CalibrationWorker;  // no parent — moved to worker_thread_
    worker_->moveToThread(worker_thread_);
    worker_thread_->start();

    connect(this, &CalibrationWizard::configure_requested,
            worker_, &CalibrationWorker::configure);
    connect(this, &CalibrationWizard::process_capture_requested,
            worker_, &CalibrationWorker::process_capture);
    connect(this, &CalibrationWizard::delete_last_capture_requested,
            worker_, &CalibrationWorker::delete_last_capture);
    connect(this, &CalibrationWizard::run_calibration_requested,
            worker_, &CalibrationWorker::run_calibration);
    connect(this, &CalibrationWizard::export_requested,
            worker_, &CalibrationWorker::export_to);
    connect(worker_, &CalibrationWorker::frame_accepted,
            this, &CalibrationWizard::on_frame_accepted);
    connect(worker_, &CalibrationWorker::frame_rejected,
            this, &CalibrationWizard::on_frame_rejected);
    connect(worker_, &CalibrationWorker::frame_deleted,
            this, &CalibrationWizard::on_frame_deleted);
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
    // Safety net: hideEvent normally restores the biases, but if the app
    // exits with the wizard still open no hideEvent fires.
    restore_diff_bias_override();
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
    // 100000 µs window to grab.
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
    restore_diff_bias_override();
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
        apply_diff_bias_override();
        tap_.clear();
        camera_->set_cd_broadcast(true);
    }
}

void CalibrationWizard::apply_diff_bias_override() {
    // Blinking-LCD noise-floor suppression (field-verified 2026-08-16): the
    // LCD backlight PWM fires both polarities on every on-screen pixel at
    // ~104 Mev/s, saturating the USB link (HAL USB Submit Errors) and
    // starving every downstream consumer. Raising diff_on/diff_off to their
    // hardware MAXIMUM filters the low-contrast PWM events on-sensor; the
    // chessboard blink is high-contrast and survives. The pre-override
    // values are snapshotted and restored on hide/close.
    if (!camera_ || !camera_->is_connected() || !saved_diff_biases_.empty()) {
        return;
    }
    auto* biases = camera_->biases_facility();
    if (!biases) return;
    try {
        const auto all = biases->get_all_biases();
        std::map<std::string, int> snapshot;
        for (const auto& [name, value] : all) {
            const bool is_diff = name.find("diff_on") != std::string::npos ||
                                 name.find("diff_off") != std::string::npos;
            if (!is_diff) continue;
            Metavision::LL_Bias_Info info;
            if (!biases->get_bias_info(name, info)) continue;
            const auto range = info.get_bias_range();
            if (range.second <= range.first) continue;  // unknown range — skip
            snapshot[name] = value;
            biases->set(name, range.second);
        }
        saved_diff_biases_ = std::move(snapshot);
    } catch (const std::exception& e) {
        set_status(tr("Warning: failed to raise diff biases (%1) — the LCD "
                      "noise floor may slow the stream.")
                       .arg(QString::fromUtf8(e.what())));
        saved_diff_biases_.clear();
        return;
    }
    if (!saved_diff_biases_.empty()) {
        emit biases_changed_externally();
    }
}

void CalibrationWizard::restore_diff_bias_override() {
    if (saved_diff_biases_.empty()) return;
    // Take the snapshot out first: if the camera is gone or a write fails,
    // the override is still considered finished (retrying against a
    // disconnecting device would just throw again).
    const auto snapshot = std::move(saved_diff_biases_);
    saved_diff_biases_.clear();
    if (camera_ && camera_->is_connected()) {
        if (auto* biases = camera_->biases_facility()) {
            try {
                for (const auto& [name, value] : snapshot) {
                    biases->set(name, value);
                }
            } catch (const std::exception&) {
                // Best effort — the camera may be mid-disconnect.
            }
        }
    }
    emit biases_changed_externally();
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
    // a misleading "No camera connected".
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
        // prerequisite. GUI is all-English.
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
        set_status(tr("No events in the last %1 µs — wait for the chessboard to blink.")
            .arg(window_us));
        return;
    }

    const int sw = camera_->sensor_info().width;
    const int sh = camera_->sensor_info().height;
    if (sw <= 0 || sh <= 0) {
        set_status(tr("Sensor geometry unknown."));
        return;
    }

    // Accumulate per-polarity event counts and hand them to the worker. The
    // worker owns the blink-frame parameters and derives the count threshold
    // adaptively, so a capture can be re-judged without re-reading events.
    BlinkCapture capture;
    gui_algo::accumulate_blink_counts(evs.data(), evs.data() + evs.size(),
                                      sw, sh, capture.on_cnt, capture.off_cnt);

    capture_in_flight_ = true;
    // Serialize captures: disable the button while the worker detects; on
    // accept/reject it is re-enabled. This makes the one-at-a-time flow
    // visible and prevents rapid clicks piling up.
    capture_btn_->setEnabled(false);
    // "Delete this capture" is also blocked while a frame is in flight — the
    // last accepted frame's preview is still shown, but deleting it mid-
    // judgment of the next frame would race the worker's accept/remove queue.
    update_delete_enabled();
    set_status(tr("Detecting (%1 events)…").arg(n));
    emit process_capture_requested(capture);
}

void CalibrationWizard::on_frame_accepted(QImage annotated,
                                          std::size_t accepted, std::size_t target,
                                          QVector<QPointF> points) {
    capture_in_flight_ = false;
    progress_->setRange(0, static_cast<int>(target));
    progress_->setValue(static_cast<int>(accepted));
    // Keep a preview stack parallel to the worker's image_points_ so a delete
    // can revert the preview to the now-last capture. Push even when annotated
    // is null (annotation can fail) so the stack stays in lock-step with the
    // accepted count — show_last_preview() handles a null top gracefully.
    captured_previews_.push_back(annotated);
    // Same lock-step for the coverage overlay's point stack.
    std::vector<cv::Point2f> view;
    view.reserve(static_cast<std::size_t>(points.size()));
    for (const auto& p : points) {
        view.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
    }
    captured_points_.push_back(std::move(view));
    update_coverage_overlay();
    show_last_preview();
    // The just-accepted frame is now the deletable "last capture". capture_done_
    // (set by on_capture_complete when the target is reached) clears this via
    // update_delete_enabled() — so delete is only available before the final
    // set is locked in.
    can_delete_last_ = true;
    set_status(tr("Captured %1 / %2 frames.").arg(accepted).arg(target));
    // Judgment done — re-enable capture for the next shot (capture_complete
    // disables it for good once the target is reached).
    if (!capture_done_) enable_capture(camera_ && camera_->is_connected());
    update_delete_enabled();
}

void CalibrationWizard::on_frame_rejected(QString reason) {
    capture_in_flight_ = false;
    set_status(tr("Rejected — %1").arg(reason));
    if (!capture_done_) enable_capture(camera_ && camera_->is_connected());
    update_delete_enabled();
}

void CalibrationWizard::on_delete_capture() {
    // One delete per accept: drop the flag immediately so a second click (or a
    // rapid double-click) cannot remove the prior capture. The worker's
    // remove_last_frame is queued and will confirm via frame_deleted, at which
    // point the progress bar and preview are rolled back to match.
    if (!can_delete_last_) return;
    can_delete_last_ = false;
    update_delete_enabled();
    set_status(tr("Discarding last capture…"));
    emit delete_last_capture_requested();
}

void CalibrationWizard::on_frame_deleted(std::size_t remaining) {
    // Worker confirmed the last observation was removed. Roll back the progress
    // bar to its authoritative count and pop the parallel preview stack so the
    // preview reverts to the now-last capture (or the placeholder if empty).
    progress_->setValue(static_cast<int>(remaining));
    if (!captured_previews_.empty()) captured_previews_.pop_back();
    if (!captured_points_.empty()) captured_points_.pop_back();
    update_coverage_overlay();
    show_last_preview();
    if (remaining > 0) {
        set_status(tr("Last capture discarded. %1 frame(s) remain.").arg(remaining));
    } else {
        set_status(tr("Last capture discarded. No frames remain."));
    }
    update_delete_enabled();
}

void CalibrationWizard::on_capture_complete(std::size_t accepted) {
    capture_in_flight_ = false;
    capture_done_ = true;
    enable_capture(false);
    // Target reached → calibration is running on the worker. Deleting now would
    // not affect the already-computed result, so block it. The user must Reset
    // (which clears the preview stack and can_delete_last_) to start over.
    can_delete_last_ = false;
    update_delete_enabled();
    // Run the two-pass cv::calibrateCamera on the worker thread.
    set_status(tr("Captured %1 frames. Running calibration…").arg(accepted));
    emit run_calibration_requested();
}

void CalibrationWizard::on_calibration_done(bool ok, double rms, int frames_used,
                                            int removed_frames, QString error) {
    if (ok) {
        set_status(tr("Calibration OK. RMS = %1 px (%2 kept, %3 removed). "
                      "Click Export to save.")
            .arg(rms, 0, 'f', 3).arg(frames_used).arg(removed_frames));
        export_btn_->setEnabled(true);
    } else {
        QMessageBox::warning(this, tr("Calibration failed"), error);
        set_status(tr("Calibration failed: %1").arg(error));
    }
}

void CalibrationWizard::on_export_pressed() {
    // Create the default parent directory BEFORE opening the dialog: it does
    // not exist until the first export (the worker auto-mkdirs on save), and
    // QFileDialog resolves a non-existent directory to the CURRENT WORKING
    // DIRECTORY — so the very first export would otherwise default to
    // <launch-dir>/intrinsic.yml (e.g. the build directory) instead of the
    // shared default, diverging from the undistort preprocessor's path.
    QDir().mkpath(QFileInfo(default_export_path()).absolutePath());
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
    can_delete_last_ = false;
    captured_previews_.clear();
    captured_points_.clear();
    update_coverage_overlay();
    progress_->setValue(0);
    preview_label_->clear();
    preview_label_->setText(tr("No frames captured yet."));
    export_btn_->setEnabled(false);
    // Reset the worker's accumulated observations (queued).
    QMetaObject::invokeMethod(worker_, "reset", Qt::QueuedConnection);
    enable_capture(camera_ && camera_->is_connected());
    update_delete_enabled();
    set_status(tr("Point the camera at the blinking chessboard and press Space to capture."));
}

void CalibrationWizard::on_config_changed() {
    apply_pattern_to_display();
    configure_worker();
    // Geometry change invalidates accumulated observations — reset the view.
    capture_in_flight_ = false;
    capture_done_ = false;
    can_delete_last_ = false;
    captured_previews_.clear();
    captured_points_.clear();
    update_coverage_overlay();
    progress_->setValue(0);
    preview_label_->clear();
    preview_label_->setText(tr("No frames captured yet."));
    export_btn_->setEnabled(false);
    QMetaObject::invokeMethod(worker_, "reset", Qt::QueuedConnection);
    enable_capture(camera_ && camera_->is_connected());
    update_delete_enabled();
}

void CalibrationWizard::set_status(const QString& text) {
    if (status_) status_->setText(text);
}

void CalibrationWizard::enable_capture(bool on) {
    capture_btn_->setEnabled(on);
    capture_btn_->setToolTip(on ? QString() :
        tr("Connect a camera first, then press Space to capture."));
}

void CalibrationWizard::update_delete_enabled() {
    // Enabled only when the last accept has not been deleted, no capture is in
    // flight, and the target has not been reached. This enforces the contract:
    // one delete per capture, before the final set is locked in. A second
    // consecutive delete (can_delete_last_ == false) is blocked here, so it can
    // never reach the worker to drop the prior capture.
    const bool on = can_delete_last_ && !capture_in_flight_ && !capture_done_;
    delete_capture_btn_->setEnabled(on);
}

void CalibrationWizard::update_coverage_overlay() {
    int sw = 0, sh = 0;
    if (camera_) {
        const auto& info = camera_->sensor_info();
        sw = info.width;
        sh = info.height;
    }
    camera_view_->set_coverage_overlay(
        compute_coverage_overlay(captured_points_, sw, sh), captured_points_);
}

void CalibrationWizard::show_last_preview() {
    // Render the top of the preview stack (the current last-accepted capture)
    // scaled to the scroll-area width, or the placeholder when empty/null.
    // QImage is implicitly shared, so QPixmap::fromImage makes the only deep
    // copy needed for display.
    if (captured_previews_.empty() || captured_previews_.back().isNull()) {
        preview_label_->clear();
        preview_label_->setText(tr("No frames captured yet."));
        return;
    }
    const QImage& img = captured_previews_.back();
    preview_label_->setPixmap(QPixmap::fromImage(img).scaledToWidth(
        preview_area_->viewport()->width(), Qt::SmoothTransformation));
    preview_label_->resize(preview_label_->pixmap().size());
}

void CalibrationWizard::apply_pattern_to_display() {
    if (pattern_) {
        pattern_->set_pattern(kGridCols, kGridRows);
        pattern_->set_square_size_mm(static_cast<float>(square_mm_->value()));
    }
}

void CalibrationWizard::configure_worker() {
    emit configure_requested(square_mm_->value(),
                             target_frames_->value());
}

QString CalibrationWizard::default_export_path() const {
    // Single source of truth shared with the Preprocessor undistort default
    // (see calib_defaults.h) — the user can rely on both defaults pointing at
    // the same file. QFileDialog prompts for overwrite if it already exists;
    // the parent directory is created on export (auto-mkdir).
    return default_intrinsic_yml_path();
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
    // annotated image. Both stay memory-light.
    auto* top_row = new QHBoxLayout();

    // Parameters column.
    auto* params_widget = new QWidget(this);
    auto* form = new QFormLayout(params_widget);
    form->setContentsMargins(0, 0, 0, 0);

    // Capture window (µs), 200–200000 in 1 µs steps. The chessboard blinks at
    // 10 ms (a 20 ms full cycle); the window must cover ≥ one full cycle so
    // every black square fires both polarities.
    capture_window_ = new QSpinBox(params_widget);
    capture_window_->setRange(kCaptureWindowMinUs, kCaptureWindowMaxUs);
    capture_window_->setSingleStep(kCaptureWindowStepUs);
    capture_window_->setValue(kDefaultCaptureWindowUs);
    capture_window_->setSuffix(tr(" µs"));
    capture_window_->setToolTip(tr("Event capture window in microseconds "
        "(200–200000). The chessboard blinks at 10 ms; the window must cover at "
        "least one full blink cycle (20 ms) so every black square fires both "
        "polarities. Longer windows gather a denser board signal."));
    form->addRow(tr("Capture window"), capture_window_);

    square_mm_ = new QDoubleSpinBox(params_widget);
    square_mm_->setRange(0.1, 500.0);
    square_mm_->setDecimals(2);
    square_mm_->setValue(kDefaultSquareMm);
    square_mm_->setSuffix(tr(" mm"));
    square_mm_->setToolTip(tr("Physical length (mm) of one chessboard square "
        "edge. Sets the real-world scale for calibration; does NOT change the "
        "on-screen pattern size (screen DPI is deliberately not used)."));
    form->addRow(tr("Square size"), square_mm_);

    // Measurement instruction: one short sentence.
    spacing_note_ = new QLabel(
        tr("Measure the length of one chessboard square edge, then enter it in mm."),
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

    // ---- Large blinking chessboard pattern (full width, dominant) ----
    pattern_ = new BlinkingChessboardDisplay(this);
    pattern_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pattern_->setMinimumSize(400, 200);
    apply_pattern_to_display();
    outer->addWidget(pattern_, 1);

    // ---- Capture hint ----
    hint_ = new QLabel(tr(
        "The chessboard blinks to generate events — hold the camera steady. "
        "Press <b>Space</b> to capture. Move the camera between captures so "
        "every pose differs, and bring the board to the center AND the "
        "corners of the view — the coverage hull on the aim view shows where "
        "you have already captured."), this);
    hint_->setWordWrap(true);
    hint_->setProperty("class", "hint");
    outer->addWidget(hint_);

    // ---- Controls row: buttons + progress ----
    auto* btns = new QHBoxLayout();
    capture_btn_ = new QPushButton(tr("Capture"), this);
    // "Delete this capture" sits immediately right of Capture. It discards the
    // most recently accepted frame after the user inspects its annotated
    // preview — that frame is then excluded from the final intrinsic
    // calculation. Gated by can_delete_last_ (one delete per accept: a second
    // consecutive delete that would drop the prior capture is blocked) and
    // disabled while a capture is in flight or once the target is reached.
    delete_capture_btn_ = new QPushButton(tr("Delete this capture"), this);
    delete_capture_btn_->setEnabled(false);
    delete_capture_btn_->setToolTip(tr(
        "Discard the last accepted capture (after inspecting its preview) so it "
        "is not used in the intrinsic calculation. Only the most recent capture "
        "can be deleted — one delete per capture; capture again to delete the "
        "next one."));
    reset_btn_   = new QPushButton(tr("Reset"), this);
    export_btn_  = new QPushButton(tr("Export..."), this);
    enable_capture(false);
    export_btn_->setEnabled(false);
    btns->addWidget(capture_btn_);
    btns->addWidget(delete_capture_btn_);
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

    // Config-change handlers. The grid is fixed at 9×6 (no cols/rows spinbox),
    // so there is no square-grid rejection. capture_window/square_mm/target all
    // trigger a re-configure + display refresh.
    connect(capture_window_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int) { on_config_changed(); });
    connect(square_mm_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
        [this](double) { on_config_changed(); });
    connect(target_frames_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int v) { progress_->setRange(0, v); on_config_changed(); });
    connect(capture_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_capture_pressed);
    connect(delete_capture_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_delete_capture);
    connect(reset_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_intrinsic_reset);
    connect(export_btn_, &QPushButton::clicked, this, &CalibrationWizard::on_export_pressed);

    on_intrinsic_reset();
}

} // namespace gui
