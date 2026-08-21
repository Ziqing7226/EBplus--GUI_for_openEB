// gui/app/camera_controller.cpp

#include "camera_controller.h"

#include <QMetaObject>
#include <QString>

#include <filesystem>

#include <cstdint>

#include <metavision/sdk/stream/camera_error_code.h>
#include <metavision/sdk/stream/camera_exception.h>
#include <metavision/sdk/stream/file_config_hints.h>

namespace gui {

namespace {
// OOM guard for file playback (audit §六-C2a). RAW Evt3 encodes events at
// ~8 bytes/event on average (CD events dominate; headers/time-high words
// amortized), so file_size / 8 is a rough event-count estimate. Buffered
// Metavision::EventCD is 16 bytes/event, so 150M events ≈ 2.4 GB resident
// in the FileFrameGenerator buffer — warn above that, but never block the
// open: the user decides whether to continue.
constexpr unsigned long long kEvt3BytesPerEventEstimate = 8;
constexpr unsigned long long kWarnEventCount = 150'000'000;
} // namespace

CameraController::CameraController(QObject* parent)
    : QObject(parent), frame_pipeline_(nullptr), statistics_(nullptr) {
    // Surface the FileFrameGenerator's OOM guard (audit §六-C2b) through
    // the existing warning chain (status bar in MainWindow). The signal
    // is emitted from the SDK streaming thread; Qt queues it here.
    connect(&frame_pipeline_, &FramePipeline::file_buffer_truncated,
            this, [this]() {
                emit runtime_warning(
                    tr("Event buffer memory limit reached; events beyond "
                       "this point were discarded."));
            });
}

CameraController::~CameraController() {
    teardown();
}

std::vector<std::pair<QString, QString>> CameraController::list_online_sources() {
    std::vector<std::pair<QString, QString>> out;
    try {
        const auto sources = Metavision::Camera::list_online_sources();
        for (const auto& kv : sources) {
            QString type_label;
            switch (kv.first) {
                case Metavision::OnlineSourceType::EMBEDDED: type_label = "Embedded"; break;
                case Metavision::OnlineSourceType::USB:      type_label = "USB"; break;
                case Metavision::OnlineSourceType::REMOTE:   type_label = "Remote"; break;
                default:                                     type_label = "Other"; break;
            }
            for (const auto& serial : kv.second) {
                out.emplace_back(type_label, QString::fromStdString(serial));
            }
        }
    } catch (const Metavision::CameraException&) {
        // ignore — return empty list
    }
    return out;
}

bool CameraController::connect_first_available() {
    teardown();
    try {
        // Use from_serial("") instead of from_first_available(). The latter
        // internally calls Camera::list_online_sources() (full local + remote
        // scan) to locate a camera — redundant with the list already shown in
        // the Devices panel. from_serial("") delegates to
        // DeviceDiscovery::open("") which opens the first available *local*
        // camera directly, skipping both the redundant scan and the slow
        // remote discovery.
        auto cam = Metavision::Camera::from_serial(std::string());
        setup_camera(std::move(cam), false);
        return true;
    } catch (const Metavision::CameraException& e) {
        // teardown() already destroyed the previous camera/pipeline but never
        // emits disconnected() — do so here so the UI cleans up its stale
        // connection state (status bar, panels, playback controls) before
        // the error dialog appears.
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::connect_serial(const std::string& serial) {
    teardown();
    try {
        auto cam = Metavision::Camera::from_serial(serial);
        setup_camera(std::move(cam), false);
        return true;
    } catch (const Metavision::CameraException& e) {
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::connect_file(const std::string& path) {
    teardown();
    // OOM guard (audit §六-C2a): estimate the event count from the file
    // size BEFORE opening (RAW Evt3 ≈ 8 bytes/event) and warn if the
    // buffer would grow huge. Non-blocking: the file still opens.
    unsigned long long estimated_events = 0;
    {
        std::error_code ec;
        const auto size = std::filesystem::file_size(path, ec);
        if (!ec) {
            estimated_events = size / kEvt3BytesPerEventEstimate;
        }
    }
    try {
        // Always use real_time_playback=false: read all events as fast as
        // possible and buffer them in the FileFrameGenerator. Playback rate
        // is controlled by the FileFrameGenerator's QTimer, not by the SDK's
        // delivery rate.
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        auto cam = Metavision::Camera::from_file(path, hints);
        setup_camera(std::move(cam), true);
        if (estimated_events > kWarnEventCount) {
            emit runtime_warning(
                tr("Very large file (est. %1M events): playback may use a "
                   "lot of memory.").arg(estimated_events / 1'000'000));
        }
        return true;
    } catch (const Metavision::CameraException& e) {
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

void CameraController::disconnect() {
    teardown();
    emit disconnected();
}

bool CameraController::start() {
    if (!camera_) {
        return false;
    }
    try {
        if (!camera_->is_running()) {
            camera_->start();
        }
        // Don't emit started() here: the status-change callback fires it
        // exactly once when the SDK confirms the STARTED transition.
        return true;
    } catch (const Metavision::CameraException& e) {
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::stop() {
    if (!camera_) {
        return false;
    }
    try {
        if (camera_->is_running()) {
            camera_->stop();
        }
        // Don't emit stopped() here: the status-change callback fires it
        // exactly once when the SDK confirms the STOPPED transition. For
        // runtime errors (file EOF, disconnect), the error callback also
        // emits stopped() so the UI is notified even if the status callback
        // never fires.
        return true;
    } catch (const Metavision::CameraException& e) {
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::is_running() const {
    return camera_ && camera_->is_running();
}

// ---------------------------------------------------------------------------
// Phase 2 facility accessors
// ---------------------------------------------------------------------------
// All go through Device::get_facility<T>() which returns a nullable pointer
// (vs Camera::get_facility<T>() which throws on unsupported features). This
// lets the GUI degrade gracefully by disabling the corresponding panel.
facility::Biases* CameraController::biases_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::Biases>();
}
facility::Roi* CameraController::roi_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::Roi>();
}

bool CameraController::set_unified_roi(bool enabled, int x, int y, int w, int h,
                                       std::optional<bool> roni) {
    const bool roni_mode = roni.value_or(roi_roni_);
    // Snapshot the OLD output geometry for the transition ordering below.
    const int old_out_w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_
                                                       : sensor_info_.width;
    const int old_out_h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_
                                                       : sensor_info_.height;
    if (is_file_) {
        // File playback: no hardware — software crop with the same
        // "sensor outputs only ROI events" semantics (Phase 2.6). RONI
        // inverts the crop (Phase 2.6 debug D-5). No ordering constraints
        // here: rendering and the bridge resize both run on the GUI thread
        // and cannot interleave.
        frame_pipeline_.set_file_roi(enabled, x, y, w, h, roni_mode);
        roi_roni_ = roni_mode;
        bool en = false;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        frame_pipeline_.file_roi(en, x0, y0, x1, y1);
        emit roi_state_changed(en, x0, y0, x1, y1);
        return true;
    }
    auto* roi = roi_facility();
    if (!roi) return false;
    try {
        // Compute the window (auto-center on -1, clamp to sensor), mirroring
        // ProcessRegion::compute so live and file paths agree.
        const int sw = sensor_info_.width > 0 ? sensor_info_.width : 1280;
        const int sh = sensor_info_.height > 0 ? sensor_info_.height : 720;
        const int rw = (w <= 0) ? sw : std::min(w, sw);
        const int rh = (h <= 0) ? sh : std::min(h, sh);
        const int rx = (x < 0) ? (sw - rw) / 2 : std::min(std::max(0, x), sw - rw);
        const int ry = (y < 0) ? (sh - rh) / 2 : std::min(std::max(0, y), sh - rh);
        if (rw <= 0 || rh <= 0) return false;
        // Phase 2.6 debug D-5: the mode is part of the unified state (was
        // hardcoded ROI, clobbering RONI set via the RoiPanel), and the
        // window/mode are configured even when disabling so callers can
        // pre-configure a rect while the ROI is off (mirrors the file path,
        // which stores the rect unconditionally).
        roi->set_mode(roni_mode ? Metavision::I_ROI::Mode::RONI
                                : Metavision::I_ROI::Mode::ROI);
        roi->set_windows({Metavision::I_ROI::Window(rx, ry, rw, rh)});
        roi->enable(enabled);
        roi_enabled_ = enabled;
        roi_roni_ = roni_mode;
        roi_x0_ = rx; roi_y0_ = ry;
        roi_x1_ = rx + rw; roi_y1_ = ry + rh;
    } catch (const std::exception&) {
        return false;
    }
    // Conditioner follows the ROI (highest priority): ROI mode → crop+shift
    // to ROI-relative (canonical for algorithms); RONI → drop-inside,
    // absolute coordinates. The DISPLAY keeps the full-sensor frame (content
    // shifted back to its absolute position by FramePipeline; the rest stays
    // the palette background) — the overlay / Zoom-to-ROI / Replace-composite
    // strategies are built around full frames.
    //
    // TRANSITION ORDERING (review 2026-08-21, race fix): the conditioner
    // decides the coordinate regime read by the SDK thread; roi_state_changed
    // synchronously resizes backends (direct connection). In-flight batches
    // must stay in-bounds in EVERY intermediate state, so order by geometry:
    //   output SHRINKS (new dims ≤ old) → conditioner first (new small
    //     coords into not-yet-shrunk big buffers — safe, no loss);
    //   output GROWS (new dims > old)   → resize first (old small coords
    //     into already-grown buffers — safe; the conditioner keeps dropping
    //     outside-the-old-rect events for the few ms until it flips, events
    //     hw I_ROI was discarding the instant before anyway).
    // No hot-path cost: this only reorders statements on a user action.
    const int new_out_w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_
                                                       : sensor_info_.width;
    const int new_out_h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_
                                                       : sensor_info_.height;
    const auto apply_conditioner = [this] {
        conditioner_.set_roi(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_,
                             roi_roni_);
        frame_pipeline_.set_display_roi_origin(roi_enabled_ && !roi_roni_,
                                               roi_x0_, roi_y0_);
    };
    const bool shrink = new_out_w <= old_out_w && new_out_h <= old_out_h;
    const bool grow = new_out_w >= old_out_w && new_out_h >= old_out_h;
    if (shrink) {
        // Conditioner first: new small coords into not-yet-shrunk buffers.
        apply_conditioner();
        emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
    } else if (grow) {
        // Resize first: old small coords into already-grown buffers. The
        // conditioner keeps dropping outside-the-old-rect events for the
        // few ms until it flips — events hw I_ROI was discarding the
        // instant before anyway.
        emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
        apply_conditioner();
    } else {
        // MIXED rect change (one axis shrinks, the other grows — reachable
        // via the ROI settings dialog): NO single order is safe; both leave
        // one axis's coordinates out of bounds in one intermediate state.
        // Route through a min-dims intermediate instead:
        //   1. conditioner -> min rect at the NEW origin: coords ≤ min dims,
        //      which fit BOTH the old and the future backend buffers;
        //   2. resize to the final rect (min coords fit the new dims);
        //   3. conditioner -> final rect.
        // Costs a transient drop of events outside the min sub-rect (ms
        // scale) — the margin hw I_ROI is already delivering at the new
        // rect. Mixed only occurs rect→rect (rect-vs-sensor is always
        // shrink-or-grow since rects are clamped inside the sensor).
        const int min_w = std::min(old_out_w, new_out_w);
        const int min_h = std::min(old_out_h, new_out_h);
        conditioner_.set_roi(true, roi_x0_, roi_y0_,
                             roi_x0_ + min_w, roi_y0_ + min_h, false);
        frame_pipeline_.set_display_roi_origin(true, roi_x0_, roi_y0_);
        emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
        apply_conditioner();
    }
    return true;
}

void CameraController::unified_roi(bool& enabled, int& x0, int& y0,
                                   int& x1, int& y1) const {
    if (is_file_) {
        frame_pipeline_.file_roi(enabled, x0, y0, x1, y1);
        return;
    }
    enabled = roi_enabled_;
    x0 = roi_x0_; y0 = roi_y0_; x1 = roi_x1_; y1 = roi_y1_;
}
facility::AntiFlicker* CameraController::anti_flicker_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::AntiFlicker>();
}
facility::TrailFilter* CameraController::trail_filter_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::TrailFilter>();
}
facility::Erc* CameraController::erc_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::Erc>();
}
facility::TriggerIn* CameraController::trigger_in_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::TriggerIn>();
}
facility::TriggerOut* CameraController::trigger_out_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::TriggerOut>();
}
facility::CameraSync* CameraController::camera_sync_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::CameraSync>();
}

void CameraController::set_conditioned_listener(ConditionedListener cb) {
    std::lock_guard<std::mutex> lk(conditioned_mutex_);
    conditioned_listener_ = std::move(cb);
}

bool CameraController::set_conditioner_param(const std::string& key,
                                             const std::string& value) {
    // Live conditioner + the file generator's conditioner (only one is ever
    // active; the inactive one just stores the value).
    const bool ok = conditioner_.set_param(key, value);
    frame_pipeline_.set_file_conditioner_param(key, value);
    return ok;
}

void CameraController::reset_conditioner() {
    conditioner_.reset_temporal();
    frame_pipeline_.reset_file_conditioner();
}

bool CameraController::conditioner_active() const {
    return conditioner_.active();
}

void CameraController::set_cd_broadcast(bool enabled) {
    cd_broadcast_.store(enabled, std::memory_order_relaxed);
}

// --- Auto bias (§4.4.6) -----------------------------------------------------

bool CameraController::set_auto_bias_enabled(bool on) {
    if (on == auto_bias_enabled()) return on;
    if (on) {
        if (is_file_ || !camera_) return false;
        if (!bias_applier_.attach(biases_facility())) return false;
        {
            std::lock_guard<std::mutex> lk(auto_bias_mutex_);
            auto_bias_ctrl_.reset();
            auto_bias_last_tick_ = -1;
        }
        auto_bias_enabled_.store(true, std::memory_order_relaxed);
    } else {
        auto_bias_enabled_.store(false, std::memory_order_relaxed);
        if (bias_applier_.attached()) {
            bias_applier_.restore();
            bias_applier_.detach();
            emit auto_bias_applied();
        }
        std::lock_guard<std::mutex> lk(auto_bias_mutex_);
        auto_bias_ctrl_.reset();
    }
    return true;
}

bool CameraController::set_auto_bias_rate_bounds(float lo_mev, float hi_mev) {
    std::lock_guard<std::mutex> lk(auto_bias_mutex_);
    return auto_bias_ctrl_.set_rate_bounds(lo_mev, hi_mev);
}

void CameraController::auto_bias_rate_bounds(float& lo_mev, float& hi_mev) const {
    std::lock_guard<std::mutex> lk(auto_bias_mutex_);
    lo_mev = auto_bias_ctrl_.rate_min_mev();
    hi_mev = auto_bias_ctrl_.rate_max_mev();
}

void CameraController::auto_bias_tick(const Metavision::EventCD* b,
                                      const Metavision::EventCD* e) {
    if (b == e) return;
    std::uint32_t n_on = 0, n_off = 0;
    for (auto* it = b; it != e; ++it) {
        if (it->p != 0) ++n_on; else ++n_off;
    }
    const std::int64_t t = (e - 1)->t;
    gui_algo::BiasCommand cmd;
    {
        std::lock_guard<std::mutex> lk(auto_bias_mutex_);
        auto_bias_ctrl_.accumulate(t, n_on, n_off);
        if (auto_bias_last_tick_ < 0) auto_bias_last_tick_ = t;
        if (t - auto_bias_last_tick_ >= gui_algo::AutoBiasController::kTickUs) {
            cmd = auto_bias_ctrl_.update(t);
            auto_bias_last_tick_ = t;
        }
    }
    if (!cmd.active) return;
    // Apply on the GUI thread: the register writes are rare (each is
    // followed by a hold + measurement refill) and the Biases panel can
    // safely re-read the hardware afterwards.
    const bool home = cmd.home;
    QMetaObject::invokeMethod(this, [this, home, cmd]() {
        // The controller may have been disabled since the tick — the
        // snapshot restore already ran, don't fight it.
        if (!auto_bias_enabled_.load(std::memory_order_relaxed)) return;
        // Home commands move both biases half the remaining distance to 0
        // (factory default), clipped to [1, kDefaultMaxStep]; at 0 they
        // no-op and emit nothing.
        const bool changed =
            home ? bias_applier_.home(gui_algo::AutoBiasController::kDefaultMaxStep)
                 : bias_applier_.apply(cmd.delta_on, cmd.delta_off) !=
                       BiasApplier::Status::NoBias;
        if (changed) emit auto_bias_applied();
    }, Qt::QueuedConnection);
}

// ---------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------

void CameraController::setup_camera(Metavision::Camera&& cam, bool is_file) {
    is_file_ = is_file;
    camera_ = std::make_unique<Metavision::Camera>(std::move(cam));
    fetch_sensor_info();

    // Runtime error callback: file EOF, disconnects, firmware errors arrive here.
    // Capture the camera pointer: this callback's queued lambdas may execute
    // AFTER the user has connected a different source — without the identity
    // check, a stale error from source A would stop the freshly-connected
    // source B and emit a spurious stopped() (audit §六-C1).
    err_cb_id_ = camera_->add_runtime_error_callback(
        [this, cam = camera_.get()](const Metavision::CameraException& e) {
            // Reaching end-of-file is a normal stop condition for playback.
            const QString msg = QString::fromUtf8(e.what());

            // Evt3 "NonMonotonicTimeHigh" is a transient HAL-layer warning
            // that occurs ~50% of the time when starting Gen3.x cameras.
            // The timestamp high bits momentarily go backwards, but the
            // camera keeps streaming and the frame pipeline handles the
            // timestamp gap gracefully. Stopping the camera on this error
            // (the previous behavior) made the camera fail half the time.
            // Treat it as a non-fatal warning and keep the stream running.
            const bool is_evt3_time_glitch =
                msg.contains(QStringLiteral("NonMonotonicTimeHigh"), Qt::CaseInsensitive) ||
                msg.contains(QStringLiteral("Evt3 protocol violation"), Qt::CaseInsensitive);

            if (is_evt3_time_glitch) {
                // Transient Evt3 timestamp glitch — ignore for BOTH live and
                // file sources.  Gen3 raw files frequently contain
                // NonMonotonicTimeHigh warnings; treating them as EOF (the
                // previous behaviour for is_file_) stopped playback before
                // any frames were visible.
                emit runtime_warning(tr("Transient timestamp glitch (ignored): %1").arg(msg));
            } else if (is_file_) {
                // File source + non-glitch error → genuine EOF.
                emit runtime_warning(tr("Playback ended: %1").arg(msg));
                QMetaObject::invokeMethod(this, [this, cam]() {
                    if (camera_.get() != cam) return;  // stale callback, see above
                    // The whole file is now buffered: allow the
                    // FileFrameGenerator's EOF handling (stop / loop wrap)
                    // to engage (audit §六-P2).
                    frame_pipeline_.set_file_loading_complete(true);
                    if (camera_->is_running()) {
                        try { camera_->stop(); } catch (...) {}
                    }
                    emit stopped();
                }, Qt::QueuedConnection);
            } else {
                // Live camera + genuine error: stop and report.
                emit error(msg);
                QMetaObject::invokeMethod(this, [this, cam]() {
                    if (camera_.get() != cam) return;  // stale callback, see above
                    if (camera_->is_running()) {
                        try { camera_->stop(); } catch (...) {}
                    }
                    emit stopped();
                }, Qt::QueuedConnection);
            }
        });

    // Status change callback.
    status_cb_id_ = camera_->add_status_change_callback(
        [this](const Metavision::CameraStatus& status) {
            if (status == Metavision::CameraStatus::STARTED) {
                emit started();
            } else {
                // A file source stopping on its own means EOF: everything
                // the file will ever yield is now buffered. Signal
                // loading-complete so the FileFrameGenerator's EOF handling
                // (stop / loop wrap) can engage. The runtime-error EOF path
                // (above) also does this, but the SDK does not guarantee an
                // error callback at EOF — relying on it alone left
                // loading_complete_ unset and loop playback stalled at the
                // buffer top forever. Idempotent; on user-initiated stops
                // the pipeline is being torn down anyway.
                if (is_file_) {
                    frame_pipeline_.set_file_loading_complete(true);
                }
                emit stopped();
            }
        });

    // CD callback: forward events to the frame pipeline + statistics.
    // The SDK dispatches this callback on its streaming/decoding thread with
    // NO try/catch, so an exception escaping the lambda would call
    // std::terminate and crash the whole GUI with no diagnostic. The
    // conditioner may reallocate its buffers, so std::bad_alloc at high event
    // rates is plausible. Wrap the body and surface failures to the GUI
    // thread.
    cd_cb_id_ = camera_->cd().add_callback(
        [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            try {
                statistics_.add_events(b, e);
                if (is_file_) {
                    // File mode: buffer RAW events — conditioning happens
                    // per-frame in FileFrameGenerator::render_frame() so
                    // toggles take effect immediately during playback.
                    frame_pipeline_.add_events(b, e);
                } else {
                    // Auto bias measures the RAW sensor output (biases act
                    // before any software conditioning).
                    if (auto_bias_enabled_.load(std::memory_order_relaxed)) {
                        auto_bias_tick(b, e);
                    }
                    // Live mode: condition ONCE (unified ROI → polarity
                    // stages → noise filter → thin → undistort → flips).
                    // Display, processed recording and the algorithm
                    // listener all consume the SAME output span.
                    const auto [cb, cn] = conditioner_.apply(b, e);
                    const auto* ce = cb + cn;
                    // cn == 0 (a stage emptied the batch): skip the display
                    // push — an empty span may carry a null data() on first
                    // use, and the old pre-rework paths guarded exactly this
                    // (`if (!filtered.empty())`). The listener still runs: it
                    // owns the raw-count profiler tick for empty batches.
                    if (cn > 0) {
                        frame_pipeline_.add_events(cb, ce);
                    }
                    ConditionedListener listener;
                    {
                        std::lock_guard<std::mutex> lk(conditioned_mutex_);
                        listener = conditioned_listener_;
                    }
                    if (listener) listener(b, e, cb, ce);
                }
                // Optional CD broadcast for calibration tools — always the
                // RAW span. The atomic check is cheap; the buffer comes from
                // the reusable pool (no allocation in steady state) and only
                // happens when a listener has explicitly opted in via
                // set_cd_broadcast(true). The emit crosses to the GUI thread
                // via Qt's queued-connection machinery (the shared_ptr is
                // captured by value).
                if (cd_broadcast_.load(std::memory_order_relaxed) && b != e) {
                    auto batch = broadcast_pool_.acquire();
                    batch->assign(b, e);
                    emit cd_events_ready(batch);
                }
            } catch (const std::exception& ex) {
                QMetaObject::invokeMethod(this, [this, msg = std::string(ex.what())]() {
                    emit runtime_warning(QString::fromUtf8(msg.c_str()));
                }, Qt::QueuedConnection);
            } catch (...) {
                // Swallow to keep the stream alive; the SDK thread must not
                // propagate exceptions out of the callback.
            }
        });

    statistics_.reset();
    filter_chain_.set_geometry(sensor_info_.width, sensor_info_.height);
    // Conditioner: live batches only (the file source conditions per-frame
    // in FileFrameGenerator). reset() clears temporal state so a reconnect's
    // timestamps never see stale surfaces.
    conditioner_.init(sensor_info_.width, sensor_info_.height);
    conditioner_.set_filter_chain(&filter_chain_);
    conditioner_.reset_temporal();

    // Start the frame pipeline for the new sensor geometry. File sources use
    // FileFrameGenerator (buffers events, controls playback rate via QTimer);
    // live sources use CDFrameGenerator (shows latest accumulation window).
    // fps_ / accumulation_us_ / fps_limit_ persist across stop/start cycles
    // so user settings survive camera reconnects and file reopens.
    const long w = sensor_info_.width;
    const long h = sensor_info_.height;
    const std::uint16_t fps = frame_pipeline_.fps();
    const Metavision::timestamp acc = frame_pipeline_.accumulation_time_us();
    if (is_file) {
        frame_pipeline_.set_file_filter_chain(&filter_chain_);
        if (!frame_pipeline_.start_file(w, h, fps, acc)) {
            // Without a running pipeline the display stays black forever —
            // abort the connection instead of reporting "Connected"
            // (audit §六-C4).
            teardown();
            emit disconnected();
            emit error(tr("Failed to start file frame pipeline."));
            return;
        }
    } else {
        if (!frame_pipeline_.start(w, h, fps, acc)) {
            teardown();
            emit disconnected();
            emit error(tr("Failed to start frame pipeline."));
            return;
        }
    }

    emit connected(sensor_info_);
}

void CameraController::teardown() {
    // 1. Remove the SDK callbacks FIRST so the SDK thread stops calling into
    //    FramePipeline / FilterChain / StatisticsController. Without this,
    //    stopping the pipeline (which resets generator_) races with the CD
    //    callback's frame_pipeline_.add_events() — a use-after-free.
    // Also disable CD broadcast so no in-flight emit references the camera.
    cd_broadcast_.store(false, std::memory_order_relaxed);
    // Auto bias: stop the control loop before the device goes away. The
    // register writes would fail — the sensor keeps its current biases.
    auto_bias_enabled_.store(false, std::memory_order_relaxed);
    bias_applier_.detach();
    {
        std::lock_guard<std::mutex> lk(auto_bias_mutex_);
        auto_bias_ctrl_.reset();
    }
    if (camera_) {
        if (cd_cb_id_) {
            camera_->cd().remove_callback(*cd_cb_id_);
            cd_cb_id_.reset();
        }
        if (err_cb_id_) {
            camera_->remove_runtime_error_callback(*err_cb_id_);
            err_cb_id_.reset();
        }
        if (status_cb_id_) {
            camera_->remove_status_change_callback(*status_cb_id_);
            status_cb_id_.reset();
        }
        if (camera_->is_running()) {
            try { camera_->stop(); } catch (...) {}
        }
        camera_.reset();
    }

    // 2. Now that no SDK thread can touch it, stop the frame pipeline.
    frame_pipeline_.stop();

    sensor_info_ = SensorInfo{};
    is_file_ = false;
}

void CameraController::fetch_sensor_info() {
    SensorInfo info;
    info.is_file = is_file_;
    if (!camera_) {
        sensor_info_ = info;
        return;
    }
    try {
        const auto& g = camera_->geometry();
        info.width = g.get_width();
        info.height = g.get_height();
    } catch (...) {}
    try {
        const auto& cfg = camera_->get_camera_configuration();
        info.serial = QString::fromStdString(cfg.serial_number);
        info.integrator = QString::fromStdString(cfg.integrator);
        info.plugin_name = QString::fromStdString(cfg.plugin_name);
        info.encoding_format = QString::fromStdString(cfg.data_encoding_format);
        info.firmware_version = QString::fromStdString(cfg.firmware_version);
    } catch (...) {}
    try {
        const auto& gen = camera_->generation();
        info.generation_name = QString::fromStdString(gen.name());
        info.generation_major = gen.version_major();
        info.generation_minor = gen.version_minor();
    } catch (...) {}
    sensor_info_ = info;
}

} // namespace gui
