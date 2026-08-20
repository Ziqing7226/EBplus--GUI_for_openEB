// gui/app/camera_controller.h — owns the Metavision::Camera lifecycle.
//
// Discovers, connects (live or file), wires the CD callback into the
// FramePipeline and StatisticsController, and exposes sensor metadata to the
// GUI. All cross-thread communication goes through queued Qt signals.

#ifndef GUI_APP_CAMERA_CONTROLLER_H
#define GUI_APP_CAMERA_CONTROLLER_H

#include <QObject>
#include <QString>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>
#include <metavision/hal/facilities/i_geometry.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_roi.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_trigger_out.h>
#include <metavision/sdk/base/utils/callback_id.h>
#include <metavision/sdk/base/utils/object_pool.h>
#include <metavision/sdk/stream/camera.h>

#include "frame_pipeline.h"
#include "stream_conditioner.h"
#include "statistics_controller.h"
#include "algo_bridge/filter_chain.h"

namespace gui {

// HAL facility aliases used by Phase 2 panels. Each is obtained via
// Camera::get_device().get_facility<T>() which returns a nullable pointer;
// panels must nullptr-check before use (graceful degradation when the
// connected sensor doesn't support a feature).
namespace facility {
using Biases       = Metavision::I_LL_Biases;
using Roi          = Metavision::I_ROI;
using AntiFlicker  = Metavision::I_AntiFlickerModule;
using TrailFilter  = Metavision::I_EventTrailFilterModule;
using Erc          = Metavision::I_ErcModule;
using TriggerIn    = Metavision::I_TriggerIn;
using TriggerOut   = Metavision::I_TriggerOut;
using CameraSync   = Metavision::I_CameraSynchronization;
using Geometry     = Metavision::I_Geometry;
} // namespace facility

/// @brief Snapshot of sensor metadata shown in the Information panel.
struct SensorInfo {
    int width{0};
    int height{0};
    QString serial;
    QString integrator;
    QString plugin_name;
    QString encoding_format;
    QString firmware_version;
    QString generation_name;
    short generation_major{0};
    short generation_minor{0};
    bool is_file{false};
};

class CameraController : public QObject {
    Q_OBJECT
public:
    explicit CameraController(QObject* parent = nullptr);
    ~CameraController();

    /// @brief Lists online camera sources as (type_label, serial) pairs.
    std::vector<std::pair<QString, QString>> list_online_sources();

    /// @brief Connects to the first available live camera. Returns false on failure.
    bool connect_first_available();
    /// @brief Connects to a camera by serial number.
    bool connect_serial(const std::string& serial);
    /// @brief Opens an event file (RAW / HDF5 / DAT) for playback. Always
    /// uses real_time_playback=false so all events are read as fast as
    /// possible and buffered in the FileFrameGenerator. Playback rate is
    /// controlled by the FileFrameGenerator's QTimer (fps * window / 1e6).
    bool connect_file(const std::string& path);

    void disconnect();

    bool start();
    bool stop();
    bool is_running() const;
    bool is_connected() const { return static_cast<bool>(camera_); }
    bool is_file_source() const { return is_file_; }

    /// @brief Returns the underlying Metavision::Camera (nullptr if none).
    Metavision::Camera* camera_handle() { return camera_.get(); }

    const SensorInfo& sensor_info() const { return sensor_info_; }
    FramePipeline* frame_pipeline() { return &frame_pipeline_; }
    StatisticsController* statistics() { return &statistics_; }
    FilterChain* filter_chain() { return &filter_chain_; }

    /// @brief Phase 2 facility accessors. Each returns nullptr when no camera
    /// is connected or the connected sensor does not support that feature.
    /// Panels must nullptr-check before invoking any method on the returned
    /// pointer. The pointer is only valid until the next disconnect()/connect.
    facility::Biases*      biases_facility();
    facility::Roi*         roi_facility();
    facility::AntiFlicker* anti_flicker_facility();
    facility::TrailFilter* trail_filter_facility();
    facility::Erc*         erc_facility();
    facility::TriggerIn*   trigger_in_facility();
    facility::TriggerOut*  trigger_out_facility();
    facility::CameraSync*  camera_sync_facility();

    /// @brief Unified ROI entry point (Phase 2.6): the single ROI concept.
    /// Live camera: applies the hardware ROI (I_ROI) so the sensor itself
    /// only outputs ROI events. File playback: forwards to FramePipeline's
    /// software crop (same semantics). @p x/@p y = -1 means auto-center the
    /// window on the sensor. @p roni selects ROI (keep-inside, default) vs
    /// RONI (drop-inside) mode; std::nullopt keeps the current mode.
    /// Returns false on failure (facility missing / invalid rect) — caller
    /// should NOT treat the ROI as applied.
    /// On success emits roi_state_changed with the computed rect (the single
    /// driver for the overlay frame, zoom button and algorithm path,
    /// Phase 2.6 debug D-5).
    bool set_unified_roi(bool enabled, int x, int y, int w, int h,
                         std::optional<bool> roni = std::nullopt);

    /// @brief Reads the current unified ROI state (computed rect
    /// [x0,x1) × [y0,y1]) for overlay rendering.
    void unified_roi(bool& enabled, int& x0, int& y0, int& x1, int& y1) const;

    /// @brief True when the unified ROI is in RONI (drop-inside) mode
    /// (Phase 2.6 debug D-5). In RONI mode events keep ABSOLUTE sensor
    /// coordinates (the source filters, nothing is translated) — consumers
    /// that shift coordinates (OverlayStrategy) or resize backends
    /// (AlgoBridge) must treat RONI as "no translation / no resize".
    bool unified_roi_roni() const { return roi_roni_; }

    /// @brief Enables/disables broadcasting of every CD batch via
    /// cd_events_ready(). When false (default), the CD callback takes the
    /// fast path with zero extra copies. Calibration tools flip this to true
    /// on start and back to false on stop so the SDK thread only pays the
    /// copy cost while a consumer is actively listening.
    void set_cd_broadcast(bool enabled);

    /// @brief Conditioned-listener signature: raw span [rb,re) as delivered
    /// by the SDK, plus the conditioned span [cb,ce) (ROI → polarity stages →
    /// noise filter → thin → undistort → flips — see StreamConditioner).
    using ConditionedListener = std::function<void(
        const Metavision::EventCD* rb, const Metavision::EventCD* re,
        const Metavision::EventCD* cb, const Metavision::EventCD* ce)>;

    /// @brief Registers the live-mode consumer of the conditioned stream
    /// (algorithm feeding, XYT display). Invoked on the SDK CD thread AFTER
    /// the display pipeline received the same span — one conditioning pass
    /// for everyone. File sources never invoke it (file playback feeds
    /// algorithms via FramePipeline::events_window_ready, synchronized with
    /// the displayed frame).
    void set_conditioned_listener(ConditionedListener cb);

    /// @brief Single entry for the panel's preproc_* parameters: forwards to
    /// the live conditioner AND the file generator's conditioner. Same keys
    /// as AlgorithmsPanel::apply_global_preproc emits.
    bool set_conditioner_param(const std::string& key, const std::string& value);

    /// @brief Clears the conditioners' temporal state (source restart, file
    /// seek/loop — event time jumps backward and stale timestamp surfaces
    /// would suppress events).
    void reset_conditioner();

    /// @brief True when any conditioning stage would modify the stream.
    /// Used by RecorderController to choose processed-stream recording.
    bool conditioner_active() const;

signals:
    void connected(const SensorInfo& info);
    void disconnected();
    void started();
    void stopped();
    void error(const QString& message);
    void runtime_warning(const QString& message);
    /// @brief Emitted after every successful set_unified_roi (Phase 2.6
    /// debug D-5): the single driver for the overlay frame, the Zoom-to-ROI
    /// button, the algorithm path (AlgoBridge::set_unified_roi_state) and
    /// GUI checkbox sync. Carries the COMPUTED rect [x0,x1) × [y0,y1).
    void roi_state_changed(bool enabled, int x0, int y0, int x1, int y1);
    /// @brief Emitted from the SDK CD callback (cross-thread, queued) when
    /// cd_broadcast_ is true. Carries a shared_ptr copy of the batch so
    /// listeners on the GUI thread can process it safely. Used by the
    /// calibration wizard's 1 ms event accumulator.
    void cd_events_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events);

private:
    /// @brief Sets up callbacks + pipeline for a new camera. Calls
    /// frame_pipeline_.start_file() for file sources (FileFrameGenerator)
    /// or frame_pipeline_.start() for live sources (CDFrameGenerator).
    void setup_camera(Metavision::Camera&& cam, bool is_file);
    /// @brief Tears down camera + callbacks + frame pipeline.
    void teardown();
    void fetch_sensor_info();

    std::unique_ptr<Metavision::Camera> camera_;
    std::optional<Metavision::CallbackId> cd_cb_id_;
    std::optional<Metavision::CallbackId> err_cb_id_;
    std::optional<Metavision::CallbackId> status_cb_id_;
    SensorInfo sensor_info_;
    bool is_file_{false};
    /// Unified ROI state (live path; file path keeps its rect in
    /// FileFrameGenerator, read via frame_pipeline_.file_roi()). roi_roni_
    /// is tracked here for BOTH paths (single writer: set_unified_roi).
    bool roi_enabled_{false};
    bool roi_roni_{false};
    int roi_x0_{0}, roi_y0_{0}, roi_x1_{0}, roi_y1_{0};

    FramePipeline frame_pipeline_;
    StatisticsController statistics_;
    FilterChain filter_chain_;
    /// Shared conditioning (live batches). The file source's conditioner
    /// lives inside FileFrameGenerator; only one is ever active.
    StreamConditioner conditioner_;
    /// Conditioned-stream listener (live mode), guarded by
    /// conditioned_mutex_. Copied into a local before invocation so a
    /// set_conditioned_listener() during a callback cannot race.
    ConditionedListener conditioned_listener_;
    std::mutex conditioned_mutex_;
    /// @brief When true, the CD callback emits cd_events_ready() with a copy
    /// of every batch. Off by default so non-calibration usage pays nothing.
    std::atomic<bool> cd_broadcast_{false};
    /// @brief Reusable event-buffer pool for the CD broadcast: buffers are
    /// returned to the pool when the last consumer (calibration tap / worker)
    /// drops its shared_ptr, so steady-state broadcasting allocates nothing.
    Metavision::ObjectPool<std::vector<Metavision::EventCD>, true> broadcast_pool_;
};

} // namespace gui

#endif // GUI_APP_CAMERA_CONTROLLER_H
