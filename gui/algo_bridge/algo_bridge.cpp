// gui/algo_bridge/algo_bridge.cpp
//
// AlgoInstance 持有真实的 AlgoBackend 实例，真正调用 algo/cv 与 algo/analytics
// 的算法类。注册表列出全部 31 个自研模块 + 30 个 openEB 能力 = 61 项。

#include "algo_bridge.h"

#include <mutex>

namespace gui {

// ---------------------------------------------------------------------------
// AlgoInstance
// ---------------------------------------------------------------------------

AlgoInstance::AlgoInstance(const AlgoInfo& info, int width, int height)
    : info_(info), width_(width), height_(height) {
    for (const auto& p : info_.params) {
        param_values_[p.key] = p.default_value;
    }
    // 创建真实后端（自研算法）。OpenEB 包装算法返回 nullptr → 透传。
    backend_ = create_algo_backend(info_.name, width_, height_);
    if (backend_) {
        // 应用默认参数到后端。
        for (const auto& p : info_.params) {
            backend_->set_param(p.key, p.default_value);
        }
    }
}

void AlgoInstance::set_param(const std::string& key, const std::string& value) {
    std::lock_guard<std::mutex> lk(mutex_);
    param_values_[key] = value;
    if (backend_) {
        backend_->set_param(key, value);
    }
}

std::string AlgoInstance::get_param(const std::string& key) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = param_values_.find(key);
    return it == param_values_.end() ? std::string{} : it->second;
}

void AlgoInstance::set_enabled(bool e) {
    std::lock_guard<std::mutex> lk(mutex_);
    enabled_ = e;
}

bool AlgoInstance::is_enabled() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return enabled_;
}

void AlgoInstance::push_events(const Metavision::EventCD* begin,
                               const Metavision::EventCD* end) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!enabled_) {
        return;
    }
    if (backend_) {
        // 真实算法后端处理事件。
        backend_->push_events(begin, end);
    } else {
        // OpenEB 包装算法：透传（由 filter_chain 处理）。
    }
}

AlgoResult AlgoInstance::pull_result() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (backend_) {
        return backend_->pull_result();
    }
    // 透传：返回空结果（OpenEB 算法由 filter_chain 处理）。
    AlgoResult r;
    r.status = "pass-through (openeb)";
    return r;
}

void AlgoInstance::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (backend_) {
        backend_->reset();
    }
}

// ---------------------------------------------------------------------------
// Small spec helpers
// ---------------------------------------------------------------------------

namespace {

AlgoParamSpec pint(const std::string& k, const std::string& disp,
                   const std::string& def, const std::string& lo,
                   const std::string& hi) {
    return {k, disp, "int", def, lo, hi, {}};
}

AlgoParamSpec pfloat(const std::string& k, const std::string& disp,
                     const std::string& def, const std::string& lo,
                     const std::string& hi) {
    return {k, disp, "float", def, lo, hi, {}};
}

AlgoParamSpec penum(const std::string& k, const std::string& disp,
                    const std::string& def, std::vector<std::string> vals) {
    return {k, disp, "enum", def, "", "", std::move(vals)};
}

AlgoParamSpec pbool(const std::string& k, const std::string& disp,
                    const std::string& def) {
    return {k, disp, "bool", def, "", "", {}};
}

} // namespace

// ---------------------------------------------------------------------------
// AlgoBridge
// ---------------------------------------------------------------------------

AlgoBridge::AlgoBridge() {
    register_openeb_filters();
    register_openeb_frame_modes();
    register_openeb_preprocessors();
    register_openeb_utils();
    register_self_cv();
    register_self_analytics();
    register_self_calibration();
}

std::vector<AlgoInfo> AlgoBridge::list_algos() const {
    std::vector<AlgoInfo> out;
    out.reserve(registry_.size());
    for (const auto& kv : registry_) {
        out.push_back(kv.second);
    }
    return out;
}

const AlgoInfo* AlgoBridge::find(const std::string& name) const {
    auto it = registry_.find(name);
    return it == registry_.end() ? nullptr : &it->second;
}

std::shared_ptr<AlgoInstance> AlgoBridge::create(const std::string& name) {
    auto it = registry_.find(name);
    if (it == registry_.end()) {
        return nullptr;
    }
    auto inst = std::make_shared<AlgoInstance>(it->second, sensor_w_, sensor_h_);
    {
        std::lock_guard<std::mutex> lk(live_mutex_);
        live_instances_[name] = inst;
    }
    return inst;
}

void AlgoBridge::set_sensor_dimensions(int width, int height) {
    sensor_w_ = (width > 0) ? width : 1280;
    sensor_h_ = (height > 0) ? height : 720;
}

std::shared_ptr<AlgoInstance> AlgoBridge::find_live(const std::string& name) {
    std::lock_guard<std::mutex> lk(live_mutex_);
    auto it = live_instances_.find(name);
    if (it == live_instances_.end()) return nullptr;
    auto inst = it->second.lock();
    if (!inst) {
        live_instances_.erase(it);
    }
    return inst;
}

std::vector<std::shared_ptr<AlgoInstance>> AlgoBridge::list_live() {
    std::vector<std::shared_ptr<AlgoInstance>> out;
    std::lock_guard<std::mutex> lk(live_mutex_);
    for (auto it = live_instances_.begin(); it != live_instances_.end(); ) {
        if (auto inst = it->second.lock()) {
            out.push_back(std::move(inst));
            ++it;
        } else {
            it = live_instances_.erase(it);
        }
    }
    return out;
}

void AlgoBridge::push_events(const std::shared_ptr<AlgoInstance>& inst,
                             const Metavision::EventCD* begin,
                             const Metavision::EventCD* end) {
    if (inst) {
        inst->push_events(begin, end);
    }
}

AlgoResult AlgoBridge::pull_result(const std::shared_ptr<AlgoInstance>& inst) {
    if (!inst) {
        return {};
    }
    return inst->pull_result();
}

// ---------------------------------------------------------------------------
// OpenEB-wrapped filters (design §4.3.1) — unchanged
// ---------------------------------------------------------------------------

void AlgoBridge::register_openeb_filters() {
    auto add = [&](AlgoInfo a) {
        a.source = "openeb";
        a.category = "openeb_filter";
        registry_[a.name] = std::move(a);
    };

    add({"roi_filter", "ROI Filter", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive,
         {pint("x0", "X start", "0", "0", ""),
          pint("y0", "Y start", "0", "0", ""),
          pint("x1", "X end", "1279", "0", ""),
          pint("y1", "Y end", "719", "0", ""),
          pbool("output_relative_coordinates", "Relative coords", "false")}});

    add({"roi_mask", "ROI Mask", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive,
         {{"mask_path", "Mask image path", "string", "", "", "", {}}}});

    add({"polarity_filter", "Polarity Filter", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive,
         {penum("polarity", "Polarity", "1", {"0=OFF", "1=ON"})}});

    add({"polarity_invert", "Polarity Invert", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"flip_x", "Flip X", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"flip_y", "Flip Y", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"rotate", "Rotate", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive,
         {penum("rotation", "Rotation (deg)", "0", {"0", "90", "180", "270"})}});

    add({"transpose", "Transpose", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"rescale", "Rescale", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive,
         {pfloat("scale_width", "Scale X", "1.0", "0.0001", "10"),
          pfloat("scale_height", "Scale Y", "1.0", "0.0001", "10")}});

    add({"adaptive_rate_split", "Adaptive Rate Split", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive,
         {pfloat("thr_var_per_event", "Var threshold", "5e-4", "1e-5", "1e-2"),
          pint("downsampling_factor", "Downsampling", "2", "1", "8")}});
}

// ---------------------------------------------------------------------------
// OpenEB-wrapped frame generators (design §4.3.2) — unchanged
// ---------------------------------------------------------------------------

void AlgoBridge::register_openeb_frame_modes() {
    auto add = [&](AlgoInfo a) {
        a.source = "openeb";
        a.category = "openeb_frame";
        a.display_mode = AlgoDisplayMode::Replace;
        registry_[a.name] = std::move(a);
    };

    add({"frame_integration", "Integration Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("decay_time_us", "Decay time (us)", "1000000", "10000", "10000000")}});

    add({"frame_diff", "Diff Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"frame_histogram", "Histogram Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"frame_time_decay", "Time Decay Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("exponential_decay_time_us", "Decay (us)", "100000", "10000", "10000000")}});

    add({"frame_contrast_map", "Contrast Map", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"frame_periodic", "Periodic Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("period_us", "Period (us)", "33000", "1000", "1000000")}});

    add({"frame_on_demand", "On-Demand Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {penum("trigger", "Trigger strategy", "N_EVENTS",
                {"N_EVENTS", "N_US", "MIXED"})}});
}

// ---------------------------------------------------------------------------
// OpenEB-wrapped preprocessors (design §4.3.3) — unchanged
// ---------------------------------------------------------------------------

void AlgoBridge::register_openeb_preprocessors() {
    auto add = [&](AlgoInfo a) {
        a.source = "openeb";
        a.category = "openeb_preproc";
        a.display_mode = AlgoDisplayMode::Passive;
        registry_[a.name] = std::move(a);
    };

    add({"preproc_diff", "Diff Preprocessor", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"preproc_histo", "Histo Preprocessor", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {pint("max_events_per_pixel", "Max events/px", "5", "1", "255"),
          pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"preproc_hw_diff", "Hardware Diff", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"preproc_hw_histo", "Hardware Histo", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"preproc_time_surface", "Time Surface Processor", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {penum("channels", "Channels", "1", {"1=merged", "2=split"})}});

    add({"preproc_event_cube", "Event Cube", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {pint("num_bins", "Num bins", "10", "2", "20"),
          pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"preproc_factory", "Preprocessor Factory", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {{"config_path", "JSON config path", "string", "", "", "", {}}}});
}

// ---------------------------------------------------------------------------
// OpenEB-wrapped utilities (design §4.3.4) — unchanged
// ---------------------------------------------------------------------------

void AlgoBridge::register_openeb_utils() {
    auto add = [&](AlgoInfo a) {
        a.source = "openeb";
        a.category = "openeb_util";
        a.display_mode = AlgoDisplayMode::Passive;
        registry_[a.name] = std::move(a);
    };

    add({"util_rate_estimator", "Rate Estimator", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
    add({"util_frame_composer", "Frame Composer", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
    add({"util_rolling_buffer", "Rolling Buffer", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
    add({"util_video_writer", "Video Writer", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
    add({"util_data_synchronizer", "Data Synchronizer", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
    add({"util_timing_profiler", "Timing Profiler", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
}

// ---------------------------------------------------------------------------
// Self-developed CV algorithms (design §4.3.5 - §4.3.27)
// 23 modules, all with real algo_backend wiring.
// ---------------------------------------------------------------------------

void AlgoBridge::register_self_cv() {
    auto add = [&](AlgoInfo a) {
        a.source = "self";
        a.category = "cv";
        registry_[a.name] = std::move(a);
    };

    // §4.3.5 Noise Filter (8 modes)
    add({"noise_filter", "Noise Filter", "cv", "self",
         AlgoDisplayMode::Passive,
         {penum("mode", "Mode", "1", {"0=BAF", "1=STCF", "2=Refractory",
           "3=DWF", "4=AgePolarity", "5=Harmonic", "6=Repetitious", "7=SpatialBP"}),
          pfloat("correlation_time_s", "Correlation time (s)", "0.005", "0.001", "0.1"),
          pint("min_neighbors", "Min neighbors", "2", "1", "8"),
          pint("baf_dt_us", "BAF dt (us)", "1000", "1000", "100000"),
          pint("refractory_us", "Refractory (us)", "1000", "100", "100000"),
          pbool("filter_hot_pixels", "Filter hot pixels", "false"),
          pbool("adaptive_correlation_time", "Adaptive corr time", "false"),
          penum("line_freq_hz", "Line freq (Hz)", "50", {"50", "60"})}});

    // §4.3.6 Hot Pixel Filter
    add({"hot_pixel_filter", "Hot Pixel Filter", "cv", "self",
         AlgoDisplayMode::Passive,
         {pfloat("learning_window_s", "Learning window (s)", "5.0", "0.1", "60.0"),
          pfloat("n_sigma", "N-sigma", "4.0", "1.0", "10.0"),
          pbool("enable_fpn_correction", "FPN correction", "false"),
          pfloat("fpn_target_rate_hz", "FPN target rate (Hz)", "100", "1", "1000")}});

    // §4.3.7 Orientation Filter
    add({"orientation_filter", "Orientation Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("tau_us", "Time constant (us)", "10000", "1000", "100000")}});

    // §4.3.8 Direction Selective Filter
    add({"direction_selective", "Direction Selective Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("direction", "Direction", "0", "0", "7"),
          pint("tau_us", "Time constant (us)", "10000", "1000", "100000")}});

    // §4.3.9 Sparse Optical Flow (4 modes: LocalPlanes/LucasKanade/BlockMatch/ClusterOF)
    add({"sparse_optical_flow", "Sparse Optical Flow", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=LocalPlanes", "1=LucasKanade",
           "2=BlockMatch", "3=ClusterOF"}),
          pint("search_radius", "Search radius (px)", "8", "3", "30")}});

    // §4.3.10 Blob Detector
    add({"blob_detector", "Blob Detector", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("threshold", "Threshold", "10", "1", "254"),
          pfloat("learning_rate", "Learning rate", "0.05", "0.001", "1.0")}});

    // §4.3.11 Object Tracker (4 modes)
    add({"object_tracker", "Object Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=RCT", "1=Median", "2=Kalman", "3=MultiHypothesis"}),
          pint("cluster_size_px", "Cluster size (px)", "10", "3", "50"),
          pint("cluster_time_us", "Cluster time (us)", "5000", "1000", "50000"),
          pint("min_cluster_events", "Min cluster events", "50", "10", "500"),
          pfloat("max_lost_age_s", "Max lost age (s)", "1.0", "0.1", "5.0"),
          pbool("enable_velocity_prediction", "Velocity prediction", "true")}});

    // §4.3.12 Corner Detector (3 modes)
    add({"corner_detector", "Corner Detector", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=Harris", "1=FAST", "2=AGAST"}),
          pfloat("min_score", "Min score", "0.01", "0", "1.0")}});

    // §4.3.13 Line Segment Detector (ELiSeD)
    add({"line_segment", "Line Segment (ELiSeD)", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("min_length", "Min length (px)", "10", "3", "100"),
          pint("gap", "Max gap (px)", "3", "1", "20")}});

    // §4.3.14 Hough Line Tracker
    add({"hough_line", "Hough Line Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("threshold", "Threshold", "50", "2", "500"),
          pint("num_theta_bins", "Theta bins", "90", "8", "360"),
          pint("num_rho_bins", "Rho bins (0=auto)", "0", "0", "4000"),
          pint("accumulator_decay_us", "Decay (us)", "100000", "1000", "5000000")}});

    // §4.3.15 Hough Circle Tracker
    add({"hough_circle", "Hough Circle Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("min_radius", "Min radius (px)", "5", "1", "100"),
          pint("max_radius", "Max radius (px)", "50", "5", "500"),
          pint("threshold", "Threshold", "30", "2", "500"),
          pint("accumulator_decay_us", "Decay (us)", "100000", "1000", "5000000")}});

    // §4.3.17 Orientation Cluster
    add({"orientation_cluster", "Orientation Cluster", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("min_events", "Min events", "20", "5", "500")}});

    // §4.3.18 Cluster LIF
    add({"cluster_lif", "Cluster LIF", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("tau_ms", "Tau (ms)", "10", "1", "1000"),
          pfloat("threshold", "Threshold", "1.0", "0.1", "10.0")}});

    // §4.3.19 Background Mask Filter
    add({"background_mask", "Background Mask Filter", "cv", "self",
         AlgoDisplayMode::Replace,
         {pfloat("learning_rate", "Learning rate", "0.05", "0.001", "1.0"),
          pfloat("threshold", "Threshold", "10", "1", "100")}});

    // §4.3.20 Perspective Undistort
    add({"perspective_undistort", "Perspective Undistort", "cv", "self",
         AlgoDisplayMode::Passive,
         {pbool("enable", "Enable", "false"),
          pfloat("zoom", "Zoom", "1.0", "0.1", "10.0")}});

    // §4.3.21 Trigger Synced Filter
    add({"trigger_synced", "Trigger Synced Filter", "cv", "self",
         AlgoDisplayMode::Passive,
         {pint("window_us", "Window (us)", "10000", "1000", "1000000")}});

    // §4.3.22 Bandpass Filter
    add({"bandpass_filter", "Bandpass Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("low_cutoff_hz", "Low cutoff (Hz)", "1.0", "0.01", "100"),
          pfloat("high_cutoff_hz", "High cutoff (Hz)", "10.0", "0.01", "1000")}});

    // §4.3.23 Optical Gyro (EIS)
    add({"optical_gyro", "EIS (Optical Gyro)", "cv", "self",
         AlgoDisplayMode::Passive,
         {pbool("stabilize", "Stabilize", "true"),
          pfloat("smoothing_window_ms", "Smoothing window (ms)", "100", "10", "1000")}});

    // §4.3.24 Ultra Slow Motion
    add({"ultra_slow_motion", "Ultra Slow Motion", "cv", "self",
         AlgoDisplayMode::Passive,
         {pint("factor", "Dilation factor", "10", "1", "1000")}});

    // §4.3.25 XYT Visualizer
    add({"xyt_visualizer", "XYT 3D Visualizer", "cv", "self",
         AlgoDisplayMode::Standalone,
         {pint("time_window_us", "Time window (us)", "1000000", "10000", "10000000"),
          pint("max_points", "Max points", "50000", "1000", "500000")}});

    // §4.3.26 Overlay
    add({"overlay", "Overlay", "cv", "self",
         AlgoDisplayMode::Overlay, {}});

    // §4.3.27 Time Surface
    add({"time_surface", "Time Surface", "cv", "self",
         AlgoDisplayMode::Standalone,
         {pint("decay_time_us", "Decay time (us)", "100000", "10000", "5000000"),
          penum("palette", "Palette", "1", {"0=Gray", "1=Hot", "2=Plasma", "3=Turbo"}),
          penum("channels", "Channels", "1", {"1=merged", "2=split"})}});
}

// ---------------------------------------------------------------------------
// Self-developed analytics (design §4.4) — 7 modules
// ---------------------------------------------------------------------------

void AlgoBridge::register_self_analytics() {
    auto add = [&](AlgoInfo a) {
        a.source = "self";
        a.category = "analytics";
        registry_[a.name] = std::move(a);
    };

    // §4.4.1 Active Marker
    add({"active_marker", "Active Marker Tracking", "analytics", "self",
         AlgoDisplayMode::Overlay,
         {pint("window_us", "Window (us)", "10000", "1000", "100000"),
          pint("min_events", "Min events", "20", "5", "500")}});

    // §4.4.2 Event To Video (3 modes)
    add({"event_to_video", "Event -> Video (E2VID)", "analytics", "self",
         AlgoDisplayMode::Standalone,
         {penum("mode", "Mode", "0", {"0=BardowVariational", "1=InteractingMaps", "2=E2VID"})}});

    // §4.4.3 Flow Statistics (requires ground-truth; Passive in real-time)
    add({"flow_statistics", "Flow Statistics", "analytics", "self",
         AlgoDisplayMode::Passive, {}});

    // §4.4.4 ISI Analyzer
    add({"isi_analyzer", "ISI Analyzer", "analytics", "self",
         AlgoDisplayMode::Standalone,
         {pbool("per_pixel", "Per pixel", "false"),
          pfloat("max_isi_ms", "Max ISI (ms)", "100", "1", "1000")}});

    // §4.4.5 Particle Counter
    add({"particle_counter", "Particle Counter", "analytics", "self",
         AlgoDisplayMode::Overlay,
         {pint("line_y", "Line Y (px)", "360", "0", ""),
          pint("min_area", "Min area (px)", "10", "1", "10000")}});

    // §4.4.6 Auto Bias Controller
    add({"auto_bias", "Auto Bias Controller", "analytics", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("target_event_rate_mev", "Target rate (Mev/s)", "5.0", "0.1", "50.0")}});

    // §4.4.7 Freq Detector
    add({"freq_detector", "Frequency Detector", "analytics", "self",
         AlgoDisplayMode::Standalone,
         {pfloat("update_interval_s", "Update interval (s)", "0.5", "0.1", "10"),
          pint("min_events", "Min events", "50", "10", "1000")}});
}

// ---------------------------------------------------------------------------
// Self-developed calibration (design §4.5) — unchanged
// ---------------------------------------------------------------------------

void AlgoBridge::register_self_calibration() {
    auto add = [&](AlgoInfo a) {
        a.source = "self";
        a.category = "calibration";
        a.display_mode = AlgoDisplayMode::Standalone;
        registry_[a.name] = std::move(a);
    };

    add({"intrinsic_calibration", "Intrinsic Calibration", "calibration", "self",
         AlgoDisplayMode::Standalone,
         {penum("board_type", "Board type", "chessboard",
                {"chessboard", "circle_grid", "aruco"}),
          pint("squares_x", "Squares X", "9", "2", "30"),
          pint("squares_y", "Squares Y", "6", "2", "30"),
          pfloat("square_size_mm", "Square size (mm)", "25", "1", "200")}});
}

} // namespace gui
