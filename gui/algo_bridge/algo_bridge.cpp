// gui/algo_bridge/algo_bridge.cpp
//
// AlgoInstance 持有真实的 AlgoBackend 实例，真正调用 algo/cv 与 algo/analytics
// 的算法类。注册表列出全部 29 个自研模块 + 30 个 openEB 能力 = 59 项。

#include "algo_bridge.h"

#include <QImage>

#include <mutex>

#include "display/display_strategy.h"  // IDisplayStrategy + concrete strategies

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
    // Select the display strategy from the declared display mode
    // (design §3.5.3). The strategy is queried later via apply_strategy().
    switch (info_.display_mode) {
        case AlgoDisplayMode::Passive:    strategy_ = std::make_unique<PassiveStrategy>(); break;
        case AlgoDisplayMode::Overlay:    strategy_ = std::make_unique<OverlayStrategy>(); break;
        case AlgoDisplayMode::Replace:    strategy_ = std::make_unique<ReplaceStrategy>(); break;
        case AlgoDisplayMode::Standalone: strategy_ = std::make_unique<StandaloneStrategy>(); break;
    }
}

// Out-of-line so std::unique_ptr<IDisplayStrategy> destroys with a complete type.
AlgoInstance::~AlgoInstance() = default;

void AlgoInstance::apply_strategy(QImage& frame, AlgoResult& result,
                                  DisplayContext& ctx) {
    ctx.instance = this;
    strategy_->apply(frame, result, info_, ctx);
}

void AlgoInstance::set_param(const std::string& key, const std::string& value) {
    std::lock_guard<std::mutex> lk(mutex_);
    // Only store known parameter keys to avoid map pollution from
    // unknown/obsolete keys (BUG-G12). The backend still receives the
    // call so it can handle backward-compat forwards (e.g. "downsample"
    // -> "preproc_downsample" in EventToVideoBackend) and global
    // preproc_* keys (handled by Preprocessor, not in info_.params).
    bool known = false;
    for (const auto& p : info_.params) {
        if (p.key == key) { known = true; break; }
    }
    if (known) param_values_[key] = value;
    if (backend_) {
        backend_->set_param(key, value);
        // BUG-G2/N11: after setting model_path (or num_bins itself) on an
        // E2VID backend, the real num_bins is dictated by the loaded ONNX
        // model's input channel count — the algo ignores the caller's value
        // when a model is loaded (e2vid_inference.h set_num_bins). The
        // backend re-syncs e2vid_num_bins_ from the algo; sync that
        // authoritative value back into param_values_ so get_param("num_bins")
        // returns the model-determined value, not the stale registry default
        // ("5") or a stale cached value. We intentionally do NOT delegate
        // get_param to the backend in general — that would change the string
        // format of every param (backend uses std::to_string(double),
        // producing "5.500000" instead of the registry's "5.5") and break
        // ParamRoundTrip tests. This targeted sync keeps the fix minimal.
        if (key == "model_path" || key == "num_bins") {
            std::string nb = backend_->get_param("num_bins");
            if (!nb.empty()) param_values_["num_bins"] = nb;
        }
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
    if (e) {
        // Re-enabling clears any prior overload state and resets the strike
        // counter so the algo gets a fresh start. The backend's internal
        // state (buffers/accumulators) is intentionally NOT reset here —
        // this preserves the pause-resume workflow where the user temporarily
        // disables an algorithm and later resumes with its accumulated state
        // intact (e.g. E2VID log_intensity_, InteractingMaps I_map_). Full
        // reset is performed separately by the caller when starting a new
        // session (e.g. MainWindow::on_camera_connected calls inst->reset()
        // for every live instance). Drop-rate counters are also reset so
        // the InformationPanel shows fresh telemetry for the new session.
        overloaded_ = false;
        flood_strikes_ = 0;
        total_pushed_ = 0;
        total_dropped_ = 0;
    }
}

bool AlgoInstance::is_enabled() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return enabled_ && !overloaded_;
}

bool AlgoInstance::is_overloaded() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return overloaded_;
}

void AlgoInstance::clear_overload() {
    std::lock_guard<std::mutex> lk(mutex_);
    overloaded_ = false;
    flood_strikes_ = 0;
}

std::size_t AlgoInstance::total_pushed() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return total_pushed_;
}

std::size_t AlgoInstance::total_dropped() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return total_dropped_;
}

void AlgoInstance::push_events(const Metavision::EventCD* begin,
                               const Metavision::EventCD* end) {
    std::lock_guard<std::mutex> lk(mutex_);
    const std::size_t n = static_cast<std::size_t>(end - begin);
    total_pushed_ += n;
    if (!enabled_ || overloaded_) {
        // Instance is disabled/overloaded — the entire batch is dropped.
        total_dropped_ += n;
        return;
    }
    if (backend_) {
        // Flood guard (design §5.6.7): cap each batch to the most recent
        // kMaxBatchEvents events. If a batch was capped, increment the strike
        // counter; kFloodStrikes consecutive capped batches mean the algo is
        // being fed faster than it can process → auto-disable to prevent
        // memory blowup and GUI stalls. A non-capped batch resets the count.
        const Metavision::EventCD* b = begin;
        const Metavision::EventCD* e = end;
        if (n > kMaxBatchEvents) {
            // Keep the most recent events (drop older ones from the front).
            b = end - kMaxBatchEvents;
            const std::size_t dropped_front = n - kMaxBatchEvents;
            total_dropped_ += dropped_front;
            if (++flood_strikes_ >= kFloodStrikes) {
                overloaded_ = true;
                enabled_ = false;
                // The capped batch is also dropped (backend never sees it).
                total_dropped_ += kMaxBatchEvents;
                return;
            }
        } else {
            flood_strikes_ = 0;
        }

        backend_->push_events(b, e);
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

void AlgoInstance::set_sensor_dimensions(int width, int height) {
    std::lock_guard<std::mutex> lk(mutex_);
    width_ = width;
    height_ = height;
    if (backend_) {
        backend_->set_sensor_dimensions(width, height);
    }
}

// ---------------------------------------------------------------------------
// Small spec helpers
// ---------------------------------------------------------------------------

namespace {

AlgoParamSpec pint(const std::string& k, const std::string& disp,
                   const std::string& def, const std::string& lo,
                   const std::string& hi, const std::string& mf = "") {
    return {k, disp, "int", def, lo, hi, {}, mf};
}

AlgoParamSpec pfloat(const std::string& k, const std::string& disp,
                     const std::string& def, const std::string& lo,
                     const std::string& hi, const std::string& mf = "") {
    return {k, disp, "float", def, lo, hi, {}, mf};
}

AlgoParamSpec penum(const std::string& k, const std::string& disp,
                    const std::string& def, std::vector<std::string> vals,
                    const std::string& mf = "") {
    return {k, disp, "enum", def, "", "", std::move(vals), mf};
}

AlgoParamSpec pbool(const std::string& k, const std::string& disp,
                    const std::string& def, const std::string& mf = "") {
    return {k, disp, "bool", def, "", "", {}, mf};
}

AlgoParamSpec pstring(const std::string& k, const std::string& disp,
                      const std::string& def, const std::string& mf = "") {
    return {k, disp, "string", def, "", "", {}, mf};
}

/// Returns the 5 ROI parameters (design §5.6.6) shared by all self-developed
/// algorithms. Defaults: enabled, center 128×128 (x/y=-1 = auto-center,
/// w/h=128). When enabled the algorithm only processes events inside the
/// ROI region; the main display frame draws a yellow ROI rectangle.
std::vector<AlgoParamSpec> roi_params() {
    return {
        pbool("roi_enabled", "ROI enable", "true"),
        pint("roi_x", "ROI x (-1=center)", "-1", "-1", ""),
        pint("roi_y", "ROI y (-1=center)", "-1", "-1", ""),
        pint("roi_w", "ROI w (0=full)", "128", "0", ""),
        pint("roi_h", "ROI h (0=full)", "128", "0", ""),
    };
}

/// Returns the shared preprocessing parameters (v1.0.9): a stackable noise
/// filter + 1/4 downsample applied AFTER the algorithm ROI, in the order
/// ROI → filter → downsample. These overlay on top of any main algorithm and
/// are NOT mutually exclusive with it. preproc_downsample defaults to "true"
/// to preserve v1.0.0 behaviour (event_to_video had downsample=true).
/// preproc_filter_enabled defaults to "false" (opt-in). The preproc_filter_*
/// params mirror the standalone NoiseFilter params (§4.3.5) so the same 8
/// denoiser modes are available as a preprocessing stage.
std::vector<AlgoParamSpec> preproc_params() {
    return {
        pbool("preproc_filter_enabled", "Preproc: noise filter", "false"),
        pbool("preproc_downsample", "Preproc: 1/4 downsample", "true"),
        penum("preproc_filter_mode", "Preproc: filter mode", "1",
              {"0=BAF", "1=STCF", "2=Refractory", "3=DWF",
               "4=AgePolarity", "5=Harmonic", "6=Repetitious", "7=SpatialBP"}),
        // STCF (mode 1)
        pfloat("preproc_filter_correlation_time_s", "Preproc STCF corr (s)", "0.005", "0.001", "0.1"),
        pint("preproc_filter_min_neighbors", "Preproc STCF min nbr", "2", "1", "8"),
        pbool("preproc_filter_require_polarity_match", "Preproc STCF pol match", "false"),
        pbool("preproc_filter_allow_coincidence", "Preproc STCF coincide", "false"),
        // BAF (mode 0)
        pint("preproc_filter_baf_dt_us", "Preproc BAF dt (us)", "1000", "1000", "100000"),
        pint("preproc_filter_baf_subsample_by", "Preproc BAF subsample", "0", "0", "4"),
        // Refractory (mode 2)
        pint("preproc_filter_refractory_us", "Preproc Refractory (us)", "1000", "100", "100000"),
        // DWF (mode 3)
        pint("preproc_filter_dwf_window_length", "Preproc DWF win len", "2", "1", "100"),
        pint("preproc_filter_dwf_dist_threshold", "Preproc DWF dist", "2", "1", "1024"),
        pint("preproc_filter_dwf_min_correlated", "Preproc DWF min corr", "2", "1", "8"),
        pbool("preproc_filter_dwf_double_mode", "Preproc DWF double", "false"),
        // AgePolarity (mode 4)
        pint("preproc_filter_agep_tau_us", "Preproc AgePol tau (us)", "3000", "1000", "100000"),
        pfloat("preproc_filter_age_threshold", "Preproc AgePol thresh", "2.0", "0.0", "8.0"),
        pint("preproc_filter_agep_radius", "Preproc AgePol radius", "2", "1", "5"),
        // Harmonic (mode 5)
        penum("preproc_filter_line_freq_hz", "Preproc Harmonic Hz", "50", {"50", "60"}),
        pfloat("preproc_filter_notch_q", "Preproc Harmonic Q", "5.0", "0.1", "100.0"),
        pfloat("preproc_filter_harmonic_threshold", "Preproc Harmonic thresh", "0.1", "0.0", "1.0"),
        // Repetitious (mode 6)
        pint("preproc_filter_rep_period_us", "Preproc Rep period (us)", "5000", "1000", "1000000"),
        pint("preproc_filter_rep_tolerance_us", "Preproc Rep tol (us)", "1000", "100", "10000"),
        pint("preproc_filter_rep_ratio_shorter", "Preproc Rep ratio short", "10", "1", "100"),
        pint("preproc_filter_rep_ratio_longer", "Preproc Rep ratio long", "10", "1", "100"),
        pint("preproc_filter_rep_min_dt_to_store_us", "Preproc Rep min dt (us)", "1000", "0", "1000000"),
        // SpatialBP (mode 7)
        pint("preproc_filter_sbp_center_radius_px", "Preproc SBP center", "2", "1", "10"),
        pint("preproc_filter_sbp_surround_radius_px", "Preproc SBP surround", "10", "5", "30"),
        pint("preproc_filter_sbp_dt_surround_us", "Preproc SBP dt (us)", "10000", "100", "1000000"),
        // Cross-mode flags
        pbool("preproc_filter_filter_hot_pixels", "Preproc filter hot px", "false"),
        pbool("preproc_filter_adaptive_correlation_time", "Preproc adaptive corr", "false"),
    };
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
        // Replay per-algorithm cached params (N1) FIRST: values loaded from
        // a config file for an algorithm that had no live instance. These
        // include preproc_*/roi_* keys (because capture_algo_state saves
        // all info.params). Then replay the global preproc_cache_/roi_cache_
        // SECOND so they override the config's preproc_*/roi_* values —
        // the user's latest global modifications take precedence over stale
        // config values (BUG-1: N1+N6 interaction). Per-algo params like
        // model_path/mode are unaffected since the global caches don't
        // contain those keys. Skip calibration algos (BUG-R8).
        auto pit = algo_param_cache_.find(name);
        if (pit != algo_param_cache_.end()) {
            for (const auto& [k, v] : pit->second) {
                inst->set_param(k, v);
            }
            algo_param_cache_.erase(pit);
        }
        if (it->second.source == "self" &&
            it->second.category != "calibration") {
            for (const auto& [k, v] : preproc_cache_) {
                inst->set_param(k, v);
            }
            for (const auto& [k, v] : roi_cache_) {
                inst->set_param(k, v);
            }
        }
    }
    return inst;
}

std::shared_ptr<AlgoInstance> AlgoBridge::find_or_create(const std::string& name) {
    if (auto existing = find_live(name)) {
        return existing;
    }
    return create(name);
}

void AlgoBridge::set_sensor_dimensions(int width, int height) {
    sensor_w_ = (width > 0) ? width : 1280;
    sensor_h_ = (height > 0) ? height : 720;
}

void AlgoBridge::apply_global_preproc(const std::string& key,
                                      const std::string& value) {
    // Apply the preproc_* parameter to every live self-developed instance.
    // OpenEB wrapper algorithms (source=="openeb") have backend_==nullptr
    // (pass-through); preproc_* params have no effect on them and would
    // pollute their param_values_ map, so they are skipped.
    std::lock_guard<std::mutex> lk(live_mutex_);
    // Cache the value so instances created later (by other code paths) inherit
    // the shared preprocessing state (BUG-R4).
    preproc_cache_[key] = value;
    for (auto it = live_instances_.begin(); it != live_instances_.end(); ) {
        if (auto inst = it->second.lock()) {
            // Skip calibration algorithms — they don't use preproc params
            // and would just accumulate pollution in param_values_ (BUG-R8).
            if (inst->info().source == "self" &&
                inst->info().category != "calibration") {
                inst->set_param(key, value);
            }
            ++it;
        } else {
            it = live_instances_.erase(it);
        }
    }
}

void AlgoBridge::apply_global_roi(const std::string& enabled,
                                  const std::string& x,
                                  const std::string& y,
                                  const std::string& w,
                                  const std::string& h) {
    // Apply the roi_* parameters to every live self-developed instance and
    // cache them so instances created later inherit the current ROI (N3).
    // Same source/category filter as apply_global_preproc.
    std::lock_guard<std::mutex> lk(live_mutex_);
    roi_cache_["roi_enabled"] = enabled;
    roi_cache_["roi_x"] = x;
    roi_cache_["roi_y"] = y;
    roi_cache_["roi_w"] = w;
    roi_cache_["roi_h"] = h;
    for (auto it = live_instances_.begin(); it != live_instances_.end(); ) {
        if (auto inst = it->second.lock()) {
            if (inst->info().source == "self" &&
                inst->info().category != "calibration") {
                inst->set_param("roi_enabled", enabled);
                inst->set_param("roi_x", x);
                inst->set_param("roi_y", y);
                inst->set_param("roi_w", w);
                inst->set_param("roi_h", h);
            }
            ++it;
        } else {
            it = live_instances_.erase(it);
        }
    }
}

void AlgoBridge::cache_algo_params(
    const std::string& name,
    const std::map<std::string, std::string>& params) {
    // Store per-algorithm params for an instance that is not yet live (N1).
    // create() will replay these when the instance is eventually created.
    std::lock_guard<std::mutex> lk(live_mutex_);
    algo_param_cache_[name] = params;
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
         {pint("bit_size", "Bit size", "8", "2", "8"),
          pbool("allow_rollover", "Allow rollover", "true")}});

    add({"frame_histogram", "Histogram Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("channel_bit_neg", "Neg bits", "4", "1", "7"),
          pint("channel_bit_pos", "Pos bits", "4", "1", "7"),
          pbool("packed", "Packed", "false")}});

    add({"frame_time_decay", "Time Decay Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("exponential_decay_time_us", "Decay (us)", "100000", "10000", "10000000"),
          penum("palette", "Palette", "1", {"0=Light", "1=Dark", "2=CoolWarm", "3=Gray"})}});

    add({"frame_contrast_map", "Contrast Map", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pfloat("contrast_on", "Contrast ON", "1.2", "0.1", "10"),
          pfloat("contrast_off", "Contrast OFF", "-1", "-10", "0")}});

    add({"frame_periodic", "Periodic Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("accumulation_time_us", "Accumulation (us)", "10000", "1000", "1000000"),
          pfloat("fps", "FPS", "30", "0", "120")}});

    add({"frame_on_demand", "On-Demand Frame", "openeb_frame", "openeb",
         AlgoDisplayMode::Replace,
         {pint("accumulation_time_us", "Accumulation (us)", "0", "0", "10000000")}});
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
         {pfloat("max_incr_per_pixel", "Max incr/px", "5", "0.1", "100"),
          pfloat("clip_value_after_normalization", "Clip value", "1", "0.1", "10")}});

    add({"preproc_histo", "Histo Preprocessor", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {pfloat("max_incr_per_pixel", "Max incr/px", "5", "0.1", "100"),
          pfloat("clip_value_after_normalization", "Clip value", "1", "0.1", "10"),
          pbool("use_CHW", "Use CHW", "true")}});

    add({"preproc_hw_diff", "Hardware Diff", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {pint("accumulation_time_us", "Accumulation (us)", "33000", "1000", "1000000")}});

    add({"preproc_hw_histo", "Hardware Histo", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"preproc_time_surface", "Time Surface Processor", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         // 'channels' triggers backend reconstruction (1→merge, 2→split)
         {penum("channels", "Channels", "1", {"1=merged", "2=split"})}});

    add({"preproc_event_cube", "Event Cube", "openeb_preproc", "openeb",
         AlgoDisplayMode::Passive,
         {pint("num_bins", "Num bins", "10", "2", "20"),
          pint("delta_t_us", "Delta t (us)", "33000", "1000", "1000000"),
          pbool("split_polarity", "Split polarity", "false"),
          pfloat("max_incr_per_pixel", "Max incr/px", "5", "0.1", "100")}});

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
         AlgoDisplayMode::Passive,
         {penum("mode", "Mode", "0", {"0=N_EVENTS", "1=N_US"}),
          pint("delta_n_events", "N events", "5000", "1", "1000000"),
          pint("delta_ts_us", "Delta t (us)", "1000000", "1000", "60000000")}});
    add({"util_video_writer", "Video Writer", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
    add({"util_data_synchronizer", "Data Synchronizer", "openeb_util", "openeb",
         AlgoDisplayMode::Passive,
         {pint("period_us", "Period (us)", "10000", "1000", "1000000000")}});
    add({"util_timing_profiler", "Timing Profiler", "openeb_util", "openeb",
         AlgoDisplayMode::Passive, {}});
}

// ---------------------------------------------------------------------------
// Self-developed CV algorithms (design §4.3.5 - §4.3.27)
// 21 modules, all with real algo_backend wiring.
// ---------------------------------------------------------------------------

void AlgoBridge::register_self_cv() {
    auto add = [&](AlgoInfo a) {
        a.source = "self";
        a.category = "cv";
        // All self-developed CV algorithms support ROI (design §5.6.6) and the
        // shared preprocessing stage (v1.0.9: ROI → filter → downsample).
        for (auto& p : roi_params()) a.params.push_back(std::move(p));
        for (auto& p : preproc_params()) a.params.push_back(std::move(p));
        registry_[a.name] = std::move(a);
    };

    // §4.3.5 Noise Filter — removed as a standalone algorithm in v1.0.9; the
    // noise filter is now a stackable preprocessing stage (see preproc_params)
    // shared by all self-developed algorithms (ROI → filter → downsample).

    // §4.3.6 Hot Pixel Filter
    add({"hot_pixel_filter", "Hot Pixel Filter", "cv", "self",
         AlgoDisplayMode::Passive,
         {pfloat("learning_window_s", "Learning window (s)", "5.0", "0.1", "60.0"),
          pfloat("n_sigma", "N-sigma", "4.0", "1.0", "10.0"),
          pbool("enable_fpn_correction", "FPN correction", "false"),
          pfloat("fpn_target_rate_hz", "FPN target rate (Hz)", "100", "1", "1000")}});

    // §4.3.7 Orientation Filter (jAER SimpleOrientationFilter min-dt WTA)
    add({"orientation_filter", "Orientation Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("tau_us", "Time window (us)", "10000", "1000", "50000"),
          pint("min_neighbors", "Min neighbors", "1", "1", "8"),
          pint("min_dt_threshold_us", "Min dt threshold (us)", "100000", "1", "1000000"),
          pbool("multi_ori_output", "Multi-ori output", "false"),
          pbool("use_average_dt", "Use average dt", "true"),
          pbool("ori_history_enabled", "Ori history smoothing", "false"),
          pbool("pass_all_events", "Pass all events", "false"),
          pint("dt_reject_threshold_us", "Dt reject threshold (us)", "100000", "1", "10000000")}});

    // §4.3.8 Direction Selective Filter (jAER DirectionSelectiveFilter)
    add({"direction_selective", "Direction Selective Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("tau_us", "Time window (us)", "10000", "1000", "50000"),
          pint("min_dt_us", "Min dt (us)", "100", "0", "1000000"),
          pint("search_distance", "Search distance (px)", "3", "1", "12"),
          pint("tau_low_ms", "Tau low-pass (ms)", "100", "1", "100000"),
          pbool("enable_global_mode", "Global motion mode", "true")}});

    // §4.3.9 Sparse Optical Flow (4 modes: LocalPlanes/LucasKanade/BlockMatch/ClusterOF)
    add({"sparse_optical_flow", "Sparse Optical Flow", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=LocalPlanes", "1=LucasKanade",
           "2=BlockMatch", "3=ClusterOF"}),
          pint("search_radius", "Search radius (px)", "8", "3", "30"),
          pint("time_window_us", "Time window (us)", "20000", "1000", "100000"),
          pfloat("cluster_ema_alpha", "Cluster EMA alpha", "0.05", "0.001", "1.0")}});

    // §4.3.10 Blob Detector
    add({"blob_detector", "Blob Detector", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("threshold", "Threshold", "10", "1", "254"),
          pfloat("learning_rate", "Learning rate", "0.05", "0.001", "1.0")}});

    // §4.3.11 Object Tracker (4 modes, jAER RectangularClusterTracker)
    add({"object_tracker", "Object Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=RCT", "1=Median", "2=Kalman", "3=MultiHypothesis"}),
          pint("cluster_size_px", "Cluster size (px)", "10", "3", "50"),
          pint("cluster_time_us", "Cluster time (us)", "5000", "1000", "50000"),
          pint("min_cluster_events", "Min cluster events", "50", "10", "500"),
          pfloat("max_lost_age_s", "Max lost age (s)", "1.0", "0.1", "5.0"),
          pbool("enable_velocity_prediction", "Velocity prediction", "true"),
          pfloat("location_mixing_factor", "Location mixing factor", "0.05", "0.0", "1.0"),
          pfloat("predictive_velocity_factor", "Predictive velocity factor", "1.0", "0.0", "10.0"),
          pint("mass_decay_tau_us", "Mass decay tau (us)", "100000", "1", "1000000"),
          pfloat("threshold_mass_for_visible", "Threshold mass visible", "10.0", "0.0", "1000000.0")}});

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

    // §4.3.14 Hough Line Tracker (jAER HoughLineTracker)
    add({"hough_line", "Hough Line Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("threshold", "Threshold", "50", "2", "500"),
          pint("num_theta_bins", "Theta bins", "90", "8", "360"),
          pint("num_rho_bins", "Rho bins (0=auto)", "0", "0", "4000"),
          pint("accumulator_decay_us", "Decay (us)", "100000", "1000", "5000000"),
          pfloat("hough_decay_factor", "Per-packet decay factor", "0.6", "0.0", "1.0")}});

    // §4.3.15 Hough Circle Tracker (jAER HoughCircleTracker) — tightened
    // defaults to reduce lag: narrower radius range (8-30 → 23 radii vs
    // 5-50 → 46) and higher threshold (50 vs 30) so find_peaks scans fewer
    // candidates.
    add({"hough_circle", "Hough Circle Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("min_radius", "Min radius (px)", "8", "1", "100"),
          pint("max_radius", "Max radius (px)", "30", "5", "500"),
          pint("threshold", "Threshold", "50", "2", "500"),
          pint("accumulator_decay_us", "Decay (us)", "100000", "1000", "5000000"),
          pfloat("decay", "Decay factor", "1.0", "0.0", "10.0"),
          pint("buffer_length", "Buffer length", "4000", "100", "100000"),
          pint("nr_max", "Max circles", "1", "1", "20"),
          pbool("decay_mode", "Decay mode", "true"),
          pbool("loc_depression", "Local depression", "true")}});

    // §4.3.17 Orientation Cluster
    add({"orientation_cluster", "Orientation Cluster", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("min_events", "Min events", "20", "5", "500"),
          pfloat("dt", "dt (us)", "10000", "100", "1000000"),
          pfloat("factor", "Factor", "1000", "1", "100000"),
          pint("rf_width", "RF width", "1", "1", "64"),
          pint("rf_height", "RF height", "1", "1", "64"),
          pfloat("tolerance", "Tolerance (deg)", "10", "0", "180"),
          pfloat("ori", "Ori (deg)", "45", "0", "180"),
          pfloat("neighbor_thr", "Neighbor thr", "10", "0", "1000"),
          pfloat("thr_gradient", "Thr gradient", "0", "0", "100"),
          pfloat("history_factor", "History factor", "1", "0", "10"),
          pbool("use_opposite_polarity", "Use opposite polarity", "true"),
          pbool("ori_history_enabled", "Ori history enabled", "false"),
          pint("display_length", "Display length", "10", "1", "500")}});

    // §4.3.18 Cluster LIF
    add({"cluster_lif", "Cluster LIF", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("tau_ms", "Tau (ms)", "10", "1", "1000"),
          pfloat("threshold", "Threshold", "1.0", "0.1", "10.0"),
          pint("receptive_field_size_pixels", "Receptive field (px)", "8", "2", "128"),
          pfloat("initial_potential_percent", "Initial potential (%)", "50", "0", "100"),
          pfloat("jump_after_firing_percent", "Jump after firing (%)", "10", "0", "100")}});

    // §4.3.19 Background Mask Filter
    add({"background_mask", "Background Mask Filter", "cv", "self",
         AlgoDisplayMode::Replace,
         {pfloat("learning_rate", "Learning rate", "0.05", "0.001", "1.0"),
          pfloat("threshold", "Threshold", "10", "1", "100"),
          pint("erosion_size", "Erosion size", "0", "0", "20")}});

    // §4.3.20 Perspective Undistort
    add({"perspective_undistort", "Perspective Undistort", "cv", "self",
         AlgoDisplayMode::Passive,
         {pbool("enable", "Enable", "false"),
          pfloat("zoom", "Zoom", "1.0", "0.1", "10.0")}});

    // §4.3.21 Trigger Synced Filter
    add({"trigger_synced", "Trigger Synced Filter", "cv", "self",
         AlgoDisplayMode::Passive,
         {pint("window_us", "Window (us)", "10000", "1000", "1000000"),
          pint("t0_us", "T0 delay (us)", "500", "0", "1000"),
          pint("trigger_channel", "Trigger channel", "0", "0", "7")}});

    // §4.3.22 Bandpass Filter
    add({"bandpass_filter", "Bandpass Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("low_cutoff_hz", "Low cutoff (Hz)", "1.0", "0.01", "100"),
          pfloat("high_cutoff_hz", "High cutoff (Hz)", "10.0", "0.01", "1000")}});

    // §4.3.23 Optical Gyro (EIS) — Overlay: draws translation/rotation
    // vectors on top of the stabilized event display (jAER OpticalGyro
    // annotates the GL canvas with motion vectors).
    add({"optical_gyro", "EIS (Optical Gyro)", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pbool("stabilize", "Stabilize", "true"),
          pbool("rotation_enabled", "Rotation estimation", "false"),
          pfloat("smoothing_window_ms", "Smoothing window (ms)", "100", "10", "1000")}});

    // §4.3.24 Ultra Slow Motion
    add({"ultra_slow_motion", "Ultra Slow Motion", "cv", "self",
         AlgoDisplayMode::Passive,
         {pint("factor", "Dilation factor", "10", "1", "1000")}});

    // §4.3.25 XYT Visualizer
    add({"xyt_visualizer", "XYT 3D Visualizer", "cv", "self",
         AlgoDisplayMode::Standalone,
         {pint("time_window_us", "Time window (us)", "500000", "10000", "10000000"),
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
        // All self-developed analytics algorithms support ROI (design §5.6.6)
        // and the shared preprocessing stage (v1.0.9: ROI → filter → downsample).
        for (auto& p : roi_params()) a.params.push_back(std::move(p));
        for (auto& p : preproc_params()) a.params.push_back(std::move(p));
        registry_[a.name] = std::move(a);
    };

    // §4.4.1 Active Marker
    add({"active_marker", "Active Marker Tracking", "analytics", "self",
         AlgoDisplayMode::Overlay,
         {pint("window_us", "Window (us)", "10000", "1000", "100000"),
          pint("min_events", "Min events", "20", "5", "500")}});

    // §4.4.2 Event To Video (3 modes). Parameters are mode-scoped via
    // mode_filter: the UI shows only the params that apply to the currently
    // selected mode (BardowVariational=0, InteractingMaps=1, E2VID=2).
    // Common params (mode, output_fps) have an empty mode_filter.
    add({"event_to_video", "Event -> Video (E2VID)", "analytics", "self",
         AlgoDisplayMode::Standalone,
         {penum("mode", "Mode", "2", {"0=BardowVariational", "1=InteractingMaps", "2=E2VID"}),
          pint("output_fps", "Output fps", "30", "1", "120"),
          // --- Shared non-DL params (mode 0,1) ---
          pfloat("window_ms", "Window (ms)", "50", "10", "500", "0,1"),
          pfloat("decay_tau_ms", "Decay tau (ms)", "500", "0", "5000", "0,1"),
          // --- BardowVariational (mode 0) ---
          pfloat("delta_t_ms", "Delta t (ms)", "15", "1", "50", "0"),
          pfloat("theta", "Theta", "0.22", "0.05", "0.5", "0"),
          pint("num_iterations", "TV iterations", "100", "10", "500", "0"),
          pfloat("lambda1", "Lambda1 (TV weight)", "0.02", "0.0", "1.0", "0"),
          pfloat("lambda2", "Lambda2", "0.05", "0.0", "1.0", "0"),
          pfloat("lambda3", "Lambda3 (TV weight)", "0.02", "0.0", "1.0", "0"),
          pfloat("lambda4", "Lambda4", "0.2", "0.0", "2.0", "0"),
          pfloat("lambda5", "Lambda5", "0.1", "0.0", "1.0", "0"),
          pfloat("lambda6", "Lambda6 (prior)", "1.0", "0.0", "2.0", "0"),
          // --- InteractingMaps (mode 1) ---
          pfloat("relaxation_step", "Relaxation step", "0.1", "0.001", "0.5", "1"),
          pint("im_iterations", "Relax iterations", "50", "10", "1000", "1"),
          pfloat("fov_deg", "Camera FOV (deg)", "60", "10", "170", "1"),
          // --- E2VID (mode 2) ---
          pstring("model_path", "Model path (ONNX)", "models/e2vid_lightweight.onnx", "2"),
          pint("num_bins", "Num bins", "5", "1", "20", "2"),
          pbool("auto_hdr", "Auto HDR", "false", "2"),
          // 1/4 downsample is now a shared preprocessing stage
          // (preproc_downsample) — removed the per-algo downsample param.
          pfloat("unsharp_amount", "Unsharp amount", "0.3", "0.0", "2.0", "2"),
          pfloat("unsharp_sigma", "Unsharp sigma", "1.0", "0.1", "5.0", "2"),
          pfloat("bilateral_sigma", "Bilateral sigma", "0.0", "0.0", "10.0", "2")}});

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
