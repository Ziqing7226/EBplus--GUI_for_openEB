// gui/algo_bridge/backends/analytics_extra_backends.cpp — ParticleCounter, AutoBias, TriggerSynced
// (design §3.4). Split from the former algo_backend.cpp monolith.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <chrono>
#include <string>
#include <vector>

#include "algo/analytics/particle_counter.h"
#include "algo/analytics/auto_bias_controller.h"
#include "algo/analytics/freq_detector.h"
#include "algo/analytics/active_marker.h"
#include "algo/cv/trigger_synced_filter.h"
#include "algo/cv/frequency_map.h"

using namespace gui::backend_detail;

namespace gui {

class ParticleCounterBackend final : public AlgoBackend {
    gui_algo::ParticleCounter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    Metavision::timestamp last_t_{0};
    static constexpr Metavision::timestamp kMinDetectIntervalUs = 50000;
    Metavision::timestamp last_detect_t_{0};
    // -1 = auto (counting line at the sensor-height midpoint). Internal
    // sentinel only: get_param() resolves it to the actual row (the y the
    // line is drawn at), so the GUI never sees a -1 that disagrees with the
    // drawn position.
    int line_y_{-1};
public:
    ParticleCounterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "line_y") {
            const int y = to_i(v, -1);
            if (y > 0) {
                // Explicit row: forwarded as-is.
                line_y_ = y;
                algo_.set_counting_line_y(y);
            } else {
                // 0 (and legacy negative values) = auto: counting line at the
                // sensor midpoint, re-centered on dimension changes. Keep the
                // sentinel so set_sensor_dimensions picks the new center.
                line_y_ = -1;
                algo_.set_counting_line_y(algo_.height() / 2);
            }
        }
        else if (k == "min_area") algo_.set_min_particle_size_px(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        // line_y: report the row the line is ACTUALLY drawn at. Auto (-1)
        // resolves to the sensor midpoint, so the GUI default matches the
        // visual position (the panel syncs its spinbox from this value).
        if (k == "line_y") return from_i(algo_.counting_line_y());
        if (k == "min_area") return from_i(algo_.min_particle_size_px());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (!passthrough_.empty()) last_t_ = passthrough_.back().t;
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        algo_.process(ev, n, last_t_);
        // Throttle the expensive detect_and_track sweep to ~20Hz.
        if (last_detect_t_ > 0 && last_t_ - last_detect_t_ < kMinDetectIntervalUs) {
            return;
        }
        last_detect_t_ = last_t_;
        algo_.detect_and_track();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Virtual counting line: full width at the counting row.
        const int ly = algo_.counting_line_y();
        r.lines.push_back({0, ly, algo_.width() - 1, ly});
        // Detected / tracked particle bounding boxes. id = -1: a bare box
        // only, no "#N" caption cluttering the view.
        for (const auto& p : algo_.tracks()) {
            r.boxes.push_back({p.last_bbox_x, p.last_bbox_y,
                               p.last_bbox_w, p.last_bbox_h, -1});
        }
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "count: " + std::to_string(algo_.cumulative_count());
        r.texts.push_back(t);
        r.status = "counter: " + std::to_string(algo_.cumulative_count()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_t_ = 0; last_detect_t_ = 0; }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::ParticleCounter(w, h);  // sensor-sized grid
        // Re-apply an explicit counting line; -1 (auto) keeps the new
        // sensor's height/2 ctor default.
        if (line_y_ >= 0) algo_.set_counting_line_y(line_y_);
    }
};

/// AutoBiasController backend — bias command as overlay text.
/// Supports ROI (design §5.6.6): rate computed from ROI events only.
class AutoBiasBackend final : public AlgoBackend {
    gui_algo::AutoBiasController algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    Metavision::timestamp last_t_{0};
    gui_algo::BiasCommand last_cmd_;
public:
    AutoBiasBackend(int w, int h) : algo_(5.0F, 0.5F, 0.01F, 0.0F, 10.0F) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "target_event_rate_mev") algo_.set_target_event_rate_mev(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "target_event_rate_mev") return from_d(algo_.target_event_rate_mev());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        Metavision::timestamp cur_t = passthrough_.empty() ? last_t_ : passthrough_.back().t;
        const auto dt = cur_t - last_t_;
        if (dt > 0) {
            const double rate_mev = static_cast<double>(n) / (static_cast<double>(dt) * 1e-6) / 1e6;
            last_cmd_ = algo_.update(rate_mev, dt);
        }
        last_t_ = cur_t;
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        OverlayText t;
        t.x = 10; t.y = 40;
        t.text = "bias: d_diff=" + std::to_string(last_cmd_.delta_diff) +
                 " d_on=" + std::to_string(last_cmd_.delta_on);
        r.texts.push_back(t);
        r.status = "auto_bias: target=" + std::to_string(algo_.target_event_rate_mev()) + "Mev" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_t_ = 0; last_cmd_ = {}; }
    void set_sensor_dimensions(int w, int h) override {
        // The algo holds no sensor-sized state (rate controller) — only the
        // ROI geometry needs updating (audit §五-D1).
        roi_.set_sensor_dimensions(w, h);
    }
};

// ===========================================================================
// Group F: Event-vector (process returns vector<Event>)
// ===========================================================================

/// TriggerSyncedFilter backend — outputs filtered event vector.
/// Supports ROI (design §5.6.6): only ROI events reach the algo, so the
/// output vector contains only ROI events.
class TriggerSyncedBackend final : public AlgoBackend {
    gui_algo::TriggerSyncedFilter algo_;
    std::vector<Metavision::EventCD> last_out_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    TriggerSyncedBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "window_us") algo_.set_trigger_window_us(to_i(v));
        else if (k == "t0_us") algo_.set_t0(to_i(v));
        else if (k == "t1_us") algo_.set_t1(to_i(v));
        else if (k == "trigger_channel") algo_.set_trigger_channel(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "window_us") return from_i(static_cast<int>(algo_.trigger_window_us()));
        if (k == "t0_us") return from_i(static_cast<int>(algo_.t0()));
        if (k == "t1_us") return from_i(static_cast<int>(algo_.t1()));
        if (k == "trigger_channel") return from_i(algo_.trigger_channel());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        auto out = algo_.process(pkt);
        last_out_.assign(out.begin(), out.end());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = last_out_;
        r.status = "trigger_synced: " + std::to_string(last_out_.size()) + " events" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); last_out_.clear(); passthrough_.clear(); roi_buf_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::TriggerSyncedFilter(w, h);  // per-pixel timestamp grids
    }
};


// ===========================================================================
// Group E: Analyzers (process events, produce text/point overlay)
// ===========================================================================

/// FreqDetector backend — detected light sources as overlay circles + text.
/// Complex algorithm (design §4.4.7): defaults to the center 128×128 ROI.
/// Events outside the ROI are dropped before reaching the analyzer; the algo
/// keeps sensor-sized internal buffers so output coordinates remain at sensor
/// scale (correct for overlay rendering). Uses RoiFilter so the shared
/// Preprocessor (noise filter + 1/4 downsample, subsample-only) applies after
/// ROI; coordinates stay at sensor scale for overlay correctness.
class FreqDetectorBackend final : public AlgoBackend {
    gui_algo::FreqDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::vector<gui_algo::LightSource> last_;
    // Persisted so set_sensor_dimensions() rebuilds (unified-ROI automation)
    // keep the user's settings instead of reverting to ctor defaults.
    float update_interval_s_{1.0f};
    int min_events_{3};
public:
    FreqDetectorBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "update_interval_s") {
            update_interval_s_ = static_cast<float>(to_d(v));
            algo_.set_update_interval_s(update_interval_s_);
        } else if (k == "min_events") {
            min_events_ = to_i(v);
            algo_.set_min_cc_area(min_events_);
        }
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "update_interval_s") return from_d(algo_.update_interval_s());
        if (k == "min_events") return from_i(algo_.min_cc_area());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
        if (algo_.should_analyze()) last_ = algo_.analyze();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& src : last_) {
            OverlayCircle c;
            c.cx = src.u; c.cy = src.v; c.r = 8;
            r.circles.push_back(c);
            OverlayText t;
            t.x = src.u + 12; t.y = src.v;
            t.text = std::to_string(static_cast<int>(src.blink_freq_hz)) + "Hz";
            r.texts.push_back(t);
        }
        r.status = "freq: " + std::to_string(last_.size()) + " sources" +
                   std::string(roi_.region.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        // The algo keeps sensor-sized internal buffers — rebuild at the new
        // dimensions and re-apply the persisted settings.
        algo_ = gui_algo::FreqDetector(w, h);
        algo_.set_update_interval_s(update_interval_s_);
        algo_.set_min_cc_area(min_events_);
        last_.clear();
    }
};

/// ActiveMarker backend — detected markers as overlay circles + text.
/// Supports ROI (design §5.6.6): processes only ROI events; overlay coords
/// remain at sensor scale. Throttled: analyze() only runs every 50ms to
/// avoid CPU saturation (the per-event process() is cheap, but the
/// cluster-detection sweep in analyze() is expensive).
class ActiveMarkerBackend final : public AlgoBackend {
    gui_algo::ActiveMarker algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::vector<gui_algo::ClusterAnnotation> last_;
    static constexpr Metavision::timestamp kMinAnalyzeIntervalUs = 50000;
    Metavision::timestamp last_analyze_t_{0};
public:
    ActiveMarkerBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "window_us") algo_.set_window_ms(static_cast<float>(to_i(v)) / 1000.0F);
        else if (k == "min_events") algo_.set_min_cluster_area(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "window_us") return from_i(static_cast<int>(algo_.window_ms() * 1000.0F));
        if (k == "min_events") return from_i(algo_.min_cluster_area());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
        // Throttle the expensive cluster-detection sweep to ~20Hz.
        const Metavision::timestamp cur_t =
            passthrough_.empty() ? last_analyze_t_ : passthrough_.back().t;
        if (last_analyze_t_ > 0 && cur_t - last_analyze_t_ < kMinAnalyzeIntervalUs) {
            return;  // keep last_ cached result
        }
        last_analyze_t_ = cur_t;
        last_ = algo_.analyze();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& m : last_) {
            OverlayCircle c;
            c.cx = m.cx; c.cy = m.cy; c.r = 6;
            r.circles.push_back(c);
            OverlayText t;
            t.x = m.cx + 10; t.y = m.cy;
            t.text = std::to_string(static_cast<int>(m.frequency_hz)) + "Hz";
            r.texts.push_back(t);
        }
        r.status = "marker: " + std::to_string(last_.size()) + " markers" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_.clear(); last_analyze_t_ = 0; }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::ActiveMarker(w, h);  // sensor-sized cluster grid
    }
};


/// FrequencyMapBackend — per-pixel flicker frequency map (Standalone frame:
/// Jet colormap) + clustered light sources (circles + Hz labels).
/// analyze() (a full-sensor BFS cluster sweep) is throttled to ~4 Hz so the
/// GUI thread is not starved; between runs the last frame + sources are reused.
class FrequencyMapBackend final : public AlgoBackend {
    gui_algo::FrequencyMap algo_;
    gui_algo::FrequencyMapParams params_;  ///< Persisted across set_sensor_dimensions() rebuilds.
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    /// Minimum interval between analyze() runs (ms).
    int update_interval_ms_{250};
    std::chrono::steady_clock::time_point last_analyze_{};
    cv::Mat cached_frame_;
    std::vector<gui_algo::FlickerSource> cached_sources_;
public:
    FrequencyMapBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "update_interval_ms") update_interval_ms_ = std::max(50, to_i(v));
        else if (k == "filter_length") params_.frequency_filter_length = to_i(v);
        else if (k == "period_diff_thresh_us") params_.period_diff_thresh_us = to_i(v);
        else if (k == "min_freq_hz") params_.min_freq_hz = static_cast<float>(to_d(v));
        else if (k == "max_freq_hz") params_.max_freq_hz = static_cast<float>(to_d(v));
        else if (k == "max_freq_diff") params_.max_cluster_frequency_diff = static_cast<float>(to_d(v));
        else if (k == "min_cluster_size") params_.min_cluster_size = to_i(v);
        else return;
        algo_.set_params(params_);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "update_interval_ms") return from_i(update_interval_ms_);
        if (k == "filter_length") return from_i(params_.frequency_filter_length);
        if (k == "period_diff_thresh_us") return from_i(static_cast<int>(params_.period_diff_thresh_us));
        if (k == "min_freq_hz") return from_d(params_.min_freq_hz);
        if (k == "max_freq_hz") return from_d(params_.max_freq_hz);
        if (k == "max_freq_diff") return from_d(params_.max_cluster_frequency_diff);
        if (k == "min_cluster_size") return from_i(params_.min_cluster_size);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        algo_.process(ev, ev + n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_analyze_).count();
        if (cached_frame_.empty() || elapsed_ms >= update_interval_ms_) {
            algo_.analyze();
            last_analyze_ = now;
            // Frequency map → Jet colormap frame (Standalone display).
            const cv::Mat f = algo_.frequency_hz();
            double mn = 0, mx = 0;
            cv::minMaxLoc(f, &mn, &mx);
            cv::Mat u8;
            if (mx > 0.0) {
                f.convertTo(u8, CV_8UC1, 255.0 / mx, 0.0);
            } else {
                u8 = cv::Mat::zeros(f.size(), CV_8UC1);
            }
            cached_frame_.create(u8.rows, u8.cols, CV_8UC3);
            cv::applyColorMap(u8, cached_frame_, cv::COLORMAP_JET);
            cached_sources_ = algo_.sources();
        }
        r.frame = cached_frame_;
        r.has_frame = true;
        for (const auto& src : cached_sources_) {
            OverlayCircle c;
            c.cx = static_cast<int>(std::lround(src.x));
            c.cy = static_cast<int>(std::lround(src.y));
            c.r = std::max(3, static_cast<int>(std::lround(src.radius_px)));
            r.circles.push_back(c);
            OverlayText t;
            t.x = static_cast<int>(std::lround(src.x)) + c.r + 4;
            t.y = static_cast<int>(std::lround(src.y));
            t.text = std::to_string(static_cast<int>(src.frequency_hz)) + " Hz";
            r.texts.push_back(t);
        }
        r.status = "freq map: " + std::to_string(cached_sources_.size()) +
                   " sources" + std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override {
        algo_.reset();
        passthrough_.clear();
        roi_buf_.clear();
        cached_frame_ = cv::Mat();
        cached_sources_.clear();
        last_analyze_ = {};
    }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::FrequencyMap(w, h);
        algo_.set_params(params_);
        reset();
    }
};


// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_analytics_extra_backend(const std::string& name,
                                          int width, int height) {
    if (name == "freq_detector")               return std::make_unique<FreqDetectorBackend>(width, height);
    if (name == "active_marker")               return std::make_unique<ActiveMarkerBackend>(width, height);
    if (name == "particle_counter")            return std::make_unique<ParticleCounterBackend>(width, height);
    if (name == "auto_bias")                   return std::make_unique<AutoBiasBackend>(width, height);
    if (name == "trigger_synced")              return std::make_unique<TriggerSyncedBackend>(width, height);
    if (name == "frequency_map")               return std::make_unique<FrequencyMapBackend>(width, height);
    return nullptr;
}

} // namespace gui
