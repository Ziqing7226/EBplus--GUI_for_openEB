// gui/algo_bridge/algo_backend.cpp — concrete backends wrapping real algo/ classes.
//
// 每个 backend 包装一个 algo/cv 或 algo/analytics 类：
//   - push_events: 零拷贝 reinterpret_cast EventCD→Event，调用 process()/filter()
//   - pull_result: 收集过滤事件 + 叠加层 + 帧
//   - set_param/get_param: 字符串↔类型化 setter 转换
//
// 分组模式：
//   A) In-place filter:  noise_filter, hot_pixel_filter, optical_gyro, perspective_undistort
//   B) Overlay detector: object_tracker, corner_detector, blob_detector, sparse_optical_flow
//   C) Result-vector:    hough_line, hough_circle, line_segment, orientation_cluster, cluster_lif
//   D) Frame producer:   time_surface, event_to_video, flow_statistics, isi_analyzer
//   E) Analyzer:         freq_detector, active_marker, particle_counter, auto_bias
//   F) Event-vector:     trigger_synced, ultra_slow_motion
//   G) Visualization:    xyt_visualizer, overlay
//   H) Misc filter:      orientation_filter, direction_selective, background_mask, bandpass

#include "algo_backend.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

// OpenEB SDK — frame generators (design §4.3.2)
#include <metavision/sdk/core/algorithms/roi_mask_algorithm.h>
#include <metavision/sdk/core/algorithms/adaptive_rate_events_splitter_algorithm.h>
#include <metavision/sdk/core/algorithms/events_integration_algorithm.h>
#include <metavision/sdk/core/algorithms/event_frame_diff_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/event_frame_histo_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/time_decay_frame_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/contrast_map_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/periodic_frame_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/base_frame_generation_algorithm.h>
#include <metavision/sdk/base/events/raw_event_frame_diff.h>
#include <metavision/sdk/base/events/raw_event_frame_histo.h>
// OpenEB SDK — preprocessors (design §4.3.3)
#include <metavision/sdk/core/preprocessors/diff_processor.h>
#include <metavision/sdk/core/preprocessors/histo_processor.h>
#include <metavision/sdk/core/preprocessors/time_surface_processor.h>
#include <metavision/sdk/core/preprocessors/event_cube_processor.h>
#include <metavision/sdk/core/preprocessors/tensor.h>
// OpenEB SDK — utilities (design §4.3.4)
#include <metavision/sdk/core/utils/frame_composer.h>
#include <metavision/sdk/core/utils/rolling_event_buffer.h>
#include <metavision/sdk/core/utils/mostrecent_timestamp_buffer.h>

// algo/cv
#include "algo/cv/noise_filter.h"
#include "algo/cv/hot_pixel_filter.h"
#include "algo/cv/orientation_filter.h"
#include "algo/cv/direction_selective_filter.h"
#include "algo/cv/sparse_optical_flow.h"
#include "algo/cv/blob_detector.h"
#include "algo/cv/object_tracker.h"
#include "algo/cv/corner_detector.h"
#include "algo/cv/line_segment_detector.h"
#include "algo/cv/hough_line_tracker.h"
#include "algo/cv/hough_circle_tracker.h"
#include "algo/cv/orientation_cluster.h"
#include "algo/cv/cluster_lif.h"
#include "algo/cv/background_mask_filter.h"
#include "algo/cv/perspective_undistort.h"
#include "algo/cv/trigger_synced_filter.h"
#include "algo/cv/bandpass_filter.h"
#include "algo/cv/optical_gyro.h"
#include "algo/cv/ultra_slow_motion.h"
#include "algo/cv/xyt_visualizer.h"
#include "algo/cv/time_surface.h"
// NOTE: algo/cv/overlay.h intentionally NOT included — it redefines
// FlowVector/TrackedObject/Corner already defined in the per-algo headers
// above (sparse_optical_flow.h, object_tracker.h, corner_detector.h).
// OverlayBackend below is a pure pass-through; overlay drawing is handled
// by gui/display/frame_annotator from AlgoResult overlay vectors.

// algo/analytics
#include "algo/analytics/active_marker.h"
#include "algo/analytics/event_to_video.h"
#include "algo/analytics/flow_statistics.h"
#include "algo/analytics/isi_analyzer.h"
#include "algo/analytics/particle_counter.h"
#include "algo/analytics/auto_bias_controller.h"
#include "algo/analytics/freq_detector.h"

namespace gui {

namespace {

// Pi as float (kPiF is not available in all OpenCV versions).
constexpr float kPiF = 3.14159265358979323846F;

// ---- string ↔ value helpers -----------------------------------------------

int to_i(const std::string& s, int def = 0) {
    try { return std::stoi(s); } catch (...) { return def; }
}
double to_d(const std::string& s, double def = 0.0) {
    try { return std::stod(s); } catch (...) { return def; }
}
bool to_b(const std::string& s) {
    return s == "1" || s == "true" || s == "True" || s == "on" || s == "yes";
}
std::string from_i(int v) { return std::to_string(v); }
std::string from_d(double v) { return std::to_string(v); }
std::string from_b(bool v) { return v ? "true" : "false"; }

/// Zero-copy view of EventCD buffer as gui_algo::Event (layout-compatible).
const gui_algo::Event* as_events(const Metavision::EventCD* p) {
    static_assert(sizeof(gui_algo::Event) == sizeof(Metavision::EventCD),
                  "layout mismatch");
    return reinterpret_cast<const gui_algo::Event*>(p);
}

/// @brief Processing region (ROI) for complex algorithms.
/// Design §4.4.2 / §4.3.25-27: complex algorithms default to the center
/// 256×256 region to bound computational cost. The user can adjust the
/// region or disable the ROI via the Algorithms panel.
struct ProcessRegion {
    bool enabled{true};
    int x{-1};   ///< -1 = auto-center on sensor
    int y{-1};
    int w{256};  ///< 0 = full sensor width
    int h{256};  ///< 0 = full sensor height

    // Computed bounds (valid after compute() is called)
    int x0{0}, y0{0}, x1{0}, y1{0};  ///< [x0,x1) × [y0,y1)
    int rw{0}, rh{0};                  ///< ROI width / height

    void compute(int sensor_w, int sensor_h) {
        rw = (w <= 0) ? sensor_w : std::min(w, sensor_w);
        rh = (h <= 0) ? sensor_h : std::min(h, sensor_h);
        int rx = (x < 0) ? (sensor_w - rw) / 2
                          : std::min(std::max(0, x), sensor_w - rw);
        int ry = (y < 0) ? (sensor_h - rh) / 2
                          : std::min(std::max(0, y), sensor_h - rh);
        x0 = rx; y0 = ry;
        x1 = std::min(rx + rw, sensor_w);
        y1 = std::min(ry + rh, sensor_h);
        rw = x1 - x0;
        rh = y1 - y0;
    }

    bool contains(int ex, int ey) const {
        return ex >= x0 && ex < x1 && ey >= y0 && ey < y1;
    }
};

/// @brief Filters @p src events to @p roi and subtracts the ROI origin so the
/// output events are in ROI-relative coordinates. Used by frame-producer
/// backends (EventToVideo / TimeSurface / ISIAnalyzer) whose algo instances
/// are created at ROI dimensions.
std::vector<gui_algo::Event> crop_to_roi(const gui_algo::Event* src,
                                          std::size_t n,
                                          const ProcessRegion& roi) {
    std::vector<gui_algo::Event> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (roi.contains(src[i].x, src[i].y)) {
            gui_algo::Event e = src[i];
            e.x -= roi.x0;
            e.y -= roi.y0;
            out.push_back(e);
        }
    }
    return out;
}

/// @brief ROI helper for backends that keep sensor-scale coordinates (overlay
/// detectors, in-place filters, analyzers, event-vector backends).
///
/// Events outside the ROI are dropped; coordinates of kept events are
/// unchanged (sensor scale). This lets overlay primitives (boxes, points,
/// lines, circles) render at the correct position on the main display frame.
///
/// Usage: add as a member, call init(w,h) in the constructor, forward ROI
/// params in set_param(), call apply() in push_events() to get a
/// (pointer, count) pair for the filtered event range.
struct RoiFilter {
    int sensor_w{0};
    int sensor_h{0};
    ProcessRegion region;

    void init(int w, int h) {
        sensor_w = w;
        sensor_h = h;
        region.compute(w, h);
    }

    /// Handles the 5 ROI params. Returns true if @p k was a ROI param.
    bool set_param(const std::string& k, const std::string& v) {
        if (k == "roi_enabled") { region.enabled = to_b(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_x") { region.x = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_y") { region.y = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_w") { region.w = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_h") { region.h = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        return false;
    }

    std::string get_param(const std::string& k) const {
        if (k == "roi_enabled") return from_b(region.enabled);
        return {};
    }

    /// @brief Filters events to ROI, keeping sensor coordinates.
    /// @return (pointer, count). If ROI is disabled, returns the original
    ///         (src, n); otherwise fills @p buf with the filtered events.
    std::pair<const gui_algo::Event*, std::size_t> apply(
        const gui_algo::Event* src, std::size_t n,
        std::vector<gui_algo::Event>& buf) const {
        if (!region.enabled || region.rw <= 0 || region.rh <= 0) {
            return {src, n};
        }
        buf.clear();
        buf.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            if (region.contains(src[i].x, src[i].y)) {
                buf.push_back(src[i]);
            }
        }
        return {buf.data(), buf.size()};
    }
};

} // namespace

// ===========================================================================
// Group A: In-place event filters (compact / modify events)
// ===========================================================================

/// NoiseFilter backend — 8-mode denoiser, compacts events in place.
class NoiseFilterBackend final : public AlgoBackend {
    gui_algo::NoiseFilter algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
    std::size_t last_kept_{0};
    double last_rate_{0.0};
public:
    NoiseFilterBackend(int w, int h)
        : algo_(w, h, gui_algo::NoiseFilter::Mode::STCF) {
        roi_.init(w, h);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        // General
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 7) algo_.set_mode(static_cast<gui_algo::NoiseFilter::Mode>(m));
        }
        // STCF (mode 1)
        else if (k == "correlation_time_s") algo_.set_correlation_time_s(to_d(v));
        else if (k == "min_neighbors") algo_.set_min_neighbors(to_i(v));
        else if (k == "require_polarity_match") algo_.set_require_polarity_match(to_b(v));
        else if (k == "allow_coincidence") algo_.set_allow_coincidence(to_b(v));
        // BAF (mode 0)
        else if (k == "baf_dt_us") algo_.set_baf_dt_us(to_i(v));
        else if (k == "baf_subsample_by") algo_.set_baf_subsample_by(to_i(v));
        // Refractory (mode 2)
        else if (k == "refractory_us") algo_.set_refractory_period_us(to_i(v));
        // DWF (mode 3)
        else if (k == "dwf_window_length") algo_.set_dwf_window_length(to_i(v));
        else if (k == "dwf_dist_threshold") algo_.set_dwf_dist_threshold(to_i(v));
        else if (k == "dwf_min_correlated") algo_.set_dwf_min_correlated(to_i(v));
        else if (k == "dwf_double_mode") algo_.set_dwf_double_mode(to_b(v));
        // AgePolarity (mode 4)
        else if (k == "agep_tau_us") algo_.set_tau_us(to_i(v));
        else if (k == "age_threshold") algo_.set_age_threshold(to_d(v));
        else if (k == "agep_radius") algo_.set_agep_radius(to_i(v));
        // Harmonic (mode 5)
        else if (k == "line_freq_hz") algo_.set_line_freq(to_i(v) == 60 ? gui_algo::NoiseFilter::LineFreq::Hz60 : gui_algo::NoiseFilter::LineFreq::Hz50);
        else if (k == "notch_q") algo_.set_notch_q(to_d(v));
        else if (k == "harmonic_threshold") algo_.set_harmonic_threshold(to_d(v));
        // Repetitious (mode 6)
        else if (k == "rep_period_us") algo_.set_period_us(to_i(v));
        else if (k == "rep_tolerance_us") algo_.set_tolerance_us(to_i(v));
        else if (k == "rep_ratio_shorter") algo_.set_ratio_shorter(to_i(v));
        else if (k == "rep_ratio_longer") algo_.set_ratio_longer(to_i(v));
        else if (k == "rep_min_dt_to_store_us") algo_.set_min_dt_to_store_us(to_i(v));
        // SpatialBP (mode 7)
        else if (k == "sbp_center_radius_px") algo_.set_center_radius_px(to_i(v));
        else if (k == "sbp_surround_radius_px") algo_.set_surround_radius_px(to_i(v));
        else if (k == "sbp_dt_surround_us") algo_.set_dt_surround_us(to_i(v));
        // Cross-mode flags
        else if (k == "filter_hot_pixels") algo_.set_filter_hot_pixels(to_b(v));
        else if (k == "adaptive_correlation_time") algo_.set_adaptive_correlation_time(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        // STCF
        if (k == "correlation_time_s") return from_d(algo_.correlation_time_s());
        if (k == "min_neighbors") return from_i(algo_.min_neighbors());
        if (k == "require_polarity_match") return from_b(algo_.require_polarity_match());
        if (k == "allow_coincidence") return from_b(algo_.allow_coincidence());
        // BAF
        if (k == "baf_dt_us") return from_i(algo_.baf_dt_us());
        if (k == "baf_subsample_by") return from_i(algo_.baf_subsample_by());
        // Refractory
        if (k == "refractory_us") return from_i(algo_.refractory_period_us());
        // DWF
        if (k == "dwf_window_length") return from_i(algo_.dwf_window_length());
        if (k == "dwf_dist_threshold") return from_i(algo_.dwf_dist_threshold());
        if (k == "dwf_min_correlated") return from_i(algo_.dwf_min_correlated());
        if (k == "dwf_double_mode") return from_b(algo_.dwf_double_mode());
        // AgePolarity
        if (k == "agep_tau_us") return from_i(algo_.tau_us());
        if (k == "age_threshold") return from_d(algo_.age_threshold());
        if (k == "agep_radius") return from_i(algo_.agep_radius());
        // Harmonic
        if (k == "line_freq_hz") return from_i(algo_.line_freq_hz());
        if (k == "notch_q") return from_d(algo_.notch_q());
        if (k == "harmonic_threshold") return from_d(algo_.harmonic_threshold());
        // Repetitious
        if (k == "rep_period_us") return from_i(algo_.period_us());
        if (k == "rep_tolerance_us") return from_i(algo_.tolerance_us());
        if (k == "rep_ratio_shorter") return from_i(algo_.ratio_shorter());
        if (k == "rep_ratio_longer") return from_i(algo_.ratio_longer());
        if (k == "rep_min_dt_to_store_us") return from_i(algo_.min_dt_to_store_us());
        // SpatialBP
        if (k == "sbp_center_radius_px") return from_i(algo_.center_radius_px());
        if (k == "sbp_surround_radius_px") return from_i(algo_.surround_radius_px());
        if (k == "sbp_dt_surround_us") return from_i(algo_.dt_surround_us());
        // Cross-mode
        if (k == "filter_hot_pixels") return from_b(algo_.filter_hot_pixels());
        if (k == "adaptive_correlation_time") return from_b(algo_.adaptive_correlation_time());
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        std::size_t n = buf_.size();
        // Compact in place to ROI events (design §5.6.6).
        if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
            }
            n = kept;
        }
        buf_.resize(n);
        last_kept_ = algo_.filter(ev, n);
        last_rate_ = algo_.filter_rate();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events.assign(buf_.data(), buf_.data() + last_kept_);
        r.status = "noise_filter[" + std::to_string(static_cast<int>(algo_.mode())) +
                   "]: kept " + std::to_string(last_kept_) + "/" +
                   std::to_string(buf_.size()) + " (" + std::to_string(last_rate_ * 100) + "%)" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); last_kept_ = 0; }
};

/// HotPixelFilter backend — learns hot-pixel mask + compacts events.
class HotPixelFilterBackend final : public AlgoBackend {
    gui_algo::HotPixelFilter algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
    std::size_t last_kept_{0};
public:
    HotPixelFilterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "learning_window_s") algo_.set_learning_window_s(to_d(v));
        else if (k == "n_sigma") algo_.set_n_sigma(to_d(v));
        else if (k == "enable_fpn_correction") algo_.set_enable_fpn_correction(to_b(v));
        else if (k == "fpn_target_rate_hz") algo_.set_fpn_target_rate_hz(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "learning_window_s") return from_d(algo_.learning_window_s());
        if (k == "n_sigma") return from_d(algo_.n_sigma());
        if (k == "enable_fpn_correction") return from_b(algo_.enable_fpn_correction());
        if (k == "fpn_target_rate_hz") return from_d(algo_.fpn_target_rate_hz());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        std::size_t n = buf_.size();
        if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
            }
            n = kept;
        }
        buf_.resize(n);
        algo_.learn(as_events(buf_.data()), n);
        last_kept_ = algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events.assign(buf_.data(), buf_.data() + last_kept_);
        r.status = "hot_pixel: " + std::to_string(algo_.hot_pixel_count()) + " hot px, kept " +
                   std::to_string(last_kept_) + std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); last_kept_ = 0; }
};

/// OpticalGyro (EIS) backend — modifies event coordinates in place.
class OpticalGyroBackend final : public AlgoBackend {
    gui_algo::OpticalGyro algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
public:
    OpticalGyroBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "stabilize") algo_.set_stabilization_strength(to_b(v) ? 1.0F : 0.0F);
        else if (k == "smoothing_window_ms") algo_.set_smoothing_window_ms(static_cast<float>(to_d(v)));
        else if (k == "rotation_enabled") algo_.set_rotation_enabled(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "stabilize") return from_b(algo_.stabilization_strength() > 0.0F);
        if (k == "smoothing_window_ms") return from_d(algo_.smoothing_window_ms());
        if (k == "rotation_enabled") return from_b(algo_.rotation_enabled());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        std::size_t n = buf_.size();
        if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
            }
            n = kept;
        }
        buf_.resize(n);
        gui_algo::MutableEventPacket pkt(ev, n);
        algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = buf_;
        const auto m = algo_.smoothed_motion();
        // Translation vector (jAER OpticalGyro.drawVector):
        // arrow from chip center showing cumulative shift.
        const int cx = algo_.width() / 2;
        const int cy = algo_.height() / 2;
        const float scale = 5.0F;  // px/px for visibility
        OverlayLine tl;
        tl.x1 = cx; tl.y1 = cy;
        tl.x2 = cx + static_cast<int>(m.dx * scale);
        tl.y2 = cy + static_cast<int>(m.dy * scale);
        r.lines.push_back(tl);
        // Rotation indicator (jAER rotationAngle): an arc segment.
        if (std::fabs(m.dtheta) > 1e-3F) {
            const int R = 50;
            const float a0 = -static_cast<float>(kPiF) * 0.25F;
            const float a1 = a0 + m.dtheta * static_cast<float>(kPiF) / 180.0F * 2.0F;
            OverlayLine rl;
            rl.x1 = cx + static_cast<int>(std::cos(a0) * R);
            rl.y1 = cy + static_cast<int>(std::sin(a0) * R);
            rl.x2 = cx + static_cast<int>(std::cos(a1) * R);
            rl.y2 = cy + static_cast<int>(std::sin(a1) * R);
            r.lines.push_back(rl);
        }
        // Motion text overlay.
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "EIS trans=(" + std::to_string(static_cast<int>(m.dx)) + "," +
                  std::to_string(static_cast<int>(m.dy)) + ") rot=" +
                  std::to_string(static_cast<int>(m.dtheta)) + "deg";
        r.texts.push_back(t);
        r.status = "EIS: shift=(" + std::to_string(m.dx) + "," +
                   std::to_string(m.dy) + ") rot=" + std::to_string(m.dtheta) + "deg" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); }
};

/// PerspectiveUndistort backend — remaps event coordinates in place.
class PerspectiveUndistortBackend final : public AlgoBackend {
    gui_algo::PerspectiveUndistort algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
public:
    PerspectiveUndistortBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "enable") algo_.set_undistort(to_b(v));
        else if (k == "zoom") algo_.set_zoom(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "enable") return from_b(algo_.undistort());
        if (k == "zoom") return from_d(algo_.zoom());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        std::size_t n = buf_.size();
        if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
            }
            n = kept;
        }
        buf_.resize(n);
        gui_algo::MutableEventPacket pkt(ev, n);
        algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = buf_;
        r.status = std::string("undistort: ") + (algo_.undistort() ? "on" : "off") +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); }
};

// ===========================================================================
// Group B: Overlay detectors (process events, produce overlay data)
// ===========================================================================

/// ObjectTracker backend — tracked objects as overlay boxes + trajectories.
class ObjectTrackerBackend final : public AlgoBackend {
    gui_algo::ObjectTracker algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    ObjectTrackerBackend(int w, int h)
        : algo_(w, h, gui_algo::ObjectTracker::Mode::RCT) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::ObjectTracker::Mode>(m));
        } else if (k == "cluster_size_px") algo_.set_cluster_size_px(to_i(v));
        else if (k == "cluster_time_us") algo_.set_cluster_time_us(to_i(v));
        else if (k == "min_cluster_events") algo_.set_min_cluster_events(to_i(v));
        else if (k == "max_lost_age_s") algo_.set_max_lost_age_s(to_d(v));
        else if (k == "enable_velocity_prediction") algo_.set_enable_velocity_prediction(to_b(v));
        else if (k == "location_mixing_factor") algo_.set_location_mixing_factor(static_cast<float>(to_d(v)));
        else if (k == "predictive_velocity_factor") algo_.set_predictive_velocity_factor(static_cast<float>(to_d(v)));
        else if (k == "mass_decay_tau_us") algo_.set_mass_decay_tau_us(to_i(v));
        else if (k == "threshold_mass_for_visible") algo_.set_threshold_mass_for_visible(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "cluster_size_px") return from_i(algo_.cluster_size_px());
        if (k == "cluster_time_us") return from_i(algo_.cluster_time_us());
        if (k == "min_cluster_events") return from_i(algo_.min_cluster_events());
        if (k == "max_lost_age_s") return from_d(algo_.max_lost_age_s());
        if (k == "enable_velocity_prediction") return from_b(algo_.enable_velocity_prediction());
        if (k == "location_mixing_factor") return from_d(algo_.location_mixing_factor());
        if (k == "predictive_velocity_factor") return from_d(algo_.predictive_velocity_factor());
        if (k == "mass_decay_tau_us") return from_i(algo_.mass_decay_tau_us());
        if (k == "threshold_mass_for_visible") return from_d(algo_.threshold_mass_for_visible());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        const auto& objs = algo_.objects();
        for (const auto& o : objs) {
            if (!o.visible) continue;
            OverlayBox box;
            box.x = o.bbox.x; box.y = o.bbox.y;
            box.w = o.bbox.width; box.h = o.bbox.height;
            box.id = o.id;
            r.boxes.push_back(box);
            // Velocity arrow (jAER Cluster.drawVelocityVector).
            OverlayLine v;
            v.x1 = static_cast<int>(o.x);
            v.y1 = static_cast<int>(o.y);
            // Scale velocity (px/s) to a visible length (assume 30ms window).
            v.x2 = v.x1 + static_cast<int>(o.vx * 0.03F);
            v.y2 = v.y1 + static_cast<int>(o.vy * 0.03F);
            r.lines.push_back(v);
            // Trajectory (jAER ClusterPath).
            if (!o.trajectory.empty()) {
                OverlayTrajectory tr;
                tr.id = o.id;
                tr.points.reserve(o.trajectory.size());
                for (const auto& p : o.trajectory) {
                    tr.points.emplace_back(static_cast<int>(p.x),
                                            static_cast<int>(p.y));
                }
                r.trajectories.push_back(tr);
            }
            OverlayText t;
            t.x = o.bbox.x; t.y = o.bbox.y > 12 ? o.bbox.y - 12 : 0;
            t.text = "#" + std::to_string(o.id) + " v=(" +
                     std::to_string(static_cast<int>(o.vx)) + "," +
                     std::to_string(static_cast<int>(o.vy)) + ")";
            r.texts.push_back(t);
        }
        r.status = "tracker: " + std::to_string(objs.size()) + " objects" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// CornerDetector backend — corners as overlay points.
class CornerDetectorBackend final : public AlgoBackend {
    gui_algo::CornerDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    CornerDetectorBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 2) algo_.set_mode(static_cast<gui_algo::CornerDetector::Mode>(m));
        } else if (k == "min_score") algo_.set_threshold(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "min_score") return from_d(algo_.threshold());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : algo_.corners()) {
            OverlayPoint p;
            p.x = static_cast<int>(c.x);
            p.y = static_cast<int>(c.y);
            p.strength = c.strength;
            r.points.push_back(p);
        }
        r.status = "corners: " + std::to_string(algo_.corners().size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// BlobDetector backend — blobs as overlay boxes.
class BlobDetectorBackend final : public AlgoBackend {
    gui_algo::BlobDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    BlobDetectorBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "threshold") algo_.set_threshold(static_cast<int>(to_d(v)));
        else if (k == "learning_rate") algo_.set_learning_rate(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "threshold") return from_d(algo_.threshold());
        if (k == "learning_rate") return from_d(algo_.learning_rate());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& blob : algo_.blobs()) {
            OverlayBox box;
            box.x = blob.bbox.x; box.y = blob.bbox.y;
            box.w = blob.bbox.width; box.h = blob.bbox.height;
            r.boxes.push_back(box);
        }
        r.status = "blobs: " + std::to_string(algo_.blobs().size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// SparseOpticalFlow backend — flow vectors as overlay lines.
class SparseOpticalFlowBackend final : public AlgoBackend {
    gui_algo::SparseOpticalFlow algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::FlowVector> flows_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    SparseOpticalFlowBackend(int w, int h)
        : algo_(w, h, gui_algo::SparseOpticalFlow::Mode::LocalPlanes) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::SparseOpticalFlow::Mode>(m));
        } else if (k == "search_radius") algo_.set_search_radius_px(to_i(v));
        else if (k == "time_window_us") algo_.set_time_window_us(to_i(v));
        else if (k == "cluster_ema_alpha") algo_.set_cluster_ema_alpha(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "search_radius") return from_i(algo_.search_radius_px());
        if (k == "time_window_us") return from_i(algo_.time_window_us());
        if (k == "cluster_ema_alpha") return from_d(algo_.cluster_ema_alpha());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        flows_.clear();
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n, flows_);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Optical-flow visualization: render each flow vector as a single
        // colored pixel/square at its origin. Color encodes direction (HSV
        // hue 0..360° mapped to the (vx,vy) angle) and brightness encodes
        // magnitude. This is the standard dense-flow visualization style
        // (Farneback/OpenCV hue-intensity scheme) and avoids the previous
        // "long lines all over the screen" problem caused by drawing
        // vx*10/vy*10 line segments.
        r.colored_points.reserve(flows_.size());
        for (const auto& f : flows_) {
            OverlayColoredPoint cp;
            cp.x = f.x;
            cp.y = f.y;
            // Direction -> hue (atan2 returns -pi..pi, map to 0..1).
            const float angle_rad = std::atan2(f.vy, f.vx);
            float hue = (angle_rad + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI));
            if (hue < 0.0f) hue = 0.0f;
            if (hue > 1.0f) hue = 1.0f;
            // Magnitude -> value (normalize against a typical px/s reference).
            const float mag = std::sqrt(f.vx * f.vx + f.vy * f.vy);
            const float val = std::min(1.0f, mag / 1000.0f);  // 1000 px/s -> full brightness
            hsv_to_rgb(hue, 1.0f, val, cp.r, cp.g, cp.b);
            r.colored_points.push_back(cp);
        }
        r.status = "flow: " + std::to_string(flows_.size()) + " vectors" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); flows_.clear(); }
private:
    /// HSV (h,s,v in [0,1]) -> RGB (r,g,b in [0,255]).
    static void hsv_to_rgb(float h, float s, float v,
                           std::uint8_t& r, std::uint8_t& g, std::uint8_t& b) {
        const float c = v * s;
        const float hp = h * 6.0f;
        const float x = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
        float rf = 0.0f, gf = 0.0f, bf = 0.0f;
        if (hp < 1.0f)      { rf = c; gf = x; bf = 0.0f; }
        else if (hp < 2.0f) { rf = x; gf = c; bf = 0.0f; }
        else if (hp < 3.0f) { rf = 0.0f; gf = c; bf = x; }
        else if (hp < 4.0f) { rf = 0.0f; gf = x; bf = c; }
        else if (hp < 5.0f) { rf = x; gf = 0.0f; bf = c; }
        else                { rf = c; gf = 0.0f; bf = x; }
        const float m = v - c;
        r = static_cast<std::uint8_t>((rf + m) * 255.0f + 0.5f);
        g = static_cast<std::uint8_t>((gf + m) * 255.0f + 0.5f);
        b = static_cast<std::uint8_t>((bf + m) * 255.0f + 0.5f);
    }
};

// ===========================================================================
// Group C: Result-vector detectors (process returns vector<Result>)
// ===========================================================================

/// HoughLineTracker backend — detected lines as overlay lines.
///
/// Complex algorithm (design §4.3.14): the Hough accumulator is built at ROI
/// dimensions (not sensor dimensions) to bound memory and per-frame scan cost.
/// Events are cropped to ROI-relative coordinates; detected line endpoints are
/// shifted back by the ROI origin so they render at the correct sensor position.
class HoughLineBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    int num_theta_bins_{90};
    int num_rho_bins_{0};
    int threshold_{50};
    Metavision::timestamp decay_us_{100000};
    std::unique_ptr<gui_algo::HoughLineTracker> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    std::vector<gui_algo::HoughLine> last_;
public:
    HoughLineBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        algo_ = std::make_unique<gui_algo::HoughLineTracker>(
            aw, ah, num_theta_bins_, num_rho_bins_, threshold_, decay_us_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool need_rebuild = false;
        if (k == "threshold") { threshold_ = to_i(v); if (algo_) algo_->set_threshold(threshold_); }
        else if (k == "num_theta_bins") { num_theta_bins_ = to_i(v); need_rebuild = true; }
        else if (k == "num_rho_bins") { num_rho_bins_ = to_i(v); need_rebuild = true; }
        else if (k == "accumulator_decay_us") {
            decay_us_ = static_cast<Metavision::timestamp>(to_i(v));
            if (algo_) algo_->set_accumulator_decay_us(decay_us_);
        }
        else if (k == "hough_decay_factor") {
            if (algo_) algo_->set_hough_decay_factor(static_cast<float>(to_d(v)));
        }
        else if (k == "roi_enabled") { roi_.enabled = to_b(v); need_rebuild = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); need_rebuild = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); need_rebuild = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); need_rebuild = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); need_rebuild = true; }
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "threshold" && algo_) return from_i(algo_->threshold());
        if (k == "num_theta_bins") return from_i(num_theta_bins_);
        if (k == "num_rho_bins") return from_i(num_rho_bins_);
        if (k == "accumulator_decay_us") return from_i(static_cast<int>(decay_us_));
        if (k == "hough_decay_factor" && algo_) return from_d(algo_->hough_decay_factor());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            roi_events_ = crop_to_roi(as_events(passthrough_.data()),
                                       passthrough_.size(), roi_);
            gui_algo::EventPacket pkt(roi_events_.data(), roi_events_.size());
            last_ = algo_->process(pkt);
        } else {
            gui_algo::EventPacket pkt(as_events(passthrough_.data()),
                                      passthrough_.size());
            last_ = algo_->process(pkt);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Shift ROI-relative endpoints back to sensor coordinates.
        const int dx = (roi_.enabled && roi_.rw > 0) ? roi_.x0 : 0;
        const int dy = (roi_.enabled && roi_.rh > 0) ? roi_.y0 : 0;
        for (const auto& hl : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(hl.start.x) + dx;
            l.y1 = static_cast<int>(hl.start.y) + dy;
            l.x2 = static_cast<int>(hl.end.x) + dx;
            l.y2 = static_cast<int>(hl.end.y) + dy;
            r.lines.push_back(l);
        }
        // Aux frame: Hough θ-ρ accumulator (jAER HoughLineTracker GL render).
        if (algo_) {
            const auto& accum = algo_->accum();
            const int nt = algo_->num_theta_bins();
            const int nr = algo_->num_rho_bins();
            if (nt > 0 && nr > 0 && static_cast<int>(accum.size()) == nt * nr) {
                cv::Mat hough(nt, nr, CV_32F, const_cast<float*>(accum.data()));
                double mn, mx;
                cv::minMaxLoc(hough, &mn, &mx);
                cv::Mat vis;
                if (mx > mn) {
                    hough.convertTo(vis, CV_8U, 255.0 / (mx - mn), -mn * 255.0 / (mx - mn));
                } else {
                    vis = cv::Mat::zeros(nt, nr, CV_8U);
                }
                cv::applyColorMap(vis, r.aux_frame, cv::COLORMAP_JET);
                r.has_aux_frame = true;
            }
        }
        r.status = "hough_line: " + std::to_string(last_.size()) + " lines" +
                   std::string(roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear(); roi_events_.clear(); last_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

/// HoughCircleTracker backend — detected circles as overlay circles.
///
/// Complex algorithm (design §4.3.15): the 3D accumulator (a, b, r) is built
/// at ROI dimensions, NOT sensor dimensions. At sensor scale (e.g. 1280×720)
/// with max_radius=50 the accumulator would be ~42M cells (168 MB) and
/// find_peaks would scan every cell per frame — a memory/CPU blowup that
/// freezes/crashes the GUI. Using the ROI (default 128×128) bounds this to
/// ~750K cells. Events are cropped to ROI-relative coordinates; detected
/// circle centers are shifted back by the ROI origin for overlay rendering.
class HoughCircleBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    int min_radius_{8};
    int max_radius_{30};
    int threshold_{50};
    Metavision::timestamp decay_us_{100000};
    // jAER params (persisted so rebuild() preserves them)
    float decay_{1.0f};
    int buffer_length_{4000};
    int nr_max_{1};
    bool decay_mode_{true};
    bool loc_depression_{true};
    std::unique_ptr<gui_algo::HoughCircleTracker> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    std::vector<gui_algo::HoughCircle> last_;
    /// Throttle: only run find_peaks every kMinProcessIntervalUs to avoid
    /// CPU saturation. Between runs the last cached result is returned.
    /// 50ms = 20Hz update, smooth enough for visual tracking.
    static constexpr Metavision::timestamp kMinProcessIntervalUs = 50000;
    Metavision::timestamp last_process_t_{0};
public:
    HoughCircleBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        algo_ = std::make_unique<gui_algo::HoughCircleTracker>(
            aw, ah, min_radius_, max_radius_, threshold_, decay_us_,
            decay_, buffer_length_, nr_max_, decay_mode_, loc_depression_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool need_rebuild = false;
        if (k == "min_radius") { min_radius_ = to_i(v); need_rebuild = true; }
        else if (k == "max_radius") { max_radius_ = to_i(v); need_rebuild = true; }
        else if (k == "threshold") { threshold_ = to_i(v); if (algo_) algo_->set_threshold(threshold_); }
        else if (k == "accumulator_decay_us") {
            decay_us_ = static_cast<Metavision::timestamp>(to_i(v));
            if (algo_) algo_->set_accumulator_decay_us(decay_us_);
        }
        else if (k == "decay") { decay_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_decay(decay_); }
        else if (k == "buffer_length") { buffer_length_ = to_i(v); if (algo_) algo_->set_buffer_length(buffer_length_); }
        else if (k == "nr_max") { nr_max_ = to_i(v); if (algo_) algo_->set_nr_max(nr_max_); }
        else if (k == "decay_mode") { decay_mode_ = to_b(v); if (algo_) algo_->set_decay_mode(decay_mode_); }
        else if (k == "loc_depression") { loc_depression_ = to_b(v); if (algo_) algo_->set_loc_depression(loc_depression_); }
        else if (k == "roi_enabled") { roi_.enabled = to_b(v); need_rebuild = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); need_rebuild = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); need_rebuild = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); need_rebuild = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); need_rebuild = true; }
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "min_radius") return from_i(min_radius_);
        if (k == "max_radius") return from_i(max_radius_);
        if (k == "threshold") return from_i(threshold_);
        if (k == "accumulator_decay_us") return from_i(static_cast<int>(decay_us_));
        if (k == "decay" && algo_) return from_d(algo_->decay());
        if (k == "buffer_length" && algo_) return from_i(algo_->buffer_length());
        if (k == "nr_max" && algo_) return from_i(algo_->nr_max());
        if (k == "decay_mode" && algo_) return from_b(algo_->decay_mode());
        if (k == "loc_depression" && algo_) return from_b(algo_->loc_depression());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        // Throttle: skip the expensive process() call if not enough time has
        // elapsed since the last run. The accumulator still receives events
        // (accumulate is cheap), but find_peaks (the O(W×H×R) scan) only
        // runs at ~20Hz. This eliminates the "满屏幕卡顿" lag.
        const Metavision::timestamp cur_t =
            passthrough_.empty() ? last_process_t_ : passthrough_.back().t;
        if (last_process_t_ > 0 && cur_t - last_process_t_ < kMinProcessIntervalUs) {
            return;  // keep last_ cached result
        }
        last_process_t_ = cur_t;
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            roi_events_ = crop_to_roi(as_events(passthrough_.data()),
                                       passthrough_.size(), roi_);
            gui_algo::EventPacket pkt(roi_events_.data(), roi_events_.size());
            last_ = algo_->process(pkt);
        } else {
            gui_algo::EventPacket pkt(as_events(passthrough_.data()),
                                      passthrough_.size());
            last_ = algo_->process(pkt);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Shift ROI-relative centers back to sensor coordinates.
        const int dx = (roi_.enabled && roi_.rw > 0) ? roi_.x0 : 0;
        const int dy = (roi_.enabled && roi_.rh > 0) ? roi_.y0 : 0;
        for (const auto& c : last_) {
            OverlayCircle oc;
            oc.cx = static_cast<int>(c.center.x) + dx;
            oc.cy = static_cast<int>(c.center.y) + dy;
            oc.r = static_cast<int>(c.radius);
            r.circles.push_back(oc);
        }
        // Aux frame: per-pixel Hough accumulator (jAER HoughCircleTracker GL).
        if (algo_) {
            const auto& accum = algo_->accum();
            const int aw = (roi_.enabled && roi_.rw > 0) ? roi_.rw : sensor_w_;
            const int ah = (roi_.enabled && roi_.rh > 0) ? roi_.rh : sensor_h_;
            if (aw > 0 && ah > 0 &&
                static_cast<int>(accum.size()) == aw * ah) {
                cv::Mat hough(ah, aw, CV_32F, const_cast<float*>(accum.data()));
                double mn, mx;
                cv::minMaxLoc(hough, &mn, &mx);
                cv::Mat vis;
                if (mx > mn) {
                    hough.convertTo(vis, CV_8U, 255.0 / (mx - mn), -mn * 255.0 / (mx - mn));
                } else {
                    vis = cv::Mat::zeros(ah, aw, CV_8U);
                }
                cv::applyColorMap(vis, r.aux_frame, cv::COLORMAP_JET);
                r.has_aux_frame = true;
            }
        }
        r.status = "hough_circle: " + std::to_string(last_.size()) + " circles" +
                   std::string(roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear(); roi_events_.clear(); last_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

/// LineSegmentDetector (ELiSeD) backend — detected segments as overlay lines.
class LineSegmentBackend final : public AlgoBackend {
    gui_algo::LineSegmentDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::LineSegment> last_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    LineSegmentBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "min_length") algo_.set_min_line_length_px(to_i(v));
        else if (k == "gap") algo_.set_max_line_gap_px(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "min_length") return from_i(algo_.min_line_length_px());
        if (k == "gap") return from_i(algo_.max_line_gap_px());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& seg : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(seg.start.x); l.y1 = static_cast<int>(seg.start.y);
            l.x2 = static_cast<int>(seg.end.x); l.y2 = static_cast<int>(seg.end.y);
            r.lines.push_back(l);
        }
        r.status = "elised: " + std::to_string(last_.size()) + " segments" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// OrientationCluster backend — detected orientation clusters as overlay lines.
class OrientationClusterBackend final : public AlgoBackend {
    gui_algo::OrientationCluster algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::OrientationClusterResult> last_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    OrientationClusterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "min_events") algo_.set_min_cluster_size(to_i(v));
        else if (k == "dt") algo_.set_dt(static_cast<float>(to_d(v)));
        else if (k == "factor") algo_.set_factor(static_cast<float>(to_d(v)));
        else if (k == "rf_width") algo_.set_rf_width(to_i(v));
        else if (k == "rf_height") algo_.set_rf_height(to_i(v));
        else if (k == "tolerance") algo_.set_tolerance(static_cast<float>(to_d(v)));
        else if (k == "ori") algo_.set_ori(static_cast<float>(to_d(v)));
        else if (k == "neighbor_thr") algo_.set_neighbor_thr(static_cast<float>(to_d(v)));
        else if (k == "thr_gradient") algo_.set_thr_gradient(static_cast<float>(to_d(v)));
        else if (k == "history_factor") algo_.set_history_factor(static_cast<float>(to_d(v)));
        else if (k == "use_opposite_polarity") algo_.set_use_opposite_polarity(to_b(v));
        else if (k == "ori_history_enabled") algo_.set_ori_history_enabled(to_b(v));
        else if (k == "display_length") algo_.set_display_length(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "min_events") return from_i(algo_.min_cluster_size());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(c.centroid.x); l.y1 = static_cast<int>(c.centroid.y);
            l.x2 = static_cast<int>(c.centroid.x) + static_cast<int>(std::cos(c.orientation) * c.size);
            l.y2 = static_cast<int>(c.centroid.y) + static_cast<int>(std::sin(c.orientation) * c.size);
            r.lines.push_back(l);
        }
        r.status = "orient_cluster: " + std::to_string(last_.size()) + " clusters" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// ClusterLIF backend — LIF clusters as overlay boxes.
class ClusterLifBackend final : public AlgoBackend {
    gui_algo::ClusterLIF algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::LifCluster> last_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    ClusterLifBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "tau_ms") algo_.set_tau_ms(static_cast<float>(to_d(v)));
        else if (k == "threshold") algo_.set_threshold(static_cast<float>(to_d(v)));
        else if (k == "receptive_field_size_pixels") algo_.set_receptive_field_size_pixels(to_i(v));
        else if (k == "initial_potential_percent") algo_.set_initial_potential_percent(static_cast<float>(to_d(v)));
        else if (k == "jump_after_firing_percent") algo_.set_jump_after_firing_percent(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "tau_ms") return from_d(algo_.tau_ms());
        if (k == "threshold") return from_d(algo_.threshold());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : last_) {
            OverlayBox box;
            box.x = static_cast<int>(c.position.x) - 5;
            box.y = static_cast<int>(c.position.y) - 5;
            box.w = 10; box.h = 10;
            r.boxes.push_back(box);
        }
        r.status = "lif: " + std::to_string(last_.size()) + " clusters" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

// ===========================================================================
// Group D: Frame producers (process events, produce cv::Mat frame)
// ===========================================================================

/// TimeSurface backend — produces a time-decay pseudo-color frame.
/// Complex algorithm (design §4.3.27): defaults to the center 128×128 ROI.
class TimeSurfaceBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    int decay_time_us_{100000};
    gui_algo::TimeSurface::Palette palette_{gui_algo::TimeSurface::Palette::Hot};
    gui_algo::TimeSurface::Channels channels_{gui_algo::TimeSurface::Channels::Merged};
    std::unique_ptr<gui_algo::TimeSurface> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
public:
    TimeSurfaceBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        algo_ = std::make_unique<gui_algo::TimeSurface>(
            aw, ah, channels_, decay_time_us_, palette_, 30);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool need_rebuild = false;
        if (k == "decay_time_us") {
            decay_time_us_ = to_i(v);
            if (algo_) algo_->set_decay_time_us(decay_time_us_);
        } else if (k == "palette") {
            int p = to_i(v);
            if (p >= 0 && p <= 3) {
                palette_ = static_cast<gui_algo::TimeSurface::Palette>(p);
                if (algo_) algo_->set_palette(palette_);
            }
        } else if (k == "channels") {
            int c = to_i(v);
            channels_ = (c == 2) ? gui_algo::TimeSurface::Channels::Split
                                  : gui_algo::TimeSurface::Channels::Merged;
            if (algo_) algo_->set_channels(channels_);
        } else if (k == "roi_enabled") { roi_.enabled = to_b(v); need_rebuild = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); need_rebuild = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); need_rebuild = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); need_rebuild = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); need_rebuild = true; }
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "decay_time_us") return from_i(decay_time_us_);
        if (k == "palette") return from_i(static_cast<int>(palette_));
        if (k == "channels") return from_i(channels_ == gui_algo::TimeSurface::Channels::Split ? 2 : 1);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            roi_events_ = crop_to_roi(as_events(passthrough_.data()),
                                       passthrough_.size(), roi_);
            algo_->process(roi_events_.data(), roi_events_.size());
        } else {
            algo_->process(as_events(passthrough_.data()), passthrough_.size());
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_->render().clone();
        r.status = "time_surface: " + std::to_string(algo_->width()) + "x" +
                   std::to_string(algo_->height()) +
                   (roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        roi_events_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

/// EventToVideo backend — produces reconstructed intensity frame.
/// Complex algorithm (design §4.4.2): defaults to the center 128×128 ROI to
/// bound computational cost. The ROI, output fps, window and theta params
/// are exposed in the Algorithms panel and this backend.
class EventToVideoBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    // Current param values (re-applied after ROI rebuild so the fresh
    // EventToVideo instance inherits every setting, including E2VID model
    // path and post-processing params — otherwise ROI changes would silently
    // drop the E2VID configuration and fall back to the heuristic path).
    gui_algo::EventToVideo::Mode mode_{gui_algo::EventToVideo::Mode::E2VID};
    int output_fps_{30};
    // BardowVariational params.
    float window_ms_{50.0F};
    float delta_t_ms_{15.0F};
    float theta_{0.22F};
    int num_iterations_{100};
    float lambda1_{0.02F}, lambda2_{0.05F}, lambda3_{0.02F};
    float lambda4_{0.2F}, lambda5_{0.1F}, lambda6_{0.1F};
    float decay_tau_ms_{0.0F};
    // InteractingMaps params.
    float relaxation_step_{0.1F};
    int im_iterations_{50};
    float fov_deg_{60.0F};
    // E2VID params.
    std::string model_path_;
    int e2vid_num_bins_{5};
    bool e2vid_auto_hdr_{false};
    bool downsample_{true};  // 1/4 downsample (default on, all modes)
    float unsharp_amount_{0.3F};
    float unsharp_sigma_{1.0F};
    float bilateral_sigma_{0.0F};
    std::unique_ptr<gui_algo::EventToVideo> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
public:
    EventToVideoBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        algo_ = std::make_unique<gui_algo::EventToVideo>(aw, ah, mode_, output_fps_);
        // BardowVariational
        algo_->set_window_ms(window_ms_);
        algo_->set_delta_t_ms(delta_t_ms_);
        algo_->set_theta(theta_);
        algo_->set_num_iterations(num_iterations_);
        algo_->set_lambda1(lambda1_);
        algo_->set_lambda2(lambda2_);
        algo_->set_lambda3(lambda3_);
        algo_->set_lambda4(lambda4_);
        algo_->set_lambda5(lambda5_);
        algo_->set_lambda6(lambda6_);
        algo_->set_decay_tau_ms(decay_tau_ms_);
        // InteractingMaps
        algo_->set_relaxation_step(relaxation_step_);
        algo_->set_im_iterations(im_iterations_);
        algo_->set_fov_deg(fov_deg_);
        // E2VID (model reload is intentionally deferred to set_model_path so
        // an empty path keeps the heuristic fallback; non-empty path triggers
        // load_model which may fail silently and also fall back).
        if (!model_path_.empty()) algo_->set_model_path(model_path_);
        algo_->set_e2vid_num_bins(e2vid_num_bins_);
        algo_->set_e2vid_auto_hdr(e2vid_auto_hdr_);
        algo_->set_e2vid_downsample(downsample_);
        algo_->set_downsample(downsample_);
        algo_->set_unsharp_amount(unsharp_amount_);
        algo_->set_unsharp_sigma(unsharp_sigma_);
        algo_->set_bilateral_sigma(bilateral_sigma_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool need_rebuild = false;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 2) {
                mode_ = static_cast<gui_algo::EventToVideo::Mode>(m);
                if (algo_) algo_->set_mode(mode_);
            }
        } else if (k == "output_fps") {
            output_fps_ = to_i(v);
            if (algo_) algo_->set_output_fps(output_fps_);
        } else if (k == "window_ms") {
            window_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_window_ms(window_ms_);
        } else if (k == "delta_t_ms") {
            delta_t_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_delta_t_ms(delta_t_ms_);
        } else if (k == "theta") {
            theta_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_theta(theta_);
        } else if (k == "num_iterations") {
            num_iterations_ = to_i(v);
            if (algo_) algo_->set_num_iterations(num_iterations_);
        } else if (k == "lambda1") {
            lambda1_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda1(lambda1_);
        } else if (k == "lambda2") {
            lambda2_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda2(lambda2_);
        } else if (k == "lambda3") {
            lambda3_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda3(lambda3_);
        } else if (k == "lambda4") {
            lambda4_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda4(lambda4_);
        } else if (k == "lambda5") {
            lambda5_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda5(lambda5_);
        } else if (k == "lambda6") {
            lambda6_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda6(lambda6_);
        } else if (k == "decay_tau_ms") {
            decay_tau_ms_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_decay_tau_ms(decay_tau_ms_);
        } else if (k == "relaxation_step") {
            relaxation_step_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_relaxation_step(relaxation_step_);
        } else if (k == "im_iterations") {
            im_iterations_ = to_i(v);
            if (algo_) algo_->set_im_iterations(im_iterations_);
        } else if (k == "fov_deg") {
            fov_deg_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_fov_deg(fov_deg_);
        } else if (k == "model_path") {
            model_path_ = v;
            if (algo_) {
                algo_->set_model_path(model_path_);
                // num_bins is dictated by the loaded model (rpg_e2vid:
                // model.num_bins). Sync the persisted value so subsequent
                // ROI rebuilds keep the model's channel count.
                e2vid_num_bins_ = algo_->e2vid_num_bins();
            }
        } else if (k == "num_bins") {
            e2vid_num_bins_ = to_i(v);
            if (algo_) algo_->set_e2vid_num_bins(e2vid_num_bins_);
        } else if (k == "auto_hdr") {
            e2vid_auto_hdr_ = to_b(v);
            if (algo_) algo_->set_e2vid_auto_hdr(e2vid_auto_hdr_);
        } else if (k == "downsample") {
            downsample_ = to_b(v);
            if (algo_) {
                algo_->set_e2vid_downsample(downsample_);
                algo_->set_downsample(downsample_);
            }
        } else if (k == "unsharp_amount") {
            unsharp_amount_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_unsharp_amount(unsharp_amount_);
        } else if (k == "unsharp_sigma") {
            unsharp_sigma_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_unsharp_sigma(unsharp_sigma_);
        } else if (k == "bilateral_sigma") {
            bilateral_sigma_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_bilateral_sigma(bilateral_sigma_);
        } else if (k == "roi_enabled") { roi_.enabled = to_b(v); need_rebuild = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); need_rebuild = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); need_rebuild = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); need_rebuild = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); need_rebuild = true; }
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "mode") return from_i(static_cast<int>(mode_));
        if (k == "output_fps") return from_i(output_fps_);
        if (k == "window_ms") return from_d(window_ms_);
        if (k == "delta_t_ms") return from_d(delta_t_ms_);
        if (k == "theta") return from_d(theta_);
        if (k == "num_iterations") return from_i(num_iterations_);
        if (k == "lambda1") return from_d(lambda1_);
        if (k == "lambda2") return from_d(lambda2_);
        if (k == "lambda3") return from_d(lambda3_);
        if (k == "lambda4") return from_d(lambda4_);
        if (k == "lambda5") return from_d(lambda5_);
        if (k == "lambda6") return from_d(lambda6_);
        if (k == "decay_tau_ms") return from_d(decay_tau_ms_);
        if (k == "relaxation_step") return from_d(relaxation_step_);
        if (k == "im_iterations") return from_i(im_iterations_);
        if (k == "fov_deg") return from_d(fov_deg_);
        if (k == "model_path") return model_path_;
        if (k == "num_bins") return from_i(e2vid_num_bins_);
        if (k == "auto_hdr") return from_b(e2vid_auto_hdr_);
        if (k == "downsample") return from_b(downsample_);
        if (k == "unsharp_amount") return from_d(unsharp_amount_);
        if (k == "unsharp_sigma") return from_d(unsharp_sigma_);
        if (k == "bilateral_sigma") return from_d(bilateral_sigma_);
        if (k == "roi_enabled") return from_b(roi_.enabled);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            roi_events_ = crop_to_roi(as_events(passthrough_.data()),
                                       passthrough_.size(), roi_);
            algo_->process(roi_events_.data(), roi_events_.size());
        } else {
            algo_->process(as_events(passthrough_.data()), passthrough_.size());
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_->get_frame().clone();
        r.status = "e2v: " + std::to_string(algo_->width()) + "x" +
                   std::to_string(algo_->height()) +
                   (roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        roi_events_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w;
        sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
};

/// FlowStatistics backend — requires ground-truth flow samples (not available
/// in real-time). Counts events and reports a status; no frame is produced.
/// Supports ROI (design §5.6.6): when enabled, only ROI events are counted.
class FlowStatisticsBackend final : public AlgoBackend {
    gui_algo::FlowStatistics algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::size_t total_events_{0};
public:
    FlowStatisticsBackend(int w, int h)
        : algo_(gui_algo::FlowStatistics::Source::Synthetic, 5) {
        roi_.init(w, h);
    }
    void set_param(const std::string& k, const std::string& v) override {
        roi_.set_param(k, v);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        total_events_ += n;
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "flow_stats: " + std::to_string(total_events_) +
                   " events (no GT — Passive mode)" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); total_events_ = 0; }
};

/// ISIAnalyzer backend — renders ISI histogram as frame.
/// Complex algorithm (design §4.4.4): defaults to the center 128×128 ROI.
class ISIAnalyzerBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    bool per_pixel_{false};
    float max_isi_ms_{100.0F};
    std::unique_ptr<gui_algo::ISIAnalyzer> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
public:
    ISIAnalyzerBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        algo_ = std::make_unique<gui_algo::ISIAnalyzer>(aw, ah, 32, max_isi_ms_, per_pixel_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool need_rebuild = false;
        if (k == "per_pixel") {
            per_pixel_ = to_b(v);
            need_rebuild = true;
        } else if (k == "max_isi_ms") {
            max_isi_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_max_isi_ms(max_isi_ms_);
        } else if (k == "roi_enabled") { roi_.enabled = to_b(v); need_rebuild = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); need_rebuild = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); need_rebuild = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); need_rebuild = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); need_rebuild = true; }
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "per_pixel") return from_b(per_pixel_);
        if (k == "max_isi_ms") return from_d(max_isi_ms_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            roi_events_ = crop_to_roi(as_events(passthrough_.data()),
                                       passthrough_.size(), roi_);
            algo_->process(roi_events_.data(), roi_events_.size());
        } else {
            algo_->process(as_events(passthrough_.data()), passthrough_.size());
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_->render();
        r.status = "isi: histogram" + std::string(roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        roi_events_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

// ===========================================================================
// Group E: Analyzers (process events, produce text/point overlay)
// ===========================================================================

/// FreqDetector backend — detected light sources as overlay circles + text.
/// Complex algorithm (design §4.4.7): defaults to the center 128×128 ROI.
/// Events outside the ROI are dropped before reaching the analyzer; the algo
/// keeps sensor-sized internal buffers so output coordinates remain at sensor
/// scale (correct for overlay rendering).
class FreqDetectorBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    gui_algo::FreqDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    std::vector<gui_algo::LightSource> last_;
public:
    FreqDetectorBackend(int w, int h) : sensor_w_(w), sensor_h_(h), algo_(w, h) {
        roi_.compute(sensor_w_, sensor_h_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "update_interval_s") algo_.set_update_interval_s(static_cast<float>(to_d(v)));
        else if (k == "min_events") algo_.set_min_cc_area(to_i(v));
        else if (k == "roi_enabled") { roi_.enabled = to_b(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_x") { roi_.x = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_y") { roi_.y = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_w") { roi_.w = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_h") { roi_.h = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "update_interval_s") return from_d(algo_.update_interval_s());
        if (k == "min_events") return from_i(algo_.min_cc_area());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            // Filter events to ROI (no coordinate adjustment — sensor-scale
            // buffers + overlay coordinates are preserved).
            roi_events_.clear();
            const auto* ev = as_events(passthrough_.data());
            for (std::size_t i = 0; i < passthrough_.size(); ++i) {
                if (roi_.contains(ev[i].x, ev[i].y)) roi_events_.push_back(ev[i]);
            }
            algo_.process(roi_events_.data(), roi_events_.size());
        } else {
            algo_.process(as_events(passthrough_.data()), passthrough_.size());
        }
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
                   std::string(roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_events_.clear(); last_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_);
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
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_.clear(); }
};

/// ParticleCounter backend — count as overlay text.
/// Supports ROI (design §5.6.6): processes only ROI events. Throttled:
/// detect_and_track() only runs every 50ms to avoid CPU saturation.
class ParticleCounterBackend final : public AlgoBackend {
    gui_algo::ParticleCounter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    Metavision::timestamp last_t_{0};
    static constexpr Metavision::timestamp kMinDetectIntervalUs = 50000;
    Metavision::timestamp last_detect_t_{0};
public:
    ParticleCounterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "line_y") algo_.set_counting_line_y(to_i(v));
        else if (k == "min_area") algo_.set_min_particle_size_px(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
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
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "count: " + std::to_string(algo_.cumulative_count());
        r.texts.push_back(t);
        r.status = "counter: " + std::to_string(algo_.cumulative_count()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_t_ = 0; }
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
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_t_ = 0; }
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
    RoiFilter roi_;
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
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        std::vector<Metavision::EventCD> inp(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(inp.data()));
        std::size_t n = inp.size();
        // Compact in place to ROI events (design §5.6.6).
        if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
            }
            n = kept;
        }
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
    void reset() override { algo_.reset(); last_out_.clear(); }
};

/// UltraSlowMotion backend — outputs time-dilated event vector.
/// Supports ROI (design §5.6.6): only ROI events are time-dilated.
class UltraSlowMotionBackend final : public AlgoBackend {
    gui_algo::UltraSlowMotion algo_;
    std::vector<Metavision::EventCD> last_out_;
    RoiFilter roi_;
public:
    UltraSlowMotionBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "factor") algo_.set_dilation_factor(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "factor") return from_d(algo_.dilation_factor());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        std::vector<Metavision::EventCD> inp(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(inp.data()));
        std::size_t n = inp.size();
        // Compact in place to ROI events (design §5.6.6).
        if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
            }
            n = kept;
        }
        auto out = algo_.process(ev, n);
        last_out_.assign(out.begin(), out.end());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = last_out_;
        r.status = "slow_motion: " + std::to_string(last_out_.size()) + " events" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); last_out_.clear(); }
};

// ===========================================================================
// Group G: Visualization
// ===========================================================================

/// XYTVisualizer backend — 3D point cloud data for space_time_display.
/// Complex algorithm (design §4.3.25): defaults to the center 128×128 ROI to
/// bound the point-cloud size. Events outside the ROI are dropped.
///
/// NOTE: this backend does NOT produce overlay output. The 3D point cloud is
/// rendered by SpaceTimeDisplay, which owns its own XYTVisualizer instance and
/// receives events directly from the SDK callback (see install_algo_callback).
/// This backend exists only so the AlgoWindow can expose ROI/parameters and
/// reflect the buffer state in its status label.
class XYTVisualizerBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    gui_algo::XYTVisualizer algo_;
    int max_points_{50000};
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
public:
    XYTVisualizerBackend(int w, int h)
        : sensor_w_(w), sensor_h_(h),
          algo_(200.0f,
                gui_algo::XYTVisualizer::ColorMode::Age,
                2.5f,
                false,
                true) {
        roi_.compute(sensor_w_, sensor_h_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "time_window_us") algo_.set_time_window_ms(static_cast<float>(to_i(v)) / 1000.0F);
        else if (k == "max_points") max_points_ = to_i(v);
        else if (k == "roi_enabled") { roi_.enabled = to_b(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_x") { roi_.x = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_y") { roi_.y = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_w") { roi_.w = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
        else if (k == "roi_h") { roi_.h = to_i(v); roi_.compute(sensor_w_, sensor_h_); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "time_window_us") return from_i(static_cast<int>(algo_.time_window_ms() * 1000.0F));
        if (k == "max_points") return from_i(max_points_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            roi_events_.clear();
            const auto* ev = as_events(passthrough_.data());
            for (std::size_t i = 0; i < passthrough_.size(); ++i) {
                if (roi_.contains(ev[i].x, ev[i].y)) roi_events_.push_back(ev[i]);
            }
            algo_.process(roi_events_.data(), roi_events_.size());
        } else {
            algo_.process(as_events(passthrough_.data()), passthrough_.size());
        }
        // NOTE: do NOT call render() here — the 3D point cloud is rendered by
        // SpaceTimeDisplay which owns its own XYTVisualizer instance. This
        // backend only forwards events so the AlgoWindow status reflects the
        // ROI filter. Generating 50K OverlayPoints here would splatter 2D
        // points across the main display (wrong) and stall the GUI thread.
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // No overlay/frame: the 3D rendering is handled by SpaceTimeDisplay.
        r.status = "xyt: " + std::to_string(algo_.size()) + " buffered events" +
                   std::string(roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override { algo_.clear(); passthrough_.clear(); roi_events_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_);
    }
};

// ===========================================================================
// Group H: Misc filters
// ===========================================================================

/// OrientationFilter backend — orientation histogram as overlay text.
/// Supports ROI (design §5.6.6): only ROI events feed the histogram.
class OrientationFilterBackend final : public AlgoBackend {
    gui_algo::OrientationFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::vector<int> last_orientations_;  // per-event orientation labels
    // Decaying histogram (jAER uses a per-packet decay factor on the
    // orientation counters). A cumulative counter would grow without bound.
    float hist_[gui_algo::OrientationFilter::kNumOrientations]{};
    static constexpr float kHistDecay = 0.9F;  // per-packet decay
public:
    OrientationFilterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "tau_us") algo_.set_time_window_us(to_i(v));
        else if (k == "min_neighbors") algo_.set_min_neighbors(to_i(v));
        else if (k == "min_dt_threshold_us") algo_.set_min_dt_threshold_us(to_i(v));
        else if (k == "multi_ori_output") algo_.set_multi_ori_output(to_b(v));
        else if (k == "use_average_dt") algo_.set_use_average_dt(to_b(v));
        else if (k == "ori_history_enabled") algo_.set_ori_history_enabled(to_b(v));
        else if (k == "pass_all_events") algo_.set_pass_all_events(to_b(v));
        else if (k == "dt_reject_threshold_us") algo_.set_dt_reject_threshold_us(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "tau_us") return from_i(algo_.time_window_us());
        if (k == "min_neighbors") return from_i(algo_.min_neighbors());
        if (k == "min_dt_threshold_us") return from_i(algo_.min_dt_threshold_us());
        if (k == "multi_ori_output") return from_b(algo_.multi_ori_output());
        if (k == "use_average_dt") return from_b(algo_.use_average_dt());
        if (k == "ori_history_enabled") return from_b(algo_.ori_history_enabled());
        if (k == "pass_all_events") return from_b(algo_.pass_all_events());
        if (k == "dt_reject_threshold_us") return from_i(algo_.dt_reject_threshold_us());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        last_orientations_.resize(n);
        // Per-packet exponential decay (jAER oriCountsMap decay).
        for (int i = 0; i < gui_algo::OrientationFilter::kNumOrientations; ++i) {
            hist_[i] *= kHistDecay;
        }
        for (std::size_t i = 0; i < n; ++i) {
            last_orientations_[i] = algo_.classify(ev[i]);
            if (last_orientations_[i] >= 0 &&
                last_orientations_[i] < gui_algo::OrientationFilter::kNumOrientations) {
                hist_[last_orientations_[i]] += 1.0F;
            }
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Per-event colored events (jAER DvsOrientationEvent coloring).
        r.colored_events.reserve(last_orientations_.size());
        for (std::size_t i = 0; i < last_orientations_.size() && i < passthrough_.size(); ++i) {
            const int ori = last_orientations_[i];
            if (ori < 0) continue;
            const cv::Vec3b c = algo_.color(ori);
            ColoredEvent ce;
            ce.event = passthrough_[i];
            ce.r = c[2]; ce.g = c[1]; ce.b = c[0];  // BGR -> RGB
            r.colored_events.push_back(ce);
        }
        // Global orientation vector (from chip center, jAER computeGlobalOriVector).
        {
            float gx = 0, gy = 0;
            for (int i = 0; i < gui_algo::OrientationFilter::kNumOrientations; ++i) {
                const float angle = i * kPiF * 0.25F;
                gx += std::cos(angle) * hist_[i];
                gy += std::sin(angle) * hist_[i];
            }
            const int cx = algo_.width() / 2;
            const int cy = algo_.height() / 2;
            const float scale = 0.02F;
            OverlayLine gl;
            gl.x1 = cx - static_cast<int>(gx * scale);
            gl.y1 = cy - static_cast<int>(gy * scale);
            gl.x2 = cx + static_cast<int>(gx * scale);
            gl.y2 = cy + static_cast<int>(gy * scale);
            r.lines.push_back(gl);
        }
        // Histogram text + legend.
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "orient: 0=" + std::to_string(static_cast<int>(hist_[0])) +
                 " 45=" + std::to_string(static_cast<int>(hist_[1])) +
                 " 90=" + std::to_string(static_cast<int>(hist_[2])) +
                 " 135=" + std::to_string(static_cast<int>(hist_[3]));
        r.texts.push_back(t);
        r.status = "orient_filter: " + std::to_string(r.colored_events.size()) +
                   " colored events" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_orientations_.clear(); std::fill(hist_, hist_ + 4, 0.0F); }
};

/// DirectionSelectiveFilter backend — direction histogram as overlay text.
/// Supports ROI (design §5.6.6): only ROI events feed the histogram.
class DirectionSelectiveBackend final : public AlgoBackend {
    gui_algo::DirectionSelectiveFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::vector<int> last_dirs_;  // per-event direction labels
public:
    DirectionSelectiveBackend(int w, int h) : algo_(w, h) {
        roi_.init(w, h);
        algo_.set_enable_global_mode(true);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "tau_us") algo_.set_time_window_us(to_i(v));
        else if (k == "min_dt_us") algo_.set_min_dt_us(to_i(v));
        else if (k == "search_distance") algo_.set_search_distance(to_i(v));
        else if (k == "tau_low_ms") algo_.set_tau_low_ms(to_i(v));
        else if (k == "enable_global_mode") algo_.set_enable_global_mode(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "tau_us") return from_i(algo_.time_window_us());
        if (k == "min_dt_us") return from_i(algo_.min_dt_us());
        if (k == "search_distance") return from_i(algo_.search_distance());
        if (k == "tau_low_ms") return from_i(algo_.tau_low_ms());
        if (k == "enable_global_mode") return from_b(algo_.enable_global_mode());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        last_dirs_.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            last_dirs_[i] = algo_.classify(ev[i]);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Per-event colored events (jAER DirEvent coloring, 8 colors).
        // 8-direction palette: 0=E red, 1=NE orange, 2=N yellow, 3=NW green,
        // 4=W cyan, 5=SW blue, 6=S magenta, 7=SE white.
        static const std::uint8_t pal[8][3] = {
            {255, 0, 0}, {255, 165, 0}, {255, 255, 0}, {0, 255, 0},
            {0, 255, 255}, {0, 0, 255}, {255, 0, 255}, {255, 255, 255}};
        r.colored_events.reserve(last_dirs_.size());
        for (std::size_t i = 0; i < last_dirs_.size() && i < passthrough_.size(); ++i) {
            const int d = last_dirs_[i];
            if (d < 0 || d >= 8) continue;
            ColoredEvent ce;
            ce.event = passthrough_[i];
            ce.r = pal[d][0]; ce.g = pal[d][1]; ce.b = pal[d][2];
            r.colored_events.push_back(ce);
        }
        // Global motion vectors (jAER translation/rotation/expansion).
        // Translation arrow from chip center scaled for visibility.
        const int cx = algo_.width() / 2;
        const int cy = algo_.height() / 2;
        const auto tr = algo_.translation();
        const float tlen = std::hypot(tr.x, tr.y);
        if (tlen > 1.0F) {
            const float scale = 50.0F / std::max(1.0F, tlen);
            OverlayLine tl;
            tl.x1 = cx; tl.y1 = cy;
            tl.x2 = cx + static_cast<int>(tr.x * scale);
            tl.y2 = cy + static_cast<int>(tr.y * scale);
            r.lines.push_back(tl);
        }
        // Rotation indicator: an arc-like pair of segments at chip center.
        const float rot = algo_.rotation();
        if (std::fabs(rot) > 1e-3F) {
            const int R = 40;
            const float a0 = 0.0F;
            const float a1 = rot * 0.5F;
            OverlayLine rl;
            rl.x1 = cx + static_cast<int>(std::cos(a0) * R);
            rl.y1 = cy + static_cast<int>(std::sin(a0) * R);
            rl.x2 = cx + static_cast<int>(std::cos(a1) * R);
            rl.y2 = cy + static_cast<int>(std::sin(a1) * R);
            r.lines.push_back(rl);
        }
        // Direction histogram text.
        const auto& hist = algo_.global_histogram();
        OverlayText t;
        t.x = 10; t.y = 40;
        t.text = "dir: ";
        for (int i = 0; i < gui_algo::DirectionSelectiveFilter::kNumDirections; ++i) {
            t.text += std::to_string(i * 45) + "=" + std::to_string(hist[i]) + " ";
        }
        r.texts.push_back(t);
        // Motion text overlay.
        OverlayText mt;
        mt.x = 10; mt.y = 60;
        mt.text = "trans=(" + std::to_string(static_cast<int>(tr.x)) + "," +
                  std::to_string(static_cast<int>(tr.y)) + ") rot=" +
                  std::to_string(static_cast<int>(rot * 180.0F / static_cast<float>(kPiF))) +
                  " exp=" + std::to_string(algo_.expansion());
        r.texts.push_back(mt);
        const int dom = algo_.global_direction();
        r.status = "dir_selective: dominant=" + std::to_string(dom) +
                   " colored=" + std::to_string(r.colored_events.size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_dirs_.clear(); }
};

/// BackgroundMaskFilter backend — produces background mask frame.
/// Supports ROI (design §5.6.6): only ROI events update the mask; the mask
/// remains sensor-sized so only the ROI region is updated.
class BackgroundMaskBackend final : public AlgoBackend {
    gui_algo::BackgroundMaskFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    BackgroundMaskBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "learning_rate") algo_.set_learning_window_s(static_cast<float>(to_d(v)));
        else if (k == "threshold") algo_.set_background_rate_threshold_hz(static_cast<float>(to_d(v)));
        else if (k == "erosion_size") algo_.set_erosion_size(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "learning_rate") return from_d(algo_.learning_window_s());
        if (k == "threshold") return from_d(algo_.background_rate_threshold_hz());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_.mask().clone();
        r.status = "bg_mask: active" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); }
};

/// BandpassFilter backend — event-rate band-pass, text overlay.
/// Supports ROI (design §5.6.6): rate computed from ROI events only.
class BandpassFilterBackend final : public AlgoBackend {
    gui_algo::BandpassFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    Metavision::timestamp last_t_{0};
    double low_cutoff_hz_{1.0};
    double high_cutoff_hz_{10.0};
public:
    BandpassFilterBackend(int w, int h) : algo_(1.0F, 10.0F, 1, 0.033) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "low_cutoff_hz") {
            low_cutoff_hz_ = to_d(v);
            algo_.set_cutoffs(low_cutoff_hz_, high_cutoff_hz_);
        } else if (k == "high_cutoff_hz") {
            high_cutoff_hz_ = to_d(v);
            algo_.set_cutoffs(low_cutoff_hz_, high_cutoff_hz_);
        }
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "low_cutoff_hz") return from_d(low_cutoff_hz_);
        if (k == "high_cutoff_hz") return from_d(high_cutoff_hz_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        if (!passthrough_.empty()) {
            const auto t = passthrough_.back().t;
            algo_.add_events(n, t);
            last_t_ = t;
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        OverlayText t;
        t.x = 10; t.y = 60;
        t.text = "bp: " + std::to_string(algo_.value()) + " ev/s";
        r.texts.push_back(t);
        r.status = "bandpass: " + std::to_string(algo_.value()) + " ev/s" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_t_ = 0; }
};

/// Overlay backend — pass-through (overlay is drawn by frame_annotator from
/// other algos). Supports ROI (design §5.6.6): when enabled, only ROI events
/// pass through to filtered_events.
class OverlayBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
public:
    OverlayBackend(int w, int h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        roi_.set_param(k, v);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        // Compact in place to ROI events (design §5.6.6).
        if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
            auto* ev = const_cast<gui_algo::Event*>(as_events(passthrough_.data()));
            std::size_t n = passthrough_.size();
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
            }
            passthrough_.resize(kept);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "overlay: pass-through" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

// ===========================================================================
// OpenEB SDK — frame generators & filters (design §4.3.1 / §4.3.2)
//
// These backends wrap the real Metavision SDK algorithm classes. Events are
// stored in a std::vector<Metavision::EventCD> and the raw pointer is fed to
// the SDK process_events() (InputIt = Metavision::EventCD*).
// ===========================================================================

/// RoiMaskAlgorithm backend — propagates events inside a pixel mask.
/// The SDK operator() accesses the mask via cv::Mat::at<double>, so the mask
/// is kept in CV_64F to match.
class RoiMaskBackend final : public AlgoBackend {
    std::unique_ptr<Metavision::RoiMaskAlgorithm> algo_;
    std::vector<Metavision::EventCD> buf_;
    std::vector<Metavision::EventCD> out_;
    std::string mask_path_;
public:
    RoiMaskBackend(int w, int h) {
        cv::Mat mask = cv::Mat::ones(h, w, CV_64FC1); // all pixels pass
        algo_ = std::make_unique<Metavision::RoiMaskAlgorithm>(mask);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "mask_path") {
            mask_path_ = v;
            if (!v.empty()) {
                cv::Mat m = cv::imread(v, cv::IMREAD_GRAYSCALE);
                if (!m.empty()) {
                    m.convertTo(m, CV_64F);
                    algo_->set_pixel_mask(m);
                }
            }
        }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "mask_path") return mask_path_;
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        out_.clear();
        algo_->process_events(buf_.data(), buf_.data() + buf_.size(),
                              std::back_inserter(out_));
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = out_;
        r.status = "roi_mask: " + std::to_string(out_.size()) + "/" +
                   std::to_string(buf_.size()) +
                   (mask_path_.empty() ? "" : " (" + mask_path_ + ")");
        return r;
    }
    void reset() override { buf_.clear(); out_.clear(); }
};

/// AdaptiveRateEventsSplitterAlgorithm backend — splits the event stream into
/// sharp slices. retrieve_events() is called when a slice is ready.
class AdaptiveRateSplitBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    float thr_{5e-4F};
    int downsample_{2};
    std::unique_ptr<Metavision::AdaptiveRateEventsSplitterAlgorithm> algo_;
    std::vector<Metavision::EventCD> buf_;
    std::vector<Metavision::EventCD> out_;
    int slices_{0};
public:
    AdaptiveRateSplitBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::AdaptiveRateEventsSplitterAlgorithm>(
            h_, w_, thr_, downsample_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool dirty = false;
        if (k == "thr_var_per_event") { thr_ = static_cast<float>(to_d(v)); dirty = true; }
        else if (k == "downsampling_factor") { downsample_ = to_i(v); dirty = true; }
        if (dirty) rebuild();
    }
    std::string get_param(const std::string& k) const override {
        if (k == "thr_var_per_event") return from_d(thr_);
        if (k == "downsampling_factor") return from_i(downsample_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        if (algo_->process_events(buf_.data(), buf_.data() + buf_.size())) {
            out_.clear();
            algo_->retrieve_events(out_);
            ++slices_;
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = out_;
        r.status = "adaptive_rate_split: " + std::to_string(slices_) +
                   " slices, " + std::to_string(out_.size()) + " ev";
        return r;
    }
    void reset() override {
        if (algo_) algo_->retrieve_events(out_);
        buf_.clear(); out_.clear(); slices_ = 0;
    }
};

/// EventsIntegrationAlgorithm backend — integrates events into a grayscale
/// frame. decay_time_us requires reconstruction (no setter).
class FrameIntegrationBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    Metavision::timestamp decay_us_{1000000};
    std::unique_ptr<Metavision::EventsIntegrationAlgorithm> algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    FrameIntegrationBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::EventsIntegrationAlgorithm>(
            static_cast<unsigned>(w_), static_cast<unsigned>(h_), decay_us_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "decay_time_us") {
            auto nv = static_cast<Metavision::timestamp>(to_i(v));
            if (nv != decay_us_) { decay_us_ = nv; rebuild(); }
        }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "decay_time_us") return from_i(static_cast<int>(decay_us_));
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        algo_->generate(r.frame);
        r.status = "frame_integration: decay=" + std::to_string(decay_us_) + "us";
        return r;
    }
    void reset() override { if (algo_) algo_->reset(); passthrough_.clear(); }
};

/// EventFrameDiffGenerationAlgorithm backend — diff event frame (sum of
/// polarities per pixel). bit_size / allow_rollover require reconstruction.
class FrameDiffBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    unsigned bit_size_{8};
    bool rollover_{true};
    std::unique_ptr<Metavision::EventFrameDiffGenerationAlgorithm<Metavision::EventCD*>> algo_;
    Metavision::RawEventFrameDiff frame_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    FrameDiffBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::EventFrameDiffGenerationAlgorithm<Metavision::EventCD*>>(
            static_cast<unsigned>(w_), static_cast<unsigned>(h_), bit_size_, rollover_);
        frame_ = Metavision::RawEventFrameDiff(static_cast<unsigned>(h_),
                                               static_cast<unsigned>(w_), bit_size_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool dirty = false;
        if (k == "bit_size") {
            int nv = to_i(v); if (nv >= 2 && nv <= 8) { bit_size_ = static_cast<unsigned>(nv); dirty = true; }
        } else if (k == "allow_rollover") { rollover_ = to_b(v); dirty = true; }
        if (dirty) rebuild();
    }
    std::string get_param(const std::string& k) const override {
        if (k == "bit_size") return from_i(static_cast<int>(bit_size_));
        if (k == "allow_rollover") return from_b(rollover_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        algo_->generate(frame_);
        const auto& data = frame_.get_data();
        // int8 diff → CV_8U grayscale centred at 128 (0 = no activity)
        cv::Mat src(h_, w_, CV_8SC1,
                    const_cast<int8_t*>(data.data()));
        src.convertTo(r.frame, CV_8U, 1.0, 128.0);
        r.status = "frame_diff: " + std::to_string(passthrough_.size()) + " ev";
        return r;
    }
    void reset() override { if (algo_) algo_->reset(); passthrough_.clear(); }
};

/// EventFrameHistoGenerationAlgorithm backend — histo event frame (separate
/// neg/pos counts). channel_bit_neg/pos & packed require reconstruction.
class FrameHistoBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    unsigned bit_neg_{4}, bit_pos_{4};
    bool packed_{false};
    std::unique_ptr<Metavision::EventFrameHistoGenerationAlgorithm<Metavision::EventCD*>> algo_;
    Metavision::RawEventFrameHisto frame_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    FrameHistoBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::EventFrameHistoGenerationAlgorithm<Metavision::EventCD*>>(
            static_cast<unsigned>(w_), static_cast<unsigned>(h_), bit_neg_, bit_pos_, packed_);
        frame_ = Metavision::RawEventFrameHisto(static_cast<unsigned>(h_),
                                                static_cast<unsigned>(w_),
                                                bit_neg_, bit_pos_, packed_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool dirty = false;
        if (k == "channel_bit_neg") {
            int nv = to_i(v); if (nv >= 1 && nv <= 7) { bit_neg_ = static_cast<unsigned>(nv); dirty = true; }
        } else if (k == "channel_bit_pos") {
            int nv = to_i(v); if (nv >= 1 && nv <= 7) { bit_pos_ = static_cast<unsigned>(nv); dirty = true; }
        } else if (k == "packed") { packed_ = to_b(v); dirty = true; }
        if (dirty) rebuild();
    }
    std::string get_param(const std::string& k) const override {
        if (k == "channel_bit_neg") return from_i(static_cast<int>(bit_neg_));
        if (k == "channel_bit_pos") return from_i(static_cast<int>(bit_pos_));
        if (k == "packed") return from_b(packed_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        algo_->generate(frame_);
        const auto& data = frame_.get_data();
        const int channels = packed_ ? 1 : 2;
        // Build a BGR frame: negative events → blue, positive events → red.
        r.frame = cv::Mat::zeros(h_, w_, CV_8UC3);
        if (channels == 2) {
            const int plane = w_ * h_;
            for (int y = 0; y < h_; ++y) {
                for (int x = 0; x < w_; ++x) {
                    const int idx = y * w_ + x;
                    r.frame.at<cv::Vec3b>(y, x)[0] = data[idx];          // B = neg
                    r.frame.at<cv::Vec3b>(y, x)[2] = data[idx + plane];  // R = pos
                }
            }
        } else {
            cv::Mat src(h_, w_, CV_8UC1, const_cast<uint8_t*>(data.data()));
            cv::cvtColor(src, r.frame, cv::COLOR_GRAY2BGR);
        }
        r.status = "frame_histogram: " + std::to_string(passthrough_.size()) + " ev";
        return r;
    }
    void reset() override { if (algo_) algo_->reset(); passthrough_.clear(); }
};

/// TimeDecayFrameGenerationAlgorithm backend — time-decay visualization.
class FrameTimeDecayBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    std::unique_ptr<Metavision::TimeDecayFrameGenerationAlgorithm> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    Metavision::timestamp decay_us_{100000};
    int palette_idx_{1}; // Dark
public:
    FrameTimeDecayBackend(int w, int h) : w_(w), h_(h) {
        algo_ = std::make_unique<Metavision::TimeDecayFrameGenerationAlgorithm>(
            w_, h_, decay_us_, to_palette(palette_idx_));
    }
    static Metavision::ColorPalette to_palette(int i) {
        switch (i) {
            case 0: return Metavision::ColorPalette::Light;
            case 2: return Metavision::ColorPalette::CoolWarm;
            case 3: return Metavision::ColorPalette::Gray;
            case 1: default: return Metavision::ColorPalette::Dark;
        }
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "exponential_decay_time_us") {
            decay_us_ = static_cast<Metavision::timestamp>(to_i(v));
            algo_->set_exponential_decay_time_us(decay_us_);
        } else if (k == "palette") {
            palette_idx_ = to_i(v);
            algo_->set_color_palette(to_palette(palette_idx_));
        }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "exponential_decay_time_us") return from_i(static_cast<int>(decay_us_));
        if (k == "palette") return from_i(palette_idx_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        algo_->generate(r.frame);
        r.status = "frame_time_decay: decay=" + std::to_string(decay_us_) + "us";
        return r;
    }
    void reset() override { if (algo_) algo_->reset(); passthrough_.clear(); }
};

/// ContrastMapGenerationAlgorithm backend — contrast map (log intensity).
class FrameContrastMapBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    float contrast_on_{1.2F};
    float contrast_off_{-1.0F};
    std::unique_ptr<Metavision::ContrastMapGenerationAlgorithm> algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    FrameContrastMapBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::ContrastMapGenerationAlgorithm>(
            static_cast<unsigned>(w_), static_cast<unsigned>(h_), contrast_on_, contrast_off_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool dirty = false;
        if (k == "contrast_on") { contrast_on_ = static_cast<float>(to_d(v)); dirty = true; }
        else if (k == "contrast_off") { contrast_off_ = static_cast<float>(to_d(v)); dirty = true; }
        if (dirty) rebuild();
    }
    std::string get_param(const std::string& k) const override {
        if (k == "contrast_on") return from_d(contrast_on_);
        if (k == "contrast_off") return from_d(contrast_off_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        cv::Mat_<float> map;
        algo_->generate(map);
        // Normalize float contrast map to CV_8U for display.
        double mn, mx;
        cv::minMaxLoc(map, &mn, &mx);
        double range = (mx - mn > 1e-6) ? (mx - mn) : 1.0;
        map.convertTo(r.frame, CV_8U, 255.0 / range, -mn * 255.0 / range);
        cv::cvtColor(r.frame, r.frame, cv::COLOR_GRAY2BGR);
        r.status = "frame_contrast_map: " + std::to_string(passthrough_.size()) + " ev";
        return r;
    }
    void reset() override { if (algo_) algo_->reset(); passthrough_.clear(); }
};

/// PeriodicFrameGenerationAlgorithm backend — fixed-fps frame generation via
/// async callback. The callback stores the latest frame for pull_result().
class FramePeriodicBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    std::unique_ptr<Metavision::PeriodicFrameGenerationAlgorithm> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    cv::Mat last_frame_;
    uint32_t accum_us_{10000};
    double fps_{30.0};
public:
    FramePeriodicBackend(int w, int h) : w_(w), h_(h) {
        algo_ = std::make_unique<Metavision::PeriodicFrameGenerationAlgorithm>(
            w_, h_, accum_us_, fps_);
        algo_->set_output_callback(
            [this](Metavision::timestamp, cv::Mat& frame) { last_frame_ = frame.clone(); });
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "accumulation_time_us") {
            accum_us_ = static_cast<uint32_t>(to_i(v));
            algo_->set_accumulation_time_us(accum_us_);
        } else if (k == "fps") {
            fps_ = to_d(v);
            algo_->set_fps(fps_);
        }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "accumulation_time_us") return from_i(static_cast<int>(accum_us_));
        if (k == "fps") return from_d(fps_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        if (!last_frame_.empty()) {
            r.has_frame = true;
            r.frame = last_frame_.clone();
        }
        r.status = "frame_periodic: " + from_d(fps_) + " fps, accum=" +
                   std::to_string(accum_us_) + "us";
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear(); last_frame_.release();
    }
};

/// OnDemandFrameGenerationAlgorithm backend — generates a frame at the last
/// event timestamp on each pull_result().
class FrameOnDemandBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    uint32_t accum_us_{0};
    Metavision::timestamp last_ts_{0};
public:
    FrameOnDemandBackend(int w, int h) : w_(w), h_(h) {
        algo_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
            w_, h_, accum_us_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "accumulation_time_us") {
            accum_us_ = static_cast<uint32_t>(to_i(v));
            algo_->set_accumulation_time_us(accum_us_);
        }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "accumulation_time_us") return from_i(static_cast<int>(accum_us_));
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (!passthrough_.empty()) last_ts_ = passthrough_.back().t;
        algo_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        algo_->generate(last_ts_, r.frame);
        r.status = "frame_on_demand: accum=" + std::to_string(accum_us_) +
                   "us, ts=" + std::to_string(last_ts_);
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear(); last_ts_ = 0;
    }
};

// ===========================================================================
// OpenEB SDK — preprocessors (design §4.3.3)
//
// Preprocessors fill a Tensor (float). The tensor is accumulated across
// push_events() calls and reset after each pull_result() so each displayed
// frame shows activity since the last pull. The Tensor is converted to a
// cv::Mat for visualization.
// ===========================================================================

/// DiffProcessor backend — single-channel diff tensor → grayscale frame.
class PreprocDiffBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    float max_incr_{5.0F};
    float clip_{1.0F};
    std::unique_ptr<Metavision::DiffProcessor<Metavision::EventCD*>> algo_;
    Metavision::Tensor tensor_;
    Metavision::timestamp frame_start_ts_{-1};
    std::vector<Metavision::EventCD> passthrough_;
public:
    PreprocDiffBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::DiffProcessor<Metavision::EventCD*>>(
            w_, h_, max_incr_, clip_);
        tensor_.create(algo_->get_output_shape(), algo_->get_output_type());
        tensor_.set_to(0.f);
        frame_start_ts_ = -1;
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool dirty = false;
        if (k == "max_incr_per_pixel") { max_incr_ = static_cast<float>(to_d(v)); dirty = true; }
        else if (k == "clip_value_after_normalization") { clip_ = static_cast<float>(to_d(v)); dirty = true; }
        if (dirty) rebuild();
    }
    std::string get_param(const std::string& k) const override {
        if (k == "max_incr_per_pixel") return from_d(max_incr_);
        if (k == "clip_value_after_normalization") return from_d(clip_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (passthrough_.empty()) return;
        if (frame_start_ts_ < 0) frame_start_ts_ = passthrough_.front().t;
        algo_->process_events(frame_start_ts_, passthrough_.data(),
                              passthrough_.data() + passthrough_.size(), tensor_);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        const float* d = tensor_.data<float>();
        // Tensor layout CHW with C=1 → contiguous HxW float plane.
        cv::Mat src(h_, w_, CV_32F, const_cast<float*>(d));
        // Values in [-clip, clip] → map to [0, 255] centred at 128.
        double scale = (clip_ > 0) ? (127.0 / clip_) : 127.0;
        src.convertTo(r.frame, CV_8U, scale, 128.0);
        cv::cvtColor(r.frame, r.frame, cv::COLOR_GRAY2BGR);
        r.status = "preproc_diff: max_incr=" + from_d(max_incr_) + " clip=" + from_d(clip_);
        // Reset accumulation for the next frame.
        tensor_.set_to(0.f);
        frame_start_ts_ = -1;
        return r;
    }
    void reset() override { tensor_.set_to(0.f); frame_start_ts_ = -1; passthrough_.clear(); }
};

/// HistoProcessor backend — two-channel histogram tensor → BGR frame.
class PreprocHistoBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    float max_incr_{5.0F};
    float clip_{1.0F};
    bool use_chw_{true};
    std::unique_ptr<Metavision::HistoProcessor<Metavision::EventCD*>> algo_;
    Metavision::Tensor tensor_;
    Metavision::timestamp frame_start_ts_{-1};
    std::vector<Metavision::EventCD> passthrough_;
public:
    PreprocHistoBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::HistoProcessor<Metavision::EventCD*>>(
            w_, h_, max_incr_, clip_, use_chw_);
        tensor_.create(algo_->get_output_shape(), algo_->get_output_type());
        tensor_.set_to(0.f);
        frame_start_ts_ = -1;
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool dirty = false;
        if (k == "max_incr_per_pixel") { max_incr_ = static_cast<float>(to_d(v)); dirty = true; }
        else if (k == "clip_value_after_normalization") { clip_ = static_cast<float>(to_d(v)); dirty = true; }
        else if (k == "use_CHW") { use_chw_ = to_b(v); dirty = true; }
        if (dirty) rebuild();
    }
    std::string get_param(const std::string& k) const override {
        if (k == "max_incr_per_pixel") return from_d(max_incr_);
        if (k == "clip_value_after_normalization") return from_d(clip_);
        if (k == "use_CHW") return from_b(use_chw_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (passthrough_.empty()) return;
        if (frame_start_ts_ < 0) frame_start_ts_ = passthrough_.front().t;
        algo_->process_events(frame_start_ts_, passthrough_.data(),
                              passthrough_.data() + passthrough_.size(), tensor_);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        const float* d = tensor_.data<float>();
        const int plane = w_ * h_;
        r.frame = cv::Mat::zeros(h_, w_, CV_8UC3);
        double scale = (clip_ > 0) ? (255.0 / clip_) : 255.0;
        // Channel 0 = negative, channel 1 = positive (CHW: c-plane; HWC: interleaved).
        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) {
                const int idx = y * w_ + x;
                float neg, pos;
                if (use_chw_) { neg = d[idx]; pos = d[idx + plane]; }
                else { neg = d[idx * 2]; pos = d[idx * 2 + 1]; }
                r.frame.at<cv::Vec3b>(y, x)[0] = static_cast<uint8_t>(
                    std::min(255.0, std::max(0.0, neg * scale)));
                r.frame.at<cv::Vec3b>(y, x)[2] = static_cast<uint8_t>(
                    std::min(255.0, std::max(0.0, pos * scale)));
            }
        }
        r.status = "preproc_histo: CHW=" + from_b(use_chw_);
        tensor_.set_to(0.f);
        frame_start_ts_ = -1;
        return r;
    }
    void reset() override { tensor_.set_to(0.f); frame_start_ts_ = -1; passthrough_.clear(); }
};

/// TimeSurfaceProcessor backend — most-recent-timestamp buffer → time surface
/// image. The CHANNELS template parameter (1 or 2) is selected at runtime;
/// changing it reconstructs the processor.
class PreprocTimeSurfaceBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    int channels_{1};
    std::unique_ptr<Metavision::TimeSurfaceProcessor<Metavision::EventCD*, 1>> ts1_;
    std::unique_ptr<Metavision::TimeSurfaceProcessor<Metavision::EventCD*, 2>> ts2_;
    Metavision::MostRecentTimestampBuffer buf_;
    std::vector<Metavision::EventCD> passthrough_;
    Metavision::timestamp last_ts_{0};
public:
    PreprocTimeSurfaceBackend(int w, int h) : w_(w), h_(h) {
        buf_.create(h_, w_, 1);
        rebuild();
    }
    void rebuild() {
        ts1_.reset();
        ts2_.reset();
        buf_.create(h_, w_, channels_);
        buf_.set_to(0);
        if (channels_ == 1) ts1_ = std::make_unique<Metavision::TimeSurfaceProcessor<Metavision::EventCD*, 1>>(w_, h_);
        else ts2_ = std::make_unique<Metavision::TimeSurfaceProcessor<Metavision::EventCD*, 2>>(w_, h_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "channels") {
            int c = to_i(v);
            if (c == 1 || c == 2) { channels_ = c; rebuild(); }
        }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "channels") return from_i(channels_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (passthrough_.empty()) return;
        last_ts_ = passthrough_.back().t;
        if (channels_ == 1) ts1_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size(), buf_);
        else ts2_->process_events(passthrough_.data(), passthrough_.data() + passthrough_.size(), buf_);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        // Decay window: 100 ms by default for visualization.
        const Metavision::timestamp delta_t = 100000;
        if (channels_ == 2) buf_.generate_img_time_surface(last_ts_, delta_t, r.frame);
        else buf_.generate_img_time_surface_collapsing_channels(last_ts_, delta_t, r.frame);
        cv::cvtColor(r.frame, r.frame, cv::COLOR_GRAY2BGR);
        r.status = "preproc_time_surface: channels=" + from_i(channels_);
        return r;
    }
    void reset() override { buf_.set_to(0); passthrough_.clear(); last_ts_ = 0; }
};

/// EventCubeProcessor backend — event-cube tensor → grayscale projection.
class PreprocEventCubeBackend final : public AlgoBackend {
    int w_{0}, h_{0};
    int num_bins_{10};
    Metavision::timestamp delta_t_us_{33000};
    bool split_polarity_{false};
    float max_incr_{5.0F};
    std::unique_ptr<Metavision::EventCubeProcessor<Metavision::EventCD*>> algo_;
    Metavision::Tensor tensor_;
    Metavision::timestamp frame_start_ts_{-1};
    std::vector<Metavision::EventCD> passthrough_;
public:
    PreprocEventCubeBackend(int w, int h) : w_(w), h_(h) { rebuild(); }
    void rebuild() {
        algo_ = std::make_unique<Metavision::EventCubeProcessor<Metavision::EventCD*>>(
            delta_t_us_, w_, h_, num_bins_, split_polarity_, max_incr_);
        tensor_.create(algo_->get_output_shape(), algo_->get_output_type());
        tensor_.set_to(0.f);
        frame_start_ts_ = -1;
    }
    void set_param(const std::string& k, const std::string& v) override {
        bool dirty = false;
        if (k == "num_bins") { int n = to_i(v); if (n >= 2 && n <= 20) { num_bins_ = n; dirty = true; } }
        else if (k == "delta_t_us") { delta_t_us_ = static_cast<Metavision::timestamp>(to_i(v)); dirty = true; }
        else if (k == "split_polarity") { split_polarity_ = to_b(v); dirty = true; }
        else if (k == "max_incr_per_pixel") { max_incr_ = static_cast<float>(to_d(v)); dirty = true; }
        if (dirty) rebuild();
    }
    std::string get_param(const std::string& k) const override {
        if (k == "num_bins") return from_i(num_bins_);
        if (k == "delta_t_us") return from_i(static_cast<int>(delta_t_us_));
        if (k == "split_polarity") return from_b(split_polarity_);
        if (k == "max_incr_per_pixel") return from_d(max_incr_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (passthrough_.empty()) return;
        if (frame_start_ts_ < 0) frame_start_ts_ = passthrough_.front().t;
        algo_->process_events(frame_start_ts_, passthrough_.data(),
                              passthrough_.data() + passthrough_.size(), tensor_);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        const float* d = tensor_.data<float>();
        const int plane = w_ * h_;
        const int c = static_cast<int>(tensor_.shape().get_nb_values()) / plane;
        // Project the cube across channels into a single HxW plane.
        cv::Mat acc = cv::Mat::zeros(h_, w_, CV_32F);
        for (int ch = 0; ch < c; ++ch) {
            for (int y = 0; y < h_; ++y) {
                for (int x = 0; x < w_; ++x) {
                    acc.at<float>(y, x) += d[ch * plane + y * w_ + x];
                }
            }
        }
        double mn, mx;
        cv::minMaxLoc(acc, &mn, &mx);
        double range = (mx - mn > 1e-6) ? (mx - mn) : 1.0;
        acc.convertTo(r.frame, CV_8U, 255.0 / range, -mn * 255.0 / range);
        cv::cvtColor(r.frame, r.frame, cv::COLOR_GRAY2BGR);
        r.status = "preproc_event_cube: bins=" + from_i(num_bins_) +
                   " dt=" + std::to_string(delta_t_us_) + "us";
        tensor_.set_to(0.f);
        frame_start_ts_ = -1;
        return r;
    }
    void reset() override { tensor_.set_to(0.f); frame_start_ts_ = -1; passthrough_.clear(); }
};

/// EventPreprocessorFactory backend — stub. The factory requires a JSON config
/// parser; until a config_path is provided, this is a pass-through with an
/// informative status.
class PreprocFactoryBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
    std::string config_path_;
public:
    PreprocFactoryBackend(int, int) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "config_path") config_path_ = v;
    }
    std::string get_param(const std::string& k) const override {
        if (k == "config_path") return config_path_;
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "preproc_factory: stub — set config_path to load JSON" +
                   (config_path_.empty() ? "" : " (" + config_path_ + ")");
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

// ===========================================================================
// OpenEB SDK — utilities (design §4.3.4)
//
// Most utilities are passive containers / recorders with no per-event frame
// output. They pass events through and report an informative status. The
// RollingEventBuffer is implemented fully.
// ===========================================================================

/// FrameComposer backend — passive container (status only).
class UtilFrameComposerBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
public:
    UtilFrameComposerBackend(int, int) {}
    void set_param(const std::string&, const std::string&) override {}
    std::string get_param(const std::string&) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "frame_composer: ready — use API to add subimages";
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

/// RollingEventBuffer backend — keeps a rolling window of events (by count or
/// duration) and passes them through.
class UtilRollingBufferBackend final : public AlgoBackend {
    int mode_{0}; // 0=N_EVENTS, 1=N_US
    Metavision::timestamp delta_ts_{1000000};
    std::size_t delta_n_{5000};
    Metavision::RollingEventBuffer<Metavision::EventCD> buf_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    UtilRollingBufferBackend(int, int) {
        buf_ = Metavision::RollingEventBuffer<Metavision::EventCD>(
            Metavision::RollingEventBufferConfig::make_n_events(delta_n_));
    }
    void rebuild() {
        if (mode_ == 1)
            buf_ = Metavision::RollingEventBuffer<Metavision::EventCD>(
                Metavision::RollingEventBufferConfig::make_n_us(delta_ts_));
        else
            buf_ = Metavision::RollingEventBuffer<Metavision::EventCD>(
                Metavision::RollingEventBufferConfig::make_n_events(delta_n_));
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "mode") { mode_ = to_i(v); rebuild(); }
        else if (k == "delta_n_events") { delta_n_ = static_cast<std::size_t>(to_i(v)); rebuild(); }
        else if (k == "delta_ts_us") { delta_ts_ = static_cast<Metavision::timestamp>(to_i(v)); rebuild(); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "mode") return from_i(mode_);
        if (k == "delta_n_events") return from_i(static_cast<int>(delta_n_));
        if (k == "delta_ts_us") return from_i(static_cast<int>(delta_ts_));
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        buf_.insert_events(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "rolling_buffer: " + std::to_string(buf_.size()) + "/" +
                   std::to_string(buf_.capacity()) + " ev" +
                   (mode_ == 1 ? " (N_US)" : " (N_EVENTS)");
        return r;
    }
    void reset() override { buf_.clear(); passthrough_.clear(); }
};

/// DataSynchronizerFromTriggers backend — stub (complex API).
class UtilDataSynchronizerBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
    int period_us_{10000};
public:
    UtilDataSynchronizerBackend(int, int) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "period_us") period_us_ = to_i(v);
    }
    std::string get_param(const std::string& k) const override {
        if (k == "period_us") return from_i(period_us_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "data_synchronizer: ready — period_us=" + std::to_string(period_us_);
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

/// TimingProfiler backend — stub (singleton, no per-event output).
class UtilTimingProfilerBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
public:
    UtilTimingProfilerBackend(int, int) {}
    void set_param(const std::string&, const std::string&) override {}
    std::string get_param(const std::string&) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "timing_profiler: active";
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

/// Rate estimator backend — stub (see Statistics panel).
class UtilRateEstimatorBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
public:
    UtilRateEstimatorBackend(int, int) {}
    void set_param(const std::string&, const std::string&) override {}
    std::string get_param(const std::string&) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "rate_estimator: see Statistics panel";
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

/// Video writer backend — stub (use Export menu).
class UtilVideoWriterBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
public:
    UtilVideoWriterBackend(int, int) {}
    void set_param(const std::string&, const std::string&) override {}
    std::string get_param(const std::string&) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "video_writer: use Export menu";
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

// ===========================================================================
// Factory
// ===========================================================================

std::unique_ptr<AlgoBackend> create_algo_backend(const std::string& name,
                                                  int width, int height) {
    // algo/cv filters
    if (name == "noise_filter")          return std::make_unique<NoiseFilterBackend>(width, height);
    if (name == "hot_pixel_filter")      return std::make_unique<HotPixelFilterBackend>(width, height);
    if (name == "orientation_filter")    return std::make_unique<OrientationFilterBackend>(width, height);
    if (name == "direction_selective")   return std::make_unique<DirectionSelectiveBackend>(width, height);
    if (name == "sparse_optical_flow")   return std::make_unique<SparseOpticalFlowBackend>(width, height);
    if (name == "blob_detector")         return std::make_unique<BlobDetectorBackend>(width, height);
    if (name == "object_tracker")        return std::make_unique<ObjectTrackerBackend>(width, height);
    if (name == "corner_detector")       return std::make_unique<CornerDetectorBackend>(width, height);
    if (name == "line_segment")          return std::make_unique<LineSegmentBackend>(width, height);
    if (name == "hough_line")            return std::make_unique<HoughLineBackend>(width, height);
    if (name == "hough_circle")          return std::make_unique<HoughCircleBackend>(width, height);
    if (name == "orientation_cluster")   return std::make_unique<OrientationClusterBackend>(width, height);
    if (name == "cluster_lif")           return std::make_unique<ClusterLifBackend>(width, height);
    if (name == "background_mask")       return std::make_unique<BackgroundMaskBackend>(width, height);
    if (name == "perspective_undistort") return std::make_unique<PerspectiveUndistortBackend>(width, height);
    if (name == "trigger_synced")        return std::make_unique<TriggerSyncedBackend>(width, height);
    if (name == "bandpass_filter")       return std::make_unique<BandpassFilterBackend>(width, height);
    if (name == "optical_gyro")          return std::make_unique<OpticalGyroBackend>(width, height);
    if (name == "ultra_slow_motion")     return std::make_unique<UltraSlowMotionBackend>(width, height);
    if (name == "xyt_visualizer")        return std::make_unique<XYTVisualizerBackend>(width, height);
    if (name == "time_surface")          return std::make_unique<TimeSurfaceBackend>(width, height);
    if (name == "overlay")               return std::make_unique<OverlayBackend>(width, height);
    // algo/analytics
    if (name == "active_marker")         return std::make_unique<ActiveMarkerBackend>(width, height);
    if (name == "event_to_video")        return std::make_unique<EventToVideoBackend>(width, height);
    if (name == "flow_statistics")       return std::make_unique<FlowStatisticsBackend>(width, height);
    if (name == "isi_analyzer")          return std::make_unique<ISIAnalyzerBackend>(width, height);
    if (name == "particle_counter")      return std::make_unique<ParticleCounterBackend>(width, height);
    if (name == "auto_bias")             return std::make_unique<AutoBiasBackend>(width, height);
    if (name == "freq_detector")         return std::make_unique<FreqDetectorBackend>(width, height);
    // OpenEB SDK — filters
    if (name == "roi_mask")              return std::make_unique<RoiMaskBackend>(width, height);
    if (name == "adaptive_rate_split")   return std::make_unique<AdaptiveRateSplitBackend>(width, height);
    // OpenEB SDK — frame generators
    if (name == "frame_integration")     return std::make_unique<FrameIntegrationBackend>(width, height);
    if (name == "frame_diff")            return std::make_unique<FrameDiffBackend>(width, height);
    if (name == "frame_histogram")       return std::make_unique<FrameHistoBackend>(width, height);
    if (name == "frame_time_decay")      return std::make_unique<FrameTimeDecayBackend>(width, height);
    if (name == "frame_contrast_map")    return std::make_unique<FrameContrastMapBackend>(width, height);
    if (name == "frame_periodic")        return std::make_unique<FramePeriodicBackend>(width, height);
    if (name == "frame_on_demand")       return std::make_unique<FrameOnDemandBackend>(width, height);
    // OpenEB SDK — preprocessors
    if (name == "preproc_diff")          return std::make_unique<PreprocDiffBackend>(width, height);
    if (name == "preproc_histo")         return std::make_unique<PreprocHistoBackend>(width, height);
    if (name == "preproc_time_surface")  return std::make_unique<PreprocTimeSurfaceBackend>(width, height);
    if (name == "preproc_event_cube")    return std::make_unique<PreprocEventCubeBackend>(width, height);
    if (name == "preproc_factory")       return std::make_unique<PreprocFactoryBackend>(width, height);
    // OpenEB SDK — utilities
    if (name == "util_frame_composer")   return std::make_unique<UtilFrameComposerBackend>(width, height);
    if (name == "util_rolling_buffer")   return std::make_unique<UtilRollingBufferBackend>(width, height);
    if (name == "util_data_synchronizer") return std::make_unique<UtilDataSynchronizerBackend>(width, height);
    if (name == "util_timing_profiler")  return std::make_unique<UtilTimingProfilerBackend>(width, height);
    if (name == "util_rate_estimator")   return std::make_unique<UtilRateEstimatorBackend>(width, height);
    if (name == "util_video_writer")     return std::make_unique<UtilVideoWriterBackend>(width, height);
    return nullptr;
}

} // namespace gui
