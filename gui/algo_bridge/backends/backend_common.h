// gui/algo_bridge/backends/backend_common.h — shared helpers for all backend
// .cpp files. Extracted from the former anonymous namespace in algo_backend.cpp
// (design §3.4). Everything lives in gui::backend_detail so it does not pollute
// the gui namespace; each backend .cpp uses `using namespace gui::backend_detail;`
// to reference these helpers unqualified (matching the original code).

#ifndef GUI_ALGO_BRIDGE_BACKENDS_BACKEND_COMMON_H
#define GUI_ALGO_BRIDGE_BACKENDS_BACKEND_COMMON_H

#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo/calibration/intrinsic.h"
#include "algo/common/event.h"
#include "algo/cv/noise_filter.h"

namespace gui {
namespace backend_detail {

// Pi as float (kPiF is not available in all OpenCV versions).
inline constexpr float kPiF = 3.14159265358979323846F;

// ---- string ↔ value helpers -----------------------------------------------

inline int to_i(const std::string& s, int def = 0) {
    try { return std::stoi(s); } catch (...) { return def; }
}
inline double to_d(const std::string& s, double def = 0.0) {
    try { return std::stod(s); } catch (...) { return def; }
}
inline bool to_b(const std::string& s) {
    return s == "1" || s == "true" || s == "True" || s == "on" || s == "yes";
}
inline std::string from_i(int v) { return std::to_string(v); }
inline std::string from_d(double v) { return std::to_string(v); }
inline std::string from_b(bool v) { return v ? "true" : "false"; }

/// Zero-copy view of EventCD buffer as gui_algo::Event (layout-compatible).
inline const gui_algo::Event* as_events(const Metavision::EventCD* p) {
    static_assert(sizeof(gui_algo::Event) == sizeof(Metavision::EventCD),
                  "layout mismatch");
    return reinterpret_cast<const gui_algo::Event*>(p);
}

/// @brief Processing region (ROI) for complex algorithms.
struct ProcessRegion {
    bool enabled{true};
    int x{-1};   ///< -1 = auto-center on sensor
    int y{-1};
    int w{128};  ///< 0 = full sensor width
    int h{128};  ///< 0 = full sensor height

    int x0{0}, y0{0}, x1{0}, y1{0};  ///< [x0,x1) × [y0,y1)
    int rw{0}, rh{0};

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

// Forward declaration — Preprocessor is defined below; crop_to_roi takes an
// optional Preprocessor*.
struct Preprocessor;

// ---- noise filter param helpers (shared by Preprocessor) -------------------

inline bool apply_noise_filter_param(gui_algo::NoiseFilter& nf,
                              const std::string& k, const std::string& v) {
    if (k == "mode") {
        int m = to_i(v);
        if (m >= 0 && m <= 7) nf.set_mode(static_cast<gui_algo::NoiseFilter::Mode>(m));
    }
    else if (k == "correlation_time_s") nf.set_correlation_time_s(to_d(v));
    else if (k == "min_neighbors") nf.set_min_neighbors(to_i(v));
    else if (k == "require_polarity_match") nf.set_require_polarity_match(to_b(v));
    else if (k == "allow_coincidence") nf.set_allow_coincidence(to_b(v));
    else if (k == "baf_dt_us") nf.set_baf_dt_us(to_i(v));
    else if (k == "baf_subsample_by") nf.set_baf_subsample_by(to_i(v));
    else if (k == "refractory_us") nf.set_refractory_period_us(to_i(v));
    else if (k == "dwf_window_length") nf.set_dwf_window_length(to_i(v));
    else if (k == "dwf_dist_threshold") nf.set_dwf_dist_threshold(to_i(v));
    else if (k == "dwf_min_correlated") nf.set_dwf_min_correlated(to_i(v));
    else if (k == "dwf_double_mode") nf.set_dwf_double_mode(to_b(v));
    else if (k == "agep_tau_us") nf.set_tau_us(to_i(v));
    else if (k == "age_threshold") nf.set_age_threshold(to_d(v));
    else if (k == "agep_radius") nf.set_agep_radius(to_i(v));
    else if (k == "line_freq_hz") nf.set_line_freq(to_i(v) == 60 ? gui_algo::NoiseFilter::LineFreq::Hz60 : gui_algo::NoiseFilter::LineFreq::Hz50);
    else if (k == "notch_q") nf.set_notch_q(to_d(v));
    else if (k == "harmonic_threshold") nf.set_harmonic_threshold(to_d(v));
    // rep_period_us / rep_tolerance_us are intentionally NOT forwarded: the
    // algo stores them but never uses them (audit §7.3); the registration
    // entries were removed.
    else if (k == "rep_ratio_shorter") nf.set_ratio_shorter(to_i(v));
    else if (k == "rep_ratio_longer") nf.set_ratio_longer(to_i(v));
    else if (k == "rep_min_dt_to_store_us") nf.set_min_dt_to_store_us(to_i(v));
    else if (k == "sbp_center_radius_px") nf.set_center_radius_px(to_i(v));
    else if (k == "sbp_surround_radius_px") nf.set_surround_radius_px(to_i(v));
    else if (k == "sbp_dt_surround_us") nf.set_dt_surround_us(to_i(v));
    else if (k == "filter_hot_pixels") nf.set_filter_hot_pixels(to_b(v));
    else if (k == "adaptive_correlation_time") nf.set_adaptive_correlation_time(to_b(v));
    else return false;
    return true;
}

inline std::string get_noise_filter_param(const gui_algo::NoiseFilter& nf,
                                   const std::string& k) {
    if (k == "mode") return from_i(static_cast<int>(nf.mode()));
    if (k == "correlation_time_s") return from_d(nf.correlation_time_s());
    if (k == "min_neighbors") return from_i(nf.min_neighbors());
    if (k == "require_polarity_match") return from_b(nf.require_polarity_match());
    if (k == "allow_coincidence") return from_b(nf.allow_coincidence());
    if (k == "baf_dt_us") return from_i(nf.baf_dt_us());
    if (k == "baf_subsample_by") return from_i(nf.baf_subsample_by());
    if (k == "refractory_us") return from_i(nf.refractory_period_us());
    if (k == "dwf_window_length") return from_i(nf.dwf_window_length());
    if (k == "dwf_dist_threshold") return from_i(nf.dwf_dist_threshold());
    if (k == "dwf_min_correlated") return from_i(nf.dwf_min_correlated());
    if (k == "dwf_double_mode") return from_b(nf.dwf_double_mode());
    if (k == "agep_tau_us") return from_i(nf.tau_us());
    if (k == "age_threshold") return from_d(nf.age_threshold());
    if (k == "agep_radius") return from_i(nf.agep_radius());
    if (k == "line_freq_hz") return from_i(nf.line_freq_hz());
    if (k == "notch_q") return from_d(nf.notch_q());
    if (k == "harmonic_threshold") return from_d(nf.harmonic_threshold());
    if (k == "rep_ratio_shorter") return from_i(nf.ratio_shorter());
    if (k == "rep_ratio_longer") return from_i(nf.ratio_longer());
    if (k == "rep_min_dt_to_store_us") return from_i(nf.min_dt_to_store_us());
    if (k == "sbp_center_radius_px") return from_i(nf.center_radius_px());
    if (k == "sbp_surround_radius_px") return from_i(nf.surround_radius_px());
    if (k == "sbp_dt_surround_us") return from_i(nf.dt_surround_us());
    if (k == "filter_hot_pixels") return from_b(nf.filter_hot_pixels());
    if (k == "adaptive_correlation_time") return from_b(nf.adaptive_correlation_time());
    return {};
}

/// @brief Stackable preprocessing chain: noise filter + 1/4 downsample + undistort.
struct Preprocessor {
    bool filter_enabled_{false};
    bool downsample_enabled_{false};
    bool halve_coords_{false};
    int filter_w_{0}, filter_h_{0};
    gui_algo::NoiseFilter::Mode filter_mode_{gui_algo::NoiseFilter::Mode::STCF};
    std::unique_ptr<gui_algo::NoiseFilter> filter_;
    std::unordered_map<std::string, std::string> filter_params_;
    std::vector<gui_algo::Event> buf_;

    // Undistort stage (applied AFTER filter + downsample). The LUT is built
    // at (filter_w_/factor, filter_h_/factor) with K adjusted for the ROI
    // origin and downsample factor, so it maps distorted event addresses to
    // undistorted addresses in the same coordinate system the algorithm sees.
    bool undistort_enabled_{false};
    std::string undistort_path_;
    cv::Mat undistort_K_;          ///< Loaded from YAML (sensor resolution, CV_64F 3x3)
    cv::Mat undistort_dist_;       ///< Loaded from YAML (CV_64F 1xN)
    cv::Size undistort_image_size_{0, 0};
    std::vector<cv::Point2f> undistort_lut_;  ///< Forward LUT, row-major [y*eff_w + x]
    int undistort_eff_w_{0}, undistort_eff_h_{0};
    int last_roi_x_{-1}, last_roi_y_{-1}, last_factor_{-1};
    bool undistort_lut_valid_{false};

    void init(int w, int h) {
        filter_w_ = w; filter_h_ = h;
        rebuild_filter();
        undistort_lut_valid_ = false;  // geometry changed → LUT needs rebuild
    }
    void rebuild_filter() {
        if (filter_enabled_ && filter_w_ > 0 && filter_h_ > 0) {
            filter_ = std::make_unique<gui_algo::NoiseFilter>(
                filter_w_, filter_h_, filter_mode_);
            for (const auto& kv : filter_params_) {
                apply_noise_filter_param(*filter_, kv.first, kv.second);
            }
        } else {
            filter_.reset();
        }
    }
    /// @brief Rebuilds the forward undistort LUT for the current ROI origin
    ///        and downsample factor. Defined in backend_common.cpp (uses
    ///        cv::undistortPoints from opencv2/calib3d.hpp).
    void rebuild_undistort_lut(int roi_x, int roi_y, int factor);
    bool set_param(const std::string& k, const std::string& v) {
        if (k == "preproc_filter_enabled") {
            filter_enabled_ = to_b(v);
            rebuild_filter();
            return true;
        }
        if (k == "preproc_downsample") {
            downsample_enabled_ = to_b(v);
            undistort_lut_valid_ = false;  // factor may have changed
            return true;
        }
        if (k == "preproc_undistort_enabled") {
            undistort_enabled_ = to_b(v);
            return true;
        }
        if (k == "preproc_undistort_path") {
            undistort_path_ = v;
            cv::Mat K, dist;
            cv::Size sz;
            if (gui_algo::load_intrinsics_yml(v, K, dist, sz)) {
                undistort_K_ = K;
                undistort_dist_ = dist;
                undistort_image_size_ = sz;
            } else {
                undistort_K_.release();
                undistort_dist_.release();
                undistort_image_size_ = cv::Size(0, 0);
            }
            undistort_lut_valid_ = false;
            return true;
        }
        static const std::string pfp = "preproc_filter_";
        if (k.size() > pfp.size() && k.compare(0, pfp.size(), pfp) == 0) {
            const std::string bare = k.substr(pfp.size());
            filter_params_[bare] = v;
            if (bare == "mode") {
                int m = to_i(v);
                if (m >= 0 && m <= 7) {
                    filter_mode_ = static_cast<gui_algo::NoiseFilter::Mode>(m);
                    rebuild_filter();
                }
            } else if (filter_) {
                apply_noise_filter_param(*filter_, bare, v);
            }
            return true;
        }
        return false;
    }
    std::string get_param(const std::string& k) const {
        if (k == "preproc_filter_enabled") return from_b(filter_enabled_);
        if (k == "preproc_downsample") return from_b(downsample_enabled_);
        if (k == "preproc_undistort_enabled") return from_b(undistort_enabled_);
        if (k == "preproc_undistort_path") return undistort_path_;
        static const std::string pfp = "preproc_filter_";
        if (k.size() > pfp.size() && k.compare(0, pfp.size(), pfp) == 0) {
            const std::string bare = k.substr(pfp.size());
            auto it = filter_params_.find(bare);
            if (it != filter_params_.end()) return it->second;
            if (filter_) return get_noise_filter_param(*filter_, bare);
        }
        return {};
    }
    int factor() const { return (downsample_enabled_ && halve_coords_) ? 2 : 1; }
    bool active() const {
        return filter_enabled_ || downsample_enabled_ ||
               (undistort_enabled_ && !undistort_K_.empty());
    }

    std::pair<const gui_algo::Event*, std::size_t> apply(
        const gui_algo::Event* src, std::size_t n,
        int roi_x = 0, int roi_y = 0) {
        if (!active() || n == 0) return {src, n};
        const gui_algo::Event* p = src;
        std::size_t m = n;
        if (filter_) {
            buf_.assign(src, src + n);
            m = filter_->filter(buf_.data(), n);
            p = buf_.data();
        }
        if (downsample_enabled_) {
            if (p != buf_.data()) {
                buf_.assign(p, p + m);
                p = buf_.data();
            }
            std::size_t kept = 0;
            if (halve_coords_) {
                for (std::size_t i = 0; i < m; ++i) {
                    if ((buf_[i].x & 1) == 0 && (buf_[i].y & 1) == 0) {
                        buf_[kept] = buf_[i];
                        buf_[kept].x = static_cast<std::uint16_t>(buf_[i].x >> 1);
                        buf_[kept].y = static_cast<std::uint16_t>(buf_[i].y >> 1);
                        ++kept;
                    }
                }
            } else {
                for (std::size_t i = 0; i < m; ++i) {
                    if ((buf_[i].x & 1) == 0 && (buf_[i].y & 1) == 0) {
                        buf_[kept] = buf_[i];
                        ++kept;
                    }
                }
            }
            m = kept;
        }
        // Undistort is the last stage (order: ROI → filter → downsample → undistort).
        // The LUT is indexed by the post-downsample coordinate system the
        // algorithm sees, so it must be rebuilt whenever the ROI origin or
        // downsample factor changes. Out-of-bounds results drop the event
        // (mirrors JAER SingleCameraCalibration.undistortEvent).
        if (undistort_enabled_ && !undistort_K_.empty() && !undistort_dist_.empty()) {
            const int f = factor();
            if (!undistort_lut_valid_ ||
                roi_x != last_roi_x_ || roi_y != last_roi_y_ || f != last_factor_) {
                rebuild_undistort_lut(roi_x, roi_y, f);
            }
            if (undistort_lut_valid_ && undistort_eff_w_ > 0 && undistort_eff_h_ > 0) {
                if (p != buf_.data()) {
                    buf_.assign(p, p + m);
                    p = buf_.data();
                }
                const int eff_w = undistort_eff_w_;
                const int eff_h = undistort_eff_h_;
                std::size_t kept = 0;
                for (std::size_t i = 0; i < m; ++i) {
                    const int x = buf_[i].x;
                    const int y = buf_[i].y;
                    if (x < 0 || y < 0 || x >= eff_w || y >= eff_h) continue;
                    const cv::Point2f& mapped = undistort_lut_[
                        static_cast<std::size_t>(y) * eff_w + x];
                    const int nx = static_cast<int>(std::lround(mapped.x));
                    const int ny = static_cast<int>(std::lround(mapped.y));
                    if (nx < 0 || ny < 0 || nx >= eff_w || ny >= eff_h) continue;
                    buf_[kept] = buf_[i];
                    buf_[kept].x = static_cast<std::uint16_t>(nx);
                    buf_[kept].y = static_cast<std::uint16_t>(ny);
                    ++kept;
                }
                m = kept;
            }
        }
        return {p, m};
    }
};

/// @brief Filters events to ROI and subtracts ROI origin (ROI-relative coords).
inline void crop_to_roi(const gui_algo::Event* src, std::size_t n,
                        const ProcessRegion& roi, Preprocessor* preproc,
                        std::vector<gui_algo::Event>& out) {
    out.clear();
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (roi.contains(src[i].x, src[i].y)) {
            gui_algo::Event e = src[i];
            e.x -= roi.x0;
            e.y -= roi.y0;
            out.push_back(e);
        }
    }
    if (preproc && preproc->active() && !out.empty()) {
        // Pass the ROI origin so the undistort LUT can be built with K
        // adjusted for the ROI offset (cx -= roi.x0, cy -= roi.y0).
        auto [p, m] = preproc->apply(out.data(), out.size(), roi.x0, roi.y0);
        out.assign(p, p + m);
    }
}

/// @brief ROI helper for backends that keep sensor-scale coordinates.
struct RoiFilter {
    int sensor_w{0};
    int sensor_h{0};
    ProcessRegion region;
    Preprocessor preproc;

    void init(int w, int h) {
        sensor_w = w;
        sensor_h = h;
        region.compute(w, h);
        preproc.init(w, h);
    }

    bool set_param(const std::string& k, const std::string& v) {
        if (preproc.set_param(k, v)) return true;
        if (k == "roi_enabled") { region.enabled = to_b(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_x") { region.x = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_y") { region.y = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_w") { region.w = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        if (k == "roi_h") { region.h = to_i(v); region.compute(sensor_w, sensor_h); return true; }
        return false;
    }

    std::string get_param(const std::string& k) const {
        auto pp = preproc.get_param(k);
        if (!pp.empty()) return pp;
        if (k == "roi_enabled") return from_b(region.enabled);
        if (k == "roi_x") return from_i(region.x);
        if (k == "roi_y") return from_i(region.y);
        if (k == "roi_w") return from_i(region.w);
        if (k == "roi_h") return from_i(region.h);
        return {};
    }

    std::pair<const gui_algo::Event*, std::size_t> apply(
        const gui_algo::Event* src, std::size_t n,
        std::vector<gui_algo::Event>& buf) {
        const gui_algo::Event* p = src;
        std::size_t m = n;
        if (region.enabled && region.rw > 0 && region.rh > 0) {
            buf.clear();
            buf.reserve(n);
            for (std::size_t i = 0; i < n; ++i) {
                if (region.contains(src[i].x, src[i].y)) {
                    buf.push_back(src[i]);
                }
            }
            p = buf.data();
            m = buf.size();
        }
        if (preproc.active()) {
            auto [pp, pm] = preproc.apply(p, m);
            p = pp; m = pm;
        }
        return {p, m};
    }
};

} // namespace backend_detail
} // namespace gui

#endif // GUI_ALGO_BRIDGE_BACKENDS_BACKEND_COMMON_H
