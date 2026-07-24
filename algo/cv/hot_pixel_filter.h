// algo/cv/hot_pixel_filter.h — hot pixel learning, lookup and FPN correction.
//
// Port of jAER HotPixelFilter (eu.seebetter.ini.chips.davis) and
// ProbFPNCorrectionFilter (net.sf.jaer.eventprocessing.filter), design §4.3.6.
// Learns which pixels fire at abnormally high rates by accumulating counts
// over a learning window then selecting the top-N hottest pixels (count >= 2
// floor), and maintains an HxW uint8 hot-pixel mask for O(1) lookup
// filtering. Optionally applies probabilistic FPN correction that throttles
// each pixel toward a target event rate via per-pixel ISI adaptation; when
// FPN correction is enabled it is applied to ALL events (not just hot-mask
// pixels). Header-only.
//
// 与 jAER 的差异（有意）：
//   * 热像素集合每个学习窗重算、不跨窗累积（jAER hotPixelSet 只增不清）。
//   * 学习窗默认 5s（jAER 为 500ms 且为手动触发学习）。
//   * 按像素滤除（两路极性全滤），等价 jAER use2DBooleanArray=true 变体；
//     jAER 默认按键控地址（含极性）粒度。
//   * reset() 连热像素掩码一起清除（jAER resetFilter 保留热像素集）。

#ifndef GUI_ALGO_CV_HOT_PIXEL_FILTER_H
#define GUI_ALGO_CV_HOT_PIXEL_FILTER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"

namespace gui_algo {

/// @brief Hot pixel learning + lookup filter with optional FPN correction.
class HotPixelFilter {
public:
    /// @brief Sentinel value marking an uninitialized timestamp slot.
    static constexpr Metavision::timestamp kSentinel =
        std::numeric_limits<Metavision::timestamp>::min();

    HotPixelFilter(int width, int height)
        : width_(width), height_(height),
          counts_(static_cast<std::size_t>(width) * height, 0),
          last_ts_(static_cast<std::size_t>(width) * height, kSentinel),
          hot_mask_(static_cast<std::size_t>(width) * height, 0),
          rng_(0x5EED1234ULL) {}

    // Parameter setters with range clamping -------------------------------
    void set_learning_window_s(double v) { learning_window_s_ = clamp(v, 1.0, 60.0); }
    void set_num_hot_pixels_max(int v) { num_hot_pixels_max_ = clamp_i(v, 1, 1000000); }
    void set_enable_fpn_correction(bool v) { enable_fpn_correction_ = v; }
    void set_fpn_target_rate_hz(double v) { fpn_target_rate_hz_ = clamp(v, 1.0, 1000.0); }

    double learning_window_s() const { return learning_window_s_; }
    int num_hot_pixels_max() const { return num_hot_pixels_max_; }
    bool enable_fpn_correction() const { return enable_fpn_correction_; }
    double fpn_target_rate_hz() const { return fpn_target_rate_hz_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Feeds events into the learner and refreshes the hot-pixel mask
    ///        when the learning window elapses.
    void learn(const Event* events, std::size_t count) {
        // Anchor the learning window to the first event seen (§四-低1 /
        // §一-1.4): with a 0-initialised anchor and large-timestamp sources
        // (live camera t≈1e9us) the first window closed immediately and
        // recomputed the mask from a handful of events.
        if (learn_start_s_ < 0.0 && count > 0) {
            learn_start_s_ = static_cast<double>(events[0].t) * 1e-6;
        }
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const std::size_t idx = idx_of(e.x, e.y);
            ++counts_[idx];
            if (e.t > last_learn_t_) last_learn_t_ = e.t;
            ++total_events_;
        }
        if (learn_start_s_ < 0.0) return;  // no events seen yet
        const double elapsed_s =
            static_cast<double>(last_learn_t_) * 1e-6 - learn_start_s_;
        if (elapsed_s >= learning_window_s_ && total_events_ > 0) {
            recompute_mask();
            learn_start_s_ = static_cast<double>(last_learn_t_) * 1e-6;
        }
    }

    /// @brief Returns the current hot-pixel mask (HxW, row-major, uint8).
    const std::vector<std::uint8_t>& hot_mask() const { return hot_mask_; }

    /// @brief Returns the number of hot pixels currently marked.
    std::size_t hot_pixel_count() const {
        std::size_t n = 0;
        for (auto v : hot_mask_) if (v) ++n;
        return n;
    }

    /// @brief Filters @p events in place by compacting the array.
    /// @return New (kept) event count.
    /// When FPN correction is disabled, hot-mask pixels are dropped. When FPN
    /// correction is enabled, the correction is applied to ALL events
    /// (regardless of hot_mask_ membership): each event is throttled toward
    /// fpn_target_rate_hz with transmission probability p = isi / target_isi
    /// (proportional to ISI, so hot pixels with short ISI are suppressed).
    std::size_t process(Event* events, std::size_t count) {
        std::size_t out = 0;
        const double target_isi_us = 1e6 / fpn_target_rate_hz_;
        for (std::size_t i = 0; i < count; ++i) {
            Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) {
                events[out++] = e;
                continue;
            }
            const std::size_t idx = idx_of(e.x, e.y);
            if (enable_fpn_correction_) {
                // M33: FPN correction applies to ALL events.
                const Metavision::timestamp prev = last_ts_[idx];
                last_ts_[idx] = e.t;
                if (prev == kSentinel) { events[out++] = e; continue; }
                const double isi_d = static_cast<double>(e.t - prev);
                // C1: transmission probability proportional to ISI
                // (re-designed; only loosely follows jAER's alpha*isi/avgIsi,
                // which uses an IIR-smoothed ISI and adaptive avgIsi).
                const double p = isi_d / target_isi_us;
                if (p >= 1.0 || u01_(rng_) < p) events[out++] = e;
            } else {
                // Hot-pixel filtering mode: drop hot-mask pixels.
                if (!hot_mask_[idx]) events[out++] = e;
            }
        }
        return out;
    }

    void reset() {
        std::fill(counts_.begin(), counts_.end(), 0);
        std::fill(last_ts_.begin(), last_ts_.end(), kSentinel);
        std::fill(hot_mask_.begin(), hot_mask_.end(), 0);
        total_events_ = 0;
        last_learn_t_ = 0;
        learn_start_s_ = -1.0;  // re-anchor on the next first event
    }

private:
    static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    std::size_t idx_of(int x, int y) const {
        return static_cast<std::size_t>(y) * width_ + x;
    }

    // M34: jAER selects the top num_hot_pixels_max pixels by count, with a
    // count >= 2 floor (counts of 1 are not considered hot).
    void recompute_mask() {
        std::fill(hot_mask_.begin(), hot_mask_.end(), 0);
        const std::size_t max_hot = static_cast<std::size_t>(num_hot_pixels_max_);
        for (std::size_t sel = 0; sel < max_hot; ++sel) {
            std::size_t best = counts_.size();
            std::uint32_t max_count = 1; // floor: require count >= 2
            for (std::size_t i = 0; i < counts_.size(); ++i) {
                if (counts_[i] > max_count) {
                    max_count = counts_[i];
                    best = i;
                }
            }
            if (best == counts_.size()) break; // no pixel with count >= 2
            hot_mask_[best] = 1;
            counts_[best] = 0; // clear so it is not reselected
        }
        std::fill(counts_.begin(), counts_.end(), 0);
        total_events_ = 0;
    }

    int width_;
    int height_;
    double learning_window_s_{5.0};
    int num_hot_pixels_max_{1000}; // jAER numHotPixelsMax default
    bool enable_fpn_correction_{false};
    double fpn_target_rate_hz_{50.0};
    std::vector<std::uint32_t> counts_;
    std::vector<Metavision::timestamp> last_ts_;
    std::vector<std::uint8_t> hot_mask_;
    std::uint64_t total_events_{0};
    Metavision::timestamp last_learn_t_{0};
    double learn_start_s_{-1.0};  // -1 = anchored on the first event seen
    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> u01_{0.0, 1.0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_HOT_PIXEL_FILTER_H
