// algo/cv/orientation_filter.h — 4-orientation edge labelling from time surfaces.
//
// Faithful port of jAER SimpleOrientationFilter / AbstractOrientationFilter
// (design §4.3.7). For each event the lastTimesMap is updated first, then for
// each of 4 orientations the delta-times along the receptive-field offsets are
// gathered (same polarity only). The average (or max) dt per orientation is
// computed with outlier rejection; a WTA selects the minimum-dt orientation.
// An orientation is emitted only if its dt is below minDtThresholdUs. An
// optional oriHistory map provides temporal smoothing. Output is an orientation
// index per event plus a colour for pseudo-colour rendering. Header-only.

#ifndef GUI_ALGO_CV_ORIENTATION_FILTER_H
#define GUI_ALGO_CV_ORIENTATION_FILTER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Quantises event edges into 4 orientations from a local time surface.
/// Faithful port of jAER SimpleOrientationFilter (min-dt WTA method).
class OrientationFilter {
public:
    enum class ColorMap { Fixed4, HSV };

    /// @brief Orientation indices returned by classify().
    /// 0 = 0 deg, 1 = 45 deg, 2 = 90 deg, 3 = 135 deg, -1 = undetermined.
    static constexpr int kNumOrientations = 4;
    /// jAER receptive-field half-length along the orientation (default length=3).
    static constexpr int kRfLength = 3;

    OrientationFilter(int width, int height)
        : width_(width), height_(height),
          surface_(static_cast<std::size_t>(2) * static_cast<std::size_t>(width)
                       * static_cast<std::size_t>(height), 0),
          ori_history_(static_cast<std::size_t>(width)
                           * static_cast<std::size_t>(height), 0) {}

    void set_color_map(ColorMap m) { color_map_ = m; }
    /// @brief jAER minDtThresholdUs: an orientation is emitted only if the
    /// per-orientation aggregate delta-time is below this (us). Default 100000.
    void set_min_dt_threshold_us(int v) { min_dt_threshold_us_ = clamp_i(v, 1, 1000000); }
    /// @brief jAER useAverageDtEnabled: true = average dt, false = max dt.
    void set_use_average_dt(bool v) { use_average_dt_ = v; }
    /// @brief jAER oriHistoryEnabled: temporal smoothing of orientation labels.
    void set_ori_history_enabled(bool v) { ori_history_enabled_ = v; }
    /// @brief jAER dtRejectThreshold: delta-times above this are rejected as
    /// outliers when computing per-orientation average/max dt.
    void set_dt_reject_threshold_us(int v) { dt_reject_threshold_us_ = clamp_i(v, 1, 10000000); }

    ColorMap color_map() const { return color_map_; }
    int min_dt_threshold_us() const { return min_dt_threshold_us_; }
    bool use_average_dt() const { return use_average_dt_; }
    bool ori_history_enabled() const { return ori_history_enabled_; }
    int dt_reject_threshold_us() const { return dt_reject_threshold_us_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Classifies a single event, updating the internal time surface.
    /// @return Orientation index in [0,3] or -1 if too few recent neighbours or
    /// the per-orientation coincidence gate fails.
    int classify(const Event& e) {
        if (e.x >= width_ || e.y >= height_) return -1;
        const int pol = e.p ? 1 : 0;

        // jAER L135: update lastTimesMap BEFORE computing orientation.
        surface_[idx_of(e.x, e.y, pol)] = e.t;

        // For each orientation, gather delta-times along the RF offsets.
        // jAER L143-157.
        std::array<Metavision::timestamp, kNumOrientations> oridts;
        std::array<Metavision::timestamp, kNumOrientations> ori_decide_helper;
        oridts.fill(std::numeric_limits<Metavision::timestamp>::max());
        ori_decide_helper.fill(std::numeric_limits<Metavision::timestamp>::max());

        for (int ori = 0; ori < kNumOrientations; ++ori) {
            const int bdx = kBaseDx[ori];
            const int bdy = kBaseDy[ori];
            std::array<Metavision::timestamp, kRfSize> dts{};
            int count = 0;
            for (int s = -kRfLength; s <= kRfLength; ++s) {
                if (s == 0) continue;
                const int idx = (s + kRfLength) - (s > 0 ? 1 : 0); // compact index
                const int nx = e.x + s * bdx;
                const int ny = e.y + s * bdy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
                    dts[idx < kRfSize ? idx : 0] = std::numeric_limits<Metavision::timestamp>::max();
                    continue;
                }
                const Metavision::timestamp lt = surface_[idx_of(nx, ny, pol)];
                if (lt == 0) {
                    dts[idx < kRfSize ? idx : 0] = std::numeric_limits<Metavision::timestamp>::max();
                    continue;
                }
                dts[idx < kRfSize ? idx : 0] = e.t - lt;
                ++count;
            }

            if (use_average_dt_) {
                // jAER L160-191: average dt with outlier rejection + variance.
                Metavision::timestamp sum = 0;
                int valid = 0;
                for (int i = 0; i < kRfSize; ++i) {
                    const Metavision::timestamp dt = dts[i];
                    if (dt < 0 || dt > dt_reject_threshold_us_) continue;
                    sum += dt;
                    ++valid;
                }
                if (valid > 0) {
                    oridts[ori] = sum / valid;
                    // Variance estimator (jAER oriDecideHelper).
                    double var = 0.0;
                    for (int i = 0; i < kRfSize; ++i) {
                        const Metavision::timestamp dt = dts[i];
                        if (dt < 0 || dt > dt_reject_threshold_us_) continue;
                        const double diff = static_cast<double>(dt) - static_cast<double>(oridts[ori]);
                        var += diff * diff;
                    }
                    var /= static_cast<double>(valid);
                    ori_decide_helper[ori] = static_cast<Metavision::timestamp>(var);
                }
            } else {
                // jAER L193-218: max dt with outlier rejection.
                Metavision::timestamp maxdt = std::numeric_limits<Metavision::timestamp>::min();
                Metavision::timestamp second_max = std::numeric_limits<Metavision::timestamp>::min();
                for (int i = 0; i < kRfSize; ++i) {
                    const Metavision::timestamp dt = dts[i];
                    if (dt < 0 || dt > dt_reject_threshold_us_) continue;
                    if (dt > maxdt) {
                        second_max = maxdt;
                        maxdt = dt;
                    } else if (dt > second_max) {
                        second_max = dt;
                    }
                }
                if (maxdt > std::numeric_limits<Metavision::timestamp>::min()) {
                    oridts[ori] = maxdt;
                    ori_decide_helper[ori] = second_max;
                }
            }
        }

        // jAER L220-254: WTA — find the orientation with the minimum dt.
        // Ties are broken by the decideHelper (variance or second-max).
        Metavision::timestamp mindt = static_cast<Metavision::timestamp>(min_dt_threshold_us_);
        Metavision::timestamp decide_helper = 0;
        int dir = -1;
        for (int ori = 0; ori < kNumOrientations; ++ori) {
            if (oridts[ori] < mindt) {
                mindt = oridts[ori];
                decide_helper = ori_decide_helper[ori];
                dir = ori;
            } else if (oridts[ori] == mindt) {
                if (ori_decide_helper[ori] <= decide_helper) {
                    mindt = oridts[ori];
                    dir = ori;
                }
            }
        }

        if (dir == -1) {
            // jAER L256-258: no good orientation found.
            return -1;
        }

        // jAER L260-275: oriHistory. 语义差异（§二-2.7）：jAER 的 oriHistory
        // 是**门控**（float map 初值 -1 避免水平偏置，|f-dir|>0.5 拒绝事件，
        // 输出原始 dir，oriHistoryMixingFactor 默认 0.1）；本实现把历史当
        // **平滑器**（IIR 混合后取整输出，默认因子 0.25）。
        if (ori_history_enabled_) {
            const std::size_t hidx = static_cast<std::size_t>(e.y) * width_ + e.x;
            const int hist = ori_history_[hidx];
            const float mix = ori_history_mixing_factor_;
            const float smoothed = hist * (1.0F - mix) + static_cast<float>(dir) * mix;
            int smoothed_ori = static_cast<int>(std::round(smoothed)) % kNumOrientations;
            if (smoothed_ori < 0) smoothed_ori += kNumOrientations;
            ori_history_[hidx] = smoothed_ori;
            return smoothed_ori;
        }

        return dir;
    }

    /// @brief Classifies a batch; fills @p out (resized to @p count).
    void process(const Event* events, std::size_t count, std::vector<int>& out) {
        out.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            out[i] = classify(events[i]);
        }
    }

    /// @brief Processes an event packet (updates the time surface).
    void process(EventPacket& events) {
        for (const auto& e : events) {
            (void)classify(e);
        }
    }

    /// @brief Renders an orientation index to an RGB colour.
    cv::Vec3b color(int orient) const {
        if (orient < 0 || orient >= kNumOrientations) return cv::Vec3b(0, 0, 0);
        if (color_map_ == ColorMap::HSV) {
            const int hue = orient * 45;               // 0,45,90,135 deg
            return hsv_to_bgr(hue, 255, 255);
        }
        // Fixed 4-colour palette.
        static const cv::Vec3b palette[kNumOrientations] = {
            cv::Vec3b(0, 0, 255),     // 0   deg -> red
            cv::Vec3b(0, 255, 0),     // 45  deg -> green
            cv::Vec3b(255, 0, 0),     // 90  deg -> blue
            cv::Vec3b(0, 255, 255),   // 135 deg -> yellow
        };
        return palette[orient];
    }

    void reset() {
        std::fill(surface_.begin(), surface_.end(), 0);
        std::fill(ori_history_.begin(), ori_history_.end(), 0);
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    /// @brief Two-channel (polarity) time-surface index. @p p must be 0 or 1.
    std::size_t idx_of(int x, int y, int p) const {
        return static_cast<std::size_t>(p) * static_cast<std::size_t>(width_)
                   * static_cast<std::size_t>(height_)
             + static_cast<std::size_t>(y) * width_ + x;
    }

    /// RF size = 2 * kRfLength (offsets at s = -3,-2,-1,1,2,3).
    static constexpr int kRfSize = 2 * kRfLength;

    /// @brief Per-orientation along-direction unit offsets (jAER baseOffsets).
    /// ori 0 = horizontal (1,0), 1 = 45 deg (1,1), 2 = vertical (0,1),
    /// 3 = 135 deg (-1,1).
    static constexpr int kBaseDx[kNumOrientations] = {1, 1, 0, -1};
    static constexpr int kBaseDy[kNumOrientations] = {0, 1, 1, 1};

    static cv::Vec3b hsv_to_bgr(int h, int s, int v) {
        // OpenCV uses BGR; convert 8-bit HSV.
        cv::Mat m(1, 1, CV_8UC3, cv::Vec3b(
            static_cast<std::uint8_t>(h / 2),
            static_cast<std::uint8_t>(s),
            static_cast<std::uint8_t>(v)));
        cv::Mat out;
        cv::cvtColor(m, out, cv::COLOR_HSV2BGR);
        return out.at<cv::Vec3b>(0, 0);
    }

    int width_;
    int height_;
    int min_dt_threshold_us_{100000};  // jAER minDtThresholdUs
    bool use_average_dt_{true};        // jAER useAverageDtEnabled (default true)
    bool ori_history_enabled_{false};  // jAER oriHistoryEnabled
    int dt_reject_threshold_us_{200000};  // jAER dtRejectThreshold 默认 500000（此处 200000 为有意收紧）
    float ori_history_mixing_factor_{0.25F};  // 平滑器语义，jAER 门控版默认 0.1（见 classify 注释）
    ColorMap color_map_{ColorMap::Fixed4};
    // Polarity-separated time surface: 2 * width * height, channel = polarity.
    std::vector<Metavision::timestamp> surface_;
    // Per-pixel orientation history for temporal smoothing.
    std::vector<int> ori_history_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_ORIENTATION_FILTER_H
