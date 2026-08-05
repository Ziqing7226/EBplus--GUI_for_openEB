// algo/cv/background_mask_filter.h — Background mask learning (histogram + threshold + erosion).
//
// ✅ 移植自 jAER Histogram2DFilter
// (ref/jaer/src/ch/unizh/ini/jaer/projects/virtualslotcar/Histogram2DFilter.java)。
//
// 算法：在学习窗口 learning_window_s 内对每个像素累计事件直方图 histogram[x][y]++，
// 窗口结束后按阈值 threshold 分类 (count > threshold => 前景)，并对结果做形态学腐蚀
// (窗口 (2*erosion_size+1)²，边界像素复制/夹紧)，输出前景掩码 (255=前景, 0=背景)。
// 学习窗口结束后掩码冻结，直至 reset() 重新学习——对应 jAER 的 collect→freeze 语义。
//
// 与原 C++ 自研版本 (指数衰减活动量 activity = activity*exp(-dt/tau)+1, 稳态率阈值) 的
// 区别：改为 jAER 的直方图计数 + 阈值 + 形态学腐蚀；不再使用指数衰减或事件率。
//
// API 兼容：保留 BackgroundMaskFilter 类名与 process(EventPacket)->const cv::Mat&、
// mask()、reset()、set_learning_window_s/learning_window_s、
// set_background_rate_threshold_hz/background_rate_threshold_hz 等方法以不破坏后端与测试。
// 参数语义变化：background_rate_threshold_hz 现为 jAER 风格的原始计数阈值 (默认 20，
// 对应 jAER `threshold`)，不再是 Hz；learning_window_s 仍为学习窗口时长 (秒)。新增
// erosion_size (默认 0=不腐蚀，对应 jAER `erosionSize`)。Header-only.
//
// 与 jAER 的其余差异：collect_ 初值 true（启动即自动学习并在窗口结束后
// 冻结；jAER 默认 false，需手动触发学习）；setThreshold 仅在学习结束后
// 重算 bitmap（jAER 在 collect 期间也重算）；jAER invertFiltering 未移植。

#ifndef GUI_ALGO_CV_BACKGROUND_MASK_FILTER_H
#define GUI_ALGO_CV_BACKGROUND_MASK_FILTER_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Background mask learner producing a foreground mask.
///
/// Faithfully ported from jAER Histogram2DFilter: per-pixel histogram counting
/// + threshold classification + morphological erosion, driven by a
/// learning-window (collect → freeze) cycle.
class BackgroundMaskFilter {
public:
    BackgroundMaskFilter(int width, int height,
                         float learning_window_s = 5.0f,
                         float background_rate_threshold_hz = 20.0f,
                         int erosion_size = 0)
        : width_(width), height_(height),
          learning_window_s_(static_cast<double>(learning_window_s)),
          threshold_(background_rate_threshold_hz),
          erosion_size_(erosion_size),
          histogram_(static_cast<std::size_t>(width) * height, 0),
          mask_(cv::Mat::zeros(height, width, CV_8UC1)) {}

    /// @brief Accumulates the per-pixel histogram during the learning window,
    ///        then freezes and computes the eroded foreground mask.
    /// @return Current mask (255 = foreground, 0 = background). During the
    ///         collection phase the mask is all-zero (no decision yet, jAER
    ///         leaves the bitmap null while collecting); after the learning
    ///         window elapses it becomes the thresholded + eroded bitmap.
    const cv::Mat& process(const EventPacket& packet) {
        if (collect_ && !packet.empty()) {
            // Packet end time = max event timestamp (jAER uses last timestamp).
            Metavision::timestamp t_now = packet[0].t;
            if (learn_start_t_ < 0) learn_start_t_ = packet[0].t;

            // histogram[x][y]++  (jAER filterPacket collect branch).
            for (const Event& e : packet) {
                if (e.t > t_now) t_now = e.t;
                if (e.x >= width_ || e.y >= height_) continue; // ignore OOB / special
                const std::size_t idx =
                    static_cast<std::size_t>(e.y) * width_ + e.x;
                const int v = ++histogram_[idx];
                ++total_sum_;
                if (v > histmax_) histmax_ = v;
            }

            // Auto-freeze once the learning window has elapsed.
            const double win_us =
                learning_window_s_ * static_cast<double>(1.0e6);
            if (learn_start_t_ >= 0 &&
                static_cast<double>(t_now - learn_start_t_) >= win_us) {
                collect_ = false;
                compute_eroded_bitmap();
            }
        }
        return mask_;
    }

    /// @brief Returns the current foreground mask without updating the histogram.
    const cv::Mat& mask() const { return mask_; }

    // Parameter accessors ---------------------------------------------------

    /// @brief Learning window duration in seconds (accumulate-then-decide).
    float learning_window_s() const {
        return static_cast<float>(learning_window_s_);
    }
    /// @brief Threshold on accumulated event count (jAER `threshold`).
    /// @note Kept under the legacy name `background_rate_threshold_hz` for API
    ///       compatibility with the backend/tests; the value is now a raw count
    ///       (default 20, matching jAER), not a rate in Hz.
    float background_rate_threshold_hz() const { return threshold_; }

    void set_learning_window_s(float v) {
        learning_window_s_ = static_cast<double>(v);
    }
    void set_background_rate_threshold_hz(float v) {
        const float old = threshold_;
        threshold_ = v;
        // jAER recomputes the bitmap when the threshold changes.
        if (old != threshold_ && !collect_) compute_eroded_bitmap();
    }

    /// @brief Erosion radius in pixels (jAER `erosionSize`). 0 = no erosion,
    ///        1 = 3x3 window, 2 = 5x5, ... Window is (2*r+1)² with border
    ///        pixels replicated (clamped) as in jAER.
    int erosion_size() const { return erosion_size_; }
    void set_erosion_size(int v) {
        const int old = erosion_size_;
        erosion_size_ = v;
        if (old != erosion_size_ && !collect_) compute_eroded_bitmap();
    }

    /// @brief Total accumulated event count across all pixels (jAER totalSum).
    long total_sum() const { return total_sum_; }
    /// @brief Maximum single-pixel histogram count (jAER histmax).
    int histmax() const { return histmax_; }
    /// @brief True while still inside the learning (collect) phase.
    bool collect() const { return collect_; }

    /// @brief Clears the histogram and restarts the learning window.
    void reset() {
        std::fill(histogram_.begin(), histogram_.end(), 0);
        total_sum_ = 0;
        histmax_ = 0;
        collect_ = true;
        learn_start_t_ = -1;
        mask_.setTo(0);
    }

private:
    /// @brief Computes the thresholded + eroded bitmap into mask_.
    /// Faithful port of jAER Histogram2DFilter.computeErodedBitmap():
    ///   - erosion_size <= 0: bitmap[x][y] = (histogram[x][y] > threshold)
    ///   - else: keep (x,y) iff every neighbor in the (2*r+1)² window
    ///           (clamped to image bounds) has histogram >= threshold
    ///           (jAER rejects the pixel as soon as any neighbor is
    ///           `< threshold`).
    void compute_eroded_bitmap() {
        if (total_sum_ == 0) {
            mask_.setTo(0);
            return;
        }
        const float thr = threshold_;
        const int er = erosion_size_;
        const int W = width_;
        const int H = height_;

        if (er <= 0) {
            for (int y = 0; y < H; ++y) {
                auto* row = mask_.ptr<std::uint8_t>(y);
                const int* hrow = histogram_.data() +
                    static_cast<std::size_t>(y) * W;
                for (int x = 0; x < W; ++x) {
                    row[x] = (hrow[x] > thr) ? 255 : 0;
                }
            }
            return;
        }

        for (int y = 0; y < H; ++y) {
            auto* row = mask_.ptr<std::uint8_t>(y);
            for (int x = 0; x < W; ++x) {
                bool keep = true;
                for (int k = -er; k <= er && keep; ++k) {
                    const int pixY = clip(y + k, H - 1);
                    const int* hrow = histogram_.data() +
                        static_cast<std::size_t>(pixY) * W;
                    for (int l = -er; l <= er; ++l) {
                        const int pixX = clip(x + l, W - 1);
                        if (hrow[pixX] < thr) {
                            keep = false;
                            break;
                        }
                    }
                }
                row[x] = keep ? 255 : 0;
            }
        }
    }

    /// @brief Clamps val to [0, limit] (jAER `clip`, replicate border).
    static int clip(int val, int limit) {
        if (limit <= 0) return 0;
        if (val >= limit) return limit;
        if (val < 0) return 0;
        return val;
    }

    int width_;
    int height_;
    double learning_window_s_;
    float threshold_;            ///< jAER `threshold` (raw count); legacy getter name retained.
    int erosion_size_;           ///< jAER `erosionSize`.
    std::vector<int> histogram_; ///< histogram[y*width + x], mirrors jAER histogram[x][y].
    long total_sum_{0};          ///< jAER totalSum.
    int histmax_{0};             ///< jAER histmax.
    bool collect_{true};         ///< jAER collect flag (true during learning window).
    Metavision::timestamp learn_start_t_{-1}; ///< First event timestamp of current learning cycle.
    cv::Mat mask_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_BACKGROUND_MASK_FILTER_H
