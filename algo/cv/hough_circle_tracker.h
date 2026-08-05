// algo/cv/hough_circle_tracker.h — event-driven Hough circle tracker.
//
// ✅ 移植自 jAER HoughCircleTracker (net.sf.jaer.eventprocessing.tracking.
// HoughCircleTracker, by Jan Funke / Lorenz Muller). 本实现忠实复刻 jAER 的
// 算法结构：单固定半径 + 2D 累加器 (width×height) + 非指数衰减
// 1/(0.0001*decay*dt) + FIFO 事件历史 + 整数椭圆绘制 (8 扇区) + 位置抑制
// (locDepression) NMS，无持久航迹 ID。对应设计 §4.3.15。Header-only.
//
// 与 jAER 的少量差异（为兼容既有 backend/test API 所做，不影响算法语义）：
//   * jAER 的单一 float 半径 `radius` (默认 0.8px) 在此映射为构造参数
//     `max_radius_px`（整数像素），即本类使用的固定半径。
//   * threshold 默认 30 vs jAER 15（有意，GUI 噪声环境下减少误检）。
//   * 修正了 jAER locDepression 的笔误（jAER [x-1][y-1] 写了两次、
//     [x+1][y-1] 缺失）；本实现 8 邻域全部正确抑制。
//   * max_coord 未被任何 above-threshold 检测命中时不输出（jAER 会输出
//     (0,0)）；避免在 GUI 上画出多余的左上角圆。
//   * `buffer_length` 下限为 1（jAER 允许 0 但会触发除零）。
//   * HoughCircle.track_id 恒为 -1（jAER 无持久航迹）。

#ifndef GUI_ALGO_CV_HOUGH_CIRCLE_TRACKER_H
#define GUI_ALGO_CV_HOUGH_CIRCLE_TRACKER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Detected circle. Ported jAER has no persistent tracks, so
/// `track_id` is always -1; the field is retained for API compatibility.
struct HoughCircle {
    cv::Point2f center;
    float radius{0.0f};
    int track_id{-1};   ///< Always -1 (jAER has no persistent track IDs).
};

/// @brief Event-driven Hough circle tracker, ported from jAER.
///
/// Maintains a single fixed-radius 2D accumulator. Each event votes for all
/// circle centers on the integer ellipse of radius `max_radius_px` around it
/// (8-sector integer ellipse drawing). The accumulator decays each packet by
/// the non-exponential factor 1/(0.0001*decay*dt). A FIFO event history is
/// kept; when `decay_mode` is off, the least recent event's votes are
/// subtracted. Local maxima above `threshold` are reported, and their
/// neighborhoods are suppressed via `loc_depression`.
class HoughCircleTracker {
public:
    /// @brief Constructor.
    /// @param width, height      Accumulator dimensions (pixels).
    /// @param max_radius_px      The single fixed circle radius (jAER `radius`).
    /// @param threshold          Detection threshold (jAER `threshold`; default
    ///                           30 here vs jAER 15, intentional).
    /// @param decay              jAER decay coefficient (default 1.0).
    /// @param buffer_length      FIFO event history length (default 4000).
    /// @param nr_max             Number of maxima to track (default 1).
    /// @param decay_mode         If true, apply time-based decay (default true).
    /// @param loc_depression     If true, suppress detected neighborhoods.
    HoughCircleTracker(int width, int height,
                       int max_radius_px = 50,
                       int threshold = 30,
                       float decay = 1.0f,
                       int buffer_length = 4000,
                       int nr_max = 1,
                       bool decay_mode = true,
                       bool loc_depression = true)
        : width_(width), height_(height),
          max_radius_px_(max_radius_px),
          threshold_(threshold),
          decay_(decay),
          buffer_length_(buffer_length),
          nr_max_(nr_max),
          decay_mode_(decay_mode),
          loc_depression_(loc_depression) {
        if (max_radius_px_ < 0) max_radius_px_ = 0;
        if (buffer_length_ < 1) buffer_length_ = 1;   // jAER allows 0 (div-by-zero)
        if (nr_max_ < 0) nr_max_ = 0;
        if (decay_ < 0.0f) decay_ = 0.0f;
        rebuild();
    }

    /// 衰减 + 事件累积，不做峰值扫描（供节流路径每包调用）。
    ///
    /// @param cur_t 显式时间戳供衰减计算使用。审计 §11.2-H：当 ROI/预处理
    /// 滤掉了包尾事件、或整个包被滤空时，algo 的 last_t_ 会停滞，下一包
    /// 的 dt 被夸大、衰减失真。调用方（HoughCircleBackend）应传
    /// passthrough_.back().t 以保证 last_t_ 单调推进。默认 -1 = 用
    /// packet.back().t（向后兼容 process() 与单元测试）。
    void accumulate_only(const EventPacket& packet,
                         Metavision::timestamp cur_t = -1) {
        // Resolve cur_t: if not provided, use packet.back().t. If neither is
        // available (empty packet, no explicit cur_t), bail — nothing to do.
        if (cur_t < 0) {
            if (packet.empty()) return;
            cur_t = packet[packet.size() - 1].t;
        }

        // jAER non-exponential decay: factor = 1/(0.0001*decay*dt).
        // Apply even when the packet is empty (cur_t provided) so last_t_
        // stays monotonic across ROI-filtered empty packets (§11.2-H).
        if (width_ > 0 && height_ > 0 && decay_mode_ && decay_ > 0.0f) {
            const float dt = static_cast<float>(cur_t - last_t_);
            if (dt > 0.0f) {
                const float decay_factor = 1.0f / (0.0001f * decay_ * dt);
                for (float& v : accum_) v *= decay_factor;
            }
        }
        last_t_ = cur_t;

        // Empty packet (with explicit cur_t): timestamp advanced, nothing
        // else to do.
        if (packet.empty()) return;
        if (width_ <= 0 || height_ <= 0) return;

        // Reset running maxima for this packet (jAER resets maxValue, keeps
        // maxCoordinate so the last known position persists across packets).
        std::fill(max_value_.begin(), max_value_.end(), 0.0f);

        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            const float weight = 1.0f;  // C++ Event has no WeightedEvent field

            // save event in history
            event_history_[static_cast<std::size_t>(buffer_index_)] =
                Coord{static_cast<int>(e.x), static_cast<int>(e.y)};

            // accumulate all possible circle centers for the current event
            accumulate(static_cast<int>(e.x), static_cast<int>(e.y), weight);

            // advance buffer index
            buffer_index_ = (buffer_index_ + 1) % buffer_length_;

            // remove the least recent event from hough space (only when decay
            // is off; otherwise decay handles expiration).
            if (!decay_mode_) {
                const Coord& old =
                    event_history_[static_cast<std::size_t>(buffer_index_)];
                if (old.x >= 0) accumulate(old.x, old.y, -1.0f);
            }
        }
    }

    /// 全量扫描累加器局部极大，返回检测到的圆（供节流路径按节奏调用）。
    std::vector<HoughCircle> find_peaks() {
        std::vector<HoughCircle> result;
        if (width_ <= 0 || height_ <= 0) return result;

        // Re-scan the whole accumulator for local maxima above threshold
        // (overwrites the running maxima found during accumulation).
        std::fill(max_value_.begin(), max_value_.end(), 0.0f);
        for (int x = 0; x < width_; ++x) {
            for (int y = 0; y < height_; ++y) {
                const float v = accum_[idx(x, y)];
                for (int k = 0; k < nr_max_; ++k) {
                    if (v >= max_value_[k] && is_loc_max(x, y)) {
                        max_value_[k] = v;
                        if (v > static_cast<float>(threshold_)) {
                            max_coord_[k].x = x;
                            max_coord_[k].y = y;
                        }
                        k += nr_max_;  // jAER: i += nrMax (break inner loop)
                    }
                }
            }
        }

        // Output + position suppression (locDepression).
        for (int k = 0; k < nr_max_; ++k) {
            const int x = max_coord_[k].x;
            const int y = max_coord_[k].y;
            // Skip never-detected slots (jAER emits (0,0); we omit to avoid a
            // spurious top-left circle in the GUI overlay).
            if (x < 0 || y < 0) continue;

            if (loc_depression_ &&
                (x - 1) > 0 && (x + 1) < (width_ - 1) &&
                (y - 1) > 0 && (y + 1) < (height_ - 1)) {
                accum_[idx(x, y)]     *= 0.01f;
                accum_[idx(x, y + 1)] *= 0.1f;
                accum_[idx(x, y - 1)] *= 0.1f;
                accum_[idx(x + 1, y)] *= 0.1f;
                accum_[idx(x - 1, y)] *= 0.1f;
                accum_[idx(x + 1, y + 1)] *= 0.1f;
                accum_[idx(x - 1, y + 1)] *= 0.1f;
                accum_[idx(x + 1, y - 1)] *= 0.1f;
                accum_[idx(x - 1, y - 1)] *= 0.1f;
            }

            HoughCircle hc;
            hc.center = cv::Point2f(static_cast<float>(x),
                                    static_cast<float>(y));
            hc.radius = static_cast<float>(max_radius_px_);
            hc.track_id = -1;  // jAER has no persistent tracks
            result.push_back(hc);
        }
        return result;
    }

    /// process() 保持原签名 = accumulate_only(packet) 后调用 find_peaks()（兼容现有调用方）。
    std::vector<HoughCircle> process(const EventPacket& packet) {
        accumulate_only(packet);
        return find_peaks();
    }

    // Parameter accessors ---------------------------------------------------
    int max_radius_px() const { return max_radius_px_; }
    int threshold() const { return threshold_; }
    float decay() const { return decay_; }
    int buffer_length() const { return buffer_length_; }
    int nr_max() const { return nr_max_; }
    bool decay_mode() const { return decay_mode_; }
    bool loc_depression() const { return loc_depression_; }
    /// @brief Read-only access to the per-pixel Hough accumulator
    /// (x major, y minor; size = width_*height_). Used by the GUI backend
    /// to render the Hough space as an aux frame.
    const std::vector<float>& accum() const { return accum_; }

    void set_max_radius_px(int v) {
        if (v < 0) v = 0;
        if (v == max_radius_px_) return;
        max_radius_px_ = v;  // this is the active radius (jAER `radius`)
        rebuild();
    }
    void set_threshold(int v) { threshold_ = v; }
    void set_decay(float v) {
        if (v < 0.0f) v = 0.0f;
        decay_ = v;
    }
    void set_buffer_length(int v) {
        if (v < 1) v = 1;
        if (v == buffer_length_) return;
        buffer_length_ = v;
        rebuild();
    }
    void set_nr_max(int v) {
        if (v < 0) v = 0;
        if (v == nr_max_) return;
        nr_max_ = v;
        rebuild();
    }
    void set_decay_mode(bool v) {
        if (v != decay_mode_) {
            decay_mode_ = v;
            rebuild();  // jAER resetFilter on change
        }
    }
    void set_loc_depression(bool v) {
        if (v != loc_depression_) {
            loc_depression_ = v;
            rebuild();  // jAER resetFilter on change
        }
    }

    void reset() {
        std::fill(accum_.begin(), accum_.end(), 0.0f);
        std::fill(event_history_.begin(), event_history_.end(), Coord{-1, -1});
        buffer_index_ = 0;
        std::fill(max_value_.begin(), max_value_.end(), 0.0f);
        std::fill(max_coord_.begin(), max_coord_.end(), Coord{-1, -1});
        last_t_ = 0;
    }

private:
    struct Coord { int x{-1}; int y{-1}; };

    void rebuild() {
        accum_.assign(static_cast<std::size_t>(width_) *
                          static_cast<std::size_t>(height_),
                      0.0f);
        event_history_.assign(static_cast<std::size_t>(buffer_length_),
                              Coord{-1, -1});
        buffer_index_ = 0;
        max_value_.assign(static_cast<std::size_t>(nr_max_), 0.0f);
        max_coord_.assign(static_cast<std::size_t>(nr_max_), Coord{-1, -1});
        last_t_ = 0;
    }

    inline std::size_t idx(int x, int y) const {
        return static_cast<std::size_t>(x) * static_cast<std::size_t>(height_) +
               static_cast<std::size_t>(y);
    }

    /// @brief Increase a hough point and update the running maxima (jAER
    /// increaseHoughPoint).
    void increase_hough_point(int x, int y, float weight) {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
        const std::size_t i = idx(x, y);
        accum_[i] += weight;
        for (int k = 0; k < nr_max_; ++k) {
            if (accum_[i] >= max_value_[k]) {
                max_value_[k] = accum_[i];
                if (max_value_[k] > static_cast<float>(threshold_)) {
                    max_coord_[k].x = x;
                    max_coord_[k].y = y;
                }
                k += nr_max_;  // jAER: i += nrMax (break)
            }
        }
    }

    /// @brief 8-neighbor local maximum check (jAER islocmax, locMaxRad = 1).
    bool is_loc_max(int x, int y) const {
        const int loc_max_rad = 1;
        if ((x - loc_max_rad) < 0 || (x + loc_max_rad) > (width_ - 1) ||
            (y - loc_max_rad) < 0 || (y + loc_max_rad) > (height_ - 1)) {
            return false;
        }
        const float v = accum_[idx(x, y)];
        if (v < accum_[idx(x, y + 1)]) return false;
        if (v < accum_[idx(x, y - 1)]) return false;
        if (v < accum_[idx(x + 1, y)]) return false;
        if (v < accum_[idx(x - 1, y)]) return false;
        if (v < accum_[idx(x + 1, y - 1)]) return false;
        if (v < accum_[idx(x - 1, y + 1)]) return false;
        if (v < accum_[idx(x + 1, y + 1)]) return false;
        if (v < accum_[idx(x - 1, y - 1)]) return false;
        return true;
    }

    /// @brief Fast integer ellipse drawing (jAER accumulate). Draws the
    /// circle of radius `max_radius_px_` centered at (centerX, centerY) into
    /// the accumulator, adding `weight` to each outlined cell. Uses only
    /// integer addition/subtraction. Ellipse eqn: A*x^2 + B*y^2 + C*x*y - 1
    /// = 0, with A = B = radius^2, C = 0 (a circle). All accumulator math is
    /// int64: radius^2 overflows 32-bit int for radius >= 46341 (§四-低10).
    void accumulate(int centerX, int centerY, float weight) {
        const std::int64_t aa =
            static_cast<std::int64_t>(max_radius_px_) * max_radius_px_;
        const std::int64_t bb = aa;
        const std::int64_t twoC = 0;

        int x = 0;
        int y = static_cast<int>(std::lround(std::sqrt(static_cast<double>(bb))));
        const std::int64_t twoaa = 2 * aa;
        const std::int64_t twobb = 2 * bb;
        std::int64_t dx = (twoaa * y) + (twoC * x);
        std::int64_t dy = -((twobb * x) + (twoC * y));
        std::int64_t ellipseError = aa * ((static_cast<std::int64_t>(y) * y) - bb);

        // first sector: (dy/dx > 1) -> y+1 (x+1)
        while (dy > dx) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = ellipseError + dx + aa;
            dx = dx + twoaa;
            dy = dy - twoC;
            y = y + 1;
            if ((((2 * ellipseError) - dy) + bb) > 0) {
                ellipseError = (ellipseError - dy) + bb;
                dx = dx + twoC;
                dy = dy - twobb;
                x = x + 1;
            }
        }

        // second sector: (dy/dx > 0) -> x+1 (y+1)
        while (dy > 0) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = (ellipseError - dy) + bb;
            dx = dx + twoC;
            dy = dy - twobb;
            x = x + 1;
            if (((2 * ellipseError) + dx + aa) < 0) {
                ellipseError = ellipseError + dx + aa;
                dx = dx + twoaa;
                dy = dy - twoC;
                y = y + 1;
            }
        }

        // third sector: (dy/dx > -1) -> x+1 (y-1)
        while (dy > -dx) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = (ellipseError - dy) + bb;
            dx = dx + twoC;
            dy = dy - twobb;
            x = x + 1;
            if ((((2 * ellipseError) - dx) + aa) > 0) {
                ellipseError = (ellipseError - dx) + aa;
                dx = dx - twoaa;
                dy = dy + twoC;
                y = y - 1;
            }
        }

        // fourth sector: (dy/dx < 0) -> y-1 (x+1)
        while (dx > 0) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = (ellipseError - dx) + aa;
            dx = dx - twoaa;
            dy = dy + twoC;
            y = y - 1;
            if ((((2 * ellipseError) - dy) + bb) < 0) {
                ellipseError = (ellipseError - dy) + bb;
                dx = dx + twoC;
                dy = dy - twobb;
                x = x + 1;
            }
        }

        // fifth sector (dy/dx > 1) -> y-1 (x-1)
        while ((dy < dx) && (x > 0)) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = (ellipseError - dx) + aa;
            dx = dx - twoaa;
            dy = dy + twoC;
            y = y - 1;
            if (((2 * ellipseError) + dy + bb) > 0) {
                ellipseError = ellipseError + dy + bb;
                dx = dx - twoC;
                dy = dy + twobb;
                x = x - 1;
            }
        }

        // sixth sector: (dy/dx > 0) -> x-1 (y-1)
        while ((dy < 0) && (x > 0)) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = ellipseError + dy + bb;
            dx = dx - twoC;
            dy = dy + twobb;
            x = x - 1;
            if ((((2 * ellipseError) - dx) + aa) < 0) {
                ellipseError = (ellipseError - dx) + aa;
                dx = dx - twoaa;
                dy = dy + twoC;
                y = y - 1;
            }
        }

        // seventh sector: (dy/dx > -1) -> x-1 (y+1)
        while ((dy < -dx) && (x > 0)) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = ellipseError + dy + bb;
            dx = dx - twoC;
            dy = dy + twobb;
            x = x - 1;
            if (((2 * ellipseError) + dx + aa) > 0) {
                ellipseError = ellipseError + dx + aa;
                dx = dx + twoaa;
                dy = dy - twoC;
                y = y + 1;
            }
        }

        // eight sector: (dy/dx < 0) -> y+1 (x-1)
        while (((dy > 0) && (dx < 0)) && (x > 0)) {
            increase_hough_point(centerX + x, centerY + y, weight);
            increase_hough_point(centerX - x, centerY - y, weight);
            ellipseError = ellipseError + dx + aa;
            dx = dx + twoaa;
            dy = dy - twoC;
            y = y + 1;
            if (((2 * ellipseError) + dy + bb) < 0) {
                ellipseError = ellipseError + dy + bb;
                dx = dx - twoC;
                dy = dy + twobb;
                x = x - 1;
            }
        }
    }

    // Parameters ------------------------------------------------------------
    int width_;
    int height_;
    int max_radius_px_;   ///< The single fixed circle radius (jAER `radius`).
    int threshold_;
    float decay_;          ///< jAER `decay` (default 1.0).
    int buffer_length_;    ///< jAER `bufferLength` (default 4000).
    int nr_max_;           ///< jAER `nrMax` (default 1).
    bool decay_mode_;      ///< jAER `decayMode` (default true).
    bool loc_depression_;  ///< jAER `locDepression` (default true).

    // State -----------------------------------------------------------------
    std::vector<float> accum_;          ///< 2D accumulator (width*height).
    std::vector<Coord> event_history_;  ///< FIFO event history.
    int buffer_index_{0};
    std::vector<float> max_value_;      ///< Running maxima values (size nr_max_).
    std::vector<Coord> max_coord_;      ///< Running maxima positions (size nr_max_).
    Metavision::timestamp last_t_{0};   ///< jAER `timeStamp`.
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_HOUGH_CIRCLE_TRACKER_H
