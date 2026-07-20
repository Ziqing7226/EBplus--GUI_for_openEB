// algo/analytics/e2vid/event_voxel_grid.h — Event -> voxel grid tensor.
//
// Design §4.4.2 (E2VID path). Builds a num_bins x H x W voxel grid from a
// batch of events using bilinear interpolation in the time domain, exactly
// as in rpg_e2vid (utils/inference_utils.py::events_to_voxel_grid).
// Each event's polarity (+1/-1) is split between the two nearest temporal
// bins proportional to the fractional time offset. The resulting tensor is
// the standard input to the E2VID / E2VID-Recurrent UNet model.
// Also provides optional hot-pixel masking and per-tensor normalization
// (zero-mean, unit-stddev of nonzero elements) matching the rpg_e2vid
// EventPreprocessor. Header-only.

#ifndef GUI_ALGO_ANALYTICS_E2VID_EVENT_VOXEL_GRID_H
#define GUI_ALGO_ANALYTICS_E2VID_EVENT_VOXEL_GRID_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "algo/common/event.h"

namespace gui_algo {

/// @brief Builds event voxel grids (num_bins x H x W) for E2VID inference.
class EventVoxelGrid {
public:
    /// @brief Constructs the voxel grid builder.
    /// @param width,height Sensor dimensions.
    /// @param num_bins Number of temporal bins (E2VID default: 5).
    EventVoxelGrid(int width, int height, int num_bins = 5)
        : width_(width), height_(height),
          num_bins_(clamp_bins(num_bins)),
          grid_(static_cast<std::size_t>(num_bins_) * width * height, 0.0f) {}

    /// @brief Sets the hot-pixel mask (HxW, uint8, nonzero = hot pixel to zero out).
    /// The size is validated against the grid resolution (§四-M3): an exact
    /// HxW mask is used directly; a full-resolution (2H)x(2W) mask — e.g. a
    /// sensor-wide mask passed while the grid runs at half resolution — is
    /// downsampled by 2x2 majority vote. Any other size is rejected (masking
    /// disabled) and recorded in hot_mask_rejected().
    void set_hot_pixel_mask(const std::vector<std::uint8_t>& mask) {
        hot_mask_rejected_ = false;
        const std::size_t expect =
            static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
        if (mask.size() == expect) {
            hot_mask_ = mask;
            return;
        }
        if (expect > 0 && mask.size() == expect * 4) {
            // 2x2 majority-vote downsample from full to effective resolution.
            const int fw = width_ * 2;
            hot_mask_.assign(expect, 0);
            for (int y = 0; y < height_; ++y) {
                for (int x = 0; x < width_; ++x) {
                    const std::size_t b =
                        static_cast<std::size_t>(2 * y) * fw + 2 * x;
                    const int cnt = (mask[b] != 0) + (mask[b + 1] != 0) +
                                    (mask[b + fw] != 0) + (mask[b + fw + 1] != 0);
                    hot_mask_[static_cast<std::size_t>(y) * width_ + x] =
                        (cnt >= 2) ? 1 : 0;
                }
            }
            return;
        }
        hot_mask_.clear();
        hot_mask_rejected_ = true;
    }

    /// @brief True if the last set_hot_pixel_mask() call was rejected due to
    ///        a size mismatch (masking is disabled in that case).
    bool hot_mask_rejected() const { return hot_mask_rejected_; }

    /// @brief Clears the hot-pixel mask (disables hot-pixel removal).
    void clear_hot_pixel_mask() { hot_mask_.clear(); }

    /// @brief Builds a voxel grid from a batch of events.
    /// @param events Event array.
    /// @param n Number of events.
    /// @return Reference to the internal grid (num_bins * H * W, row-major
    ///         bin-major: grid[b * H * W + y * W + x]).
    const std::vector<float>& build(const Event* events, std::size_t n) {
        std::fill(grid_.begin(), grid_.end(), 0.0f);
        if (events == nullptr || n == 0 || width_ <= 0 || height_ <= 0) {
            return grid_;
        }

        const double first_t = static_cast<double>(events[0].t);
        const double last_t = static_cast<double>(events[n - 1].t);
        const double deltaT = last_t - first_t;
        const double scale = (deltaT > 0.0)
            ? static_cast<double>(num_bins_ - 1) / deltaT
            : 0.0;

        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            if (!hot_mask_.empty()) {
                const std::size_t idx =
                    static_cast<std::size_t>(e.y) * width_ + e.x;
                if (hot_mask_[idx]) continue;  // hot pixel -> skip
            }
            // Normalized timestamp in [0, num_bins-1].
            const double t_norm =
                (static_cast<double>(e.t) - first_t) * scale;
            const int ti = static_cast<int>(t_norm);
            const double dt = t_norm - static_cast<double>(ti);
            const float pol = e.is_on() ? 1.0f : -1.0f;
            const float val_left = pol * static_cast<float>(1.0 - dt);
            const float val_right = pol * static_cast<float>(dt);

            const std::size_t base =
                static_cast<std::size_t>(e.y) * width_ + e.x;
            if (ti >= 0 && ti < num_bins_) {
                grid_[static_cast<std::size_t>(ti) * stride_hw() + base] +=
                    val_left;
            }
            if (ti + 1 >= 0 && ti + 1 < num_bins_) {
                grid_[static_cast<std::size_t>(ti + 1) * stride_hw() + base] +=
                    val_right;
            }
        }
        return grid_;
    }

    /// @brief Normalizes the grid in-place: zero-mean, unit-stddev of nonzero
    ///        elements (matching rpg_e2vid EventPreprocessor).
    void normalize() {
        double sum = 0.0;
        std::size_t count = 0;
        for (const float v : grid_) {
            if (v != 0.0f) {
                sum += static_cast<double>(v);
                ++count;
            }
        }
        if (count == 0) return;
        const double mean = sum / static_cast<double>(count);
        double sq_sum = 0.0;
        for (const float v : grid_) {
            if (v != 0.0f) {
                const double d = static_cast<double>(v) - mean;
                sq_sum += d * d;
            }
        }
        const double stddev = std::sqrt(sq_sum / static_cast<double>(count));
        if (stddev < 1e-12) return;
        for (float& v : grid_) {
            if (v != 0.0f) {
                v = static_cast<float>((static_cast<double>(v) - mean) / stddev);
            }
        }
    }

    /// @brief Returns the grid as a cv::Mat (num_bins rows, each row is HxW
    ///        flattened — or use as 1xCxHxW tensor). Mainly for visualization.
    cv::Mat to_mat() const {
        cv::Mat m(num_bins_, width_ * height_, CV_32FC1,
                  const_cast<float*>(grid_.data()));
        return m.clone();
    }

    /// @brief Renders a 2D event preview (sum across bins) as a red-blue image.
    ///        Positive events: blue, negative events: red.
    cv::Mat render_preview() const {
        cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                const std::size_t base =
                    static_cast<std::size_t>(y) * width_ + x;
                float sum = 0.0f;
                for (int b = 0; b < num_bins_; ++b) {
                    sum += grid_[static_cast<std::size_t>(b) * stride_hw() + base];
                }
                if (sum > 0.0f) {
                    img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);  // blue
                } else if (sum < 0.0f) {
                    img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);  // red
                }
            }
        }
        return img;
    }

    /// @brief Returns a const pointer to the raw grid data (bin-major).
    const float* data() const { return grid_.data(); }
    std::size_t size() const { return grid_.size(); }
    int num_bins() const { return num_bins_; }
    int width() const { return width_; }
    int height() const { return height_; }

    void reset() { std::fill(grid_.begin(), grid_.end(), 0.0f); }

private:
    static int clamp_bins(int b) {
        if (b < 1) return 1;
        if (b > 20) return 20;
        return b;
    }
    std::size_t stride_hw() const {
        return static_cast<std::size_t>(width_) * height_;
    }

    int width_;
    int height_;
    int num_bins_;
    std::vector<float> grid_;
    std::vector<std::uint8_t> hot_mask_;
    bool hot_mask_rejected_{false};  ///< Last mask rejected (size mismatch).
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_E2VID_EVENT_VOXEL_GRID_H
