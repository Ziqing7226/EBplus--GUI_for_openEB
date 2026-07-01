// algo/cv/background_mask_filter.h — Background mask learning (motion/still sep).
//
// Implements design §4.3.19 (jAER BackgroundActivityFilter + Histogram2DFilter).
// Maintains a per-pixel exponentially-decaying event activity whose steady
// state approximates the local event rate (activity / tau). Pixels whose rate
// stays below background_rate_threshold_hz over the learning window are learned
// as static background; the filter outputs a foreground mask (255 = moving,
// 0 = background) for downstream trackers. Header-only.

#ifndef GUI_ALGO_CV_BACKGROUND_MASK_FILTER_H
#define GUI_ALGO_CV_BACKGROUND_MASK_FILTER_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Background mask learner producing a foreground mask.
class BackgroundMaskFilter {
public:
    BackgroundMaskFilter(int width, int height,
                         float learning_window_s = 5.0f,
                         float background_rate_threshold_hz = 1.0f)
        : width_(width), height_(height),
          learning_window_s_(static_cast<double>(learning_window_s)),
          tau_us_(static_cast<Metavision::timestamp>(
              static_cast<double>(learning_window_s) * 1.0e6)),
          threshold_hz_(static_cast<double>(background_rate_threshold_hz)),
          activity_(static_cast<std::size_t>(width) * height, 0.0),
          last_t_(static_cast<std::size_t>(width) * height, -1),
          mask_(cv::Mat::zeros(height, width, CV_8UC1)) {}

    /// @brief Updates per-pixel activity and returns the current foreground mask.
    ///        255 = foreground (moving), 0 = background (static).
    const cv::Mat& process(const EventPacket& packet) {
        // Determine the current packet end time (max event timestamp). For an
        // empty packet fall back to the last seen timestamp so the staleness
        // decay below remains a no-op rather than operating on stale data.
        Metavision::timestamp t_now = last_packet_t_;
        if (!packet.empty()) {
            t_now = packet[0].t;
            for (const Event& e : packet) {
                if (e.t > t_now) t_now = e.t;
            }
            if (t_now > last_packet_t_) last_packet_t_ = t_now;
        }

        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            const std::size_t idx =
                static_cast<std::size_t>(e.y) * width_ + e.x;
            const Metavision::timestamp prev = last_t_[idx];
            last_t_[idx] = e.t;
            if (prev >= 0 && e.t > prev) {
                const double dt = static_cast<double>(e.t - prev);
                const double decay = std::exp(-dt / static_cast<double>(tau_us_));
                activity_[idx] = activity_[idx] * decay + 1.0;
            } else {
                activity_[idx] = 1.0;
            }
        }

        // Decay every pixel's activity to the current packet end time so that
        // pixels which have not received a recent event do not retain stale
        // (overstated) activity when the mask is evaluated.
        if (t_now >= 0) {
            for (std::size_t i = 0; i < activity_.size(); ++i) {
                const Metavision::timestamp lt = last_t_[i];
                if (lt >= 0 && t_now > lt) {
                    const double dt = static_cast<double>(t_now - lt);
                    activity_[i] *= std::exp(-dt / static_cast<double>(tau_us_));
                    last_t_[i] = t_now;
                }
            }
        }

        const double tau_s = learning_window_s_;
        for (std::size_t i = 0; i < activity_.size(); ++i) {
            const double rate = activity_[i] / tau_s; // events/second
            mask_.at<std::uint8_t>(static_cast<int>(i)) =
                (rate >= threshold_hz_) ? 255 : 0;
        }
        return mask_;
    }

    /// @brief Returns the current foreground mask without updating activity.
    const cv::Mat& mask() const { return mask_; }

    // Parameter accessors ---------------------------------------------------
    float learning_window_s() const {
        return static_cast<float>(learning_window_s_);
    }
    float background_rate_threshold_hz() const {
        return static_cast<float>(threshold_hz_);
    }
    void set_learning_window_s(float v) {
        learning_window_s_ = static_cast<double>(v);
        tau_us_ = static_cast<Metavision::timestamp>(static_cast<double>(v) * 1.0e6);
    }
    void set_background_rate_threshold_hz(float v) {
        threshold_hz_ = static_cast<double>(v);
    }

    void reset() {
        std::fill(activity_.begin(), activity_.end(), 0.0);
        std::fill(last_t_.begin(), last_t_.end(),
                  static_cast<Metavision::timestamp>(-1));
        last_packet_t_ = -1;
        mask_.setTo(0);
    }

private:
    int width_;
    int height_;
    double learning_window_s_;
    Metavision::timestamp tau_us_;
    double threshold_hz_;
    std::vector<double> activity_;
    std::vector<Metavision::timestamp> last_t_;
    Metavision::timestamp last_packet_t_{-1}; ///< High-water mark of packet end times.
    cv::Mat mask_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_BACKGROUND_MASK_FILTER_H
