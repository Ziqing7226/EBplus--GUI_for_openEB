// algo/common/performance_meter.h — performance profiler (FPS / latency / rate).
//
// Inspired by jAER PerformanceMeter / EventProcessingPerformanceMeter. Measures
// rendering/processing throughput:
//   - FPS (frames per second): IIR-smoothed over frame completions
//   - Latency (us): time from event batch arrival to frame display
//   - Event rate (eps/Meps): forwarded from EventRateEstimator (windowed)
//   - Drop rate: fraction of batches dropped due to overload
//   - Per-filter cost: ns/event, eps, average ± stderr over repeated start/stop
//     samples (matching jAER EventProcessingPerformanceMeter.start/stop).
// Designed for low overhead: call tick_frame() once per rendered frame and
// start()/stop() around the section being profiled.

#ifndef GUI_ALGO_COMMON_PERFORMANCE_METER_H
#define GUI_ALGO_COMMON_PERFORMANCE_METER_H

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include "event_rate_estimator.h"

namespace gui_algo {

/// @brief Performance profiler for the display/processing pipeline.
class PerformanceMeter {
public:
    using Clock = std::chrono::steady_clock;

    /// @param smoothing IIR factor in (0, 1] for FPS / latency smoothing.
    explicit PerformanceMeter(float smoothing = 0.1f)
        : smoothing_(smoothing) {}

    /// @brief Marks the arrival of an event batch (start of pipeline latency).
    void tick_events(std::size_t n, Metavision::timestamp t_us) {
        rate_estimator_.add_events(n, t_us);
        last_event_tick_ = Clock::now();
        total_events_ += n;
    }

    /// @brief Marks the completion of a rendered frame (end of pipeline latency).
    /// Computes FPS and instantaneous latency.
    void tick_frame() {
        const auto now = Clock::now();
        if (last_frame_tick_.has_value()) {
            const double dt_s =
                std::chrono::duration<double>(now - *last_frame_tick_).count();
            if (dt_s > 0.0) {
                const double instant_fps = 1.0 / dt_s;
                fps_ = smoothing_ * instant_fps + (1.0 - smoothing_) * fps_;
            }
        } else {
            fps_ = 0.0;
        }
        last_frame_tick_ = now;

        // Latency: event-batch-arrival → frame-completion.
        if (last_event_tick_.has_value()) {
            const double latency_us =
                std::chrono::duration<double, std::micro>(now - *last_event_tick_).count();
            latency_us_ = smoothing_ * latency_us + (1.0 - smoothing_) * latency_us_;
        }
        ++total_frames_;
    }

    /// @brief Records a dropped batch (overload backpressure).
    void tick_drop(std::size_t n) {
        total_dropped_ += n;
    }

    /// @brief Starts a per-section timing measurement over @p n_events events.
    /// Matches jAER EventProcessingPerformanceMeter.start(int nEvents).
    void start(std::uint64_t n_events) {
        start_n_events_ = n_events;
        start_time_ns_ = now_ns();
    }

    /// @brief Stops the per-section timing measurement and accumulates the
    /// ns/event sample for averaging. Matches jAER
    /// EventProcessingPerformanceMeter.stop().
    void stop() {
        const std::int64_t duration_ns = now_ns() - start_time_ns_;
        duration_ns_ = duration_ns;
        const double this_nspe = (start_n_events_ == 0)
            ? 0.0
            : static_cast<double>(duration_ns) / static_cast<double>(start_n_events_);
        nspe_sum_ += this_nspe;
        nspe_sq_ += this_nspe * this_nspe;
        ++n_samples_;
    }

    /// @brief ns/event for the most recent start/stop sample.
    double ns_per_event() const {
        if (start_n_events_ == 0 || duration_ns_ == 0) return 0.0;
        return static_cast<double>(duration_ns_) / static_cast<double>(start_n_events_);
    }

    /// @brief events/second for the most recent start/stop sample.
    double eps() const {
        if (duration_ns_ <= 0) return 0.0;
        return static_cast<double>(start_n_events_) /
               (static_cast<double>(duration_ns_) * 1.0e-9);
    }

    /// @brief Average ns/event over all accumulated start/stop samples.
    double avg_ns_per_event() const {
        if (n_samples_ == 0) return 0.0;
        return nspe_sum_ / static_cast<double>(n_samples_);
    }

    /// @brief Standard error of the mean ns/event over all accumulated samples.
    double stderr_ns_per_event() const {
        if (n_samples_ < 2) return 0.0;
        const double n = static_cast<double>(n_samples_);
        const double var = (nspe_sq_ - nspe_sum_ * nspe_sum_ / n) / (n - 1.0);
        return var <= 0.0 ? 0.0 : std::sqrt(var);
    }

    std::uint64_t n_samples() const { return n_samples_; }

    double fps() const { return fps_; }
    double latency_us() const { return latency_us_; }
    double event_rate_eps() const { return rate_estimator_.rate_eps(); }
    double event_rate_meps() const { return rate_estimator_.rate_meps(); }

    /// @brief Drop ratio in [0, 1]: dropped / (processed + dropped).
    double drop_ratio() const {
        const std::uint64_t denom = total_events_ + total_dropped_;
        return denom == 0 ? 0.0 : static_cast<double>(total_dropped_) / denom;
    }

    std::uint64_t total_events() const { return total_events_; }
    std::uint64_t total_frames() const { return total_frames_; }
    std::uint64_t total_dropped() const { return total_dropped_; }

    void reset() {
        fps_ = 0.0;
        latency_us_ = 0.0;
        total_events_ = 0;
        total_frames_ = 0;
        total_dropped_ = 0;
        last_event_tick_.reset();
        last_frame_tick_.reset();
        rate_estimator_.reset();
        // Per-section statistics.
        start_n_events_ = 0;
        start_time_ns_ = 0;
        duration_ns_ = 0;
        nspe_sum_ = 0.0;
        nspe_sq_ = 0.0;
        n_samples_ = 0;
    }

private:
    static std::int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   Clock::now().time_since_epoch()).count();
    }

    float smoothing_;
    EventRateEstimator rate_estimator_;
    double fps_{0.0};
    double latency_us_{0.0};
    std::uint64_t total_events_{0};
    std::uint64_t total_frames_{0};
    std::uint64_t total_dropped_{0};
    std::optional<Clock::time_point> last_event_tick_;
    std::optional<Clock::time_point> last_frame_tick_;
    // Per-section timing (jAER EventProcessingPerformanceMeter semantics).
    std::uint64_t start_n_events_{0};
    std::int64_t start_time_ns_{0};
    std::int64_t duration_ns_{0};
    double nspe_sum_{0.0};
    double nspe_sq_{0.0};
    std::uint64_t n_samples_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_PERFORMANCE_METER_H
