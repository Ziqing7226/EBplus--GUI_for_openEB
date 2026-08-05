// algo/common/performance_meter.h — performance profiler (latency / counters).
//
// Inspired by jAER PerformanceMeter. Measures display-pipeline latency:
//   - Latency (us): time from event batch arrival to frame display
//   - Counters: total events / frames / dropped batches, drop ratio
// Trimmed (audit §三-S4): removed the APIs that had no production caller —
// the FPS IIR (computed every tick_frame but never read), the windowed
// EventRateEstimator (the GUI uses the SDK's own rate for throughput), and
// the jAER start/stop per-section timing suite. Kept: latency (read by the
// 10Hz perf flush in MainWindow) and the cheap counters/drop-ratio
// diagnostics.
// Designed for low overhead: call tick_frame() once per rendered frame.

#ifndef GUI_ALGO_COMMON_PERFORMANCE_METER_H
#define GUI_ALGO_COMMON_PERFORMANCE_METER_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

#include <metavision/sdk/base/utils/timestamp.h>

namespace gui_algo {

/// @brief Performance profiler for the display/processing pipeline.
class PerformanceMeter {
public:
    using Clock = std::chrono::steady_clock;

    /// @param smoothing IIR factor in (0, 1] for latency smoothing.
    explicit PerformanceMeter(float smoothing = 0.1f)
        : smoothing_(smoothing) {}

    /// @brief Marks the arrival of an event batch (start of pipeline latency).
    /// Thread-safe: called from the SDK data thread in live-camera mode.
    /// @p t_us is the batch timestamp, kept for call-site symmetry / future
    /// event-time latency measurements; currently unused.
    void tick_events(std::size_t n, Metavision::timestamp /*t_us*/) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_event_tick_ = Clock::now();
        total_events_ += n;
    }

    /// @brief Marks the completion of a rendered frame (end of pipeline latency).
    /// Thread-safe: called from the GUI thread.
    void tick_frame() {
        std::lock_guard<std::mutex> lk(mutex_);
        const auto now = Clock::now();
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
        std::lock_guard<std::mutex> lk(mutex_);
        total_dropped_ += n;
    }

    double latency_us() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return latency_us_;
    }

    /// @brief Drop ratio in [0, 1]: dropped / (processed + dropped).
    double drop_ratio() const {
        std::lock_guard<std::mutex> lk(mutex_);
        const std::uint64_t denom = total_events_ + total_dropped_;
        return denom == 0 ? 0.0 : static_cast<double>(total_dropped_) / denom;
    }

    std::uint64_t total_events() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return total_events_;
    }
    std::uint64_t total_frames() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return total_frames_;
    }
    std::uint64_t total_dropped() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return total_dropped_;
    }

    void reset() {
        std::lock_guard<std::mutex> lk(mutex_);
        latency_us_ = 0.0;
        total_events_ = 0;
        total_frames_ = 0;
        total_dropped_ = 0;
        last_event_tick_.reset();
    }

private:
    float smoothing_;
    // Protects all members: tick_events() is called from the SDK data thread
    // (live-camera mode) while tick_frame()/reset()/getters are called from
    // the GUI thread. Without this, concurrent read/write of last_event_tick_
    // (std::optional<time_point>) is undefined behaviour.
    mutable std::mutex mutex_;
    double latency_us_{0.0};
    std::uint64_t total_events_{0};
    std::uint64_t total_frames_{0};
    std::uint64_t total_dropped_{0};
    std::optional<Clock::time_point> last_event_tick_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_PERFORMANCE_METER_H
