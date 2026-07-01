// algo/common/event_rate_estimator.h — windowed event rate estimator.
//
// Inspired by jAER EventRateEstimator. Estimates the event rate
// (events/second, also reported in Mev/s) from the event stream by counting
// events over a sliding time window. When the elapsed time since the last rate
// computation reaches window_us, the rate is computed as
//   instantaneousRate = numEventsSinceLastUpdate / (dt * 1e-6)
// and filteredRate is set equal to instantaneousRate (jAER does no IIR
// smoothing here — the windowing itself provides the smoothing). The estimator
// is fed event batches and queried for the current rate.

#ifndef GUI_ALGO_COMMON_EVENT_RATE_ESTIMATOR_H
#define GUI_ALGO_COMMON_EVENT_RATE_ESTIMATOR_H

#include <cstddef>

#include <metavision/sdk/base/utils/timestamp.h>

namespace gui_algo {

/// @brief Windowed event rate estimator (events/second).
class EventRateEstimator {
public:
    /// @brief Constructs the estimator.
    /// @param window_us Accumulation window in microseconds. The rate is
    ///        recomputed each time the elapsed time since the last computation
    ///        reaches this value.
    explicit EventRateEstimator(Metavision::timestamp window_us = 10000)
        : window_us_(window_us) {}

    /// @brief Notifies the estimator of a batch of events.
    /// @param n Number of events in the batch.
    /// @param t Timestamp of the last event in the batch (us).
    void add_events(std::size_t n, Metavision::timestamp t) {
        if (n == 0) return;
        if (!initialized_) {
            // First batch: seed the reference timestamp, no rate yet.
            window_count_ = 0;
            last_compute_t_ = t;
            initialized_ = true;
            return;
        }
        const auto dt = t - last_compute_t_;
        if (dt < 0) {
            // Nonmonotonic timestamp: re-initialize (matches jAER behaviour).
            initialized_ = false;
            return;
        }
        window_count_ += n;
        if (dt >= window_us_) {
            last_compute_t_ = t;
            instantaneous_rate_ =
                static_cast<double>(window_count_) / (static_cast<double>(dt) * 1.0e-6);
            // jAER: filteredRate = instantaneousRate (windowing does the smoothing).
            rate_ = instantaneous_rate_;
            window_count_ = 0;
        }
    }

    /// @brief Returns the filtered event rate in events/second, or 0 if unknown.
    double rate_eps() const { return rate_ < 0.0 ? 0.0 : rate_; }

    /// @brief Returns the filtered event rate in Mev/s (mega-events/second).
    double rate_meps() const { return rate_eps() * 1.0e-6; }

    /// @brief Returns the last instantaneous event rate in events/second
    ///        (computed over the most recent completed window), or 0 if unknown.
    double instantaneous_rate() const {
        return instantaneous_rate_ < 0.0 ? 0.0 : instantaneous_rate_;
    }

    /// @brief Resets the estimator state.
    void reset() {
        rate_ = -1.0;
        instantaneous_rate_ = -1.0;
        window_count_ = 0;
        last_compute_t_ = 0;
        initialized_ = false;
    }

    Metavision::timestamp window_us() const { return window_us_; }
    void set_window_us(Metavision::timestamp w) { window_us_ = w; }

private:
    Metavision::timestamp window_us_;
    double rate_{-1.0};                 // events/second, -1 = uninitialized
    double instantaneous_rate_{-1.0};   // events/second, -1 = uninitialized
    std::size_t window_count_{0};
    Metavision::timestamp last_compute_t_{0};
    bool initialized_{false};
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_EVENT_RATE_ESTIMATOR_H
