// algo/common/histogram_ring_buffer.h — rolling histogram statistics.
//
// Maintains a bounded rolling window of scalar samples (ISI, direction,
// polarity, latency) and computes a histogram + summary statistics (mean, std,
// percentiles). Used by isi_analyzer, direction consensus, and latency probes.
// Header-only; O(1) push, O(bins) stats.

#ifndef GUI_ALGO_COMMON_HISTOGRAM_RING_BUFFER_H
#define GUI_ALGO_COMMON_HISTOGRAM_RING_BUFFER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace gui_algo {

/// @brief Rolling histogram over a sliding window of scalar samples.
class HistogramRingBuffer {
public:
    /// @brief Constructs the histogram.
    /// @param window_size Maximum samples retained (FIFO).
    /// @param num_bins Number of histogram bins.
    /// @param value_min,value_max Range covered by the bins.
    HistogramRingBuffer(std::size_t window_size, std::size_t num_bins,
                        double value_min, double value_max)
        : window_size_(window_size == 0 ? 1 : window_size),
          num_bins_(num_bins == 0 ? 1 : num_bins),
          vmin_(value_min), vmax_(value_max),
          samples_(), counts_(num_bins_, 0) {}

    /// @brief Pushes a new sample, evicting the oldest if the window is full.
    void push(double v) {
        samples_.push_back(v);
        if (samples_.size() > window_size_) {
            const double old = samples_.front();
            samples_.pop_front();
            const int b = bin_of(old);
            if (b >= 0 && b < static_cast<int>(num_bins_)) --counts_[b];
        }
        const int b = bin_of(v);
        if (b >= 0 && b < static_cast<int>(num_bins_)) ++counts_[b];
    }

    /// @brief Returns the histogram bin counts.
    const std::vector<std::uint64_t>& counts() const { return counts_; }

    /// @brief Returns the current number of samples in the window.
    std::size_t size() const { return samples_.size(); }

    std::size_t num_bins() const { return num_bins_; }
    double value_min() const { return vmin_; }
    double value_max() const { return vmax_; }

    /// @brief Computes the arithmetic mean of the current window.
    double mean() const {
        if (samples_.empty()) return 0.0;
        double s = 0.0;
        for (const auto v : samples_) s += v;
        return s / static_cast<double>(samples_.size());
    }

    /// @brief Computes the population standard deviation of the current window.
    double std_dev() const {
        if (samples_.size() < 2) return 0.0;
        const double m = mean();
        double s = 0.0;
        for (const auto v : samples_) {
            const double d = v - m;
            s += d * d;
        }
        return std::sqrt(s / static_cast<double>(samples_.size()));
    }

    /// @brief Returns the @p q-th percentile in [0, 100] of the current window.
    double percentile(double q) const {
        if (samples_.empty()) return 0.0;
        // Clamp q to [0, 100] (§四-低7): out-of-range q was UB via the
        // (tmp.size()-1) scaled index.
        if (q < 0.0) q = 0.0;
        else if (q > 100.0) q = 100.0;
        std::vector<double> tmp(samples_.begin(), samples_.end());
        std::sort(tmp.begin(), tmp.end());
        const double pos = (q / 100.0) * (tmp.size() - 1);
        const std::size_t lo = static_cast<std::size_t>(pos);
        const std::size_t hi = lo + 1 < tmp.size() ? lo + 1 : lo;
        const double frac = pos - static_cast<double>(lo);
        return tmp[lo] * (1.0 - frac) + tmp[hi] * frac;
    }

    void clear() {
        samples_.clear();
        std::fill(counts_.begin(), counts_.end(), 0);
    }

private:
    int bin_of(double v) const {
        if (v < vmin_ || v >= vmax_) return -1;
        const double range = vmax_ - vmin_;
        if (range <= 0.0) return 0;
        return static_cast<int>((v - vmin_) / range * num_bins_);
    }

    std::size_t window_size_;
    std::size_t num_bins_;
    double vmin_;
    double vmax_;
    // deque gives O(1) push/pop at both ends for the sliding window.
    std::deque<double> samples_;
    std::vector<std::uint64_t> counts_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_HISTOGRAM_RING_BUFFER_H
