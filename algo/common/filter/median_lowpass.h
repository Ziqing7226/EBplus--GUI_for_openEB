// algo/common/filter/median_lowpass.h — median low-pass filter (impulse denoiser).
//
// Inspired by jAER MedianLowpassFilter. A running median filter that removes
// impulsive outliers (spikes) from a scalar signal while preserving edges.
// Maintains a sliding window of recent samples and returns their median on each
// update. Matches jAER: the window is zero-filled until full (so the output is
// zero for at least length/2 samples at startup), and for even window sizes the
// upper-middle element samples[length/2] is used (not the average of the two
// middle values). Header-only.

#ifndef GUI_ALGO_COMMON_FILTER_MEDIAN_LOWPASS_H
#define GUI_ALGO_COMMON_FILTER_MEDIAN_LOWPASS_H

#include <algorithm>
#include <cstddef>
#include <deque>
#include <vector>

namespace gui_algo {

/// @brief Running median filter for scalar signals (impulse noise removal).
class MedianLowpassFilter {
public:
    /// @brief Constructs the filter.
    /// @param window_size Number of samples in the sliding window.
    explicit MedianLowpassFilter(std::size_t window_size = 5)
        : window_size_(window_size == 0 ? 1 : window_size) {
        window_.assign(window_size_, 0.0);
    }

    /// @brief Pushes a new sample and returns the median of the current window.
    double process(double x) {
        window_.push_back(x);
        if (window_.size() > window_size_) window_.pop_front();
        return median();
    }

    /// @brief Current median of the window (without pushing).
    double value() const { return median(); }

    std::size_t window_size() const { return window_size_; }
    void set_window_size(std::size_t n) {
        window_size_ = n == 0 ? 1 : n;
        // Keep the most recent samples when shrinking; zero-pad when growing.
        while (window_.size() > window_size_) window_.pop_front();
        while (window_.size() < window_size_) window_.push_front(0.0);
    }

    void reset() { window_.assign(window_size_, 0.0); }

private:
    double median() const {
        if (window_.empty()) return 0.0;
        std::vector<double> tmp(window_.begin(), window_.end());
        std::sort(tmp.begin(), tmp.end());
        // jAER uses samples[length/2] (upper-middle for even length).
        return tmp[tmp.size() / 2];
    }

    std::size_t window_size_;
    std::deque<double> window_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_FILTER_MEDIAN_LOWPASS_H
