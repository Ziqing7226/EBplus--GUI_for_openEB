// algo/common/filter/bandpass.h — first-order IIR band-pass filter.
//
// Inspired by jAER BandpassFilter, which cascades low-pass then high-pass:
//   out = hp(lp(x))
// The low-pass removes high-frequency noise, the high-pass removes DC / slow
// drift, leaving the band [low_hz, high_hz]. Supports N-order cascading for
// steeper roll-off. Header-only.

#ifndef GUI_ALGO_COMMON_FILTER_BANDPASS_H
#define GUI_ALGO_COMMON_FILTER_BANDPASS_H

#include <vector>

#include "highpass.h"
#include "lowpass.h"

namespace gui_algo {

/// @brief First-order (cascade) IIR band-pass filter.
class BandpassFilter {
public:
    /// @brief Constructs the band-pass filter.
    /// @param low_cutoff_hz High-pass cutoff (removes frequencies below this).
    /// @param high_cutoff_hz Low-pass cutoff (removes frequencies above this).
    /// @param sample_dt Time between samples in seconds.
    /// @param order Number of cascaded stages (1 = first-order, 4 = 4th-order).
    BandpassFilter(double low_cutoff_hz = 1.0, double high_cutoff_hz = 10.0,
                   double sample_dt = 0.033, int order = 1)
        : dt_(sample_dt) {
        const int n = order < 1 ? 1 : order;
        hp_.reserve(n);
        lp_.reserve(n);
        for (int i = 0; i < n; ++i) {
            hp_.emplace_back(low_cutoff_hz, sample_dt);
            lp_.emplace_back(high_cutoff_hz, sample_dt);
        }
    }

    /// @brief Filters a new sample and returns the band-pass output.
    /// jAER order: low-pass first, then high-pass (hp(lp(x))).
    double process(double x) {
        double y = x;
        for (auto& lp : lp_) y = lp.process(y);
        for (auto& hp : hp_) y = hp.process(y);
        last_ = y;
        return y;
    }

    double value() const { return last_; }

    void set_cutoffs(double low_hz, double high_hz) {
        for (auto& hp : hp_) hp.set_cutoff_hz(low_hz);
        for (auto& lp : lp_) lp.set_cutoff_hz(high_hz);
    }

    void set_sample_dt(double dt) {
        dt_ = dt;
        for (auto& hp : hp_) hp.set_sample_dt(dt);
        for (auto& lp : lp_) lp.set_sample_dt(dt);
    }

    void reset() {
        for (auto& hp : hp_) hp.reset();
        for (auto& lp : lp_) lp.reset();
        last_ = 0.0;
    }

private:
    double dt_;
    std::vector<HighPassFilter> hp_;
    std::vector<LowPassFilter> lp_;
    double last_{0.0};
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_FILTER_BANDPASS_H
