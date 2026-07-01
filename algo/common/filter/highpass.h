// algo/common/filter/highpass.h — first-order IIR high-pass filter.
//
// Inspired by jAER HighPassFilter, which implements the high-pass as
// x - LP(x) with LP using forward Euler (fac = dt/tau clamped to [0, 1]).
// The equivalent direct form is
//   y[n] = (1 - fac) * (y[n-1] + x[n] - x[n-1])
// with fac = clamp(dt / tau, 0, 1), tau = RC = 1 / (2π fc). Used to detrend
// event-rate signals and detect transients. Header-only.

#ifndef GUI_ALGO_COMMON_FILTER_HIGHPASS_H
#define GUI_ALGO_COMMON_FILTER_HIGHPASS_H

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gui_algo {

/// @brief First-order IIR high-pass filter.
class HighPassFilter {
public:
    /// @brief Constructs the filter from a cutoff frequency and sample period.
    /// @param cutoff_hz Cutoff frequency (-3 dB point) in Hz.
    /// @param sample_dt Time between samples in seconds.
    HighPassFilter(double cutoff_hz = 10.0, double sample_dt = 0.033)
        : dt_(sample_dt), rc_(1.0 / (2.0 * M_PI * cutoff_hz)),
          prev_x_(0.0), y_(0.0), init_(false) {}

    /// @brief Filters a new sample and returns the high-pass output.
    double process(double x) {
        if (!init_) { prev_x_ = x; y_ = 0.0; init_ = true; return y_; }
        // HP = x - LP(x); direct-form coefficient is 1 - fac (fac = dt/tau).
        const double alpha = 1.0 - std::clamp(dt_ / rc_, 0.0, 1.0);
        y_ = alpha * (y_ + x - prev_x_);
        prev_x_ = x;
        return y_;
    }

    double value() const { return y_; }
    bool initialized() const { return init_; }

    void set_cutoff_hz(double fc) { rc_ = 1.0 / (2.0 * M_PI * fc); }
    void set_sample_dt(double dt) { dt_ = dt; }

    void reset() { prev_x_ = 0.0; y_ = 0.0; init_ = false; }

private:
    double dt_;
    double rc_;
    double prev_x_;
    double y_;
    bool init_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_FILTER_HIGHPASS_H
