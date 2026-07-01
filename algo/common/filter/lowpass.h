// algo/common/filter/lowpass.h — first-order IIR low-pass filter.
//
// Inspired by jAER LowPassFilter. Discrete one-pole low-pass (forward Euler,
// matching jAER): fac = dt / tau clamped to [0, 1], then
//   y[n] = y[n-1] + fac * (x[n] - y[n-1]) = fac * x[n] + (1 - fac) * y[n-1]
// where tau = RC = 1 / (2π fc). Used to smooth event-rate signals, latency
// probes, and tracker velocities. Header-only.

#ifndef GUI_ALGO_COMMON_FILTER_LOWPASS_H
#define GUI_ALGO_COMMON_FILTER_LOWPASS_H

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gui_algo {

/// @brief First-order IIR low-pass filter.
class LowPassFilter {
public:
    /// @brief Constructs the filter from a cutoff frequency and sample period.
    /// @param cutoff_hz Cutoff frequency (-3 dB point) in Hz.
    /// @param sample_dt Time between samples in seconds.
    LowPassFilter(double cutoff_hz = 10.0, double sample_dt = 0.033)
        : dt_(sample_dt), rc_(1.0 / (2.0 * M_PI * cutoff_hz)), y_(0.0), init_(false) {}

    /// @brief Constructs the filter directly from a smoothing coefficient.
    /// alpha = 1 → no filtering; alpha → 0 → heavy smoothing.
    static LowPassFilter from_alpha(double alpha) {
        LowPassFilter f(10.0, 0.033);
        f.alpha_ = alpha;
        f.use_alpha_ = true;
        return f;
    }

    /// @brief Filters a new sample and returns the smoothed output.
    double process(double x) {
        if (!init_) { y_ = x; init_ = true; return y_; }
        const double a = use_alpha_ ? alpha_ : compute_alpha();
        y_ = a * x + (1.0 - a) * y_;
        return y_;
    }

    double value() const { return y_; }
    bool initialized() const { return init_; }

    void set_cutoff_hz(double fc) { rc_ = 1.0 / (2.0 * M_PI * fc); use_alpha_ = false; }
    void set_sample_dt(double dt) { dt_ = dt; use_alpha_ = false; }

    void reset() { y_ = 0.0; init_ = false; }

private:
    double compute_alpha() const {
        // jAER forward Euler: fac = dt / tau clamped to [0, 1] (rc_ == tau).
        return std::clamp(dt_ / rc_, 0.0, 1.0);
    }

    double dt_;
    double rc_;
    double alpha_{0.0};       // used only when use_alpha_ is true
    bool use_alpha_{false};
    double y_;
    bool init_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_FILTER_LOWPASS_H
