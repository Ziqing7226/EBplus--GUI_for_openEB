// algo/common/filter/angular_lowpass.h — angular low-pass filter (circular).
//
// Inspired by jAER AngularLowpassFilter. Low-pass filter for angular quantities
// that wrap around at a period (e.g. 2π for motion direction, orientation).
// Computes the shortest signed angular distance from the current filtered value
// to the new sample (crossing the 0/period cut when shorter) and applies a
// time-based IIR update with fac = dt/tau clamped to [0, 1], then wraps the
// result into [0, period). Header-only.

#ifndef GUI_ALGO_COMMON_FILTER_ANGULAR_LOWPASS_H
#define GUI_ALGO_COMMON_FILTER_ANGULAR_LOWPASS_H

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gui_algo {

/// @brief Low-pass filter for angular quantities with wrap-around.
class AngularLowpassFilter {
public:
    /// @brief Constructs the filter.
    /// @param period Circular period (e.g. 2π for radians, 360 for degrees).
    /// @param tau Time constant in seconds (same units as the timestamps
    ///            passed to process()). fac = dt/tau clamped to [0, 1].
    AngularLowpassFilter(double period = 2.0 * M_PI, double tau = 0.1)
        : tau_(tau), period_(period), period_by2_(period / 2.0),
          lp_val_(0.0), last_time_(0.0), init_(false) {}

    /// @brief Filters a new angle and returns the smoothed angle in [0, period).
    /// @param val New sample (any real; wrapped into [0, period)).
    /// @param t Timestamp in seconds (same units as tau).
    double process(double val, double t) {
        if (!init_) {
            lp_val_ = wrap(val);
            last_time_ = t;
            init_ = true;
            return lp_val_;
        }
        double dt = t - last_time_;
        if (dt < 0.0) dt = 0.0;
        last_time_ = t;
        const double fac = std::clamp(dt / tau_, 0.0, 1.0);
        const double d = angular_distance(lp_val_, val);
        lp_val_ += fac * d;
        // Wrap into [0, period) (one step suffices: |fac*d| <= period/2).
        if (lp_val_ > period_) lp_val_ -= period_;
        if (lp_val_ < 0.0) lp_val_ += period_;
        return lp_val_;
    }

    /// @brief Returns the current smoothed angle in [0, period).
    double value() const { return lp_val_; }

    bool initialized() const { return init_; }

    void set_tau(double tau) { tau_ = tau; }
    void set_period(double period) {
        period_ = period;
        period_by2_ = period / 2.0;
    }

    double tau() const { return tau_; }
    double period() const { return period_; }

    void reset() { lp_val_ = 0.0; last_time_ = 0.0; init_ = false; }

private:
    /// @brief Shortest signed distance from v1 to v2 across the 0/period cut.
    /// Matches jAER AngularLowpassFilter.angularDistance.
    double angular_distance(double v1, double v2) const {
        const double d = v2 - v1;
        if (d > -period_by2_ && d <= period_by2_) return d;
        else if (d > 0.0) return d - period_;
        else return d + period_;
    }

    /// @brief Wraps v into [0, period).
    double wrap(double v) const {
        v = std::fmod(v, period_);
        if (v < 0.0) v += period_;
        return v;
    }

    double tau_;
    double period_;
    double period_by2_;
    double lp_val_;
    double last_time_;
    bool init_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_FILTER_ANGULAR_LOWPASS_H
