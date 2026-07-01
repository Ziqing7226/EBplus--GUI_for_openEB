// algo/common/lif_integrator.h — Leaky Integrate-and-Fire neuron integrator.
//
// Inspired by jAER BlurringTunnelFilter / LIFNeuron. Models a 2D grid of
// leaky integrate-and-fire neurons driven by events: each event adds +1.0 to
// the membrane potential of the corresponding pixel (regardless of polarity,
// per jAER BlurringTunnelFilter), which then leaks exponentially toward 0.
// When a pixel's potential crosses the firing threshold, it emits a spike
// (cluster) and resets. Used by ClusterLIF (4.3.18) and related clustering
// algorithms. Header-only.

#ifndef GUI_ALGO_COMMON_LIF_INTEGRATOR_H
#define GUI_ALGO_COMMON_LIF_INTEGRATOR_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

namespace gui_algo {

/// @brief 2D grid of leaky integrate-and-fire neurons driven by events.
class LifIntegrator {
public:
    /// @brief Constructs the neuron grid.
    /// @param width,height Grid (sensor) dimensions.
    /// @param tau_us Membrane time constant (leak). Larger = slower decay.
    /// @param threshold Firing threshold. Crossing it emits a spike + reset.
    /// @param reset_value Potential applied after a spike.
    /// @param decay_step Controls discrete leak granularity (see leak()).
    LifIntegrator(int width, int height,
                  Metavision::timestamp tau_us = 10000,
                  double threshold = 1.0,
                  double reset_value = 0.0,
                  Metavision::timestamp decay_step_us = 1000)
        : width_(width), height_(height), tau_us_(tau_us),
          threshold_(threshold), reset_value_(reset_value),
          decay_step_us_(decay_step_us),
          potential_(static_cast<std::size_t>(width) * height, reset_value),
          last_ts_(static_cast<std::size_t>(width) * height, -1) {}

    /// @brief Feeds an event. Returns true if this pixel fired on this event.
    bool add_event(std::uint16_t x, std::uint16_t y, short p,
                   Metavision::timestamp t) {
        if (x >= width_ || y >= height_) return false;
        const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
        // Apply leak since the last update at this pixel (decays toward 0).
        const auto prev = last_ts_[idx];
        last_ts_[idx] = t;
        if (prev >= 0 && t > prev) {
            const double dt = static_cast<double>(t - prev);
            potential_[idx] *= std::exp(-dt / static_cast<double>(tau_us_));
        }
        // Integrate: every event adds +1.0 regardless of polarity (per jAER
        // BlurringTunnelFilter, which feeds all neighbors uniformly).
        (void)p;
        potential_[idx] += 1.0;
        if (potential_[idx] >= threshold_) {
            potential_[idx] = reset_value_;
            return true;
        }
        return false;
    }

    /// @brief Globally leaks all neurons by @p dt_us. Call periodically to
    /// prevent stale potentials from accumulating on quiet pixels.
    void leak_global(Metavision::timestamp dt_us) {
        if (dt_us <= 0) return;
        const double decay = std::exp(-static_cast<double>(dt_us) /
                                      static_cast<double>(tau_us_));
        for (auto& v : potential_) {
            v *= decay;
        }
    }

    /// @brief Returns the membrane potential at (x, y).
    double potential(int x, int y) const {
        return potential_[static_cast<std::size_t>(y) * width_ + x];
    }

    /// @brief Returns a flat potential image (CV_32F-compatible layout).
    const std::vector<double>& potential_grid() const { return potential_; }

    void clear() {
        std::fill(potential_.begin(), potential_.end(), reset_value_);
        std::fill(last_ts_.begin(), last_ts_.end(),
                  static_cast<Metavision::timestamp>(-1));
    }

    int width() const { return width_; }
    int height() const { return height_; }
    double threshold() const { return threshold_; }
    Metavision::timestamp tau_us() const { return tau_us_; }

    void set_threshold(double t) { threshold_ = t; }
    void set_tau_us(Metavision::timestamp tau) { tau_us_ = tau; }

private:
    int width_;
    int height_;
    Metavision::timestamp tau_us_;
    double threshold_;
    double reset_value_;
    Metavision::timestamp decay_step_us_;
    std::vector<double> potential_;
    std::vector<Metavision::timestamp> last_ts_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_LIF_INTEGRATOR_H
