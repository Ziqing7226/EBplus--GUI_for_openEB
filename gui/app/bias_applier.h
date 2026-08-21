// gui/app/bias_applier.h — hardware side of the AutoBias controller.
//
// Owns the I_LL_Biases interaction for auto_bias (§4.4.6 rework): locates
// the two writable sensitivity biases (names matched by substring
// "diff_on"/"diff_off", same convention as the calibration wizard's LCD
// noise-floor override), snapshots them on attach, applies integer deltas
// with clamping to the hardware range, and restores the snapshot on
// disable. apply() re-READS the current register value before each write,
// so a manual edit in the Biases panel silently becomes the new baseline —
// the controller never fights the user. Header-only.

#ifndef GUI_APP_BIAS_APPLIER_H
#define GUI_APP_BIAS_APPLIER_H

#include <algorithm>
#include <string>
#include <utility>

#include <metavision/hal/facilities/i_ll_biases.h>

namespace gui {

class BiasApplier {
public:
    enum class Status {
        Ok,        ///< Both deltas applied in full.
        Clamped,   ///< Applied, but at least one bias hit its range limit.
        NoBias,    ///< Sensor exposes no bias_diff_on/off — not attachable.
        Error,     ///< Facility call failed (device went away, …).
    };

    /// @brief Locates the diff biases, reads their ranges and snapshots the
    ///        current values. Returns false when the sensor does not expose
    ///        bias_diff_on/off (the caller should keep auto_bias inactive).
    bool attach(Metavision::I_LL_Biases* biases) {
        detach();
        if (!biases) return false;
        try {
            std::string name_on, name_off;
            int lo_on = 0, hi_on = 0, lo_off = 0, hi_off = 0;
            int cur_on = 0, cur_off = 0;
            for (const auto& [name, value] : biases->get_all_biases()) {
                if (name.find("diff_on") != std::string::npos) {
                    Metavision::LL_Bias_Info info;
                    if (!biases->get_bias_info(name, info)) continue;
                    const auto range = info.get_bias_range();
                    if (range.second <= range.first) continue;
                    name_on = name; lo_on = range.first; hi_on = range.second;
                    cur_on = value;
                } else if (name.find("diff_off") != std::string::npos) {
                    Metavision::LL_Bias_Info info;
                    if (!biases->get_bias_info(name, info)) continue;
                    const auto range = info.get_bias_range();
                    if (range.second <= range.first) continue;
                    name_off = name; lo_off = range.first; hi_off = range.second;
                    cur_off = value;
                }
            }
            if (name_on.empty() || name_off.empty()) return false;
            biases_ = biases;
            name_on_ = std::move(name_on);
            name_off_ = std::move(name_off);
            lo_on_ = lo_on; hi_on_ = hi_on;
            lo_off_ = lo_off; hi_off_ = hi_off;
            saved_on_ = cur_on;
            saved_off_ = cur_off;
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    bool attached() const { return biases_ != nullptr; }

    /// @brief Applies integer deltas: reads the CURRENT register values,
    ///        adds the deltas, clamps to the hardware range, writes back.
    Status apply(int delta_on, int delta_off) {
        if (!biases_) return Status::NoBias;
        try {
            const int cur_on = biases_->get(name_on_);
            const int cur_off = biases_->get(name_off_);
            const int target_on = std::clamp(cur_on + delta_on, lo_on_, hi_on_);
            const int target_off = std::clamp(cur_off + delta_off, lo_off_, hi_off_);
            if (target_on != cur_on) biases_->set(name_on_, target_on);
            if (target_off != cur_off) biases_->set(name_off_, target_off);
            const bool clamped =
                target_on != cur_on + delta_on || target_off != cur_off + delta_off;
            return clamped ? Status::Clamped : Status::Ok;
        } catch (const std::exception&) {
            return Status::Error;
        }
    }

    /// @brief Moves both diff biases one step (at most @p max_step units)
    ///        toward 0 — the factory default. Returns true when a register
    ///        actually changed (already at 0 → false, no write).
    bool home(int max_step) {
        if (!biases_) return false;
        try {
            const int cur_on = biases_->get(name_on_);
            const int cur_off = biases_->get(name_off_);
            const int tgt_on = step_toward(cur_on, max_step, lo_on_, hi_on_);
            const int tgt_off = step_toward(cur_off, max_step, lo_off_, hi_off_);
            if (tgt_on == cur_on && tgt_off == cur_off) return false;
            if (tgt_on != cur_on) biases_->set(name_on_, tgt_on);
            if (tgt_off != cur_off) biases_->set(name_off_, tgt_off);
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    /// @brief Writes the attach-time snapshot back. Returns false when the
    ///        facility calls fail (device already gone).
    bool restore() {
        if (!biases_) return false;
        try {
            biases_->set(name_on_, saved_on_);
            biases_->set(name_off_, saved_off_);
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    void detach() {
        biases_ = nullptr;
        name_on_.clear();
        name_off_.clear();
    }

private:
    /// One step from @p v toward 0, staying inside [lo, hi] (0 may sit
    /// outside the writable range — then we stop at the nearest limit).
    static int step_toward(int v, int max_step, int lo, int hi) {
        int target = v > 0 ? v - max_step : v + max_step;
        if (v > 0 && target < 0) target = 0;
        if (v < 0 && target > 0) target = 0;
        return std::clamp(target, lo, hi);
    }

    Metavision::I_LL_Biases* biases_{nullptr};
    std::string name_on_, name_off_;
    int lo_on_{0}, hi_on_{0}, lo_off_{0}, hi_off_{0};
    int saved_on_{0}, saved_off_{0};
};

} // namespace gui

#endif // GUI_APP_BIAS_APPLIER_H
