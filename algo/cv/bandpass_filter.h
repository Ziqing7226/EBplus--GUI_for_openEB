// algo/cv/bandpass_filter.h — IIR band-pass filter on event-rate signal.
//
// Implements design §4.3.22 (jAER LowPassFilter / HighPassFilter combination).
// Applies a cascaded IIR band-pass (high-pass at low_cutoff_hz to remove DC /
// slow drift, then low-pass at high_cutoff_hz to remove high-frequency noise)
// to the event-rate time series, extracting motion within the band
// [low_cutoff_hz, high_cutoff_hz] (e.g. rotating-machinery vibration).
//
// Reuses the cascaded HP+LP building blocks from algo/common/filter/bandpass.h
// (algo/common/filter/highpass.h and lowpass.h). Header-only.

#ifndef GUI_ALGO_CV_BANDPASS_FILTER_H
#define GUI_ALGO_CV_BANDPASS_FILTER_H

#include <cstddef>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/common/filter/highpass.h"
#include "algo/common/filter/lowpass.h"

namespace gui_algo {

/// @brief IIR band-pass filter applied to the event-rate time series.
class BandpassFilter {
public:
    BandpassFilter(float low_cutoff_hz = 1.0f,
                   float high_cutoff_hz = 10.0f,
                   int order = 1,
                   double sample_dt = 0.033) {
        const int n = order < 1 ? 1 : (order > 4 ? 4 : order);
        hp_.reserve(static_cast<std::size_t>(n));
        lp_.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            hp_.emplace_back(static_cast<double>(low_cutoff_hz), sample_dt);
            lp_.emplace_back(static_cast<double>(high_cutoff_hz), sample_dt);
        }
    }

    /// @brief Filters a scalar rate sample through the cascaded band-pass.
    /// jAER order: low-pass first, then high-pass (hp(lp(x))). Matches
    /// algo/common/filter/bandpass.h.
    double process(double x) {
        double y = x;
        for (auto& lp : lp_) y = lp.process(y);
        for (auto& hp : hp_) y = hp.process(y);
        value_ = y;
        return y;
    }

    /// @brief Feeds a batch of @p n_events ending at time @p t (us), computes
    /// the instantaneous event rate (events/second) and band-passes it.
    /// @return The filtered event-rate sample.
    double add_events(std::size_t n_events, Metavision::timestamp t) {
        if (n_events == 0) return value_;
        // 首批样本只初始化 last_t_ 后被丢弃（jAER 会用首样本初始化 LP；
        // 此处首样本不产生输出，§一-1.8）。
        if (last_t_ < 0) { last_t_ = t; return value_; }
        const auto dt = t - last_t_;
        last_t_ = t;
        // dt <= 0（同时间戳/时间回退）直接返回旧值，不更新滤波器状态。
        if (dt <= 0) return value_;
        const double instant =
            static_cast<double>(n_events) / (static_cast<double>(dt) * 1.0e-6);
        // Mirror jAER's per-update recomputation of the filter coefficient
        // from the actual inter-sample dt: push the real sample spacing into
        // every stage before processing, instead of relying on the fixed
        // constructor-time sample_dt.
        const double dt_s = static_cast<double>(dt) * 1.0e-6;
        for (auto& hp : hp_) hp.set_sample_dt(dt_s);
        for (auto& lp : lp_) lp.set_sample_dt(dt_s);
        value_ = process(instant);
        return value_;
    }

    /// @brief Convenience: feeds a whole event packet (counts events per batch).
    double add_events(const EventPacket& packet, Metavision::timestamp t) {
        return add_events(packet.size(), t);
    }

    double value() const { return value_; }

    void set_cutoffs(double low_hz, double high_hz) {
        for (auto& hp : hp_) hp.set_cutoff_hz(low_hz);
        for (auto& lp : lp_) lp.set_cutoff_hz(high_hz);
    }

    void set_sample_dt(double dt) {
        for (auto& hp : hp_) hp.set_sample_dt(dt);
        for (auto& lp : lp_) lp.set_sample_dt(dt);
    }

    void reset() {
        for (auto& hp : hp_) hp.reset();
        for (auto& lp : lp_) lp.reset();
        value_ = 0.0;
        last_t_ = -1;
    }

private:
    std::vector<HighPassFilter> hp_;
    std::vector<LowPassFilter> lp_;
    double value_{0.0};
    Metavision::timestamp last_t_{-1};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_BANDPASS_FILTER_H
