// algo/cv/noise_filter.h — multi-mode event noise filter.
//
// Event noise filter (design §4.3.5) porting the jAER filter suite
// (net/sf/jaer/eventprocessing/filter/). Provides 8 denoising modes:
//   BAF          — Background Activity Filter (Delbruck 2008): 3x3 neighbour
//                  correlation within dt_us.
//   STCF         — SpatioTemporal Correlation Filter (Guo & Delbruck 2022):
//                  >= min_neighbors correlated neighbours in 3x3 within
//                  correlation_time_s, optional polarity match / coincidence.
//   Refractory   — per-pixel refractory period (ISI threshold).
//   DWF          — ✅ 移植自 jAER DoubleWindowFilter (Guo & Delbruck,
//                  T-PAMI 2022): dual global FIFO rings of recent event
//                  positions; passes when enough prior entries lie within
//                  an L1 distance threshold (disThr).
//   AgePolarity  — ✅ 移植自 jAER AgePolarityDenoiser (Delbruck 2025):
//                  sums neighbour age scores (1 - dt/tau) over a square
//                  neighbourhood; passes when the score reaches threshold.
//   Harmonic     — ✅ 移植自 jAER HarmonicFilter: a global driven damped
//                  oscillator resonant at the mains frequency filters out
//                  events near zero crossings of the oscillator.
//   Repetitious  — drops events recurring at a learned per-pixel ISI
//                  (port of jAER RepetitiousFilter).
//   SpatialBP    — center/surround time-surface differencing (port of jAER
//                  SpatialBandpassFilter: surround-timestamp splat).
// Two common options link to the rest of the pipeline: filter_hot_pixels
// (internal n_sigma hot-pixel suppression, see §4.3.6) and
// adaptive_correlation_time (scales the active time threshold by the local
// event rate, jAER NoiseFilterControl). Uses a 2D most-recent-timestamp map
// (per-polarity + any-polarity) for spatial correlation. Header-only.

#ifndef GUI_ALGO_CV_NOISE_FILTER_H
#define GUI_ALGO_CV_NOISE_FILTER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Multi-mode event denoiser operating on a streaming event packet.
class NoiseFilter {
public:
    enum class Mode {
        BAF,            ///< Background activity filter
        STCF,           ///< Spatio-temporal correlation filter
        Refractory,     ///< Per-pixel refractory period
        DWF,            ///< ✅ 移植自 jAER DoubleWindowFilter
        AgePolarity,    ///< ✅ 移植自 jAER AgePolarityDenoiser
        Harmonic,       ///< ✅ 移植自 jAER HarmonicFilter
        Repetitious,    ///< Periodic repetition filter (jAER port)
        SpatialBP,      ///< Spatial band-pass (jAER port)
    };

    enum class LineFreq { Hz50, Hz60 };

    /// @brief Sentinel value marking an uninitialized timestamp slot.
    static constexpr Metavision::timestamp kSentinel =
        std::numeric_limits<Metavision::timestamp>::min();

    NoiseFilter(int width, int height, Mode mode = Mode::STCF)
        : width_(width), height_(height), mode_(mode),
          last_any_(static_cast<std::size_t>(width) * height, kSentinel),
          last_pol_sign_(static_cast<std::size_t>(width) * height, 0),
          surround_ts_(static_cast<std::size_t>(width) * height, kSentinel),
          hp_counts_(static_cast<std::size_t>(width) * height, 0) {
        last_pol_[0].assign(static_cast<std::size_t>(width) * height, kSentinel);
        last_pol_[1].assign(static_cast<std::size_t>(width) * height, kSentinel);
        for (int p = 0; p < 2; ++p) {
            rep_last_times_[p].assign(static_cast<std::size_t>(width) * height,
                                      std::array<Metavision::timestamp, 2>{kSentinel, kSentinel});
            rep_avg_dt_[p].assign(static_cast<std::size_t>(width) * height, 0.0);
        }
        dwf_init();
        harm_recompute();
    }

    // Mode + common options ------------------------------------------------
    void set_mode(Mode m) { mode_ = m; }
    Mode mode() const { return mode_; }
    void set_filter_hot_pixels(bool v) { filter_hot_pixels_ = v; }
    void set_adaptive_correlation_time(bool v) { adaptive_correlation_time_ = v; }
    bool filter_hot_pixels() const { return filter_hot_pixels_; }
    bool adaptive_correlation_time() const { return adaptive_correlation_time_; }

    // BAF ------------------------------------------------------------------
    void set_baf_dt_us(int v) { baf_dt_us_ = clamp_i(v, 1000, 100000); }
    void set_baf_subsample_by(int v) { baf_subsample_by_ = clamp_i(v, 0, 4); }
    int baf_dt_us() const { return baf_dt_us_; }
    int baf_subsample_by() const { return baf_subsample_by_; }

    // STCF -----------------------------------------------------------------
    void set_correlation_time_s(double v) { stcf_corr_s_ = clamp_d(v, 0.001, 0.1); }
    void set_min_neighbors(int v) { stcf_min_neighbors_ = clamp_i(v, 1, 8); }
    void set_require_polarity_match(bool v) { stcf_require_polarity_match_ = v; }
    void set_allow_coincidence(bool v) { stcf_allow_coincidence_ = v; }
    double correlation_time_s() const { return stcf_corr_s_; }
    int min_neighbors() const { return stcf_min_neighbors_; }
    bool require_polarity_match() const { return stcf_require_polarity_match_; }
    bool allow_coincidence() const { return stcf_allow_coincidence_; }

    // Refractory -----------------------------------------------------------
    void set_refractory_period_us(int v) { refractory_period_us_ = clamp_i(v, 100, 100000); }
    int refractory_period_us() const { return refractory_period_us_; }

    // DWF (✅ 移植自 jAER DoubleWindowFilter) -------------------------------
    void set_dwf_window_length(int v) {
        const int nv = clamp_i(v, 2, 4096);
        if (nv != dwf_wlen_) { dwf_wlen_ = nv; dwf_init(); }
    }
    void set_dwf_dist_threshold(int v) { dwf_dis_thr_ = clamp_i(v, 1, 1024); }
    void set_dwf_min_correlated(int v) { dwf_min_corr_ = clamp_i(v, 1, 8); }
    void set_dwf_double_mode(bool v) { dwf_double_mode_ = v; }
    int dwf_window_length() const { return dwf_wlen_; }
    int dwf_dist_threshold() const { return dwf_dis_thr_; }
    int dwf_min_correlated() const { return dwf_min_corr_; }
    bool dwf_double_mode() const { return dwf_double_mode_; }

    // AgePolarity (✅ 移植自 jAER AgePolarityDenoiser) ----------------------
    void set_tau_us(int v) { agep_tau_us_ = clamp_i(v, 1000, 100000); }
    void set_age_threshold(double v) { agep_threshold_ = clamp_d(v, 0.0, 8.0); }
    void set_agep_radius(int v) { agep_radius_ = clamp_i(v, 1, 5); }
    int tau_us() const { return agep_tau_us_; }
    double age_threshold() const { return agep_threshold_; }
    int agep_radius() const { return agep_radius_; }

    // Harmonic (✅ 移植自 jAER HarmonicFilter) ------------------------------
    void set_line_freq(LineFreq f) {
        harm_line_freq_hz_ = (f == LineFreq::Hz50) ? 50 : 60;
        harm_recompute();
    }
    void set_notch_q(double v) {
        harm_notch_q_ = clamp_d(v, 1.0, 50.0);
        harm_recompute();
    }
    void set_harmonic_threshold(double v) { harm_threshold_ = clamp_d(v, 0.0, 1.0); }
    int line_freq_hz() const { return harm_line_freq_hz_; }
    double notch_q() const { return harm_notch_q_; }
    double harmonic_threshold() const { return harm_threshold_; }

    // Repetitious (jAER port) ----------------------------------------------
    void set_period_us(int v) { rep_period_us_ = clamp_i(v, 1000, 1000000); }
    void set_tolerance_us(int v) { rep_tolerance_us_ = clamp_i(v, 100, 10000); }
    int period_us() const { return rep_period_us_; }
    int tolerance_us() const { return rep_tolerance_us_; }
    void set_ratio_shorter(int v) { rep_ratio_shorter_ = clamp_i(v, 1, 100); }
    void set_ratio_longer(int v) { rep_ratio_longer_ = clamp_i(v, 1, 100); }
    int ratio_shorter() const { return rep_ratio_shorter_; }
    int ratio_longer() const { return rep_ratio_longer_; }

    // SpatialBP (jAER port) ------------------------------------------------
    void set_center_radius_px(int v) { sbp_center_ = clamp_i(v, 1, 10); }
    void set_surround_radius_px(int v) { sbp_surround_ = clamp_i(v, 5, 30); }
    void set_dt_surround_us(int v) { sbp_dt_surround_us_ = clamp_i(v, 100, 1000000); }
    int center_radius_px() const { return sbp_center_; }
    int surround_radius_px() const { return sbp_surround_; }
    int dt_surround_us() const { return sbp_dt_surround_us_; }

    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Processes an event packet, updating internal correlation maps.
    /// @return Number of events that PASSED the filter (the filtered stream).
    std::size_t process(EventPacket& events) {
        std::size_t kept = 0;
        std::size_t total = 0;
        for (const auto& e : events) {
            if (decide_and_update(e)) ++kept;
            ++total;
        }
        last_total_ = total;
        last_kept_ = kept;
        return kept;
    }

    /// @brief Filters a mutable packet in place (compaction). Returns the new
    ///        (kept) event count.
    std::size_t filter(MutableEventPacket& events) {
        std::size_t out = 0;
        for (std::size_t i = 0; i < events.size(); ++i) {
            if (decide_and_update(events[i])) {
                events[out] = events[i];
                ++out;
            }
        }
        last_total_ = events.size();
        last_kept_ = out;
        return out;
    }

    /// @brief Per-event decision for a raw pointer buffer; returns the kept
    ///        count and compacts in place.
    std::size_t filter(Event* events, std::size_t count) {
        std::size_t out = 0;
        for (std::size_t i = 0; i < count; ++i) {
            if (decide_and_update(events[i])) {
                events[out] = events[i];
                ++out;
            }
        }
        last_total_ = count;
        last_kept_ = out;
        return out;
    }

    std::size_t last_total() const { return last_total_; }
    std::size_t last_kept() const { return last_kept_; }
    std::size_t last_filtered() const {
        return last_total_ >= last_kept_ ? last_total_ - last_kept_ : 0;
    }
    /// @brief Fraction of events removed in the last process/filter call [0,1].
    double filter_rate() const {
        return last_total_ > 0
            ? static_cast<double>(last_filtered()) / static_cast<double>(last_total_)
            : 0.0;
    }

    void reset() {
        std::fill(last_any_.begin(), last_any_.end(), kSentinel);
        std::fill(last_pol_[0].begin(), last_pol_[0].end(), kSentinel);
        std::fill(last_pol_[1].begin(), last_pol_[1].end(), kSentinel);
        std::fill(last_pol_sign_.begin(), last_pol_sign_.end(), 0);
        std::fill(surround_ts_.begin(), surround_ts_.end(), kSentinel);
        for (int p = 0; p < 2; ++p) {
            std::fill(rep_last_times_[p].begin(), rep_last_times_[p].end(),
                      std::array<Metavision::timestamp, 2>{kSentinel, kSentinel});
            std::fill(rep_avg_dt_[p].begin(), rep_avg_dt_[p].end(), 0.0);
        }
        std::fill(hp_counts_.begin(), hp_counts_.end(), 0);
        hp_start_ = kSentinel;
        hp_thresh_ = 0.0;
        rate_start_ = kSentinel;
        rate_events_ = 0;
        rate_eps_ = 0.0;
        scale_ = 1.0;
        last_total_ = 0;
        last_kept_ = 0;
        dwf_init();
        harm_reset_state();
        harm_recompute();
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static double clamp_d(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    std::size_t idx_of(int x, int y) const {
        return static_cast<std::size_t>(y) * width_ + x;
    }

    /// @brief Applies the adaptive-correlation-time scaling to a base threshold.
    Metavision::timestamp thr(Metavision::timestamp base_us) const {
        if (!adaptive_correlation_time_ || scale_ <= 0.0) return base_us;
        const double v = static_cast<double>(base_us) * scale_;
        if (v < 1.0) return 1;
        return static_cast<Metavision::timestamp>(v);
    }

    /// @brief Updates the event-rate estimate and the adaptive scale factor.
    void update_rate(Metavision::timestamp t) {
        ++rate_events_;
        if (rate_start_ == kSentinel) rate_start_ = t;
        const Metavision::timestamp span = t - rate_start_;
        if (span >= rate_window_us_) {
            const double eps = static_cast<double>(rate_events_) /
                               static_cast<double>(span); // events/us
            rate_eps_ = eps;
            const double ref = 0.05; // ~50 kEvents/s reference rate
            double s = (eps > 0.0) ? (ref / eps) : 1.0;
            if (s < 0.25) s = 0.25;
            if (s > 4.0) s = 4.0;
            scale_ = s;
            rate_start_ = t;
            rate_events_ = 0;
        }
    }

    /// @brief Core per-event decision; updates timestamp surfaces.
    bool decide_and_update(const Event& e) {
        update_rate(e.t);
        if (e.x >= width_ || e.y >= height_) return false; // OOB: drop
        bool pass = mode_pass(e);
        if (pass && filter_hot_pixels_ && !hot_ok(e)) pass = false;
        const std::size_t idx = idx_of(e.x, e.y);
        last_any_[idx] = e.t;
        last_pol_[e.p ? 1 : 0][idx] = e.t;
        last_pol_sign_[idx] = e.p ? 1 : -1;
        return pass;
    }

    bool mode_pass(const Event& e) {
        switch (mode_) {
            case Mode::BAF:         return baf_pass(e);
            case Mode::STCF:        return stcf_pass(e);
            case Mode::Refractory:  return refractory_pass(e);
            case Mode::DWF:         return dwf_pass(e);
            case Mode::AgePolarity: return age_polarity_pass(e);
            case Mode::Harmonic:    return harmonic_pass(e);
            case Mode::Repetitious: return repetitious_pass(e);
            case Mode::SpatialBP:   return spatialbp_pass(e);
        }
        return true; // unreachable
    }

    bool baf_pass(const Event& e) const {
        const Metavision::timestamp dt = thr(baf_dt_us_);
        const int step = 1 << baf_subsample_by_;
        for (int dy = -1; dy <= 1; dy += step) {
            const int ny = e.y + dy;
            if (ny < 0 || ny >= height_) continue;
            for (int dx = -1; dx <= 1; dx += step) {
                if (dx == 0 && dy == 0) continue;
                const int nx = e.x + dx;
                if (nx < 0 || nx >= width_) continue;
                const Metavision::timestamp lt = last_any_[idx_of(nx, ny)];
                if (lt == kSentinel) continue;
                const Metavision::timestamp diff = e.t - lt;
                if (diff >= 0 && diff < dt) return true; // strict < dt
            }
        }
        return false;
    }

    bool stcf_pass(const Event& e) const {
        const Metavision::timestamp dt =
            thr(static_cast<Metavision::timestamp>(stcf_corr_s_ * 1e6));
        const int cur_sign = e.p ? 1 : -1;
        int count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            const int ny = e.y + dy;
            if (ny < 0 || ny >= height_) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const int nx = e.x + dx;
                if (nx < 0 || nx >= width_) continue;
                const std::size_t nidx = idx_of(nx, ny);
                const Metavision::timestamp lt = last_any_[nidx];
                if (lt == kSentinel) continue;
                const Metavision::timestamp diff = e.t - lt;
                if (diff < 0 || diff >= dt) continue; // strict < dt (jAER deltaT < dt)
                if (diff == 0 && !stcf_allow_coincidence_) continue; // coincidence
                // M31: compare current polarity to the polarity of the LAST
                // event (any polarity) recorded at the neighbour.
                if (stcf_require_polarity_match_ &&
                    last_pol_sign_[nidx] != cur_sign) continue;
                ++count;
            }
        }
        return count >= stcf_min_neighbors_;
    }

    bool refractory_pass(const Event& e) const {
        const Metavision::timestamp lt = last_any_[idx_of(e.x, e.y)];
        if (lt == kSentinel || e.t < lt) return true;
        return (e.t - lt) > thr(refractory_period_us_); // strict > refractory
    }

    // ✅ 移植自 jAER DoubleWindowFilter (Guo & Delbruck, T-PAMI 2022).
    // Two global FIFO rings of recent event (x,y) positions (signal + noise).
    // An event passes if enough prior FIFO entries lie within an L1 distance
    // (disThr) of it. Signal events push to the signal ring; noise events
    // push to the noise ring. The first half-window of events is filtered
    // out to seed the signal ring.
    bool dwf_pass(const Event& e) {
        if (!dwf_signal_.full()) {
            dwf_signal_.push(static_cast<short>(e.x), static_cast<short>(e.y));
            return false; // fill phase: filter out, seed signal ring
        }
        const short ex = static_cast<short>(e.x);
        const short ey = static_cast<short>(e.y);
        int ncorrelated = 0;
        const auto count_match = [&](const DwRing& r) {
            for (std::size_t i = 0; i < r.count; ++i) {
                const std::array<short, 2>& v = r.buf[(r.head + i) % r.cap];
                const int dx = static_cast<int>(ex) - static_cast<int>(v[0]);
                const int dy = static_cast<int>(ey) - static_cast<int>(v[1]);
                const int d = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
                if (d < dwf_dis_thr_) {
                    ++ncorrelated;
                    if (ncorrelated >= dwf_min_corr_) return;
                }
            }
        };
        count_match(dwf_signal_);
        if (ncorrelated < dwf_min_corr_ && dwf_double_mode_) {
            count_match(dwf_noise_);
        }
        if (ncorrelated >= dwf_min_corr_) {
            dwf_signal_.push(ex, ey);
            return true;
        }
        if (dwf_double_mode_) dwf_noise_.push(ex, ey);
        return false;
    }

    // ✅ 移植自 jAER AgePolarityDenoiser (Delbruck 2025). Scores each
    // neighbour by its relative age age = 1 - dt/tau (clipped to [0,1]) and
    // sums the scores over a (2r+1)^2 neighbourhood (same-polarity only when
    // require_polarity_match is set). Passes when the summed score reaches
    // the correlation threshold.
    bool age_polarity_pass(const Event& e) const {
        const double tau = static_cast<double>(thr(agep_tau_us_));
        if (tau <= 0.0) return true;
        const int cur_sign = e.p ? 1 : -1;
        const int r = agep_radius_;
        double score = 0.0;
        for (int dy = -r; dy <= r; ++dy) {
            const int ny = e.y + dy;
            if (ny < 0 || ny >= height_) continue;
            for (int dx = -r; dx <= r; ++dx) {
                const int nx = e.x + dx;
                if (nx < 0 || nx >= width_) continue;
                if (filter_hot_pixels_ && dx == 0 && dy == 0) continue;
                const std::size_t nidx = idx_of(nx, ny);
                const Metavision::timestamp lt = last_any_[nidx];
                if (lt == kSentinel) continue;
                const double dt = static_cast<double>(e.t - lt);
                if (dt <= 0.0 || dt >= tau) continue;
                if (stcf_require_polarity_match_ && last_pol_sign_[nidx] != cur_sign) continue;
                score += 1.0 - dt / tau;
            }
        }
        return score >= agep_threshold_;
    }

    // ✅ 移植自 jAER HarmonicFilter. A global driven damped harmonic
    // oscillator (resonant at the mains frequency) is kicked by each event:
    // ON events push velocity +1, OFF events push velocity -1. The
    // oscillator's lowpassed power (x^2) estimates the flicker amplitude;
    // events near a zero crossing (x^2/power < threshold) are filtered out,
    // events near a peak pass through.
    bool harmonic_pass(const Event& e) {
        harm_update(e.t, static_cast<int>(e.p));
        if (harm_power_ <= 0.0f) return true; // no amplitude estimate yet
        const float ratio = (harm_x_ * harm_x_) / harm_power_;
        return ratio >= static_cast<float>(harm_threshold_);
    }

    // Port of jAER RepetitiousFilter: per-(x,y,polarity) last-two timestamps
    // and IIR-smoothed average ISI. An event is repetitious when its ISI is
    // within [avgDt/ratio_longer, avgDt*ratio_shorter] of the learned ISI.
    bool repetitious_pass(const Event& e) {
        const int pol = e.p ? 1 : 0;
        const std::size_t idx = idx_of(e.x, e.y);
        std::array<Metavision::timestamp, 2>& last2 = rep_last_times_[pol][idx];
        double& avg_dt = rep_avg_dt_[pol][idx];
        const Metavision::timestamp lastt = last2[1];
        if (lastt == kSentinel) {
            last2[0] = e.t;
            last2[1] = e.t;
            avg_dt = 0.0;
            return true;
        }
        const double thisdt = static_cast<double>(e.t - lastt);
        const double avg = avg_dt;
        bool repetitious = false;
        if (avg > 0.0) {
            repetitious = (thisdt > avg / rep_ratio_longer_) &&
                          (thisdt < avg * rep_ratio_shorter_);
        }
        if (thisdt < 0.0) {
            // Non-monotonic / wrap: reset the running estimate.
            last2[0] = e.t;
            last2[1] = e.t;
            avg_dt = 0.0;
        } else {
            const double alpha =
                1.0 / static_cast<double>(rep_averaging_samples_);
            last2[0] = lastt;
            last2[1] = e.t;
            avg_dt = avg * (1.0 - alpha) + thisdt * alpha;
        }
        // passRepetitiousEvents == false: drop repetitious, pass the rest.
        return !repetitious;
    }

    // Port of jAER SpatialBandpassFilter: maintain a dedicated surround
    // timestamp map; pass when (e.t - surroundTs[x][y]) > dt_surround_us,
    // then splat e.t to the square surround ring on every event.
    bool spatialbp_pass(const Event& e) {
        const std::size_t idx = idx_of(e.x, e.y);
        const Metavision::timestamp st = surround_ts_[idx];
        const bool pass = (st == kSentinel) ||
                          (e.t - st > thr(sbp_dt_surround_us_));
        const int sr = sbp_surround_;
        const int cr = sbp_center_;
        for (int dy = -sr; dy <= sr; ++dy) {
            const int ny = e.y + dy;
            if (ny < 0 || ny >= height_) continue;
            for (int dx = -sr; dx <= sr; ++dx) {
                if (dx == 0 && dy == 0) continue;
                // Skip the square center region; splat only the surround ring.
                if (dx >= -cr && dx <= cr && dy >= -cr && dy <= cr) continue;
                const int nx = e.x + dx;
                if (nx < 0 || nx >= width_) continue;
                surround_ts_[idx_of(nx, ny)] = e.t;
            }
        }
        return pass;
    }

    // Internal hot-pixel suppression (links to §4.3.6) --------------------
    bool hot_ok(const Event& e) {
        const std::size_t idx = idx_of(e.x, e.y);
        ++hp_counts_[idx];
        if (hp_start_ == kSentinel) hp_start_ = e.t;
        if (e.t - hp_start_ >= hp_window_us_) {
            recompute_hp_threshold();
            hp_start_ = e.t;
        }
        return static_cast<double>(hp_counts_[idx]) <= hp_thresh_;
    }

    void recompute_hp_threshold() {
        const double n = static_cast<double>(hp_counts_.size());
        double sum = 0.0;
        for (auto c : hp_counts_) sum += static_cast<double>(c);
        const double mean = n > 0.0 ? sum / n : 0.0;
        double var = 0.0;
        if (n > 0.0) {
            for (auto c : hp_counts_) {
                const double d = static_cast<double>(c) - mean;
                var += d * d;
            }
            var /= n;
        }
        hp_thresh_ = mean + 4.0 * std::sqrt(var);
        std::fill(hp_counts_.begin(), hp_counts_.end(), 0);
    }

    // DWF (jAER DoubleWindowFilter) state helpers -------------------------
    /// Fixed-capacity FIFO ring of recent event (x,y) positions.
    struct DwRing {
        std::vector<std::array<short, 2>> buf;
        std::size_t head{0};
        std::size_t count{0};
        std::size_t cap{0};
        void init(std::size_t c) {
            cap = c;
            buf.assign(c > 0 ? c : 1, std::array<short, 2>{-1, -1});
            head = 0;
            count = 0;
        }
        bool full() const { return count >= cap; }
        void push(short x, short y) {
            if (cap == 0) return;
            if (count < cap) {
                buf[(head + count) % cap] = {{x, y}};
                ++count;
            } else {
                buf[head] = {{x, y}};
                head = (head + 1) % cap;
            }
        }
    };

    void dwf_init() {
        const int dwlen = dwf_wlen_ > 1 ? dwf_wlen_ / 2 : 1;
        const std::size_t c = static_cast<std::size_t>(dwlen);
        dwf_signal_.init(c);
        dwf_noise_.init(c);
    }

    // Harmonic (jAER HarmonicFilter) oscillator helpers -------------------
    void harm_recompute() {
        const double f0 = static_cast<double>(harm_line_freq_hz_);
        constexpr double kTwoPi = 6.28318530717958647693;
        const double omega = (f0 > 0.0) ? kTwoPi * f0 : 1.0;
        const double tau = (f0 > 0.0) ? 1.0 / omega : 1.0;
        const double q = (harm_notch_q_ > 0.0) ? harm_notch_q_ : 1.0;
        harm_tauoverq_ = tau / q;
        harm_reciptausq_ = 1.0 / (tau * tau);
        harm_dtlim_ = tau / 20.0; // GEARRATIO
    }

    void harm_reset_state() {
        harm_x_ = 0.0f;
        harm_y_ = 0.0f;
        harm_power_ = 0.0f;
        harm_was_reset_ = true;
    }

    void harm_update(Metavision::timestamp ts, int p) {
        if (harm_was_reset_) {
            harm_t_ = ts;
            harm_was_reset_ = false;
            return;
        }
        harm_y_ += static_cast<float>(2 * p - 1);
        const double dt = 1e-6 * static_cast<double>(ts - harm_t_);
        if (dt < 0.0) { harm_reset_state(); return; }
        int nsteps = (harm_dtlim_ > 0.0)
            ? static_cast<int>(std::ceil(dt / harm_dtlim_)) : 1;
        if (nsteps < 1) nsteps = 1;
        if (nsteps > 100000) nsteps = 100000; // bound the integration loop
        const double ddt = (dt / static_cast<double>(nsteps)) * harm_reciptausq_;
        const double ddt2 = dt / static_cast<double>(nsteps);
        double x = harm_x_;
        double y = harm_y_;
        for (int i = 0; i < nsteps; ++i) {
            y -= ddt * (harm_tauoverq_ * y + x);
            x += ddt2 * y;
        }
        harm_x_ = static_cast<float>(x);
        harm_y_ = static_cast<float>(y);
        const double sq = x * x;
        double alpha = (dt * static_cast<double>(harm_line_freq_hz_)) / 10.0;
        if (alpha < 0.0) alpha = 0.0;
        if (alpha > 1.0) alpha = 1.0;
        harm_power_ = static_cast<float>(harm_power_ * (1.0 - alpha) + sq * alpha);
        if (std::isnan(harm_x_) || std::isnan(harm_y_) || std::isnan(harm_power_)) {
            harm_reset_state();
            return;
        }
        harm_t_ = ts;
    }

    int width_;
    int height_;
    Mode mode_;

    // Mode parameters (defaults per design §4.3.5) ------------------------
    Metavision::timestamp baf_dt_us_{25000};
    int baf_subsample_by_{0};
    double stcf_corr_s_{0.025};
    int stcf_min_neighbors_{2};
    bool stcf_require_polarity_match_{true};
    bool stcf_allow_coincidence_{true}; // jAER: no coincidence exclusion
    Metavision::timestamp refractory_period_us_{1000};
    int dwf_wlen_{512};           // jAER wLen (total window length, events)
    int dwf_dis_thr_{5};          // jAER disThr (L1 distance threshold)
    int dwf_min_corr_{1};         // jAER numMustBeCorrelated
    bool dwf_double_mode_{true};  // jAER useDoubleMode
    Metavision::timestamp agep_tau_us_{10000};
    double agep_threshold_{5.0};  // jAER correlationThreshold (default 5, max 8)
    int agep_radius_{1};          // neighbourhood radius (1 = 3x3)
    int harm_line_freq_hz_{50};   // jAER f0 (natural frequency, Hz)
    double harm_notch_q_{3.0};    // jAER quality factor Q
    double harm_threshold_{0.1};  // jAER threshold (fraction of power)
    Metavision::timestamp rep_period_us_{20000};
    Metavision::timestamp rep_tolerance_us_{1000};
    int rep_ratio_shorter_{2};   // jAER RepetitiousFilter default
    int rep_ratio_longer_{2};    // jAER RepetitiousFilter default
    int rep_averaging_samples_{3}; // jAER averagingSamples default
    int sbp_center_{2};
    int sbp_surround_{10};
    Metavision::timestamp sbp_dt_surround_us_{8000}; // jAER dtSurround default

    bool filter_hot_pixels_{false};
    bool adaptive_correlation_time_{false};

    // Timestamp surfaces ---------------------------------------------------
    std::vector<Metavision::timestamp> last_any_;
    std::array<std::vector<Metavision::timestamp>, 2> last_pol_;
    std::vector<std::int8_t> last_pol_sign_; ///< Sign (+1/-1/0) of last event at each pixel (STCF M31).
    std::vector<Metavision::timestamp> surround_ts_; ///< Dedicated surround-timestamp map (SpatialBP M30).

    // Repetitious state (M29) ----------------------------------------------
    std::array<std::vector<std::array<Metavision::timestamp, 2>>, 2> rep_last_times_;
    std::array<std::vector<double>, 2> rep_avg_dt_;

    // DWF state (jAER DoubleWindowFilter) ---------------------------------
    DwRing dwf_signal_;
    DwRing dwf_noise_;

    // Harmonic oscillator state (jAER HarmonicFilter) ---------------------
    float harm_x_{0.0f};          // oscillator position
    float harm_y_{0.0f};          // oscillator velocity
    float harm_power_{0.0f};      // lowpassed x^2 (amplitude^2 estimate)
    Metavision::timestamp harm_t_{kSentinel};
    bool harm_was_reset_{true};
    double harm_tauoverq_{0.0};   // tau / Q
    double harm_reciptausq_{0.0}; // 1 / tau^2
    double harm_dtlim_{0.0};      // tau / GEARRATIO (max stable timestep)

    // Hot-pixel bookkeeping ------------------------------------------------
    std::vector<std::uint32_t> hp_counts_;
    Metavision::timestamp hp_start_{kSentinel};
    double hp_thresh_{0.0};
    static constexpr Metavision::timestamp hp_window_us_{1000000};

    // Adaptive correlation-time bookkeeping --------------------------------
    Metavision::timestamp rate_start_{kSentinel};
    std::uint64_t rate_events_{0};
    double rate_eps_{0.0};
    double scale_{1.0};
    static constexpr Metavision::timestamp rate_window_us_{50000};

    // Last-call statistics -------------------------------------------------
    std::size_t last_total_{0};
    std::size_t last_kept_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_NOISE_FILTER_H
