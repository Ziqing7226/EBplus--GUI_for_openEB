// algo/cv/noise_filter.h — multi-mode event noise filter.
//
// Event noise filter (design §4.3.5) porting the jAER filter suite
// (net/sf/jaer/eventprocessing/filter/). Provides 9 denoising modes:
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
//                  oscillator resonant at DOUBLE the line frequency (line
//                  doubling; ON/OFF activity peaks alternate at 2x line)
//                  filters out events near zero crossings of the oscillator.
//   Repetitious  — drops events recurring at a learned per-pixel ISI
//                  (port of jAER RepetitiousFilter).
//   SpatialBP    — center/surround time-surface differencing (port of jAER
//                  SpatialBandpassFilter: surround-timestamp splat).
//   KNoise       — ✅ 移植自 dv-processing KNoiseFilter (Khodamoradi &
//                  Kastner, "O(N)-Space Spatiotemporal Filter", 2021):
//                  one memory cell per row and per column (W+H cells, no
//                  per-pixel surface); an event passes when any of the 3
//                  neighbouring row/column cells reports a recent same-
//                  polarity event within 1 px along the other axis.
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
        KNoise,         ///< ✅ 移植自 dv-processing KNoiseFilter
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
        knoise_init();
        sbp_rebuild_offsets();
    }

    // Mode + common options ------------------------------------------------
    void set_mode(Mode m) { mode_ = m; }
    Mode mode() const { return mode_; }
    /// @brief C++ 语义：开启额外的 n-sigma 统计热像素抑制（hot_ok）。与
    /// jAER filterHotPixels（相关性测试排除中心自身）同名不同义（§一-1.3）。
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
        const int nv = clamp_i(v, 1, 4096);
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
    void set_ratio_shorter(int v) { rep_ratio_shorter_ = clamp_i(v, 1, 100); }
    void set_ratio_longer(int v) { rep_ratio_longer_ = clamp_i(v, 1, 100); }
    void set_rep_averaging_samples(int v) {
        rep_averaging_samples_ = clamp_i(v, 1, 100);  // jAER setAveragingSamples
    }
    int rep_averaging_samples() const { return rep_averaging_samples_; }
    int ratio_shorter() const { return rep_ratio_shorter_; }
    int ratio_longer() const { return rep_ratio_longer_; }
    void set_min_dt_to_store_us(int v) { rep_min_dt_to_store_ = clamp_i(v, 0, 1000000); }
    int min_dt_to_store_us() const { return rep_min_dt_to_store_; }

    // SpatialBP (jAER port) ------------------------------------------------
    // jAER clips center to [0, surround-1] and surround to >= center+1 so a
    // surround ring always exists; a degenerate surround <= center would
    // splat nothing and the filter would pass every event.
    void set_center_radius_px(int v) {
        int c = clamp_i(v, 0, 10);
        if (c >= sbp_surround_) c = sbp_surround_ - 1;
        if (c != sbp_center_) {
            sbp_center_ = c;
            sbp_rebuild_offsets();
        }
    }
    void set_surround_radius_px(int v) {
        int s = clamp_i(v, 1, 30);
        if (s < sbp_center_ + 1) s = sbp_center_ + 1;
        if (s != sbp_surround_) {
            sbp_surround_ = s;
            sbp_rebuild_offsets();
        }
    }
    void set_dt_surround_us(int v) { sbp_dt_surround_us_ = clamp_i(v, 100, 1000000); }
    int center_radius_px() const { return sbp_center_; }
    int surround_radius_px() const { return sbp_surround_; }
    int dt_surround_us() const { return sbp_dt_surround_us_; }

    // KNoise (✅ 移植自 dv-processing KNoiseFilter) -------------------------
    void set_knoise_dt_us(int v) { knoise_dt_us_ = clamp_i(v, 100, 100000); }
    int knoise_dt_us() const { return knoise_dt_us_; }

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
        hp_thresh_ = std::numeric_limits<double>::infinity();
        rate_start_ = kSentinel;
        rate_events_ = 0;
        rate_eps_ = 0.0;
        scale_ = 1.0;
        last_total_ = 0;
        last_kept_ = 0;
        dwf_init();
        harm_reset_state();
        harm_recompute();
        knoise_init();
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
            case Mode::KNoise:      return knoise_pass(e);
        }
        return true; // unreachable
    }

    bool baf_pass(const Event& e) const {
        const Metavision::timestamp dt = thr(baf_dt_us_);
        // subsample_by 语义（§一-1.3，有意不改实现）：jAER 是对地址图右移做
        // 地址降采样；本实现只改邻域遍历步长 —— step=1 时查全部 8 邻居，
        // step=2 时只查 4 个对角方向邻居。与头注释 "3x3 neighbour
        // correlation" 的描述仅在 subsample_by=0 时一致。
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
        if (lt == kSentinel) return true;
        // 对齐 jAER RefractoryFilter：仅 dt > refractoryPeriod 放行。时间回退
        // 事件（Evt3 NonMonotonicTimeHigh、文件 seek）dt <= 0，应被滤掉而非
        // 放行（audit §一-1.3）。last_any_ 在 decide_and_update 中无条件更新，
        // 因此回退后每像素只牺牲一个事件，不会锁死。
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
            std::size_t idx = r.head;
            for (std::size_t i = 0; i < r.count; ++i) {
                const std::array<short, 2>& v = r.buf[idx];
                const int dx = static_cast<int>(ex) - static_cast<int>(v[0]);
                const int dy = static_cast<int>(ey) - static_cast<int>(v[1]);
                const int d = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
                if (d < dwf_dis_thr_) {
                    ++ncorrelated;
                    if (ncorrelated >= dwf_min_corr_) return;
                }
                if (++idx >= r.cap) idx = 0;
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
                // 恒排除中心自身（对齐 jAER 默认 filterHotPixels=true，§一-1.3），
                // 与 filter_hot_pixels_ 解耦。三模式语义说明：BAF/STCF 本来就
                // 跳过 (0,0) 与该开关无关；AgePolarity 此前受该开关控制（默认
                // false → 自身白拿 ~1 分）；C++ 的 filter_hot_pixels_ 实为额外
                // n-sigma 统计抑制（hot_ok），与 jAER filterHotPixels（相关性
                // 测试排除自身）同名不同义。
                if (dx == 0 && dy == 0) continue;
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
        // jAER: if(thisdt<minDtToStore){ continue; } — jAER 的 continue 不调
        // filterOut，即事件**放行**但 avgDt/lastTimes 不更新（防止突发短 ISI
        // 噪声污染 ISI 估计）。此前误移植为 return false 丢弃事件（§一-1.3）。
        if (thisdt < static_cast<double>(rep_min_dt_to_store_)) {
            return true;
        }
        const double avg = avg_dt;
        bool repetitious = false;
        if (avg > 0.0) {
            // jAER L180: repetitious = thisdt < avgDt*ratioLonger && thisdt > avgDt/ratioShorter
            repetitious = (thisdt < avg * rep_ratio_longer_) &&
                          (thisdt > avg / rep_ratio_shorter_);
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
    // then splat e.t to the surround ring on every event (pass or drop —
    // jAER's writeSurround also runs unconditionally). The ring cells are a
    // precomputed offset list (jAER computeOffsets), so the splat touches
    // only ring cells instead of iterating the full (2sr+1)^2 square and
    // branch-skipping the center.
    bool spatialbp_pass(const Event& e) {
        const std::size_t idx = idx_of(e.x, e.y);
        const Metavision::timestamp st = surround_ts_[idx];
        const bool pass = (st == kSentinel) ||
                          (e.t - st > thr(sbp_dt_surround_us_));
        for (const auto& off : sbp_offsets_) {
            const int nx = e.x + off[0];
            if (nx < 0 || nx >= width_) continue;
            const int ny = e.y + off[1];
            if (ny < 0 || ny >= height_) continue;
            surround_ts_[idx_of(nx, ny)] = e.t;
        }
        return pass;
    }

    void sbp_rebuild_offsets() {
        sbp_offsets_.clear();
        for (int dy = -sbp_surround_; dy <= sbp_surround_; ++dy) {
            for (int dx = -sbp_surround_; dx <= sbp_surround_; ++dx) {
                if (dx >= -sbp_center_ && dx <= sbp_center_ &&
                    dy >= -sbp_center_ && dy <= sbp_center_) {
                    continue;  // center region is not surround
                }
                sbp_offsets_.push_back({static_cast<std::int16_t>(dx),
                                        static_cast<std::int16_t>(dy)});
            }
        }
    }

    // ✅ 移植自 dv-processing KNoiseFilter (Khodamoradi & Kastner 2021,
    // ref/dv-processing-master/include/dv-processing/noise/k_noise_filter.hpp).
    // Memory-efficient BAF alternative: instead of a W×H per-pixel time
    // surface, keep ONE cell per column (W cells) and ONE per row (H cells) —
    // ~18 KB at 640×480. Each cell records the last event seen in that
    // row/column: timestamp, polarity, and the "other address" (y for a
    // column cell, x for a row cell). An event passes when ANY of the 6
    // neighbour cells (columns x-1/x/x+1, rows y-1/y/y+1 — dv uses OR, not
    // AND) supports it: (e.t - cell.ts) <= dt, polarity hard-matches, and
    // the stored other-address lies within 1 px of the event's own
    // coordinate along that axis. Cells are updated unconditionally AFTER
    // the decision (dv updates them even for filtered-out events).
    // Divergences from dv, both intentional:
    //  - dv requires only (t - cell.ts) <= timeDelta; a negative diff (time
    //    rewind, Evt3 NonMonotonicTimeHigh, file seek) would spuriously
    //    support an event. We additionally require diff >= 0, matching the
    //    BAF/STCF convention in this file.
    //  - dv zero-initializes its cells, so OFF events near y=0/x=0 can
    //    spuriously pass at stream start; we use kSentinel so a cell only
    //    supports after a real event was recorded in it.
    bool knoise_pass(const Event& e) {
        const Metavision::timestamp dt = thr(knoise_dt_us_);
        bool pass = false;
        for (int dx = -1; dx <= 1 && !pass; ++dx) {
            const int nx = e.x + dx;
            if (nx < 0 || nx >= width_) continue;
            const KNoiseCell& c = knoise_col_[static_cast<std::size_t>(nx)];
            if (c.ts == kSentinel) continue;
            const Metavision::timestamp diff = e.t - c.ts;
            if (diff < 0 || diff > dt) continue;      // dv: <= timeDelta
            if (c.pol != e.p) continue;               // polarity hard match
            if (std::abs(static_cast<int>(c.other) - e.y) > 1) continue;
            pass = true;
        }
        for (int dy = -1; dy <= 1 && !pass; ++dy) {
            const int ny = e.y + dy;
            if (ny < 0 || ny >= height_) continue;
            const KNoiseCell& c = knoise_row_[static_cast<std::size_t>(ny)];
            if (c.ts == kSentinel) continue;
            const Metavision::timestamp diff = e.t - c.ts;
            if (diff < 0 || diff > dt) continue;
            if (c.pol != e.p) continue;
            if (std::abs(static_cast<int>(c.other) - e.x) > 1) continue;
            pass = true;
        }
        // Unconditional update after the decision (dv retain()).
        knoise_col_[static_cast<std::size_t>(e.x)] =
            KNoiseCell{e.t, static_cast<std::int16_t>(e.y), e.p};
        knoise_row_[static_cast<std::size_t>(e.y)] =
            KNoiseCell{e.t, static_cast<std::int16_t>(e.x), e.p};
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
        // 注释定档（audit §一-1.3，有意不改实现）：jAER 的单窗（FWF）模式用
        // 完整 windowLength，且所有事件（含被滤的）都入窗；本实现两种模式的
        // 环容量都取 wlen/2，且单窗模式下被滤事件不入窗。单窗模式（非默认）
        // 因此比 jAER 弱；double 模式（默认）不受影响。
        const int dwlen = dwf_wlen_ > 1 ? dwf_wlen_ / 2 : 1;
        const std::size_t c = static_cast<std::size_t>(dwlen);
        dwf_signal_.init(c);
        dwf_noise_.init(c);
    }

    // Harmonic (jAER HarmonicFilter) oscillator helpers -------------------
    void harm_recompute() {
        // jAER: "set by user to double line frequency" — mains-driven lamps
        // flicker at 2x line (full-wave rectification / power ~ V^2), and the
        // ON and OFF activity bursts alternate at that doubled rate; the
        // oscillator crosses zero at those activity peaks only when resonant
        // there. The GUI stores the LINE frequency (50/60); the resonance is
        // doubled here.
        const double f0 = 2.0 * static_cast<double>(harm_line_freq_hz_);
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
    int dwf_wlen_{64};           // jAER wLen (total window length, events);
                                 // 512 (jAER pref default) costs an O(wLen)
                                 // scan per noise event (~474 ns at 512) and
                                 // saturates the SDK callback thread on this
                                 // sensor's event rates — 64 keeps meaningful
                                 // history at ~the STCF default's per-event cost
    int dwf_dis_thr_{5};          // jAER disThr (L1 distance threshold)
    int dwf_min_corr_{1};         // jAER numMustBeCorrelated
    bool dwf_double_mode_{true};  // jAER useDoubleMode
    Metavision::timestamp agep_tau_us_{10000};
    double agep_threshold_{5.0};  // jAER correlationThreshold (default 5, max 8)
    int agep_radius_{1};          // neighbourhood radius (1 = 3x3)
    int harm_line_freq_hz_{50};   // 市电线频率（50/60）；振荡器共振频率在其
                                  // 2 倍处（jAER "double line frequency"，见
                                  // harm_recompute——倍频整流/V² 使灯光以 2×
                                  // 线频闪烁，ON/OFF 活动峰也以 2× 线频交替）
    double harm_notch_q_{3.0};    // jAER quality factor Q
    double harm_threshold_{0.1};  // jAER threshold (fraction of power)
    int rep_ratio_shorter_{2};   // jAER RepetitiousFilter default
    int rep_ratio_longer_{2};    // jAER RepetitiousFilter default
    int rep_averaging_samples_{3}; // jAER averagingSamples default
    int rep_min_dt_to_store_{1000};  // jAER minDtToStore default (us)
    int sbp_center_{0};          // jAER centerRadius default
    int sbp_surround_{1};        // jAER surroundRadius default. The previous
                                 // defaults (center 2 / surround 10) splatted
                                 // 416 cells per event — 52x jAER's 8 — and
                                 // capped the filter at ~2.4 Mev/s/core.
    Metavision::timestamp sbp_dt_surround_us_{8000}; // jAER dtSurround default
    /// Precomputed surround-ring splat offsets (jAER computeOffsets): only
    /// the ring cells, so the per-event loop skips no center cells and runs
    /// no center-membership branches.
    std::vector<std::array<std::int16_t, 2>> sbp_offsets_;
    // dv default is 2000 us, tuned on DAVIS240; the value below is calibrated
    // on algo/tests/sparklers.raw: keep-rate vs dt is 0.718@500us, 0.720@1ms,
    // 0.721@2-3ms, plateau 0.722 from ~5ms — the elbow is below 1 ms, so
    // 3000 us sits on the plateau without an over-wide correlation window.
    Metavision::timestamp knoise_dt_us_{3000}; // dv KNoiseFilter timeDelta

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

    // KNoise state (dv-processing KNoiseFilter) ----------------------------
    /// dv KMemCell: one cell per column (other = last y) and per row
    /// (other = last x). ~16 B/cell → (W+H) cells ≈ 18 KB at 640×480.
    struct KNoiseCell {
        Metavision::timestamp ts{kSentinel};
        std::int16_t other{-1};
        short pol{0};
    };
    std::vector<KNoiseCell> knoise_col_; ///< width cells
    std::vector<KNoiseCell> knoise_row_; ///< height cells

    void knoise_init() {
        knoise_col_.assign(static_cast<std::size_t>(width_), KNoiseCell{});
        knoise_row_.assign(static_cast<std::size_t>(height_), KNoiseCell{});
    }

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
    double hp_thresh_{std::numeric_limits<double>::infinity()};
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
