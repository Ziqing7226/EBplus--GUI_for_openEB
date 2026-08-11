// algo/cv/dense_optical_flow.h — dense event-based optical flow estimation.
//
// Self-developed (design §4.3.9), complementing SparseOpticalFlow. Three modes,
// implemented from the published methods:
//   PlaneFitting    — Benosman et al. 2013: fit a local (x,y,t) plane on the
//                     time surface; the plane gradient ∇t gives the aperture-
//                     constrained normal velocity v = ∇t/|∇t|² (px/s).
//   TimeGradient    — same normal-flow formula via centered finite differences
//                     of the time surface at each event (finer, noisier).
//   TripletMatching — Shiba, Aoki & Gallego 2022: random event triplets each
//                     yield a linear least-squares velocity hypothesis; votes
//                     accumulate in a 2D velocity histogram and the peak is the
//                     dominant flow, assigned to all active cells.
// Output: dense per-pixel (vx, vy) in px/s + confidence [0,1], at sensor
//         resolution (cells without recent activity stay 0). Header-only,
//         no Qt dependency.

#ifndef GUI_ALGO_CV_DENSE_OPTICAL_FLOW_H
#define GUI_ALGO_CV_DENSE_OPTICAL_FLOW_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Dense optical-flow estimator (per-pixel normal/dominant flow).
class DenseOpticalFlow {
public:
    enum class Mode { PlaneFitting, TimeGradient, TripletMatching };

    DenseOpticalFlow(int width, int height, Mode mode = Mode::PlaneFitting)
        : width_(width), height_(height), mode_(mode),
          // 2 channels per pixel: the most recent ON and OFF timestamps are
          // kept separately (a moving edge leaves ON at its leading side and
          // OFF at its trailing side; mixing them biases the plane fit).
          sae_(2 * static_cast<std::size_t>(width) * height, kNoTs) {}

    void set_mode(Mode m) { mode_ = m; }
    Mode mode() const { return mode_; }

    void set_time_window_us(int v) { time_window_us_ = clamp_i(v, 1000, 500000); }
    void set_spatial_radius_px(int v) { spatial_radius_px_ = clamp_i(v, 1, 16); }
    void set_max_events(int v) { max_events_ = clamp_i(v, 100, 200000); }
    void set_max_velocity_px_s(float v) {
        max_velocity_px_s_ = v < 100.0f ? 100.0f : v;
    }

    int time_window_us() const { return time_window_us_; }
    int spatial_radius_px() const { return spatial_radius_px_; }
    int max_events() const { return max_events_; }
    float max_velocity_px_s() const { return max_velocity_px_s_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Feeds a batch of events. Updates the per-polarity time surface
    /// (most recent timestamp per pixel & polarity) and the sliding event
    /// window.
    void process(const Event* begin, const Event* end) {
        if (!begin || !end || begin >= end) return;
        last_ts_ = (end - 1)->t;
        for (const Event* it = begin; it != end; ++it) {
            const int x = static_cast<int>(it->x);
            const int y = static_cast<int>(it->y);
            if (x < 0 || x >= width_ || y < 0 || y >= height_) continue;
            const std::size_t base =
                2 * (static_cast<std::size_t>(y) * width_ + x);
            sae_[base + (it->p != 0 ? 1u : 0u)] = it->t;
            window_.push_back(*it);
        }
        trim_window();
    }

    /// @brief Discards temporal state (camera restart, file seek/loop).
    void reset() {
        std::fill(sae_.begin(), sae_.end(), kNoTs);
        window_.clear();
        last_ts_ = 0;
    }

    /// @brief Computes the dense flow map from the current state.
    /// @param flow CV_32FC2 per pixel (vx, vy) in px/s (0 where no activity).
    /// @param conf CV_32FC1 confidence [0,1].
    void get_flow(cv::Mat& flow, cv::Mat& conf) {
        flow.create(height_, width_, CV_32FC2);
        flow.setTo(cv::Scalar(0, 0));
        conf.create(height_, width_, CV_32FC1);
        conf.setTo(0.0f);
        if (window_.empty()) return;

        fx_ = cv::Mat_<double>::zeros(height_, width_);
        fy_ = cv::Mat_<double>::zeros(height_, width_);
        fc_ = cv::Mat_<double>::zeros(height_, width_);

        switch (mode_) {
            case Mode::PlaneFitting:  compute_plane_fit(); break;
            case Mode::TimeGradient:  compute_time_gradient(); break;
            case Mode::TripletMatching: compute_triplet(); break;
        }

        // Average per-cell contributions.
        for (int y = 0; y < height_; ++y) {
            cv::Vec2f* frow = flow.ptr<cv::Vec2f>(y);
            float* crow = conf.ptr<float>(y);
            const double* rx = fx_.ptr<double>(y);
            const double* ry = fy_.ptr<double>(y);
            const double* rc = fc_.ptr<double>(y);
            for (int x = 0; x < width_; ++x) {
                if (rc[x] > 0.0) {
                    frow[x] = cv::Vec2f(static_cast<float>(rx[x] / rc[x]),
                                        static_cast<float>(ry[x] / rc[x]));
                    crow[x] = static_cast<float>(std::min(1.0, fc_(y, x) / rc[x]));
                }
            }
        }
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    void trim_window() {
        const Metavision::timestamp t0 = last_ts_ - static_cast<Metavision::timestamp>(time_window_us_);
        std::size_t drop = 0;
        while (drop < window_.size() && window_[drop].t < t0) ++drop;
        if (drop) window_.erase(window_.begin(), window_.begin() + static_cast<std::ptrdiff_t>(drop));
        if (window_.size() > static_cast<std::size_t>(max_events_)) {
            window_.erase(window_.begin(),
                          window_.begin() + static_cast<std::ptrdiff_t>(window_.size() - max_events_));
        }
    }

    /// @brief Time-surface timestamp at (x,y) for polarity @p pol if fresh
    /// enough, else kNoTs.
    Metavision::timestamp ts_at(int x, int y, int pol,
                                Metavision::timestamp tmin) const {
        const std::size_t base =
            2 * (static_cast<std::size_t>(y) * width_ + x);
        const Metavision::timestamp t = sae_[base + pol];
        return (t >= tmin) ? t : kNoTs;
    }

    // Benosman 2013: local (x,y,t) plane fit → normal velocity ∇t/|∇t|².
    // The plane is fitted per event on the triggering event's polarity channel
    // only, and only over the newest ~25% of the neighborhood timestamps — an
    // adaptive time gate that keeps the support locally planar instead of
    // letting old events from the full time window pull the fit.
    void compute_plane_fit() {
        const int r = spatial_radius_px_;
        const Metavision::timestamp tmin = last_ts_ - static_cast<Metavision::timestamp>(time_window_us_);
        const int diameter = 2 * r + 1;
        const std::size_t n_support = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::lround(0.25 * diameter * diameter)));
        for (const Event& ev : window_) {
            const int pol = ev.p != 0 ? 1 : 0;

            // Collect the same-polarity neighborhood timestamps and find the
            // n_support-th newest: it becomes the adaptive time gate.
            nbr_ts_.clear();
            for (int y = std::max(0, ev.y - r); y <= std::min(height_ - 1, ev.y + r); ++y) {
                const Metavision::timestamp* row =
                    &sae_[2 * (static_cast<std::size_t>(y) * width_) + pol];
                for (int x = std::max(0, ev.x - r); x <= std::min(width_ - 1, ev.x + r); ++x) {
                    nbr_ts_.push_back(row[2 * x]);
                }
            }
            if (nbr_ts_.size() < n_support) continue;
            std::nth_element(nbr_ts_.begin(), nbr_ts_.begin() + n_support,
                             nbr_ts_.end(), std::greater<Metavision::timestamp>());
            const Metavision::timestamp t_limit = nbr_ts_[n_support];
            if (t_limit < tmin) continue;  // neighborhood too empty/stale

            double sx = 0, sy = 0, st = 0, sxx = 0, syy = 0, sxy = 0, sxt = 0, syt = 0;
            int n = 0;
            for (int y = std::max(0, ev.y - r); y <= std::min(height_ - 1, ev.y + r); ++y) {
                const Metavision::timestamp* row =
                    &sae_[2 * (static_cast<std::size_t>(y) * width_) + pol];
                for (int x = std::max(0, ev.x - r); x <= std::min(width_ - 1, ev.x + r); ++x) {
                    const Metavision::timestamp t = row[2 * x];
                    if (t < t_limit) continue;
                    const double dx = x, dy = y, dt = static_cast<double>(t);
                    sx += dx; sy += dy; st += dt;
                    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
                    sxt += dx * dt; syt += dy * dt;
                    ++n;
                }
            }
            if (n < 6) continue;

            // Solve the 3x3 normal system for plane t = a·x + b·y + c.
            //   |sxx sxy sx| |a|   |sxt|
            //   |sxy syy sy| |b| = |syt|
            //   |sx  sy  n | |c|   |st |
            const double det = sxx * (syy * n - sy * sy) -
                               sxy * (sxy * n - sy * sx) +
                               sx * (sxy * sy - syy * sx);
            if (std::fabs(det) < 1e-12) continue;
            const double a = (sxt * (syy * n - sy * sy) -
                              sxy * (syt * n - sy * st) +
                              sx * (syt * sy - syy * st)) / det;
            const double b = (sxx * (syt * n - sy * st) -
                              sxt * (sxy * n - sy * sx) +
                              sx * (sxy * st - syt * sx)) / det;
            const double c = (st - a * sx - b * sy) / n;

            // Normal velocity: v = (a,b)/(a²+b²), px/µs → px/s.
            const double g2 = a * a + b * b;
            if (g2 < 1e-12) continue;
            double vx = a / g2 * 1e6;
            double vy = b / g2 * 1e6;
            const double mag = std::sqrt(vx * vx + vy * vy);
            if (mag > max_velocity_px_s_) { vx *= max_velocity_px_s_ / mag; vy *= max_velocity_px_s_ / mag; }

            // Fit residual → confidence.
            double res2 = 0.0;
            for (int y = std::max(0, ev.y - r); y <= std::min(height_ - 1, ev.y + r); ++y) {
                const Metavision::timestamp* row =
                    &sae_[2 * (static_cast<std::size_t>(y) * width_) + pol];
                for (int x = std::max(0, ev.x - r); x <= std::min(width_ - 1, ev.x + r); ++x) {
                    const Metavision::timestamp t = row[2 * x];
                    if (t < t_limit) continue;
                    const double d = static_cast<double>(t) - (a * x + b * y + c);
                    res2 += d * d;
                }
            }
            const double rms = std::sqrt(res2 / n);
            const double conf = 1.0 / (1.0 + rms / 50.0);

            accumulate(ev.x, ev.y, vx, vy, conf);
        }
    }

    // Normal flow via centered finite differences of the time surface.
    // Only the cardinal pixels at distance `spatial_radius_px_` are used (not
    // the pixels in-between): the wider stencil regularizes the estimate at
    // the cost of a slightly larger association error. The default radius is 3
    // (shared with PlaneFitting).
    void compute_time_gradient() {
        const int r = spatial_radius_px_;
        const Metavision::timestamp tmin = last_ts_ - static_cast<Metavision::timestamp>(time_window_us_);
        for (const Event& ev : window_) {
            const int x = ev.x, y = ev.y;
            if (x < r || x >= width_ - r || y < r || y >= height_ - r) continue;
            const int pol = ev.p != 0 ? 1 : 0;
            const Metavision::timestamp tl = ts_at(x - r, y, pol, tmin);
            const Metavision::timestamp tr = ts_at(x + r, y, pol, tmin);
            const Metavision::timestamp tu = ts_at(x, y - r, pol, tmin);
            const Metavision::timestamp td = ts_at(x, y + r, pol, tmin);
            if (tl == kNoTs || tr == kNoTs || tu == kNoTs || td == kNoTs) continue;
            const double inv_spacing = 1.0 / (2.0 * r);
            const double gx = (static_cast<double>(tr) - static_cast<double>(tl)) * inv_spacing;
            const double gy = (static_cast<double>(td) - static_cast<double>(tu)) * inv_spacing;
            const double g2 = gx * gx + gy * gy;
            if (g2 < 1e-12) continue;
            double vx = gx / g2 * 1e6;
            double vy = gy / g2 * 1e6;
            const double mag = std::sqrt(vx * vx + vy * vy);
            if (mag > max_velocity_px_s_) { vx *= max_velocity_px_s_ / mag; vy *= max_velocity_px_s_ / mag; }
            // Strong gradient (steep time surface) = high confidence.
            const double conf = 1.0 / (1.0 + std::sqrt(g2) / 50.0);
            accumulate(x, y, vx, vy, conf);
        }
    }

    // Shiba 2022: for each event (anchor), the same-polarity events in its
    // spatial neighbourhood (within `spatial_radius_px_`) form the matching
    // pool; every anchor + pair of neighbours is a triplet whose least-squares
    // velocity is a hypothesis. The anchor's velocity is the median of its
    // hypotheses (robust average of the aligned triplets), giving a per-cell
    // dense flow instead of one dominant scene velocity. Restricting matches
    // to the neighbourhood keeps hypotheses locally coherent — triplets of
    // events far apart combine unrelated motions.
    void compute_triplet() {
        const std::size_t n = window_.size();
        if (n < 3) return;
        const float vmax = max_velocity_px_s_;
        const int r = spatial_radius_px_;

        // Per-pixel event-index lists for O(1) neighbourhood lookups.
        grid_.assign(static_cast<std::size_t>(width_) * height_, {});
        for (std::size_t i = 0; i < n; ++i) {
            const Event& ev = window_[i];
            grid_[static_cast<std::size_t>(ev.y) * width_ + ev.x].push_back(i);
        }

        constexpr std::size_t kMaxNeighbours = 8;
        for (std::size_t ai = 0; ai < n; ++ai) {
            const Event& a = window_[ai];
            if (a.x < r || a.x >= width_ - r || a.y < r || a.y >= height_ - r) continue;

            cand_.clear();
            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const auto& cell = grid_[(static_cast<std::size_t>(a.y + dy)) * width_ + (a.x + dx)];
                    for (const std::size_t bi : cell) {
                        const Event& b = window_[bi];
                        if (b.p != a.p) continue;
                        if (std::fabs(static_cast<double>(b.t) - static_cast<double>(a.t)) >= kMinDtUs) {
                            cand_.push_back(bi);
                        }
                    }
                }
            }
            if (cand_.size() < 2) continue;
            // Keep only the temporally closest matches (most coherent).
            if (cand_.size() > kMaxNeighbours) {
                std::partial_sort(cand_.begin(), cand_.begin() + kMaxNeighbours,
                                  cand_.end(),
                                  [&](std::size_t u, std::size_t v) {
                                      const long long du = std::llabs(
                                          static_cast<long long>(window_[u].t) -
                                          static_cast<long long>(a.t));
                                      const long long dv = std::llabs(
                                          static_cast<long long>(window_[v].t) -
                                          static_cast<long long>(a.t));
                                      return du < dv;
                                  });
                cand_.resize(kMaxNeighbours);
            }

            tvx_.clear();
            tvy_.clear();
            for (std::size_t p = 0; p < cand_.size(); ++p) {
                for (std::size_t q = p + 1; q < cand_.size(); ++q) {
                    const Event& b = window_[cand_[p]];
                    const Event& c = window_[cand_[q]];
                    const double dt1 = static_cast<double>(b.t) - static_cast<double>(a.t);
                    const double dt2 = static_cast<double>(c.t) - static_cast<double>(a.t);
                    if (std::fabs(dt1) < kMinDtUs || std::fabs(dt2) < kMinDtUs) continue;
                    const double denom = dt1 * dt1 + dt2 * dt2;
                    const double ux = ((b.x - a.x) * dt1 + (c.x - a.x) * dt2) / denom;
                    const double uy = ((b.y - a.y) * dt1 + (c.y - a.y) * dt2) / denom;
                    const double vx = ux * 1e6, vy = uy * 1e6;
                    if (std::fabs(vx) > vmax || std::fabs(vy) > vmax) continue;
                    tvx_.push_back(vx);
                    tvy_.push_back(vy);
                }
            }
            if (tvx_.size() < 2) continue;
            std::sort(tvx_.begin(), tvx_.end());
            std::sort(tvy_.begin(), tvy_.end());
            const std::size_t mid = tvx_.size() / 2;
            const double conf = std::min(1.0, static_cast<double>(tvx_.size()) / 6.0);
            accumulate(a.x, a.y, tvx_[mid], tvy_[mid], conf);
        }
    }

    void accumulate(int x, int y, double vx, double vy, double conf) {
        fx_(y, x) += vx * conf;
        fy_(y, x) += vy * conf;
        fc_(y, x) += conf;
    }

    static constexpr Metavision::timestamp kNoTs =
        std::numeric_limits<Metavision::timestamp>::min();
    static constexpr double kMinDtUs = 1.0;

    int width_{0};
    int height_{0};
    Mode mode_{Mode::PlaneFitting};
    int time_window_us_{10000};
    int spatial_radius_px_{3};
    int max_events_{8000};
    float max_velocity_px_s_{20000.0f};

    Metavision::timestamp last_ts_{0};
    std::vector<Metavision::timestamp> sae_;  ///< Most recent event time per pixel & polarity (2·W·H).
    std::vector<Event> window_;               ///< Sliding event window (time-trimmed).

    // Scratch buffers (reused across get_flow() calls, not thread-safe by design).
    std::vector<Metavision::timestamp> nbr_ts_;  ///< Plane-fitting neighborhood timestamps.
    std::vector<std::vector<std::size_t>> grid_; ///< Triplet: event indices per pixel.
    std::vector<std::size_t> cand_;              ///< Triplet: anchor's match candidates.
    std::vector<double> tvx_, tvy_;              ///< Triplet: per-anchor velocity hypotheses.

    // Per-cell accumulation buffers (reused across get_flow() calls).
    cv::Mat_<double> fx_;
    cv::Mat_<double> fy_;
    cv::Mat_<double> fc_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_DENSE_OPTICAL_FLOW_H
