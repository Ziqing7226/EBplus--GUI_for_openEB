// algo/cv/object_tracker.h — event-level multi-object tracking.
//
// Self-developed (design §4.3.11), inspired by jAER RectangularClusterTracker
// et al. Four tracking modes:
//   RCT             — rectangular cluster tracker (IIR location mixing).
//   Median          — median position update (robust to outliers).
//   Kalman          — per-cluster KalmanFilter2D (position + velocity).
//   MultiHypothesis — 借鉴 jAER KalmanEventFilter (labyrinthkalman) 的
//                     分配/生成骨架（逐事件 Mahalanobis 最近分配 + 阈值内
//                     correct 否则新建 + 每包 predict）；概率池、0.9 衰减、
//                     归一化、低概率剪枝、近邻合并、池上限 5、最优假设输出
//                     均为自研。门限 χ² 2D 95% = 5.99 ≠ jAER 3.0。
// Each cluster implements ClusterInterface (position, velocity, bbox,
// trajectory, mass, age). Output: vector<TrackedObject>. Header-only.

#ifndef GUI_ALGO_CV_OBJECT_TRACKER_H
#define GUI_ALGO_CV_OBJECT_TRACKER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/common/kalman_filter.h"
#include "algo/cv/cluster_interface.h"
#include "algo/cv/cluster_path_point.h"

namespace gui_algo {

/// @brief A tracked object snapshot emitted by the tracker.
struct TrackedObject {
    int id{-1};
    float x{0.0F};
    float y{0.0F};
    float vx{0.0F};
    float vy{0.0F};
    cv::Rect bbox;
    std::vector<ClusterPathPoint> trajectory;
    float age{0.0F};      ///< seconds since birth
    bool visible{false};
};

/// @brief Multi-mode event-level object tracker.
class ObjectTracker {
public:
    enum class Mode { RCT, Median, Kalman, MultiHypothesis };

    ObjectTracker(int width, int height, Mode mode = Mode::RCT)
        : width_(width), height_(height), mode_(mode) {}

    // Parameters (defaults per design §4.3.11) ----------------------------
    void set_mode(Mode m) { mode_ = m; }
    void set_cluster_size_px(int v) { cluster_size_px_ = clamp_i(v, 3, 50); }
    void set_cluster_time_us(int v) { cluster_time_us_ = clamp_i(v, 1000, 50000); }
    void set_min_cluster_events(int v) { min_cluster_events_ = clamp_i(v, 10, 500); }
    void set_max_lost_age_s(double v) { max_lost_age_s_ = clamp_d(v, 0.1, 5.0); }
    void set_enable_velocity_prediction(bool v) { enable_velocity_prediction_ = v; }
    void set_location_mixing_factor(float v) {
        location_mixing_factor_ = clamp_f(v, 0.0F, 1.0F);
    }
    void set_predictive_velocity_factor(float v) {
        predictive_velocity_factor_ = clamp_f(v, 0.0F, 10.0F);
    }
    void set_mass_decay_tau_us(int v) {
        mass_decay_tau_us_ = clamp_i(v, 1, 1000000);
    }
    void set_threshold_mass_for_visible(float v) {
        threshold_mass_for_visible_ = clamp_f(v, 0.0F, 1000000.0F);
    }

    Mode mode() const { return mode_; }
    int cluster_size_px() const { return cluster_size_px_; }
    int cluster_time_us() const { return cluster_time_us_; }
    int min_cluster_events() const { return min_cluster_events_; }
    double max_lost_age_s() const { return max_lost_age_s_; }
    bool enable_velocity_prediction() const { return enable_velocity_prediction_; }
    float location_mixing_factor() const { return location_mixing_factor_; }
    float predictive_velocity_factor() const { return predictive_velocity_factor_; }
    int mass_decay_tau_us() const { return mass_decay_tau_us_; }
    float threshold_mass_for_visible() const { return threshold_mass_for_visible_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Processes a batch of events, updating clusters.
    void process(const Event* events, std::size_t count) {
        // First packet after construction/reset: anchor prev_batch_t_ to the
        // first event (-1 sentinel). With large-timestamp sources (live camera
        // t≈1e9us, cropped playback) a 0 initial value would age every cluster
        // by t0 and instantly prune them all on the next packet (§四-M1).
        if (prev_batch_t_ < 0 && count > 0) {
            prev_batch_t_ = events[0].t;
        }
        Metavision::timestamp last_t = prev_batch_t_;
        prune_lost();
        for (auto& c : clusters_) c.begin_batch();
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            if (e.t > last_t) last_t = e.t;
            int best = -1;
            float best_d = static_cast<float>(cluster_size_px_);
            for (int k = 0; k < static_cast<int>(clusters_.size()); ++k) {
                if (e.t - clusters_[k].last_t() > cluster_time_us_) continue;
                const float d = clusters_[k].distance(e);
                if (d < best_d) { best_d = d; best = k; }
            }
            if (best < 0) {
                if (static_cast<int>(clusters_.size()) < kMaxClusters) {
                    clusters_.emplace_back(next_id_++, e, mode_, cluster_size_px_,
                                           cluster_time_us_,
                                           enable_velocity_prediction_,
                                           max_lost_us(),
                                           location_mixing_factor_,
                                           predictive_velocity_factor_,
                                           mass_decay_tau_us_,
                                           threshold_mass_for_visible_);
                }
            } else {
                clusters_[best].update(e);
            }
        }
        const Metavision::timestamp dt = last_t - prev_batch_t_;
        if (dt > 0) {
            for (auto& c : clusters_) c.age(dt);
        }
        prev_batch_t_ = last_t;
        merge_clusters();
        emit();
    }

    /// @brief Processes an event packet.
    void process(EventPacket& events) {
        process(events.data(), events.size());
    }

    /// @brief Returns the most recently emitted tracked objects.
    const std::vector<TrackedObject>& objects() const { return tracked_; }

    void reset() {
        clusters_.clear();
        tracked_.clear();
        next_id_ = 0;
        prev_batch_t_ = -1;  // -1 sentinel: re-anchor on the next first event
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static double clamp_d(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static float clamp_f(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    Metavision::timestamp max_lost_us() const {
        return static_cast<Metavision::timestamp>(max_lost_age_s_ * 1e6);
    }

    static float median(std::vector<float>& v) {
        if (v.empty()) return 0.0F;
        std::sort(v.begin(), v.end());
        const std::size_t n = v.size();
        if (n % 2 == 1) return v[n / 2];
        return (v[n / 2 - 1] + v[n / 2]) / 2.0F;
    }

    /// @brief Concrete cluster implementing ClusterInterface.
    class Cluster : public ClusterInterface {
    public:
        struct Recent { float x, y; Metavision::timestamp t; };
        /// One KalmanFilter2D hypothesis for MultiHypothesis mode (骨架借鉴
        /// jAER KalmanEventFilter: each filter = one ball-position hypothesis).
        struct MhHyp { KalmanFilter2D kf; double prob; };

        Cluster(int id, const Event& seed, Mode mode, int cluster_size_px,
                int cluster_time_us, bool enable_velocity_prediction,
                Metavision::timestamp max_lost_us,
                float location_mixing_factor,
                float predictive_velocity_factor,
                Metavision::timestamp mass_decay_tau_us,
                float threshold_mass_for_visible)
            : id_(id), mode_(mode), last_t_(seed.t),
              cluster_size_px_(cluster_size_px),
              cluster_time_us_(cluster_time_us),
              enable_velocity_prediction_(enable_velocity_prediction),
              max_lost_us_(max_lost_us),
              location_mixing_factor_(location_mixing_factor),
              predictive_velocity_factor_(predictive_velocity_factor),
              mass_decay_tau_us_(mass_decay_tau_us),
              threshold_mass_for_visible_(threshold_mass_for_visible),
              x_(static_cast<float>(seed.x)),
              y_(static_cast<float>(seed.y)),
              prev_x_(x_), prev_y_(y_), prev_pos_t_(seed.t),
              bbox_(static_cast<int>(seed.x), static_cast<int>(seed.y), 1, 1),
              radius_(static_cast<float>(cluster_size_px) * 0.5F) {
            recent_.push_back({x_, y_, seed.t});
            if (mode_ == Mode::Kalman) kf_.init(static_cast<double>(seed.x),
                                                static_cast<double>(seed.y));
            if (mode_ == Mode::MultiHypothesis) {
                // Seed the hypothesis pool with one KF at the seed event.
                MhHyp h;
                h.kf.init(static_cast<double>(seed.x),
                          static_cast<double>(seed.y));
                h.prob = 1.0;
                mh_hyps_.push_back(h);
            }
            maybe_push_trajectory(seed);
        }

        // ClusterInterface implementation --------------------------------
        void update(const Event& e) override {
            const Metavision::timestamp prev_t = last_t_;
            since_last_us_ = 0;
            last_t_ = e.t;
            // Leaky mass: mass = 1 + mass * exp(-dt / tau) (jAER
            // clusterMassDecayTauUs).
            {
                const float dt_m = e.t > prev_t
                    ? static_cast<float>(e.t - prev_t) : 0.0F;
                mass_ = 1.0F + mass_ * std::exp(-dt_m /
                                static_cast<float>(mass_decay_tau_us_));
            }
            push_recent(e);
            switch (mode_) {
                case Mode::RCT: update_rct(e, prev_t); break;
                case Mode::Median: update_median(); break;
                case Mode::Kalman: update_kalman(e, prev_t); break;
                case Mode::MultiHypothesis: update_mh(e, prev_t); break;
            }
            update_bbox(e);
            update_velocity(e);
            maybe_push_trajectory(e);
        }

        float distance(const Event& e) const override {
            const float dx = x_ - static_cast<float>(e.x);
            const float dy = y_ - static_cast<float>(e.y);
            return std::sqrt(dx * dx + dy * dy);
        }

        bool is_visible() const override {
            return get_mass_now(last_t_) >= threshold_mass_for_visible_ &&
                   since_last_us_ <= max_lost_us_;
        }

        void age(Metavision::timestamp dt_us) override {
            since_last_us_ += dt_us;
            age_us_ += dt_us;
            if (dt_us <= 0) return;
            if (mode_ == Mode::Kalman) {
                // Skip the predict if update_kalman() already advanced the
                // filter this batch (avoids double prediction).
                if (!predicted_this_batch_) {
                    kf_.set_dt(static_cast<double>(dt_us) * 1e-6);
                    kf_.predict();
                    x_ = static_cast<float>(kf_.x());
                    y_ = static_cast<float>(kf_.y());
                    vx_ = static_cast<float>(kf_.vx());
                    vy_ = static_cast<float>(kf_.vy());
                }
            } else if (mode_ == Mode::MultiHypothesis) {
                // MultiHypothesis predicts the whole KF pool once per batch
                // inside update_mh(). If no event hit this cluster this batch
                // (mh_predicted_this_batch_ still false), predict the pool
                // forward here for motion continuity, then refresh output.
                if (!mh_predicted_this_batch_ && !mh_hyps_.empty()) {
                    const double dt = static_cast<double>(dt_us) * 1e-6;
                    for (auto& h : mh_hyps_) {
                        h.kf.set_dt(dt);
                        h.kf.predict();
                    }
                    mh_predicted_this_batch_ = true;
                    set_mh_output();
                }
            } else if (enable_velocity_prediction_) {
                const float dt_s = static_cast<float>(dt_us) * 1e-6F;
                // Clamp the per-batch extrapolation to ±cluster_size_px
                // (§四-S1): without this, a noisy velocity estimate
                // integrated over a whole packet gap threw the cluster
                // hundreds of px away and every packet spawned a new cluster.
                const float lim = static_cast<float>(cluster_size_px_);
                const float dx = vx_ * dt_s;
                const float dy = vy_ * dt_s;
                x_ += (dx < -lim) ? -lim : (dx > lim ? lim : dx);
                y_ += (dy < -lim) ? -lim : (dy > lim ? lim : dy);
            }
        }

        float x() const override { return x_; }
        float y() const override { return y_; }
        float vx() const override { return vx_; }
        float vy() const override { return vy_; }
        cv::Rect bbox() const override { return bbox_; }
        const std::vector<ClusterPathPoint>& trajectory() const override {
            return trajectory_;
        }
        float mass() const override { return mass_; }
        Metavision::timestamp age_us() const override { return age_us_; }

        // Cluster-specific accessors ------------------------------------
        int id() const { return id_; }
        Metavision::timestamp last_t() const { return last_t_; }
        float radius() const { return radius_; }
        Metavision::timestamp since_last_us() const { return since_last_us_; }
        float get_mass_now(Metavision::timestamp t) const {
            return mass_ * std::exp(static_cast<float>(last_t_ - t) /
                                    static_cast<float>(mass_decay_tau_us_));
        }
        void begin_batch() {
            predicted_this_batch_ = false;
            mh_predicted_this_batch_ = false;
        }
        bool should_prune() const {
            // Hard timeout: no events for longer than max_lost_us_.
            if (since_last_us_ > max_lost_us_) return true;
            // Mass-based pruning: decayed mass below threshold for at least
            // one mass_decay_tau_us_.
            const float decayed = get_mass_now(last_t_ + since_last_us_);
            return decayed < threshold_mass_for_visible_ &&
                   since_last_us_ >= mass_decay_tau_us_;
        }

        void absorb(Cluster& o) {
            const float m1 = mass_;
            const float m2 = o.mass_;
            const float tot = m1 + m2;
            if (tot > 0.0F) {
                x_ = (x_ * m1 + o.x_ * m2) / tot;
                y_ = (y_ * m1 + o.y_ * m2) / tot;
                vx_ = (vx_ * m1 + o.vx_ * m2) / tot;
                vy_ = (vy_ * m1 + o.vy_ * m2) / tot;
            }
            mass_ += o.mass_;
            bbox_ = bbox_ | o.bbox_;
            if (o.last_t_ > last_t_) last_t_ = o.last_t_;
        }

    private:
        void push_recent(const Event& e) {
            recent_.push_back({static_cast<float>(e.x),
                               static_cast<float>(e.y), e.t});
            while (!recent_.empty() && (e.t - recent_.front().t) > cluster_time_us_) {
                recent_.pop_front();
            }
            if (recent_.size() > 64) recent_.pop_front();
        }

        void update_rct(const Event& e, Metavision::timestamp prev_t) {
            const float m = location_mixing_factor_;
            // Optional predictive velocity advance (jAER predictiveVelocityFactor).
            if (enable_velocity_prediction_ && prev_t > 0 && e.t > prev_t) {
                const float dt_s =
                    static_cast<float>(e.t - prev_t) * 1e-6F;
                x_ += vx_ * dt_s * predictive_velocity_factor_;
                y_ += vy_ * dt_s * predictive_velocity_factor_;
            }
            // Per-event IIR location mixing (jAER locationMixingFactor).
            x_ = (1.0F - m) * x_ + m * static_cast<float>(e.x);
            y_ = (1.0F - m) * y_ + m * static_cast<float>(e.y);
        }

        void update_median() {
            std::vector<float> xs, ys;
            xs.reserve(recent_.size());
            ys.reserve(recent_.size());
            for (const auto& p : recent_) { xs.push_back(p.x); ys.push_back(p.y); }
            x_ = median(xs);
            y_ = median(ys);
        }

        void update_kalman(const Event& e, Metavision::timestamp prev_t) {
            if (!kf_.initialized()) {
                kf_.init(static_cast<double>(e.x), static_cast<double>(e.y));
                x_ = static_cast<float>(e.x);
                y_ = static_cast<float>(e.y);
                return;
            }
            if (e.t > prev_t) {
                kf_.set_dt(static_cast<double>(e.t - prev_t) * 1e-6);
            }
            kf_.predict();
            kf_.update(static_cast<double>(e.x), static_cast<double>(e.y));
            x_ = static_cast<float>(kf_.x());
            y_ = static_cast<float>(kf_.y());
            predicted_this_batch_ = true;
        }

        // 借鉴 jAER KalmanEventFilter (labyrinthkalman) 的分配/生成骨架:
        // per-cluster pool of KalmanFilter2D hypotheses. Each batch: predict
        // all hypotheses once; per event compute Mahalanobis d²=(e-μ)ᵀ·S⁻¹·(e-μ),
        // S=P+R; assign to the nearest hypothesis if d² < gate (χ² 2D 95% =
        // 5.99, jAER 用 3.0), else spawn a new hypothesis (or replace the
        // lowest-probability one when the pool is capped). Then prune
        // low-probability hypotheses, merge close ones, and emit the
        // highest-probability hypothesis (概率池/剪枝/合并均为自研).
        void update_mh(const Event& e, Metavision::timestamp prev_t) {
            const double ex = static_cast<double>(e.x);
            const double ey = static_cast<double>(e.y);

            // Predict all hypotheses once per batch (on the first event).
            if (!mh_predicted_this_batch_) {
                if (prev_t > 0 && e.t > prev_t) {
                    const double dt =
                        static_cast<double>(e.t - prev_t) * 1e-6;
                    for (auto& h : mh_hyps_) {
                        h.kf.set_dt(dt);
                        h.kf.predict();
                    }
                }
                mh_predicted_this_batch_ = true;
            }

            if (mh_hyps_.empty()) {
                MhHyp h;
                h.kf.init(ex, ey);
                h.prob = 1.0;
                mh_hyps_.push_back(h);
            } else {
                // Nearest hypothesis by Mahalanobis distance d².
                int best = -1;
                double best_d2 = mh_gate_;
                for (int k = 0;
                     k < static_cast<int>(mh_hyps_.size()); ++k) {
                    const double d2 = mahalanobis2(mh_hyps_[k].kf, ex, ey);
                    if (d2 < best_d2) { best_d2 = d2; best = k; }
                }
                if (best >= 0) {
                    mh_hyps_[best].kf.update(ex, ey);
                    // Lower Mahalanobis distance -> larger Gaussian likelihood
                    // exp(-d^2/2) -> higher probability mass for this hypothesis.
                    mh_hyps_[best].prob += std::exp(-best_d2 / 2.0);
                } else if (static_cast<int>(mh_hyps_.size()) < mh_max_pool_) {
                    // Spawn a new hypothesis for an unmatched measurement.
                    MhHyp h;
                    h.kf.init(ex, ey);
                    h.prob = 1.0;
                    mh_hyps_.push_back(h);
                } else {
                    // Pool capped: replace the lowest-probability hypothesis.
                    int lo = 0;
                    for (int k = 1;
                         k < static_cast<int>(mh_hyps_.size()); ++k) {
                        if (mh_hyps_[k].prob < mh_hyps_[lo].prob) lo = k;
                    }
                    mh_hyps_[lo].kf.init(ex, ey);
                    mh_hyps_[lo].prob = 1.0;
                }
            }

            decay_prune_merge_mh();
            set_mh_output();
        }

        /// Squared Mahalanobis distance d²=(e-μ)ᵀ·S⁻¹·(e-μ) with S=P+R. The
        /// KalmanFilter2D position covariance is diagonal (no x-y cross term),
        /// so this factorises as (dx²/Sx + dy²/Sy), Sx=p_xx+r, Sy=p_yy+r.
        static double mahalanobis2(const KalmanFilter2D& kf,
                                   double ex, double ey) {
            const double sx = kf.p_xx() + kf.measurement_noise_var();
            const double sy = kf.p_yy() + kf.measurement_noise_var();
            const double ix = ex - kf.x();
            const double iy = ey - kf.y();
            if (sx <= 0.0 || sy <= 0.0) return 1e18;
            return (ix * ix) / sx + (iy * iy) / sy;
        }

        /// Decay, normalize, prune low-probability hypotheses (always keeping
        /// the highest-probability one), and merge hypotheses whose means are
        /// within cluster_size_px_/2 of each other.
        void decay_prune_merge_mh() {
            for (auto& h : mh_hyps_) h.prob *= 0.9;
            normalize_mh();
            if (static_cast<int>(mh_hyps_.size()) > 1) {
                int maxi = 0;
                for (int k = 1;
                     k < static_cast<int>(mh_hyps_.size()); ++k) {
                    if (mh_hyps_[k].prob > mh_hyps_[maxi].prob) maxi = k;
                }
                std::vector<MhHyp> kept;
                kept.reserve(mh_hyps_.size());
                for (int k = 0;
                     k < static_cast<int>(mh_hyps_.size()); ++k) {
                    if (k == maxi || mh_hyps_[k].prob >= mh_prune_prob_) {
                        kept.push_back(std::move(mh_hyps_[k]));
                    }
                }
                mh_hyps_.swap(kept);
            }
            // Merge close hypotheses: keep the higher-probability one and
            // transfer the other's probability mass.
            const float mt = static_cast<float>(cluster_size_px_) * 0.5F;
            const float mt2 = mt * mt;
            bool merged = true;
            while (merged && static_cast<int>(mh_hyps_.size()) > 1) {
                merged = false;
                for (int i = 0;
                     i < static_cast<int>(mh_hyps_.size()); ++i) {
                    for (int j = i + 1;
                         j < static_cast<int>(mh_hyps_.size()); ++j) {
                        const float dx = static_cast<float>(
                            mh_hyps_[i].kf.x() - mh_hyps_[j].kf.x());
                        const float dy = static_cast<float>(
                            mh_hyps_[i].kf.y() - mh_hyps_[j].kf.y());
                        if (dx * dx + dy * dy < mt2) {
                            if (mh_hyps_[i].prob < mh_hyps_[j].prob) {
                                std::swap(mh_hyps_[i], mh_hyps_[j]);
                            }
                            mh_hyps_[i].prob += mh_hyps_[j].prob;
                            mh_hyps_.erase(mh_hyps_.begin() + j);
                            merged = true;
                            break;
                        }
                    }
                    if (merged) break;
                }
            }
            normalize_mh();
        }

        /// Normalize hypothesis weights so they sum to 1 (probabilities).
        void normalize_mh() {
            double sum = 0.0;
            for (const auto& h : mh_hyps_) sum += h.prob;
            if (sum > 0.0) {
                for (auto& h : mh_hyps_) h.prob /= sum;
            }
        }

        /// Output position & velocity from the highest-probability hypothesis
        /// (mirrors jAER KalmanEventFilter.currentBestFilter).
        void set_mh_output() {
            if (mh_hyps_.empty()) return;
            int best = 0;
            for (int k = 1; k < static_cast<int>(mh_hyps_.size()); ++k) {
                if (mh_hyps_[k].prob > mh_hyps_[best].prob) best = k;
            }
            x_ = static_cast<float>(mh_hyps_[best].kf.x());
            y_ = static_cast<float>(mh_hyps_[best].kf.y());
            vx_ = static_cast<float>(mh_hyps_[best].kf.vx());
            vy_ = static_cast<float>(mh_hyps_[best].kf.vy());
        }

        void update_bbox(const Event& e) {
            const cv::Rect r(static_cast<int>(e.x), static_cast<int>(e.y), 1, 1);
            bbox_ = bbox_ | r;
        }

        void update_velocity(const Event& e) {
            if (mode_ == Mode::Kalman) {
                vx_ = static_cast<float>(kf_.vx());
                vy_ = static_cast<float>(kf_.vy());
                return;
            }
            if (mode_ == Mode::MultiHypothesis) {
                // Velocity already set from the highest-probability KF in
                // update_mh() / set_mh_output(); skip IIR differencing.
                return;
            }
            if (prev_pos_t_ > 0 && e.t > prev_pos_t_) {
                const float dt_s =
                    static_cast<float>(e.t - prev_pos_t_) * 1e-6F;
                if (dt_s > 0.0F) {
                    // jAER RCT velocityTauMs=100ms first-order low-pass on the
                    // instantaneous velocity (alpha = min(1, dt/tau)). Without
                    // it the IIR-smoothed per-event position step (~0.05 px)
                    // divided by a us-scale dt produced huge velocity spikes
                    // that broke tracking via age() extrapolation (§四-S1).
                    const float alpha = std::min(1.0F, dt_s / 0.1F);
                    vx_ += alpha * ((x_ - prev_x_) / dt_s - vx_);
                    vy_ += alpha * ((y_ - prev_y_) / dt_s - vy_);
                }
            }
            prev_x_ = x_;
            prev_y_ = y_;
            prev_pos_t_ = e.t;
        }

        void maybe_push_trajectory(const Event& e) {
            if (trajectory_.empty() || e.t - last_traj_t_ >= cluster_time_us_) {
                trajectory_.push_back(ClusterPathPoint(x_, y_, vx_, vy_, e.t, radius_));
                if (trajectory_.size() > 500) trajectory_.erase(trajectory_.begin());
                last_traj_t_ = e.t;
            }
        }

        int id_;
        Mode mode_;
        Metavision::timestamp last_t_;
        Metavision::timestamp since_last_us_{0};
        Metavision::timestamp age_us_{0};
        Metavision::timestamp last_traj_t_{0};
        float mass_{1.0F};
        int cluster_size_px_;
        int cluster_time_us_;
        bool enable_velocity_prediction_;
        Metavision::timestamp max_lost_us_;
        float location_mixing_factor_;
        float predictive_velocity_factor_;
        Metavision::timestamp mass_decay_tau_us_;
        float threshold_mass_for_visible_;
        bool predicted_this_batch_{false};
        float x_, y_, vx_{0.0F}, vy_{0.0F};
        float prev_x_, prev_y_;
        Metavision::timestamp prev_pos_t_;
        cv::Rect bbox_;
        float radius_;
        std::deque<Recent> recent_;
        std::vector<MhHyp> mh_hyps_;          // MultiHypothesis KF pool
        std::vector<ClusterPathPoint> trajectory_;
        KalmanFilter2D kf_;
        // MultiHypothesis tuning (骨架借鉴 jAER KalmanEventFilter，门限不同).
        double mh_gate_{5.99};                // χ² 2D 95% gating threshold (jAER 3.0)
        int mh_max_pool_{5};                  // cap on hypotheses per cluster
        bool mh_predicted_this_batch_{false};
        double mh_prune_prob_{0.01};         // prune hypotheses below this prob
    };

    void prune_lost() {
        std::vector<Cluster> kept;
        kept.reserve(clusters_.size());
        for (auto& c : clusters_) {
            if (!c.should_prune()) kept.push_back(std::move(c));
        }
        clusters_.swap(kept);
    }

    void merge_clusters() {
        if (clusters_.size() < 2) return;
        // jAER uses rectangle overlap: |dx| < r1+r2 AND |dy| < r1+r2, and
        // restarts the loop after each merge. Cap passes to avoid O(n^2).
        for (int pass = 0; pass < 10; ++pass) {
            bool merged = false;
            for (std::size_t i = 0; i < clusters_.size(); ++i) {
                for (std::size_t j = i + 1; j < clusters_.size();) {
                    const float dx = std::abs(clusters_[i].x() - clusters_[j].x());
                    const float dy = std::abs(clusters_[i].y() - clusters_[j].y());
                    const float rsum = clusters_[i].radius() + clusters_[j].radius();
                    if (dx < rsum && dy < rsum) {
                        clusters_[i].absorb(clusters_[j]);
                        clusters_.erase(clusters_.begin() +
                                        static_cast<long>(j));
                        merged = true;
                    } else {
                        ++j;
                    }
                }
            }
            if (!merged) break;
        }
    }

    void emit() {
        tracked_.clear();
        for (const auto& c : clusters_) {
            if (static_cast<int>(c.mass()) < min_cluster_events_) continue;
            if (!c.is_visible()) continue;
            TrackedObject o;
            o.id = c.id();
            o.x = c.x();
            o.y = c.y();
            o.vx = c.vx();
            o.vy = c.vy();
            o.bbox = c.bbox();
            o.trajectory = c.trajectory();
            o.age = static_cast<float>(c.age_us()) * 1e-6F;
            o.visible = true;
            tracked_.push_back(o);
        }
    }

    static constexpr int kMaxClusters = 2000;

    int width_;
    int height_;
    Mode mode_;
    int cluster_size_px_{10};
    int cluster_time_us_{5000};
    int min_cluster_events_{50};
    double max_lost_age_s_{1.0};
    bool enable_velocity_prediction_{true};
    float location_mixing_factor_{0.05F};
    float predictive_velocity_factor_{1.0F};
    int mass_decay_tau_us_{10000};
    float threshold_mass_for_visible_{10.0F};

    std::vector<Cluster> clusters_;
    std::vector<TrackedObject> tracked_;
    int next_id_{0};
    Metavision::timestamp prev_batch_t_{-1};  // -1 = no packet seen yet
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_OBJECT_TRACKER_H
