// algo/cv/hough_line_tracker.h — event-driven incremental Hough line tracking.
//
// ✅ 移植自 jAER HoughLineTracker (net.sf.jaer.eventprocessing.tracking.
// HoughLineTracker)。事件驱动增量霍夫直线变换：维护 2D 累加器 (ρ, θ)，θ ∈ [0, π)；
// 逐事件对每个 θ 角度箱计算 ρ = x·cos(θ) + y·sin(θ) 并投票；累加器按时间常数
// accumulatorDecayUs 指数衰减（事件过期）；在累加器中寻找局部极大值作为检测直线
// (ρ, θ)，转换为图像内线段端点输出；按 (ρ, θ) 最近邻关联持久航迹。对应设计 §4.3.14。
// Header-only.

#ifndef GUI_ALGO_CV_HOUGH_LINE_TRACKER_H
#define GUI_ALGO_CV_HOUGH_LINE_TRACKER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Detected Hough line segment with persistent tracking id.
struct HoughLine {
    cv::Point2f start;
    cv::Point2f end;
    float angle{0.0f};    ///< Orientation in degrees, [0, 180).
    int track_id{-1};     ///< Persistent track id, -1 if untracked.
};

/// @brief Event-driven incremental Hough line tracker, ported from jAER.
///
/// Maintains a 2D accumulator (ρ, θ). Each event votes for ρ at every θ bin.
/// The accumulator decays exponentially over time so old events expire. Peaks
/// are converted to image-spanning line segments for overlay rendering.
class HoughLineTracker {
public:
    HoughLineTracker(int width, int height,
                     int num_theta_bins = 90,
                     int num_rho_bins = 0,
                     int threshold = 50,
                     Metavision::timestamp accumulator_decay_us = 100000)
        : width_(width), height_(height),
          num_theta_bins_(num_theta_bins),
          threshold_(threshold),
          accumulator_decay_us_(accumulator_decay_us) {
        if (num_theta_bins_ < 1) num_theta_bins_ = 1;
        rebuild(num_rho_bins);
    }

    /// @brief Processes an event packet and returns detected lines.
    std::vector<HoughLine> process(const EventPacket& packet) {
        std::vector<HoughLine> result;
        if (packet.empty()) return result;
        const Metavision::timestamp cur_t = packet[packet.size() - 1].t;
        if (last_t_ >= 0 && accumulator_decay_us_ > 0) {
            const Metavision::timestamp dt = cur_t - last_t_;
            if (dt > 0) apply_decay(dt);
        }
        last_t_ = cur_t;
        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            accumulate(e.x, e.y);
        }
        find_peaks(result);
        return result;
    }

    // Parameter accessors ---------------------------------------------------
    int num_theta_bins() const { return num_theta_bins_; }
    int num_rho_bins() const { return num_rho_bins_; }
    int threshold() const { return threshold_; }
    Metavision::timestamp accumulator_decay_us() const {
        return accumulator_decay_us_;
    }
    void set_num_theta_bins(int v) {
        if (v < 1) v = 1;
        if (v == num_theta_bins_) return;
        num_theta_bins_ = v;
        rebuild(num_rho_bins_);
    }
    void set_num_rho_bins(int v) {
        if (v < 0) v = 0;
        rebuild(v);
    }
    void set_threshold(int v) { threshold_ = v; }
    void set_accumulator_decay_us(Metavision::timestamp v) {
        accumulator_decay_us_ = v;
    }

    void reset() {
        std::fill(accum_.begin(), accum_.end(), 0.0f);
        last_t_ = -1;
        tracks_.clear();
        next_track_id_ = 0;
    }

private:
    struct Track {
        int id{-1};
        int last_ti{0};
        int last_ri{0};
    };

    void rebuild(int num_rho_bins_hint) {
        cos_.assign(static_cast<std::size_t>(num_theta_bins_), 0.0f);
        sin_.assign(static_cast<std::size_t>(num_theta_bins_), 0.0f);
        for (int i = 0; i < num_theta_bins_; ++i) {
            const double th = kPi * i / num_theta_bins_;
            cos_[static_cast<std::size_t>(i)] = static_cast<float>(std::cos(th));
            sin_[static_cast<std::size_t>(i)] = static_cast<float>(std::sin(th));
        }
        const double diag = std::sqrt(static_cast<double>(width_) * width_ +
                                      static_cast<double>(height_) * height_);
        rho_max_ = static_cast<float>(diag);
        if (num_rho_bins_hint > 0) {
            num_rho_bins_ = num_rho_bins_hint;
        } else {
            num_rho_bins_ = std::max(1, static_cast<int>(2.0 * diag));
        }
        rho_step_ = (2.0f * rho_max_) / static_cast<float>(num_rho_bins_);
        accum_.assign(static_cast<std::size_t>(num_theta_bins_) *
                          static_cast<std::size_t>(num_rho_bins_),
                      0.0f);
        last_t_ = -1;
        tracks_.clear();
        next_track_id_ = 0;
    }

    inline std::size_t idx(int ti, int ri) const {
        return static_cast<std::size_t>(ti) * static_cast<std::size_t>(num_rho_bins_) +
               static_cast<std::size_t>(ri);
    }

    /// @brief Incremental Hough vote: splat ρ = x·cos(θ) + y·sin(θ) for every
    /// θ bin.
    void accumulate(int x, int y) {
        const float xf = static_cast<float>(x);
        const float yf = static_cast<float>(y);
        for (int ti = 0; ti < num_theta_bins_; ++ti) {
            const float rho = xf * cos_[static_cast<std::size_t>(ti)] +
                              yf * sin_[static_cast<std::size_t>(ti)];
            int ri = static_cast<int>((rho + rho_max_) / rho_step_);
            if (ri < 0) ri = 0;
            else if (ri >= num_rho_bins_) ri = num_rho_bins_ - 1;
            accum_[idx(ti, ri)] += 1.0f;
        }
    }

    void apply_decay(Metavision::timestamp dt) {
        const double factor = std::exp(-static_cast<double>(dt) /
                                       static_cast<double>(accumulator_decay_us_));
        const float f = static_cast<float>(factor);
        for (float& v : accum_) v *= f;
    }

    /// @brief Finds local maxima above threshold, applies non-maximum
    /// suppression in (θ, ρ) space and associates persistent tracks.
    void find_peaks(std::vector<HoughLine>& out) {
        struct Cand { int ti; int ri; float val; };
        std::vector<Cand> cands;
        for (int ti = 0; ti < num_theta_bins_; ++ti) {
            for (int ri = 0; ri < num_rho_bins_; ++ri) {
                const float v = accum_[idx(ti, ri)];
                if (v < static_cast<float>(threshold_)) continue;
                if (!is_local_max(ti, ri)) continue;
                cands.push_back(Cand{ti, ri, v});
            }
        }
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& x, const Cand& y) { return x.val > y.val; });
        const int theta_nms = std::max(1, num_theta_bins_ / 18);
        const int rho_nms = std::max(1, num_rho_bins_ / 20);
        // Greedy non-maximum suppression against accepted (ti, ri).
        std::vector<std::pair<int, int>> accepted;
        for (const Cand& c : cands) {
            bool suppress = false;
            for (const auto& tr : accepted) {
                const int dt = c.ti - tr.first;
                const int dr = c.ri - tr.second;
                const int adt = dt < 0 ? -dt : dt;
                const int adr = dr < 0 ? -dr : dr;
                if (adt <= theta_nms && adr <= rho_nms) { suppress = true; break; }
            }
            if (suppress) continue;
            accepted.emplace_back(c.ti, c.ri);
            HoughLine hl = to_segment(c.ti, c.ri);
            hl.track_id = associate(c.ti, c.ri);
            out.push_back(hl);
            if (out.size() >= kMaxDetections) break;
        }
    }

    bool is_local_max(int ti, int ri) const {
        const float v = accum_[idx(ti, ri)];
        for (int dt = -1; dt <= 1; ++dt) {
            for (int dr = -1; dr <= 1; ++dr) {
                if (dt == 0 && dr == 0) continue;
                const int nt = ti + dt;
                const int nr = ri + dr;
                if (nt < 0 || nt >= num_theta_bins_ || nr < 0 || nr >= num_rho_bins_) continue;
                if (accum_[idx(nt, nr)] > v) return false;
            }
        }
        return true;
    }

    /// @brief Converts a (θ, ρ) peak into a line segment spanning the image.
    HoughLine to_segment(int ti, int ri) const {
        const double th = kPi * ti / num_theta_bins_;
        const double cos_t = std::cos(th);
        const double sin_t = std::sin(th);
        const double rho = (ri + 0.5) * static_cast<double>(rho_step_) -
                           static_cast<double>(rho_max_);
        HoughLine hl;
        if (std::abs(sin_t) > std::abs(cos_t)) {
            // Line is more horizontal: vary x across the image width.
            const double y0 = rho / sin_t;
            const double y1 = (rho - static_cast<double>(width_) * cos_t) / sin_t;
            hl.start = cv::Point2f(0.0f, static_cast<float>(y0));
            hl.end = cv::Point2f(static_cast<float>(width_), static_cast<float>(y1));
        } else {
            // Line is more vertical: vary y across the image height.
            const double x0 = rho / cos_t;
            const double x1 = (rho - static_cast<double>(height_) * sin_t) / cos_t;
            hl.start = cv::Point2f(static_cast<float>(x0), 0.0f);
            hl.end = cv::Point2f(static_cast<float>(x1), static_cast<float>(height_));
        }
        double deg = th * 180.0 / kPi + 90.0;
        deg = std::fmod(deg, 180.0);
        if (deg < 0.0) deg += 180.0;
        hl.angle = static_cast<float>(deg);
        return hl;
    }

    int associate(int ti, int ri) {
        const int theta_tol = std::max(1, num_theta_bins_ / 9);
        const int rho_tol = std::max(1, num_rho_bins_ / 10);
        int best_id = -1;
        for (const Track& tr : tracks_) {
            const int dt = ti - tr.last_ti;
            const int dr = ri - tr.last_ri;
            const int adt = dt < 0 ? -dt : dt;
            const int adr = dr < 0 ? -dr : dr;
            if (adt <= theta_tol && adr <= rho_tol) { best_id = tr.id; break; }
        }
        if (best_id < 0) {
            best_id = next_track_id_++;
            tracks_.push_back(Track{best_id, ti, ri});
        } else {
            for (Track& tr : tracks_) {
                if (tr.id == best_id) { tr.last_ti = ti; tr.last_ri = ri; break; }
            }
        }
        return best_id;
    }

    static constexpr std::size_t kMaxDetections = 16;
    static constexpr double kPi = 3.14159265358979323846;

    int width_;
    int height_;
    int num_theta_bins_;
    int num_rho_bins_{1};
    int threshold_;
    Metavision::timestamp accumulator_decay_us_;
    float rho_max_{0.0f};
    float rho_step_{1.0f};
    std::vector<float> cos_;
    std::vector<float> sin_;
    std::vector<float> accum_;
    Metavision::timestamp last_t_{-1};
    std::vector<Track> tracks_;
    int next_track_id_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_HOUGH_LINE_TRACKER_H
