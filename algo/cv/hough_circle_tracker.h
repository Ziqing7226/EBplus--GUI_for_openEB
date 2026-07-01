// algo/cv/hough_circle_tracker.h — event-driven incremental Hough circle tracking.
//
// ✅ 移植自 jAER HoughCircleTracker (net.sf.jaer.eventprocessing.tracking.
// HoughCircleTracker)。事件驱动增量霍夫圆变换：维护 3D 累加器 (a, b, r)，其中
// (a,b) 为圆心、r 为半径；逐事件对每个候选半径 r，在以事件 (x,y) 为中心、半径为 r
// 的圆上对所有候选圆心 (a, b) 投票（a = x + r·cos(θ), b = y + r·sin(θ)）；累加器按
// 时间常数 accumulatorDecayUs 指数衰减（事件过期）；在累加器中寻找局部极大值作为
// 检测圆，并按最近邻关联持久航迹。对应设计 §4.3.15。Header-only.

#ifndef GUI_ALGO_CV_HOUGH_CIRCLE_TRACKER_H
#define GUI_ALGO_CV_HOUGH_CIRCLE_TRACKER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Detected circle with persistent tracking id.
struct HoughCircle {
    cv::Point2f center;
    float radius{0.0f};
    int track_id{-1};   ///< Persistent track id, -1 if untracked.
};

/// @brief Event-driven incremental Hough circle tracker, ported from jAER.
///
/// Maintains a 3D accumulator (a, b, r) over circle centers and radii. Each
/// event votes for all centers that could produce it at every candidate radius.
/// The accumulator decays exponentially over time so old events expire.
class HoughCircleTracker {
public:
    HoughCircleTracker(int width, int height,
                       int min_radius_px = 5,
                       int max_radius_px = 50,
                       int threshold = 30,
                       Metavision::timestamp accumulator_decay_us = 100000)
        : width_(width), height_(height),
          min_radius_px_(min_radius_px),
          max_radius_px_(max_radius_px),
          threshold_(threshold),
          accumulator_decay_us_(accumulator_decay_us) {
        if (min_radius_px_ < 1) min_radius_px_ = 1;
        if (max_radius_px_ < min_radius_px_) max_radius_px_ = min_radius_px_;
        rebuild();
    }

    /// @brief Processes an event packet and returns detected circles.
    std::vector<HoughCircle> process(const EventPacket& packet) {
        std::vector<HoughCircle> result;
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
    int min_radius_px() const { return min_radius_px_; }
    int max_radius_px() const { return max_radius_px_; }
    int threshold() const { return threshold_; }
    /// @brief Compatibility alias for threshold().
    int hough_threshold() const { return threshold_; }
    Metavision::timestamp accumulator_decay_us() const {
        return accumulator_decay_us_;
    }
    void set_min_radius_px(int v) {
        if (v < 1) v = 1;
        if (v == min_radius_px_) return;
        min_radius_px_ = v;
        if (max_radius_px_ < min_radius_px_) max_radius_px_ = min_radius_px_;
        rebuild();
    }
    void set_max_radius_px(int v) {
        if (v < min_radius_px_) v = min_radius_px_;
        if (v == max_radius_px_) return;
        max_radius_px_ = v;
        rebuild();
    }
    void set_threshold(int v) { threshold_ = v; }
    /// @brief Compatibility alias for set_threshold().
    void set_hough_threshold(int v) { threshold_ = v; }
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
    struct Offset { int dx; int dy; };
    struct Track {
        int id{-1};
        cv::Point2f last_center;
    };

    void rebuild() {
        num_radii_ = max_radius_px_ - min_radius_px_ + 1;
        if (num_radii_ < 1) num_radii_ = 1;
        accum_.assign(static_cast<std::size_t>(width_) *
                          static_cast<std::size_t>(height_) *
                          static_cast<std::size_t>(num_radii_),
                      0.0f);
        // Precompute the deduplicated circle outline offsets for each radius:
        // for an event at (x,y), the candidate centers are (x+dx, y+dy).
        offsets_.assign(static_cast<std::size_t>(num_radii_), {});
        constexpr int kAngleSamples = 72;
        for (int k = 0; k < num_radii_; ++k) {
            const float r = static_cast<float>(min_radius_px_ + k);
            std::vector<Offset> tmp;
            tmp.reserve(kAngleSamples);
            for (int i = 0; i < kAngleSamples; ++i) {
                const float a = 2.0f * kPiF * static_cast<float>(i) /
                                static_cast<float>(kAngleSamples);
                const Offset o{
                    static_cast<int>(std::lround(r * std::cos(a))),
                    static_cast<int>(std::lround(r * std::sin(a)))};
                tmp.push_back(o);
            }
            std::sort(tmp.begin(), tmp.end(), [](const Offset& p, const Offset& q) {
                return (p.dx != q.dx) ? (p.dx < q.dx) : (p.dy < q.dy);
            });
            tmp.erase(std::unique(tmp.begin(), tmp.end(),
                                  [](const Offset& p, const Offset& q) {
                                      return p.dx == q.dx && p.dy == q.dy;
                                  }),
                      tmp.end());
            offsets_[static_cast<std::size_t>(k)] = std::move(tmp);
        }
        last_t_ = -1;
        tracks_.clear();
        next_track_id_ = 0;
    }

    inline std::size_t idx(int a, int b, int k) const {
        return (static_cast<std::size_t>(k) * static_cast<std::size_t>(height_) +
                static_cast<std::size_t>(b)) *
                   static_cast<std::size_t>(width_) +
               static_cast<std::size_t>(a);
    }

    /// @brief Incremental Hough vote: for every candidate radius, splat the
    /// circle of candidate centers around the event.
    void accumulate(int x, int y) {
        for (int k = 0; k < num_radii_; ++k) {
            for (const Offset& o : offsets_[static_cast<std::size_t>(k)]) {
                const int a = x + o.dx;
                const int b = y + o.dy;
                if (a < 0 || a >= width_ || b < 0 || b >= height_) continue;
                accum_[idx(a, b, k)] += 1.0f;
            }
        }
    }

    void apply_decay(Metavision::timestamp dt) {
        const double factor = std::exp(-static_cast<double>(dt) /
                                       static_cast<double>(accumulator_decay_us_));
        const float f = static_cast<float>(factor);
        for (float& v : accum_) v *= f;
    }

    /// @brief Finds local maxima above threshold, applies non-maximum
    /// suppression across radii and associates persistent tracks.
    void find_peaks(std::vector<HoughCircle>& out) {
        struct Cand { int a; int b; int k; float val; };
        std::vector<Cand> cands;
        for (int k = 0; k < num_radii_; ++k) {
            for (int b = 0; b < height_; ++b) {
                for (int a = 0; a < width_; ++a) {
                    const float v = accum_[idx(a, b, k)];
                    if (v < static_cast<float>(threshold_)) continue;
                    if (!is_local_max(a, b, k)) continue;
                    cands.push_back(Cand{a, b, k, v});
                }
            }
        }
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& x, const Cand& y) { return x.val > y.val; });
        const float nms = static_cast<float>(min_radius_px_);
        const float nms2 = nms * nms;
        for (const Cand& c : cands) {
            bool ok = true;
            for (const HoughCircle& h : out) {
                const float dx = h.center.x - static_cast<float>(c.a);
                const float dy = h.center.y - static_cast<float>(c.b);
                if (dx * dx + dy * dy < nms2) { ok = false; break; }
            }
            if (!ok) continue;
            HoughCircle hc;
            hc.center = cv::Point2f(static_cast<float>(c.a),
                                    static_cast<float>(c.b));
            hc.radius = static_cast<float>(min_radius_px_ + c.k);
            hc.track_id = associate(hc);
            out.push_back(hc);
            if (out.size() >= kMaxDetections) break;
        }
    }

    bool is_local_max(int a, int b, int k) const {
        const float v = accum_[idx(a, b, k)];
        for (int db = -1; db <= 1; ++db) {
            for (int da = -1; da <= 1; ++da) {
                if (da == 0 && db == 0) continue;
                const int na = a + da;
                const int nb = b + db;
                if (na < 0 || na >= width_ || nb < 0 || nb >= height_) continue;
                if (accum_[idx(na, nb, k)] > v) return false;
            }
        }
        return true;
    }

    int associate(const HoughCircle& hc) {
        const float tol = static_cast<float>(max_radius_px_) * 2.0f;
        const float tol2 = tol * tol;
        int best_id = -1;
        float best_d2 = tol2;
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            const float dx = tracks_[i].last_center.x - hc.center.x;
            const float dy = tracks_[i].last_center.y - hc.center.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_id = tracks_[i].id; }
        }
        if (best_id < 0) {
            best_id = next_track_id_++;
            tracks_.push_back(Track{best_id, hc.center});
        } else {
            for (auto& tr : tracks_) {
                if (tr.id == best_id) { tr.last_center = hc.center; break; }
            }
        }
        return best_id;
    }

    static constexpr std::size_t kMaxDetections = 16;
    static constexpr float kPiF = 3.14159265358979323846f;

    int width_;
    int height_;
    int min_radius_px_;
    int max_radius_px_;
    int threshold_;
    Metavision::timestamp accumulator_decay_us_;
    int num_radii_{1};
    std::vector<float> accum_;
    std::vector<std::vector<Offset>> offsets_;
    Metavision::timestamp last_t_{-1};
    std::vector<Track> tracks_;
    int next_track_id_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_HOUGH_CIRCLE_TRACKER_H
