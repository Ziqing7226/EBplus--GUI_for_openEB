// algo/cv/cluster_lif.h — LIF neuron grid clustering.
//
// ✅ 移植自 jAER BlurringTunnelFilter (ch.unizh.ini.jaer.projects.einsteintunnel.
// sensoryprocessing.BlurringTunnelFilter). 对应设计 §4.3.18。
//
// jAER 的 BlurringTunnelFilter / NeuronGroup 将同时发放（共燃）的多个 LIF
// 神经元按连通分量分组成一个簇（NeuronGroup）：每个像素建模为漏电积分触发
// 神经元，事件使膜电位上升（ON/OFF 不分极性，按 +1 累积）并按时间常数 tau
// 指数泄漏；当某像素电位越过发放阈值时发放并复位。本 C++ 实现复用底层 LIF
// 积分器（algo/common/lif_integrator.h，其本身已与 jAER 一致），并在每个
// 事件包处理完成后，把本包内所有发放像素收集为二值掩码，对其做 8 连通连通
// 分量标记（迭代 BFS），每个连通分量即一个簇：质心按发放次数加权、记录
// 像素数 (size)、总发放数 (mass)、外接框；再按最近邻匹配跨包跟踪并估计速度。
// 输出: vector<LifCluster>，每个含 (track_id, cx, cy, size, mass, vx, vy)。
// Header-only.

#ifndef GUI_ALGO_CV_CLUSTER_LIF_H
#define GUI_ALGO_CV_CLUSTER_LIF_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/common/lif_integrator.h"

namespace gui_algo {

/// @brief A cluster emitted when one or more co-firing LIF neurons form a
/// connected component (移植自 jAER BlurringTunnelFilter NeuronGroup).
struct LifCluster {
    cv::Point2f position;             ///< Mass-weighted centroid (cx, cy).
    double potential{0.0};            ///< Membrane potential at firing (== threshold).
    int track_id{-1};                 ///< Persistent track id, -1 if untracked.
    int size{0};                      ///< Number of distinct firing pixels.
    int mass{0};                      ///< Total firings summed over the component.
    float vx{0.0F};                   ///< Track velocity x (px/s).
    float vy{0.0F};                   ///< Track velocity y (px/s).
    cv::Rect bbox;                    ///< Bounding box of the component.
    Metavision::timestamp last_t{0};  ///< Timestamp of the batch that produced it.
};

/// @brief LIF neuron grid clustering with connected-component grouping.
class ClusterLIF {
public:
    ClusterLIF(int width, int height,
               float tau_ms = 10.0f,
               float threshold = 1.0f,
               float reset_value = 0.0f)
        : width_(width), height_(height),
          tau_us_(static_cast<Metavision::timestamp>(tau_ms * 1000.0f)),
          threshold_(static_cast<double>(threshold)),
          reset_value_(static_cast<double>(reset_value)),
          lif_(width, height,
               static_cast<Metavision::timestamp>(tau_ms * 1000.0f),
               static_cast<double>(threshold),
               static_cast<double>(reset_value)),
          track_tol_px_(10.0f),
          stale_us_(1000000),
          fire_mask_(static_cast<std::size_t>(width) * height, 0),
          fire_count_(static_cast<std::size_t>(width) * height, 0) {}

    /// @brief Processes an event packet and returns one cluster per connected
    /// component of co-firing LIF neurons (移植自 jAER BlurringTunnelFilter).
    std::vector<LifCluster> process(const EventPacket& packet) {
        std::vector<LifCluster> result;
        if (packet.empty()) return result;

        // 1) Drive the LIF integrator and collect firing pixels into a binary
        //    mask (+ per-pixel firing count, used as the centroid/mass weight,
        //    mirroring jAER's effectiveMP = numSpikes weighting).
        std::fill(fire_mask_.begin(), fire_mask_.end(), uint8_t(0));
        std::fill(fire_count_.begin(), fire_count_.end(), 0);
        Metavision::timestamp batch_t = 0;
        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            const bool fired = lif_.add_event(e.x, e.y, e.p, e.t);
            if (e.t > batch_t) batch_t = e.t;
            if (fired) {
                const std::size_t idx =
                    static_cast<std::size_t>(e.y) * width_ + e.x;
                fire_mask_[idx] = 1;
                ++fire_count_[idx];
            }
        }

        // 2) Connected-component labelling (8-connectivity, iterative BFS with
        //    an explicit stack — no recursion, sensor-sized grids are fine).
        const std::size_t n = static_cast<std::size_t>(width_) * height_;
        std::vector<int> label(n, -1);
        struct Comp {
            double sx{0.0};   // sum(x * weight)
            double sy{0.0};   // sum(y * weight)
            double sm{0.0};   // sum(weight) == total firings (mass)
            int size{0};      // distinct pixels
            int minx{0}, miny{0}, maxx{0}, maxy{0};
        };
        std::vector<Comp> comps;
        std::vector<std::size_t> stack;
        stack.reserve(n);
        static const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
        static const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
        const std::size_t w = static_cast<std::size_t>(width_);
        const int wi = width_;
        const int hi = height_;
        for (int y = 0; y < hi; ++y) {
            for (int x = 0; x < wi; ++x) {
                const std::size_t idx = static_cast<std::size_t>(y) * w + x;
                if (!fire_mask_[idx] || label[idx] >= 0) continue;
                const int cid = static_cast<int>(comps.size());
                comps.emplace_back();
                Comp& c = comps.back();
                c.minx = x; c.maxx = x; c.miny = y; c.maxy = y;
                stack.clear();
                stack.push_back(idx);
                label[idx] = cid;
                while (!stack.empty()) {
                    const std::size_t cur = stack.back();
                    stack.pop_back();
                    const int cx = static_cast<int>(cur % w);
                    const int cy = static_cast<int>(cur / w);
                    const double wt = static_cast<double>(fire_count_[cur]);
                    c.sx += static_cast<double>(cx) * wt;
                    c.sy += static_cast<double>(cy) * wt;
                    c.sm += wt;
                    ++c.size;
                    if (cx < c.minx) c.minx = cx;
                    if (cx > c.maxx) c.maxx = cx;
                    if (cy < c.miny) c.miny = cy;
                    if (cy > c.maxy) c.maxy = cy;
                    for (int k = 0; k < 8; ++k) {
                        const int nx = cx + dx[k];
                        const int ny = cy + dy[k];
                        if (nx < 0 || ny < 0 || nx >= wi || ny >= hi) continue;
                        const std::size_t nidx =
                            static_cast<std::size_t>(ny) * w + nx;
                        if (!fire_mask_[nidx] || label[nidx] >= 0) continue;
                        label[nidx] = cid;
                        stack.push_back(nidx);
                    }
                }
            }
        }

        // 3) Drop stale tracks, then build one cluster per component and match
        //    it to the nearest existing track (nearest-neighbour association).
        prune_tracks(batch_t);
        result.reserve(comps.size());
        for (const auto& c : comps) {
            LifCluster out;
            const double mass = c.sm > 0.0 ? c.sm : 1.0;
            out.position = cv::Point2f(static_cast<float>(c.sx / mass),
                                       static_cast<float>(c.sy / mass));
            out.potential = threshold_;
            out.size = c.size;
            out.mass = static_cast<int>(c.sm);
            out.bbox = cv::Rect(c.minx, c.miny,
                                c.maxx - c.minx + 1,
                                c.maxy - c.miny + 1);
            out.last_t = batch_t;
            out.track_id = associate(out);
            result.push_back(out);
        }
        return result;
    }

    // Parameter accessors ---------------------------------------------------
    float tau_ms() const {
        return static_cast<float>(tau_us_) / 1000.0f;
    }
    float threshold() const { return static_cast<float>(threshold_); }
    float reset_value() const { return static_cast<float>(reset_value_); }
    void set_tau_ms(float v) {
        tau_us_ = static_cast<Metavision::timestamp>(v * 1000.0f);
        lif_.set_tau_us(tau_us_);
    }
    void set_threshold(float v) {
        threshold_ = static_cast<double>(v);
        lif_.set_threshold(threshold_);
    }
    void set_reset_value(float v) { reset_value_ = static_cast<double>(v); }

    /// @brief Returns the membrane-potential grid (CV_32F-compatible layout).
    const std::vector<double>& potential_grid() const {
        return lif_.potential_grid();
    }

    void reset() {
        lif_.clear();
        tracks_.clear();
        next_track_id_ = 0;
        std::fill(fire_mask_.begin(), fire_mask_.end(), uint8_t(0));
        std::fill(fire_count_.begin(), fire_count_.end(), 0);
    }

private:
    struct Track {
        int id{-1};
        cv::Point2f last_position;
        Metavision::timestamp last_t{0};
        float vx{0.0F};
        float vy{0.0F};
    };

    void prune_tracks(Metavision::timestamp now) {
        if (tracks_.size() <= 1) return;
        std::vector<Track> kept;
        kept.reserve(tracks_.size());
        // Keep tracks updated within stale_us_ of the current batch time.
        // (now < tr.last_t → non-monotonic clock → negative diff ≤ stale: keep.)
        for (auto& tr : tracks_) {
            if ((now - tr.last_t) <= stale_us_) kept.push_back(std::move(tr));
        }
        if (kept.empty() && !tracks_.empty()) kept.push_back(std::move(tracks_.front()));
        tracks_.swap(kept);
    }

    int associate(LifCluster& c) {
        const float tol2 = track_tol_px_ * track_tol_px_;
        int best_id = -1;
        float best_d2 = tol2;
        Track* best_track = nullptr;
        for (auto& tr : tracks_) {
            const float dx = tr.last_position.x - c.position.x;
            const float dy = tr.last_position.y - c.position.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_id = tr.id; best_track = &tr; }
        }
        if (best_id < 0) {
            best_id = next_track_id_++;
            tracks_.push_back(Track{best_id, c.position, c.last_t, 0.0F, 0.0F});
            c.vx = 0.0F;
            c.vy = 0.0F;
        } else {
            const float dt_s = (c.last_t > best_track->last_t)
                ? static_cast<float>(c.last_t - best_track->last_t) * 1e-6F
                : 0.0F;
            float vx = 0.0F;
            float vy = 0.0F;
            if (dt_s > 0.0F) {
                vx = (c.position.x - best_track->last_position.x) / dt_s;
                vy = (c.position.y - best_track->last_position.y) / dt_s;
            }
            best_track->vx = vx;
            best_track->vy = vy;
            best_track->last_position = c.position;
            best_track->last_t = c.last_t;
            c.vx = vx;
            c.vy = vy;
        }
        return best_id;
    }

    int width_;
    int height_;
    Metavision::timestamp tau_us_;
    double threshold_;
    double reset_value_;
    LifIntegrator lif_;
    float track_tol_px_;
    Metavision::timestamp stale_us_;
    std::vector<uint8_t> fire_mask_;
    std::vector<int> fire_count_;
    std::vector<Track> tracks_;
    int next_track_id_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_CLUSTER_LIF_H
