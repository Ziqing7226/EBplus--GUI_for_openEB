// algo/cv/orientation_cluster.h — Per-event orientation filter.
//
// ✅ 移植自 jAER OrientationCluster
// (ref/jaer/src/ch/unizh/ini/jaer/projects/rccar/OrientationCluster.java).
//
// This is a faithful port of the jAER `OrientationCluster` per-event
// orientation filter (by braendch), replacing the previous self-developed
// 4-bin quantized + connected-components implementation. The algorithm:
//   - Maintains a continuous direction vector (vx, vy) per pixel, accumulated
//     from the past neighborhood (receptive field of size
//     (2*rf_width+1) x (2*rf_height+1)). Each neighbor contributes its
//     normalized offset (xx, yy) weighted by factor/dt, where dt is the
//     inter-event time gap (older neighbors contribute less).
//   - Anti-polarity neighbors are rotated by 90° (the contrast gradient is
//     perpendicular to the edge): left side → 90° CW, right side → 90° CCW.
//     Same-polarity neighbors below the center are point-inverted so that
//     every vector has a positive y-component (matches jAER's convention).
//   - oriHistoryMap provides time smoothing of the direction vector when
//     `ori_history_enabled` is on (actual vector blended with the previous one
//     via history_factor).
//   - Per-event filtering (no connected-components clustering): an event
//     passes when its orientation theta agrees with the neighborhood
//     orientation within `tolerance` (degrees), lies within `ori` (degrees)
//     of vertical, and the neighborhood vector length exceeds a y-dependent
//     threshold (neighbor_thr scaled by thr_gradient).
// Each passing event is emitted as an `OrientationClusterResult` (centroid =
// event position, orientation = edge-direction angle in radians from the
// x-axis so the backend's cos/sin overlay draws along the edge, size = a
// fixed display line length). Header-only.
//
// @note The class name is kept as `OrientationCluster` (rather than
// `OrientationClusterFilter`) because `gui/algo_bridge/algo_backend.cpp`
// references `gui_algo::OrientationCluster` directly and must not be modified.

#ifndef GUI_ALGO_CV_ORIENTATION_CLUSTER_H
#define GUI_ALGO_CV_ORIENTATION_CLUSTER_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief A per-event orientation annotation emitted by the filter.
///
/// One result is produced per event that passes the orientation filter
/// (jAER-style: the output is filtered events + orientation, not clusters).
struct OrientationClusterResult {
    cv::Point2f centroid;       ///< Event position.
    float orientation{0.0f};    ///< Edge-direction angle in radians from the
                                ///< x-axis (atan2(vy, vx)); compatible with the
                                ///< backend's cos/sin overlay drawing.
    int size{0};                ///< Display line length in pixels.
    int track_id{-1};           ///< Unused; retained for API compatibility.
};

/// @brief Per-event orientation filter, ported from jAER OrientationCluster.
///
/// Faithfully mirrors the jAER `filterPacket` loop: continuous direction
/// vectors, 1/dt-weighted neighborhood accumulation, 90° rotation for
/// anti-polarity neighbors, optional oriHistoryMap time smoothing, and the
/// three-condition per-event pass filter (tolerance, ori, neighborThr with
/// thrGradient). No connected-components clustering is performed.
class OrientationCluster {
public:
    /// @brief Constructs the filter for a sensor of @p width x @p height.
    /// Defaults mirror jAER's preference defaults.
    OrientationCluster(int width, int height,
                       float dt = 10000.0f,
                       float factor = 1000.0f,
                       int rf_width = 1,
                       int rf_height = 1,
                       float tolerance_deg = 10.0f,
                       float ori_deg = 45.0f,
                       float neighbor_thr = 10.0f,
                       float thr_gradient = 0.0f,
                       float history_factor = 1.0f,
                       bool use_opposite_polarity = true,
                       bool ori_history_enabled = false)
        : width_(width), height_(height),
          dt_(dt), factor_(factor),
          rf_width_(rf_width), rf_height_(rf_height),
          tolerance_deg_(tolerance_deg), ori_deg_(ori_deg),
          neighbor_thr_(neighbor_thr), thr_gradient_(thr_gradient),
          history_factor_(history_factor),
          use_opposite_polarity_(use_opposite_polarity),
          ori_history_enabled_(ori_history_enabled),
          vector_map_(static_cast<std::size_t>(width) * height),
          history_map_(static_cast<std::size_t>(width) * height) {
        rebuild_unit_table();
    }

    /// @brief Processes an event packet per-event; returns one
    /// `OrientationClusterResult` per event that passes the orientation filter.
    std::vector<OrientationClusterResult> process(const EventPacket& packet) {
        std::vector<OrientationClusterResult> result;
        if (packet.empty()) return result;
        result.reserve(packet.size());

        // jAER uses sizex = sizeX - 1, sizey = sizeY - 1 and requires the
        // neighbor to be strictly inside (0 < n < size-1).
        const int sizex = width_ - 1;
        const int sizey = height_ - 1;
        const float tol_rad = tolerance_deg_ * kPi / 180.0f;
        const float ori_rad = ori_deg_ * kPi / 180.0f;

        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            const int x = static_cast<int>(e.x);
            const int y = static_cast<int>(e.y);
            const std::size_t idx =
                static_cast<std::size_t>(y) * width_ + x;

            // --- Reset the accumulator for the current pixel and store its
            // polarity (0 = off, 1 = on), matching jAER's data index 3. ---
            Cell& cur = vector_map_[idx];
            cur.vx = 0.0f;
            cur.vy = 0.0f;
            cur.pol = e.is_on() ? 1 : 0;

            // Sums of the neighbor direction-vector components.
            float neighbor_x = 0.0f;
            float neighbor_y = 0.0f;

            // --- Iterate over the whole receptive field. ---
            for (int h = -rf_height_; h <= rf_height_; ++h) {
                for (int w = -rf_width_; w <= rf_width_; ++w) {
                    const int nx = x + w;
                    const int ny = y + h;
                    if (nx <= 0 || nx >= sizex ||
                        ny <= 0 || ny >= sizey) continue;
                    const std::size_t nidx =
                        static_cast<std::size_t>(ny) * width_ + nx;
                    const Cell& nb = vector_map_[nidx];

                    // Time regression (e.t < nb.ts, e.g. seek without reset):
                    // skip the neighbor — factor/dt would otherwise go
                    // negative or blow up (§四-低9; jAER 原版同样不防).
                    if (e.t < nb.ts) continue;
                    // Time gap (+1 to avoid division by zero), matches jAER.
                    const float t = static_cast<float>(e.t - nb.ts) + 1.0f;
                    if (t >= dt_) continue;

                    // Precomputed unit vector for this (w, h) offset.
                    // OPT-29: (xx, yy) and vl depend only on (w, h, same_pol),
                    // not on runtime event data — safe to precompute.
                    const bool same_pol = (cur.pol == nb.pol);
                    // useOppositePolarity=false: jAER 因未重置 xx/yy 会累加
                    // 陈旧向量（jAER bug）；本实现直接跳过该邻居（§二-2.8）。
                    if (!same_pol && !use_opposite_polarity_) continue;

                    const std::size_t uidx =
                        static_cast<std::size_t>(h + rf_height_) *
                            (2 * rf_width_ + 1) +
                        static_cast<std::size_t>(w + rf_width_);
                    const UnitVec& uv = unit_table_[uidx];
                    if (!uv.valid) continue;  // vl == 0

                    // Normalized offset weighted by factor/dt.
                    const double w_decay = factor_ / t;
                    if (same_pol) {
                        cur.vx += static_cast<float>(uv.same_x * w_decay);
                        cur.vy += static_cast<float>(uv.same_y * w_decay);
                    } else {
                        cur.vx += static_cast<float>(uv.diff_x * w_decay);
                        cur.vy += static_cast<float>(uv.diff_y * w_decay);
                    }

                    // Neighborhood vector (optionally blended with history).
                    if (ori_history_enabled_) {
                        const Cell& hn = history_map_[nidx];
                        neighbor_x += nb.vx + history_factor_ * hn.vx;
                        neighbor_y += nb.vy + history_factor_ * hn.vy;
                    } else {
                        neighbor_x += nb.vx;
                        neighbor_y += nb.vy;
                    }
                }
            }

            // Neighborhood orientation and length (jAER uses atan(x/y)).
            const float neighbor_length =
                std::sqrt(neighbor_x * neighbor_x + neighbor_y * neighbor_y);
            const float neighbor_theta = std::atan(neighbor_x / neighbor_y);

            // Current event orientation theta = atan(vx/vy) (angle from the
            // vertical y-axis, jAER convention); blended with history when
            // enabled.
            float vx_eff = cur.vx;
            float vy_eff = cur.vy;
            if (ori_history_enabled_) {
                const Cell& hc = history_map_[idx];
                vx_eff = cur.vx + history_factor_ * hc.vx;
                vy_eff = cur.vy + history_factor_ * hc.vy;
            }
            const float theta = std::atan(vx_eff / vy_eff);

            // Update the history map (mirrors jAER: history := current).
            Cell& hc = history_map_[idx];
            hc.vx = cur.vx;
            hc.vy = cur.vy;
            hc.ts = cur.ts;
            hc.theta = theta;

            // --- Output: emit one result per event passing the filter. ---
            // The three jAER conditions: non-zero vector, orientation within
            // tolerance of the neighborhood, within `ori` of vertical, and a
            // strong-enough neighborhood vector (y-dependent threshold).
            if (cur.vx != 0.0f && cur.vy != 0.0f) {
                const float dy_thr = neighbor_thr_ *
                    (1.0f - (thr_gradient_ * static_cast<float>(y)) /
                            static_cast<float>(sizey));
                if (std::fabs(theta - neighbor_theta) < tol_rad &&
                    std::fabs(theta) < ori_rad &&
                    neighbor_length > dy_thr) {
                    OrientationClusterResult r;
                    r.centroid = cv::Point2f(static_cast<float>(x),
                                             static_cast<float>(y));
                    // Present the angle from the x-axis (atan2(vy, vx)) so the
                    // backend's cos(orientation)/sin(orientation) overlay
                    // draws along the edge. Internally theta is from the
                    // y-axis (jAER); PI/2 - theta == atan2(vy, vx) for vy>=0.
                    r.orientation = kPi / 2.0f - theta;
                    r.size = display_length_;
                    r.track_id = -1;
                    result.push_back(r);
                }
            }

            // Store the timestamp last (jAER does this after the RF loop).
            cur.ts = e.t;
        }
        return result;
    }

    // --- Parameter accessors (jAER-style) ---------------------------------

    float dt() const { return dt_; }
    float factor() const { return factor_; }
    int rf_width() const { return rf_width_; }
    int rf_height() const { return rf_height_; }
    float tolerance() const { return tolerance_deg_; }
    float ori() const { return ori_deg_; }
    float neighbor_thr() const { return neighbor_thr_; }
    float thr_gradient() const { return thr_gradient_; }
    float history_factor() const { return history_factor_; }
    bool use_opposite_polarity() const { return use_opposite_polarity_; }
    bool ori_history_enabled() const { return ori_history_enabled_; }

    void set_dt(float v) { dt_ = v; }
    void set_factor(float v) { factor_ = v; }
    // Clamped to [1, 64] (§四-低9): a negative value made rebuild_unit_table()
    // compute cols <= 0 and attempt a huge allocation.
    void set_rf_width(int v) { rf_width_ = clamp_rf(v); rebuild_unit_table(); }
    void set_rf_height(int v) { rf_height_ = clamp_rf(v); rebuild_unit_table(); }
    void set_tolerance(float v) { tolerance_deg_ = v; }
    void set_ori(float v) { ori_deg_ = v; }
    void set_neighbor_thr(float v) { neighbor_thr_ = v; }
    // Clamped to [0, 1] (jAER thrGradient 语义；§四-低9): values > 1 made
    // dy_thr negative for large y, silently disabling the neighbor threshold.
    void set_thr_gradient(float v) {
        thr_gradient_ = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    }
    void set_history_factor(float v) { history_factor_ = v; }
    void set_use_opposite_polarity(bool v) { use_opposite_polarity_ = v; }
    void set_ori_history_enabled(bool v) { ori_history_enabled_ = v; }

    /// @brief Backend-compatibility shim. The previous implementation used
    /// connected-components clustering with a min-cluster-size threshold; this
    /// port is a per-event filter with no clustering, so the value is stored
    /// but has no effect on filtering. Retained so `algo_backend.cpp`'s
    /// "min_events" parameter continues to compile.
    void set_min_cluster_size(int v) { min_events_ = v; }
    int min_cluster_size() const { return min_events_; }

    /// @brief Display line length used for the `size` field of results.
    void set_display_length(int v) { display_length_ = v; }
    int display_length() const { return display_length_; }

    void reset() {
        for (Cell& c : vector_map_) c = Cell{};
        for (Cell& c : history_map_) c = Cell{};
    }

    /// @brief Rebuilds the precomputed unit-vector lookup table for the
    /// current receptive-field dimensions (OPT-29).
    ///
    /// For each (w, h) offset in the RF, precomputes the normalized offset
    /// (xx/vl, yy/vl) for both same-polarity and different-polarity cases.
    /// The (xx, yy) values depend only on (w, h) and the same_pol flag —
    /// not on runtime event data — so they can be safely precomputed.
    /// Stored as double to match the original arithmetic precision exactly.
    void rebuild_unit_table() {
        const int cols = 2 * rf_width_ + 1;
        const int rows = 2 * rf_height_ + 1;
        unit_table_.assign(static_cast<std::size_t>(cols) * rows, UnitVec{});
        for (int hh = -rf_height_; hh <= rf_height_; ++hh) {
            for (int ww = -rf_width_; ww <= rf_width_; ++ww) {
                const std::size_t idx =
                    static_cast<std::size_t>(hh + rf_height_) * cols +
                    (ww + rf_width_);
                // Same-polarity (xx, yy): ensure positive y-component.
                int sx = (hh < 0) ? -ww : ww;
                int sy = (hh < 0) ? -hh : hh;
                // Different-polarity (xx, yy): rotate 90°.
                int dx = (ww < 0) ?  hh : -hh;
                int dy = (ww < 0) ? -ww :  ww;
                // vl = sqrt(xx² + yy²) = sqrt(w² + h²), same for both cases.
                const double vl = std::sqrt(static_cast<double>(sx) * sx +
                                            static_cast<double>(sy) * sy);
                if (vl > 0.0) {
                    unit_table_[idx].same_x = sx / vl;
                    unit_table_[idx].same_y = sy / vl;
                    unit_table_[idx].diff_x = dx / vl;
                    unit_table_[idx].diff_y = dy / vl;
                    unit_table_[idx].valid = true;
                }
            }
        }
    }

private:
    struct Cell {
        float vx{0.0f};                   ///< x-component of direction vector.
        float vy{0.0f};                   ///< y-component of direction vector.
        Metavision::timestamp ts{0};      ///< Last event timestamp at pixel.
        float theta{0.0f};                ///< Last computed orientation.
        int pol{0};                       ///< Polarity: 0 = off, 1 = on.
    };

    static constexpr float kPi = 3.14159265358979323846f;

    static int clamp_rf(int v) { return v < 1 ? 1 : (v > 64 ? 64 : v); }

    int width_;
    int height_;
    float dt_;                    ///< Time criterion for selection (us).
    float factor_;                ///< Excitatory synapse weight.
    int rf_width_;                ///< Receptive-field half width.
    int rf_height_;               ///< Receptive-field half height.
    float tolerance_deg_;         ///< Tolerated deviation from neighbor (deg).
    float ori_deg_;               ///< Tolerated deviation from vertical (deg).
    float neighbor_thr_;          ///< Min neighborhood vector length.
    float thr_gradient_;          ///< Slope of the neighbor-vector-gradient.
    float history_factor_;        ///< History blend weight.
    bool use_opposite_polarity_;  ///< Use anti-polarity neighbors (rotated).
    bool ori_history_enabled_;    ///< Enable oriHistoryMap time smoothing.
    int display_length_{10};      ///< Overlay line length (result.size).
    int min_events_{0};           ///< Stored, unused (backend compat).
    std::vector<Cell> vector_map_;
    std::vector<Cell> history_map_;

    /// @brief Precomputed unit vectors for each RF offset (OPT-29).
    /// Indexed by (h + rf_height_) * (2*rf_width_+1) + (w + rf_width_).
    struct UnitVec {
        double same_x{0.0}, same_y{0.0};  ///< Unit vec for same-polarity.
        double diff_x{0.0}, diff_y{0.0};  ///< Unit vec for different-polarity.
        bool valid{false};                 ///< false when (w,h) == (0,0).
    };
    std::vector<UnitVec> unit_table_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_ORIENTATION_CLUSTER_H
