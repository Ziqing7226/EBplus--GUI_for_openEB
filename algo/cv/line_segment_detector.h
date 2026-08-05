// algo/cv/line_segment_detector.h — ELiSeD event-level line segment detection.
//
// 移植自 jAER ELiSeD (EBCCSP2016) 的**时间戳 Sobel 梯度**部分；线支撑区为
// 自研的"每包朝向分桶 + 矩拟合"（jAER 核心是逐事件生长/合并/分裂的持久
// LineSupport 区域，由 8000 事件环形缓冲驱动，未移植）。jAER 的
// predictTimestamps（邻居缺失/过期时取对侧像素时间戳保持梯度对称，
// 默认开启）也未移植。
// Ported from jAER's ch.unizh.ini.jaer.projects.elised.ELiSeD. The detector
// maintains per-polarity timestamp maps, computes a 3x3 Sobel convolution on
// the timestamp values (not on event counts), derives the level-line angle
// theta = atan2(-gy, gx), quantises it into orientation bins, and accumulates
// orientation-consistent events into line support regions. Segments are
// extracted from image moments (m00..m11): centroid (m10/m00, m01/m00),
// principal axis 0.5*atan2(2*m11, m20-m02), length from the major eigenvalue.
// Persistent ID association is used for tracking.
//
// Reference: C. Brandli, J. Strubel, S. Keller, D. Scaramuzza, T. Delbruck,
// "ELiSeD - An Event-Based Line Segment Detector," EBCCSP 2016.

#ifndef GUI_ALGO_CV_LINE_SEGMENT_DETECTOR_H
#define GUI_ALGO_CV_LINE_SEGMENT_DETECTOR_H

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

/// @brief Detected line segment with persistent tracking id.
struct LineSegment {
    cv::Point2f start;
    cv::Point2f end;
    float angle{0.0f};    ///< Orientation in degrees, [0, 180).
    int track_id{-1};     ///< Persistent track id, -1 if untracked.
};

/// @brief ELiSeD line segment detector ported from jAER (EBCCSP2016).
class LineSegmentDetector {
public:
    /// @brief Maximum age (us) for a neighbour timestamp to contribute to the
    /// Sobel timestamp gradient (jAER "maxAge", default 40000us).
    static constexpr int kDefaultMaxAgeUs = 40000;
    /// @brief Default number of level-line orientation bins.
    static constexpr int kDefaultNumOrientations = 4;

    LineSegmentDetector(int width, int height,
                        int min_line_length_px = 20,
                        float orientation_threshold = 0.7f,
                        int max_line_gap_px = 5)
        : width_(width), height_(height),
          min_line_length_px_(min_line_length_px),
          orientation_threshold_(orientation_threshold),
          max_line_gap_px_(max_line_gap_px),
          on_ts_(static_cast<std::size_t>(width) * height, -1),
          off_ts_(static_cast<std::size_t>(width) * height, -1) {}

    /// @brief Processes an event packet and returns detected line segments.
    std::vector<LineSegment> process(const EventPacket& packet) {
        std::vector<LineSegment> result;
        if (packet.empty()) return result;

        const int num_bins = num_orientations_;
        if (buckets_.size() != static_cast<std::size_t>(num_bins)) buckets_.resize(num_bins);
        for (auto& b : buckets_) b.clear();
        auto& buckets = buckets_;

        Metavision::timestamp now = -1;
        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            const std::size_t idx =
                static_cast<std::size_t>(e.y) * width_ + e.x;
            // Update the per-polarity timestamp map (jAER addEvent).
            std::vector<Metavision::timestamp>& ts_map = e.p ? on_ts_ : off_ts_;
            ts_map[idx] = e.t;
            if (e.t > now) now = e.t;

            float gx = 0.0f;
            float gy = 0.0f;
            if (!compute_gradient(e.x, e.y, e.p, e.t, gx, gy)) continue;

            // Level-line angle (jAER: vx = -gy, vy = gx; theta = atan2(vx,vy)).
            const float theta = std::atan2(-gy, gx);
            const int bin = quantize_orientation(theta, num_bins);
            if (bin < 0) continue;
            buckets[static_cast<std::size_t>(bin)].emplace_back(
                static_cast<float>(e.x), static_cast<float>(e.y));
        }

        // Prune tracks unmatched for > 2 s (§四-M6: the association
        // tolerance is only 5 px, so moving segments spawned a new track
        // every packet and the list grew unbounded).
        if (now >= 0) {
            tracks_.erase(
                std::remove_if(tracks_.begin(), tracks_.end(),
                               [now](const Track& tr) {
                                   return tr.last_seen >= 0 &&
                                          now - tr.last_seen > kTrackTimeoutUs;
                               }),
                tracks_.end());
        }

        for (int o = 0; o < num_bins; ++o) {
            if (buckets[static_cast<std::size_t>(o)].size() <
                static_cast<std::size_t>(min_line_length_px_)) {
                continue;
            }
            LineSegment seg;
            if (fit_line_from_moments(buckets[static_cast<std::size_t>(o)], seg)) {
                seg.track_id = associate(seg, now);
                result.push_back(seg);
            }
        }
        return result;
    }

    // Parameter accessors ---------------------------------------------------
    int min_line_length_px() const { return min_line_length_px_; }
    float orientation_threshold() const { return orientation_threshold_; }  // legacy no-op
    int max_line_gap_px() const { return max_line_gap_px_; }
    int max_age_us() const { return max_age_us_; }
    int num_orientations() const { return num_orientations_; }
    void set_min_line_length_px(int v) { min_line_length_px_ = v; }
    void set_orientation_threshold(float v) { orientation_threshold_ = v; }  // legacy no-op
    void set_max_line_gap_px(int v) { max_line_gap_px_ = v; }
    void set_max_age_us(int v) { max_age_us_ = v; }
    void set_num_orientations(int v) { num_orientations_ = v; }

    void reset() {
        std::fill(on_ts_.begin(), on_ts_.end(),
                  static_cast<Metavision::timestamp>(-1));
        std::fill(off_ts_.begin(), off_ts_.end(),
                  static_cast<Metavision::timestamp>(-1));
        tracks_.clear();
        next_track_id_ = 0;
    }

private:
    struct Track {
        int id{-1};
        cv::Point2f last_start;
        cv::Point2f last_end;
        Metavision::timestamp last_seen{-1};  ///< last match time (us)
    };

    static float dist2(const cv::Point2f& a, const cv::Point2f& b) {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    // 3x3 Sobel on the per-polarity timestamp map (jAER assignGradient with
    // useTimestampGradient=true, sobelWidth=3). Neighbours whose timestamp is
    // uninitialized or whose |deltaT| exceeds maxAge are skipped, and the
    // gradient is normalised by the sum of absolute kernel weights actually
    // used (jAER sumAbsSobelXFieldsUsed / sumAbsSobelYFieldsUsed).
    //   filterX = [-1,0,1,-2,0,2,-1,0,1]
    //   filterY = [-1,-2,-1,0,0,0,1,2,1]
    bool compute_gradient(std::uint16_t x, std::uint16_t y, short p,
                          Metavision::timestamp t,
                          float& gx, float& gy) const {
        static constexpr int kRadius = 1;
        static constexpr int kFilterX[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
        static constexpr int kFilterY[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

        const int px = static_cast<int>(x);
        const int py = static_cast<int>(y);
        if (px < kRadius || py < kRadius ||
            px >= width_ - kRadius || py >= height_ - kRadius) {
            return false;
        }
        const std::vector<Metavision::timestamp>& ts_map = p ? on_ts_ : off_ts_;
        const Metavision::timestamp max_age =
            static_cast<Metavision::timestamp>(max_age_us_);

        float sum_abs_x = 0.0f;
        float sum_abs_y = 0.0f;
        float sx = 0.0f;
        float sy = 0.0f;
        for (int h = 0; h < 3; ++h) {
            for (int w = 0; w < 3; ++w) {
                const int nx = px - kRadius + w;
                const int ny = py - kRadius + h;
                const Metavision::timestamp nt =
                    ts_map[static_cast<std::size_t>(ny) * width_ + nx];
                if (nt < 0) continue;  // uninitialized
                const Metavision::timestamp delta = nt - t;
                if (delta > max_age || delta < -max_age) continue;  // stale
                const int fx = kFilterX[w + h * 3];
                const int fy = kFilterY[w + h * 3];
                const float deltaT = static_cast<float>(delta);
                sx += deltaT * static_cast<float>(fx);
                sy += deltaT * static_cast<float>(fy);
                sum_abs_x += static_cast<float>(fx < 0 ? -fx : fx);
                sum_abs_y += static_cast<float>(fy < 0 ? -fy : fy);
            }
        }
        if (sum_abs_x < 1e-6f || sum_abs_y < 1e-6f) return false;
        gx = sx / sum_abs_x;
        gy = sy / sum_abs_y;
        return true;
    }

    // Quantise the level-line angle (radians) into [0, num_bins) modulo 180,
    // since a line has no direction (180-periodic).
    int quantize_orientation(float theta_rad, int num_bins) const {
        float deg = theta_rad * 180.0f / static_cast<float>(CV_PI);
        deg = std::fmod(deg, 180.0f);
        if (deg < 0.0f) deg += 180.0f;
        int bin = static_cast<int>(deg / 180.0f *
                                   static_cast<float>(num_bins));
        if (bin < 0 || bin >= num_bins) return -1;
        return bin;
    }

    /// @brief Image-moment line fit (jAER LineSupport.updateLineProperties):
    /// centroid (m10/m00, m01/m00); principal axis 0.5*atan2(2*mu11,
    /// mu20-mu02); length from the major eigenvalue
    /// (L = sqrt(12 * lambda1), uniform-segment assumption matching jAER's
    /// sqrt(6*(trace+disc)) formula).
    bool fit_line_from_moments(const std::vector<cv::Point2f>& pts,
                               LineSegment& seg) const {
        const std::size_t n = pts.size();
        if (n < static_cast<std::size_t>(min_line_length_px_)) return false;

        double m00 = 0.0, m10 = 0.0, m01 = 0.0;
        double m20 = 0.0, m02 = 0.0, m11 = 0.0;
        for (const auto& p : pts) {
            const double x = p.x;
            const double y = p.y;
            m00 += 1.0;
            m10 += x;
            m01 += y;
            m20 += x * x;
            m02 += y * y;
            m11 += x * y;
        }
        if (m00 <= 0.0) return false;
        const double cx = m10 / m00;
        const double cy = m01 / m00;
        // Central moments (covariance entries).
        const double mu20 = m20 / m00 - cx * cx;
        const double mu02 = m02 / m00 - cy * cy;
        const double mu11 = m11 / m00 - cx * cy;

        const double diff = mu20 - mu02;
        const double disc = std::sqrt(diff * diff + 4.0 * mu11 * mu11);
        const double lambda1 = 0.5 * (mu20 + mu02 + disc);  // major eigenvalue
        if (lambda1 <= 0.0) return false;

        // Principal axis of the major eigenvector.
        const double axis = 0.5 * std::atan2(2.0 * mu11, diff);
        // Uniform-segment length from the major eigenvalue
        // (variance = L^2/12  =>  L = sqrt(12 * lambda1)).
        const double length = std::sqrt(12.0 * lambda1);
        if (length < static_cast<double>(min_line_length_px_)) return false;

        const double dx = std::cos(axis);
        const double dy = std::sin(axis);
        const double half = length * 0.5;
        seg.start = cv::Point2f(static_cast<float>(cx - dx * half),
                                static_cast<float>(cy - dy * half));
        seg.end   = cv::Point2f(static_cast<float>(cx + dx * half),
                                static_cast<float>(cy + dy * half));
        float deg = static_cast<float>(axis * 180.0 / CV_PI);
        if (deg < 0.0f) deg += 180.0f;
        seg.angle = deg;
        return true;
    }

    /// @brief Associates a segment with an existing track by nearest endpoint.
    int associate(const LineSegment& seg, Metavision::timestamp now) {
        const float tol = static_cast<float>(max_line_gap_px_);
        const float tol2 = tol * tol;
        int best_id = -1;
        float best_d2 = tol2;
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            const float d = std::min(std::min(dist2(tracks_[i].last_start, seg.start),
                                              dist2(tracks_[i].last_end,   seg.end)),
                                     std::min(dist2(tracks_[i].last_start, seg.end),
                                              dist2(tracks_[i].last_end,   seg.start)));
            if (d < best_d2) { best_d2 = d; best_id = tracks_[i].id; }
        }
        if (best_id < 0) {
            best_id = next_track_id_++;
            tracks_.push_back(Track{best_id, seg.start, seg.end, now});
        } else {
            for (auto& tr : tracks_) {
                if (tr.id == best_id) {
                    tr.last_start = seg.start;
                    tr.last_end = seg.end;
                    tr.last_seen = now;
                    break;
                }
            }
        }
        return best_id;
    }

    static constexpr Metavision::timestamp kTrackTimeoutUs = 2000000;  // 2 s

    int width_;
    int height_;
    int min_line_length_px_;
    float orientation_threshold_;  ///< Legacy no-op (removed in dead-code commit).
    int max_line_gap_px_;
    int max_age_us_{kDefaultMaxAgeUs};
    int num_orientations_{kDefaultNumOrientations};
    std::vector<Metavision::timestamp> on_ts_;
    std::vector<Metavision::timestamp> off_ts_;
    std::vector<std::vector<cv::Point2f>> buckets_;  // reused per-packet (OPT-28)
    std::vector<Track> tracks_;
    int next_track_id_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_LINE_SEGMENT_DETECTOR_H
