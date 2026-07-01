// algo/cv/orientation_filter.h — 4-orientation edge labelling from time surfaces.
//
// Inspired by jAER AbstractOrientationFilter (design §4.3.7). For each event a
// 3x3 neighbourhood timestamp matrix is gathered (same polarity only); the
// freshness-weighted position covariance yields a principal eigenvector whose
// angle is quantised to one of 4 orientations (0/45/90/135 deg). A
// per-orientation temporal coincidence gate (jAER minDtThresholdUs) suppresses
// events whose along-orientation neighbours are too old, and a
// polarity-separated (two-channel) time surface avoids ON/OFF contamination.
// Output is an orientation index per event plus a colour for pseudo-colour
// rendering. Header-only.

#ifndef GUI_ALGO_CV_ORIENTATION_FILTER_H
#define GUI_ALGO_CV_ORIENTATION_FILTER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Quantises event edges into 4 orientations from a local time surface.
class OrientationFilter {
public:
    enum class ColorMap { Fixed4, HSV };

    /// @brief Orientation indices returned by classify().
    /// 0 = 0 deg, 1 = 45 deg, 2 = 90 deg, 3 = 135 deg, -1 = undetermined.
    static constexpr int kNumOrientations = 4;
    /// jAER receptive-field half-length along the orientation (default length=3).
    static constexpr int kRfLength = 3;

    OrientationFilter(int width, int height)
        : width_(width), height_(height),
          surface_(static_cast<std::size_t>(2) * static_cast<std::size_t>(width)
                       * static_cast<std::size_t>(height), 0) {}

    void set_time_window_us(int v) { time_window_us_ = clamp_i(v, 1000, 50000); }
    void set_min_neighbors(int v) { min_neighbors_ = clamp_i(v, 1, 8); }
    void set_color_map(ColorMap m) { color_map_ = m; }
    /// @brief jAER minDtThresholdUs: an orientation is emitted only if the
    /// per-orientation aggregate delta-time is below this (us). Default 100000.
    void set_min_dt_threshold_us(int v) { min_dt_threshold_us_ = clamp_i(v, 1, 1000000); }
    /// @brief multi-ori output (jAER multiOriOutputEnabled). false = WTA
    /// (winner-take-all, PCA orientation gated by coincidence); true = emit the
    /// best-coincidence orientation (jAER-style min-dt selection).
    void set_multi_ori_output(bool v) { multi_ori_output_ = v; }

    int time_window_us() const { return time_window_us_; }
    int min_neighbors() const { return min_neighbors_; }
    ColorMap color_map() const { return color_map_; }
    int min_dt_threshold_us() const { return min_dt_threshold_us_; }
    bool multi_ori_output() const { return multi_ori_output_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Classifies a single event, updating the internal time surface.
    /// @return Orientation index in [0,3] or -1 if too few recent neighbours or
    /// the per-orientation coincidence gate fails.
    int classify(const Event& e) {
        int orient = -1;
        if (e.x < width_ && e.y < height_) {
            orient = compute_orientation(e);
            const int pol = e.p ? 1 : 0;
            surface_[idx_of(e.x, e.y, pol)] = e.t;
        }
        return orient;
    }

    /// @brief Classifies a batch; fills @p out (resized to @p count).
    void process(const Event* events, std::size_t count, std::vector<int>& out) {
        out.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            out[i] = classify(events[i]);
        }
    }

    /// @brief Processes an event packet (updates the time surface).
    void process(EventPacket& events) {
        for (const auto& e : events) {
            (void)classify(e);
        }
    }

    /// @brief Renders an orientation index to an RGB colour.
    cv::Vec3b color(int orient) const {
        if (orient < 0 || orient >= kNumOrientations) return cv::Vec3b(0, 0, 0);
        if (color_map_ == ColorMap::HSV) {
            const int hue = orient * 45;               // 0,45,90,135 deg
            return hsv_to_bgr(hue, 255, 255);
        }
        // Fixed 4-colour palette.
        static const cv::Vec3b palette[kNumOrientations] = {
            cv::Vec3b(0, 0, 255),     // 0   deg -> red
            cv::Vec3b(0, 255, 0),     // 45  deg -> green
            cv::Vec3b(255, 0, 0),     // 90  deg -> blue
            cv::Vec3b(0, 255, 255),   // 135 deg -> yellow
        };
        return palette[orient];
    }

    void reset() { std::fill(surface_.begin(), surface_.end(), 0); }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    /// @brief Two-channel (polarity) time-surface index. @p p must be 0 or 1.
    std::size_t idx_of(int x, int y, int p) const {
        return static_cast<std::size_t>(p) * static_cast<std::size_t>(width_)
                   * static_cast<std::size_t>(height_)
             + static_cast<std::size_t>(y) * width_ + x;
    }

    /// @brief Per-orientation along-direction unit offsets (jAER baseOffsets).
    /// ori 0 = horizontal (1,0), 1 = 45 deg (1,1), 2 = vertical (0,1),
    /// 3 = 135 deg (-1,1).
    static constexpr int kBaseDx[kNumOrientations] = {1, 1, 0, -1};
    static constexpr int kBaseDy[kNumOrientations] = {0, 1, 1, 1};

    int compute_orientation(const Event& e) const {
        const double win = static_cast<double>(time_window_us_);
        const int pol = e.p ? 1 : 0;

        // --- PCA over the 3x3 same-polarity neighbourhood (self-developed) ---
        // The centre pixel's own previous timestamp is excluded from the
        // correlation (it is not a neighbour).
        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0, wsum = 0.0;
        int recent = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            const int ny = e.y + dy;
            if (ny < 0 || ny >= height_) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;  // exclude centre
                const int nx = e.x + dx;
                if (nx < 0 || nx >= width_) continue;
                const Metavision::timestamp lt = surface_[idx_of(nx, ny, pol)];
                if (lt == 0) continue;
                const double diff = static_cast<double>(e.t - lt);
                if (diff < 0.0 || diff > win) continue;
                const double w = 1.0 - diff / win;     // freshness weight
                const double px = static_cast<double>(nx);
                const double py = static_cast<double>(ny);
                sx += w * px;
                sy += w * py;
                sxx += w * px * px;
                syy += w * py * py;
                sxy += w * px * py;
                wsum += w;
                ++recent;
            }
        }
        int pca_orient = -1;
        if (recent >= min_neighbors_ && wsum > 0.0) {
            // Weighted covariance of positions (principal-axis orientation).
            const double cxx = sxx / wsum - (sx / wsum) * (sx / wsum);
            const double cyy = syy / wsum - (sy / wsum) * (sy / wsum);
            const double cxy = sxy / wsum - (sx / wsum) * (sy / wsum);
            double theta = 0.5 * std::atan2(2.0 * cxy, cxx - cyy); // [-pi/2, pi/2]
            if (theta < 0.0) theta += CV_PI;                       // [0, pi)
            const double deg = theta * 180.0 / CV_PI;
            int q = static_cast<int>(std::floor(deg / 45.0 + 0.5));
            if (q >= kNumOrientations) q = 0;
            pca_orient = q;
        }

        // --- jAER per-orientation coincidence gate (same polarity) ---
        // oridts[ori] = average delta-time over the along-orientation RF
        // offsets (length kRfLength, width 0). Delta times outside
        // [0, dtRejectThreshold] are rejected as outliers.
        const Metavision::timestamp dt_reject =
            static_cast<Metavision::timestamp>(min_dt_threshold_us_) * 5;
        const Metavision::timestamp dt_inf =
            std::numeric_limits<Metavision::timestamp>::max();
        std::array<Metavision::timestamp, kNumOrientations> oridts{
            dt_inf, dt_inf, dt_inf, dt_inf};
        for (int ori = 0; ori < kNumOrientations; ++ori) {
            const int bdx = kBaseDx[ori];
            const int bdy = kBaseDy[ori];
            Metavision::timestamp sum = 0;
            int count = 0;
            for (int s = -kRfLength; s <= kRfLength; ++s) {
                if (s == 0) continue;  // exclude centre
                const int nx = e.x + s * bdx;
                const int ny = e.y + s * bdy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
                const Metavision::timestamp lt = surface_[idx_of(nx, ny, pol)];
                if (lt == 0) continue;
                const Metavision::timestamp dt = e.t - lt;
                if (dt < 0 || dt > dt_reject) continue;
                sum += dt;
                ++count;
            }
            if (count > 0) oridts[ori] = sum / count;
        }

        // WTA / coincidence gate: find the orientation with the minimum
        // aggregate dt. If none is below min_dt_threshold_us_, suppress.
        int best_ori = -1;
        Metavision::timestamp best_dt = dt_inf;
        for (int ori = 0; ori < kNumOrientations; ++ori) {
            if (oridts[ori] < best_dt) {
                best_dt = oridts[ori];
                best_ori = ori;
            }
        }
        if (best_ori < 0 || best_dt >= min_dt_threshold_us_) return -1;

        if (multi_ori_output_) {
            // jAER multi-ori: emit the best-coincidence orientation.
            return best_ori;
        }
        // WTA: emit the self-developed PCA orientation; it is gated by the
        // global coincidence requirement (some orientation is coincident).
        return pca_orient;
    }

    static cv::Vec3b hsv_to_bgr(int h, int s, int v) {
        // OpenCV uses BGR; convert 8-bit HSV.
        cv::Mat m(1, 1, CV_8UC3, cv::Vec3b(
            static_cast<std::uint8_t>(h / 2),
            static_cast<std::uint8_t>(s),
            static_cast<std::uint8_t>(v)));
        cv::Mat out;
        cv::cvtColor(m, out, cv::COLOR_HSV2BGR);
        return out.at<cv::Vec3b>(0, 0);
    }

    int width_;
    int height_;
    int time_window_us_{10000};
    int min_neighbors_{2};
    int min_dt_threshold_us_{100000};  // jAER minDtThresholdUs
    bool multi_ori_output_{false};     // jAER multiOriOutputEnabled
    ColorMap color_map_{ColorMap::Fixed4};
    // Polarity-separated time surface: 2 * width * height, channel = polarity.
    std::vector<Metavision::timestamp> surface_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_ORIENTATION_FILTER_H
