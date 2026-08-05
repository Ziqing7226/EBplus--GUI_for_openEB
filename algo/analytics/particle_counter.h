// algo/analytics/particle_counter.h — Generic particle counter.
//
// Design §4.4.5. Counts particles (e.g. on a conveyor/pipeline) using blob
// detection + tracking + a virtual counting line. Outputs particles/sec,
// cumulative count, and a size-distribution histogram. Inspired by the jAER
// particlecounter project. Header-only.

#ifndef GUI_ALGO_ANALYTICS_PARTICLE_COUNTER_H
#define GUI_ALGO_ANALYTICS_PARTICLE_COUNTER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"

namespace gui_algo {

/// @brief A detected/tracked particle blob.
struct Particle {
    int id{0};
    float cx{0.0f};          ///< Centroid X
    float cy{0.0f};          ///< Centroid Y
    float area{0.0f};        ///< Pixel area
    int last_bbox_x{0};      ///< Bounding-box left
    int last_bbox_y{0};      ///< Bounding-box top
    int last_bbox_w{0};      ///< Bounding-box width
    int last_bbox_h{0};      ///< Bounding-box height
    bool counted{false};     ///< Whether already counted (line crossing)
    int missed{0};           ///< Consecutive detection rounds without a match
};

/// @brief Counts particles via blob detection + tracking + counting line.
class ParticleCounter {
public:
    /// @brief Constructs the counter.
    /// @param width,height Sensor dimensions.
    /// @param min_size_px Minimum particle area in px, [1, 1000].
    /// @param max_size_px Maximum particle area in px, [10, 10000].
    ParticleCounter(int width, int height,
                    int min_size_px = 5,
                    int max_size_px = 100)
        : width_(width), height_(height),
          min_size_(clamp_size_min(min_size_px, max_size_px)),
          max_size_(clamp_size_max(max_size_px)),
          heatmap_(static_cast<std::size_t>(width) * height, 0) {
        // Default counting line: horizontal, at the vertical midpoint.
        line_y_ = height_ / 2;
    }

    /// @brief Accumulates a batch of events into the detection heatmap.
    void process(const Event* events, std::size_t n,
                 Metavision::timestamp t) {
        if (events == nullptr || n == 0) return;
        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const std::size_t idx =
                static_cast<std::size_t>(e.y) * width_ + e.x;
            if (heatmap_[idx] < 255) ++heatmap_[idx];
        }
        last_t_ = t;
    }

    /// @brief Runs blob detection + tracking + line-crossing counting.
    /// Call periodically (e.g. once per accumulation window).
    /// @param decay If > 0, decays the heatmap by this amount after detection.
    void detect_and_track(int decay = 1) {
        // Threshold the heatmap into a binary mask.
        cv::Mat mask(height_, width_, CV_8UC1);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * width_ + x;
                mask.at<std::uint8_t>(y, x) =
                    heatmap_[idx] > 0 ? 255 : 0;
            }
        }
        // Connected components with stats.
        cv::Mat labels, stats, centroids;
        const int n_labels =
            cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
        // Build the detection list (skip background label 0).
        std::vector<Particle> detections;
        for (int i = 1; i < n_labels; ++i) {
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < min_size_ || area > max_size_) continue;
            Particle p;
            p.area = static_cast<float>(area);
            p.cx = static_cast<float>(centroids.at<double>(i, 0));
            p.cy = static_cast<float>(centroids.at<double>(i, 1));
            p.last_bbox_x = stats.at<int>(i, cv::CC_STAT_LEFT);
            p.last_bbox_y = stats.at<int>(i, cv::CC_STAT_TOP);
            p.last_bbox_w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            p.last_bbox_h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            detections.push_back(p);
        }
        match_tracks(detections);
        check_line_crossings();
        // Decay the heatmap so stale pixels fade out.
        if (decay > 0) {
            for (auto& v : heatmap_) {
                if (v > 0) {
                    v = v > decay ? static_cast<std::uint8_t>(v - decay) : 0;
                }
            }
        }
        // Record size distribution (bounded history, §四-M5: ids are never
        // reused and detections accrue every round, so cap the deque).
        for (const Particle& d : detections) {
            size_hist_.push_back(static_cast<double>(d.area));
        }
        while (size_hist_.size() > kMaxSizeHist) size_hist_.pop_front();
        ++frame_count_;
    }

    /// @brief Sets the virtual counting line as a horizontal line at y.
    void set_counting_line_y(int y) {
        line_y_ = y < 0 ? 0 : (y >= height_ ? height_ - 1 : y);
    }
    int counting_line_y() const { return line_y_; }

    /// @brief Cumulative count of particles that crossed the counting line.
    std::uint64_t cumulative_count() const { return cumulative_count_; }

    /// @brief Particles per second over the recent window.
    double particles_per_sec() const {
        if (count_times_.size() < 2) return 0.0;
        const Metavision::timestamp span =
            count_times_.back() - count_times_.front();
        if (span <= 0) return 0.0;
        return static_cast<double>(count_times_.size()) *
               1.0e6 / static_cast<double>(span);
    }

    /// @brief Returns the size-distribution history (particle areas).
    const std::deque<double>& size_distribution() const { return size_hist_; }

    /// @brief Returns the currently tracked particles.
    const std::vector<Particle>& tracks() const { return tracks_; }

    /// @brief Renders the size-distribution histogram (CV_8UC3).
    cv::Mat render_size_hist(int img_w = 512, int img_h = 256,
                             int bins = 32) const {
        cv::Mat img(img_h, img_w, CV_8UC3, cv::Scalar(20, 20, 20));
        if (bins < 1) bins = 1;  // §四-低14: avoid division by zero below
        if (size_hist_.empty()) return img;
        const double lo = static_cast<double>(min_size_);
        const double hi = static_cast<double>(max_size_);
        std::vector<std::uint64_t> counts(bins, 0);
        for (const double a : size_hist_) {
            int b = static_cast<int>((a - lo) / (hi - lo) * bins);
            if (b < 0) b = 0;
            if (b >= bins) b = bins - 1;
            ++counts[b];
        }
        std::uint64_t max_c = 1;
        for (const auto c : counts) {
            if (c > max_c) max_c = c;
        }
        const int pad = 30;
        const int w = img.cols - 2 * pad;
        const int hh = img.rows - 2 * pad;
        const int bw = std::max(w / bins, 1);
        for (int i = 0; i < bins; ++i) {
            const int bh = static_cast<int>(
                static_cast<double>(counts[i]) / static_cast<double>(max_c) * hh);
            const int x = pad + i * bw;
            const int y = img.rows - pad - bh;
            cv::rectangle(img, cv::Rect(x, y, std::max(bw - 1, 1), bh),
                          cv::Scalar(255, 200, 100), cv::FILLED);
        }
        cv::putText(img, "particle size", cv::Point(pad, pad / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        return img;
    }

    void set_min_particle_size_px(int s) {
        min_size_ = clamp_size_min(s, max_size_);
    }
    int min_particle_size_px() const { return min_size_; }

    void set_max_particle_size_px(int s) {
        max_size_ = clamp_size_max(s);
        if (max_size_ < min_size_) min_size_ = max_size_;
    }
    int max_particle_size_px() const { return max_size_; }

    void reset() {
        std::fill(heatmap_.begin(), heatmap_.end(), 0);
        tracks_.clear();
        prev_cy_.clear();
        size_hist_.clear();
        count_times_.clear();
        cumulative_count_ = 0;
        next_id_ = 1;
        frame_count_ = 0;
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    static int clamp_size_max(int s) {
        if (s < 10) return 10;
        if (s > 10000) return 10000;
        return s;
    }
    static int clamp_size_min(int s, int smax) {
        if (s < 1) s = 1;
        if (s > 1000) s = 1000;
        if (s > smax) s = smax;
        return s;
    }

    /// @brief Nearest-neighbor matching of detections to existing tracks.
    void match_tracks(const std::vector<Particle>& detections) {
        const float kMaxDist = 50.0f;  // px association gate
        const int kMaxMissed = 3;       // drop tracks unmatched this many rounds
        std::vector<bool> det_used(detections.size(), false);
        for (Particle& tr : tracks_) {
            float best_d = kMaxDist;
            int best_i = -1;
            for (std::size_t i = 0; i < detections.size(); ++i) {
                if (det_used[i]) continue;
                const float dx = detections[i].cx - tr.cx;
                const float dy = detections[i].cy - tr.cy;
                const float d = std::sqrt(dx * dx + dy * dy);
                if (d < best_d) {
                    best_d = d;
                    best_i = static_cast<int>(i);
                }
            }
            if (best_i >= 0) {
                const Particle& d = detections[best_i];
                tr.cx = d.cx;
                tr.cy = d.cy;
                tr.area = d.area;
                tr.last_bbox_x = d.last_bbox_x;
                tr.last_bbox_y = d.last_bbox_y;
                tr.last_bbox_w = d.last_bbox_w;
                tr.last_bbox_h = d.last_bbox_h;
                tr.missed = 0;
                det_used[best_i] = true;
            } else {
                ++tr.missed;
            }
        }
        // Spawn new tracks for unmatched detections.
        for (std::size_t i = 0; i < detections.size(); ++i) {
            if (det_used[i]) continue;
            Particle p = detections[i];
            p.id = next_id_++;
            p.counted = false;
            p.missed = 0;
            prev_cy_[p.id] = p.cy;
            tracks_.push_back(p);
        }
        // Drop stale tracks (and their prev_cy_ entries — ids are never
        // reused, so the map would otherwise grow unbounded, §四-M5).
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [this, kMaxMissed](const Particle& t) {
                               if (t.missed > kMaxMissed) {
                                   prev_cy_.erase(t.id);
                                   return true;
                               }
                               return false;
                           }),
            tracks_.end());
    }

    /// @brief Counts particles whose centroid crossed the counting line.
    void check_line_crossings() {
        for (Particle& tr : tracks_) {
            if (tr.counted) continue;
            auto it = prev_cy_.find(tr.id);
            if (it == prev_cy_.end()) {
                prev_cy_[tr.id] = tr.cy;
                continue;
            }
            const float prev_y = it->second;
            it->second = tr.cy;
            // Crossing the horizontal line in either direction.
            if ((prev_y < line_y_ && tr.cy >= line_y_) ||
                (prev_y > line_y_ && tr.cy <= line_y_)) {
                tr.counted = true;
                ++cumulative_count_;
                count_times_.push_back(last_t_);
                // Keep a bounded window for the rate estimate (last 5 s).
                const Metavision::timestamp cutoff = last_t_ - 5000000;
                while (!count_times_.empty() &&
                       count_times_.front() < cutoff) {
                    count_times_.pop_front();
                }
            }
        }
    }

    int width_;
    int height_;
    int min_size_;
    int max_size_;
    int line_y_{0};
    std::vector<std::uint8_t> heatmap_;
    std::vector<Particle> tracks_;
    std::unordered_map<int, float> prev_cy_;  ///< previous centroid Y per track
    std::deque<double> size_hist_;
    static constexpr std::size_t kMaxSizeHist = 100000;  ///< size_hist_ cap
    std::deque<Metavision::timestamp> count_times_;
    std::uint64_t cumulative_count_{0};
    int next_id_{1};
    int frame_count_{0};
    Metavision::timestamp last_t_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_PARTICLE_COUNTER_H
