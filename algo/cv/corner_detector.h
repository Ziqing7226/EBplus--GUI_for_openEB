// algo/cv/corner_detector.h — event-native corner detection and tracking.
//
// Three modes:
//   EndStopped      — ✅ 移植自 jAER EndStoppedOrientationLabeler. End-stopped
//                     cell simulation: for each event a 4-bin orientation is
//                     computed from a 2-channel (polarity) time surface via
//                     PCA of the 3x3 neighbour timestamp deltas; the receptive
//                     field is then walked along both the orientation and its
//                     opposite — if the run of recently-active pixels ends on
//                     BOTH sides within end_stopped_distance, a corner is
//                     emitted (a line ending in both directions).
//   TypeCoincidence — ✅ 移植自 jAER TypeCoincidenceFilter. Each event is
//                     oriented via the same PCA; if any neighbour in a
//                     configurable radius stores an ORTHOGONAL orientation
//                     ((ori+numOri/2)%numOri) whose timestamp is within
//                     coincidence_window_us, a corner is emitted (a corner is
//                     where two orthogonal edges coincide spatio-temporally).
//   Harris          — frame-based Harris on the accumulation frame
//                     (cv::cornerHarris) mapped back to event positions.
//                     (untouched, self-developed design §4.3.12)
// Detected corners are tracked by nearest-neighbour matching; tracks shorter
// than min_track_len are suppressed. Output: vector<Corner>. Header-only.

#ifndef GUI_ALGO_CV_CORNER_DETECTOR_H
#define GUI_ALGO_CV_CORNER_DETECTOR_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief A detected and tracked corner.
struct Corner {
    float x{0.0F};
    float y{0.0F};
    float strength{0.0F};
    int track_id{-1};
    std::vector<cv::Point2f> trajectory;
};

/// @brief Multi-mode corner detector with nearest-neighbour tracking.
class CornerDetector {
public:
    enum class Mode { EndStopped, TypeCoincidence, Harris };

    CornerDetector(int width, int height, Mode mode = Mode::EndStopped)
        : width_(width), height_(height), mode_(mode) {
        reset();
    }

    // Parameters (defaults per design §4.3.12) ----------------------------
    void set_mode(Mode m) { mode_ = m; }
    void set_accumulation_ms(double v) { accumulation_ms_ = clamp_d(v, 1.0, 100.0); }
    void set_threshold(double v) { threshold_ = clamp_d(v, 1e-6, 1.0); }
    void set_track_radius_px(int v) { track_radius_px_ = clamp_i(v, 1, 30); }
    void set_min_track_len(int v) { min_track_len_ = clamp_i(v, 1, 100); }
    void set_output_hz(int v) { output_hz_ = clamp_i(v, 10, 500); }

    // TypeCoincidence params (✅ 移植自 jAER TypeCoincidenceFilter) ----------
    /// @brief jAER minDtThreshold: temporal coincidence window in us.
    void set_coincidence_window_us(int v) { coincidence_window_us_ = clamp_i(v, 1, 1000000); }
    /// @brief jAER dist: neighbourhood radius (pixels) searched for an
    ///        orthogonal-orientation coincidence.
    void set_neighborhood_radius(int v) { neighborhood_radius_ = clamp_i(v, 1, 5); }

    // EndStopped params (✅ 移植自 jAER EndStoppedOrientationLabeler) --------
    /// @brief jAER endStoppedLength: half-RF length walked along the orientation.
    void set_end_stopped_distance(int v) { end_stopped_distance_ = clamp_i(v, 1, 6); }
    /// @brief jAER maxDtToUse: max age (us) for a pixel to count as active.
    void set_max_age_us(int v) { max_age_us_ = clamp_i(v, 1000, 1000000); }

    // Shared orientation params (jAER NUM_TYPES = 4) ----------------------
    /// @brief Number of orientation bins. Clamped to exactly 4: the
    /// EndStopped walker tables kBaseDx/kBaseDy have only 4 entries and are
    /// indexed with %4, so values > 4 would misalign bin semantics (§四-低6).
    void set_num_orientations(int v) { num_orientations_ = clamp_i(v, 4, 4); }

    Mode mode() const { return mode_; }
    double accumulation_ms() const { return accumulation_ms_; }
    double threshold() const { return threshold_; }
    int track_radius_px() const { return track_radius_px_; }
    int min_track_len() const { return min_track_len_; }
    int output_hz() const { return output_hz_; }
    int width() const { return width_; }
    int height() const { return height_; }
    int coincidence_window_us() const { return coincidence_window_us_; }
    int neighborhood_radius() const { return neighborhood_radius_; }
    int end_stopped_distance() const { return end_stopped_distance_; }
    int max_age_us() const { return max_age_us_; }
    int num_orientations() const { return num_orientations_; }

    /// @brief Accumulates events; runs detection + tracking when the
    ///        accumulation window elapses, emitting corners at output_hz.
    void process(const Event* events, std::size_t count) {
        ensure_mats();
        ensure_event_state();
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const int x = static_cast<int>(e.x);
            const int y = static_cast<int>(e.y);
            // Frame accumulation (used by Harris; harmless for event modes).
            float& a = accum_.at<float>(y, x);
            if (a < 1e6F) a += 1.0F;
            if (e.p) {
                float& on = on_.at<float>(y, x);
                if (on < 1e6F) on += 1.0F;
            } else {
                float& off = off_.at<float>(y, x);
                if (off < 1e6F) off += 1.0F;
            }
            // Event-native corner detection (per-event).
            if (mode_ == Mode::TypeCoincidence) {
                process_event_type_coincidence(e, x, y);
            } else if (mode_ == Mode::EndStopped) {
                process_event_end_stopped(e, x, y);
            }
            if (e.t > last_event_t_) last_event_t_ = e.t;
        }
        if (last_event_t_ - last_frame_t_ >= accum_us()) {
            detect_and_track();
            last_frame_t_ = last_event_t_;
        }
        if (last_event_t_ - last_emit_t_ >= emit_us()) {
            emit_corners();
            last_emit_t_ = last_event_t_;
        }
    }

    /// @brief Processes an event packet.
    void process(EventPacket& events) {
        process(events.data(), events.size());
    }

    /// @brief Returns the most recently emitted corners.
    const std::vector<Corner>& corners() const { return corners_; }

    void reset() {
        accum_ = cv::Mat();
        on_ = cv::Mat();
        off_ = cv::Mat();
        ori_surface_.clear();
        last_ori_.clear();
        last_ori_t_.clear();
        event_detected_.clear();
        tracks_.clear();
        corners_.clear();
        last_event_t_ = 0;
        last_frame_t_ = 0;
        last_emit_t_ = 0;
        next_track_id_ = 0;
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static double clamp_d(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    Metavision::timestamp accum_us() const {
        return static_cast<Metavision::timestamp>(accumulation_ms_ * 1000.0);
    }
    Metavision::timestamp emit_us() const {
        return static_cast<Metavision::timestamp>(1e6 / static_cast<double>(output_hz_));
    }

    void ensure_mats() {
        if (accum_.empty()) {
            accum_ = cv::Mat::zeros(height_, width_, CV_32FC1);
            on_ = cv::Mat::zeros(height_, width_, CV_32FC1);
            off_ = cv::Mat::zeros(height_, width_, CV_32FC1);
        }
    }

    void ensure_event_state() {
        if (ori_surface_.empty()) {
            ori_surface_.assign(
                static_cast<std::size_t>(2) * static_cast<std::size_t>(width_)
                    * static_cast<std::size_t>(height_),
                -1);  // -1 = never seen (0 is a legal timestamp, §四-低8)
            last_ori_.assign(
                static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_),
                -1);
            last_ori_t_.assign(
                static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_),
                0);
        }
    }

    /// @brief Two-channel (polarity) time-surface index. @p p must be 0 or 1.
    std::size_t idx_of(int x, int y, int p) const {
        return static_cast<std::size_t>(p) * static_cast<std::size_t>(width_)
                   * static_cast<std::size_t>(height_)
             + static_cast<std::size_t>(y) * static_cast<std::size_t>(width_)
             + static_cast<std::size_t>(x);
    }

    /// @brief Per-orientation along-direction unit offsets (jAER baseOffsets).
    /// ori 0 = horizontal (1,0), 1 = 45 deg (1,1), 2 = vertical (0,1),
    /// 3 = 135 deg (-1,1). EndStopped walks these integer offsets.
    static constexpr int kBaseDx[4] = {1, 1, 0, -1};
    static constexpr int kBaseDy[4] = {0, 1, 1, 1};

    /// @brief PCA orientation from the 3x3 same-polarity time surface (same
    ///        method as orientation_filter.h). Returns a bin in
    ///        [0,num_orientations_) or -1 if too few recent neighbours.
    int compute_orientation_pca(const Event& e) const {
        const double win = static_cast<double>(ori_time_window_us_);
        const int pol = e.p ? 1 : 0;
        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0, wsum = 0.0;
        int recent = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            const int ny = static_cast<int>(e.y) + dy;
            if (ny < 0 || ny >= height_) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;  // exclude centre
                const int nx = static_cast<int>(e.x) + dx;
                if (nx < 0 || nx >= width_) continue;
                const Metavision::timestamp lt = ori_surface_[idx_of(nx, ny, pol)];
                if (lt < 0) continue;  // -1 = never seen
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
        if (recent < ori_min_neighbors_ || wsum <= 0.0) return -1;
        const double cxx = sxx / wsum - (sx / wsum) * (sx / wsum);
        const double cyy = syy / wsum - (sy / wsum) * (sy / wsum);
        const double cxy = sxy / wsum - (sx / wsum) * (sy / wsum);
        double theta = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);  // [-pi/2, pi/2]
        if (theta < 0.0) theta += CV_PI;                        // [0, pi)
        const double deg = theta * 180.0 / CV_PI;
        const double bin = 180.0 / static_cast<double>(num_orientations_);
        int q = static_cast<int>(std::floor(deg / bin + 0.5));
        if (q >= num_orientations_) q = 0;
        return q;
    }

    /// @brief True if pixel (x,y) has any-polarity activity within max_age_us.
    bool is_recent(int x, int y, Metavision::timestamp t) const {
        const Metavision::timestamp lt0 = ori_surface_[idx_of(x, y, 0)];
        const Metavision::timestamp lt1 = ori_surface_[idx_of(x, y, 1)];
        Metavision::timestamp lt = lt0;
        if (lt1 > lt) lt = lt1;
        if (lt < 0) return false;  // -1 = never seen
        const Metavision::timestamp dt = t - lt;
        return dt >= 0 && dt <= max_age_us_;
    }

    // ✅ 移植自 jAER TypeCoincidenceFilter -------------------------------
    // For each event: orient it via PCA, then search a (2r+1)^2 neighbourhood
    // for a pixel whose stored orientation is orthogonal ((ori+numOri/2)%numOri)
    // and whose timestamp is within coincidence_window_us. If found, emit a
    // corner. The per-pixel (orientation, timestamp) map is single-channel
    // (any polarity), per the port spec.
    void process_event_type_coincidence(const Event& e, int x, int y) {
        const int ori = compute_orientation_pca(e);
        // Update the polarity time surface (after PCA, before storing).
        const int pol = e.p ? 1 : 0;
        ori_surface_[idx_of(x, y, pol)] = e.t;

        bool corner = false;
        if (ori >= 0) {
            const int orth = (ori + num_orientations_ / 2) % num_orientations_;
            const int r = neighborhood_radius_;
            for (int dy = -r; dy <= r && !corner; ++dy) {
                const int ny = y + dy;
                if (ny < 0 || ny >= height_) continue;
                for (int dx = -r; dx <= r && !corner; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = x + dx;
                    if (nx < 0 || nx >= width_) continue;
                    const std::size_t ni =
                        static_cast<std::size_t>(ny) * static_cast<std::size_t>(width_)
                        + static_cast<std::size_t>(nx);
                    if (last_ori_[ni] != orth) continue;
                    const Metavision::timestamp dt = e.t - last_ori_t_[ni];
                    if (dt >= 0 && dt <= coincidence_window_us_) {
                        corner = true;
                    }
                }
            }
        }
        // Store current (orientation, timestamp) at pixel.
        const std::size_t ci =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width_)
            + static_cast<std::size_t>(x);
        last_ori_[ci] = ori;
        last_ori_t_[ci] = e.t;
        if (corner) {
            event_detected_.push_back(
                Detected{static_cast<float>(x), static_cast<float>(y), 1.0F});
        }
    }

    // ✅ 移植自 jAER EndStoppedOrientationLabeler ------------------------
    // For each event: orient it via PCA, then walk integer offsets along the
    // orientation and its opposite. A line ending exists in a direction if the
    // run of recently-active pixels stops within end_stopped_distance (at least
    // `threshold` active pixels, then an inactive/OOB pixel). If BOTH sides
    // have line endings, a corner is emitted.
    void process_event_end_stopped(const Event& e, int x, int y) {
        const int ori = compute_orientation_pca(e);
        const int pol = e.p ? 1 : 0;
        ori_surface_[idx_of(x, y, pol)] = e.t;
        if (ori < 0) return;

        const int opp = (ori + num_orientations_ / 2) % num_orientations_;
        const int dx0 = kBaseDx[ori % 4];
        const int dy0 = kBaseDy[ori % 4];
        const int dx1 = kBaseDx[opp % 4];
        const int dy1 = kBaseDy[opp % 4];

        const bool end0 = has_line_ending(x, y, dx0, dy0, e.t);
        const bool end1 = has_line_ending(x, y, dx1, dy1, e.t);
        if (end0 && end1) {
            event_detected_.push_back(
                Detected{static_cast<float>(x), static_cast<float>(y), 1.0F});
        }
    }

    /// @brief Walks (dx,dy) from (x,y). A line ending exists if the run of
    ///        recently-active pixels (d=1..) has length in [min_run, dmax] and
    ///        is followed by an inactive / out-of-bounds pixel.
    bool has_line_ending(int x, int y, int dx, int dy,
                         Metavision::timestamp t) const {
        const int dmax = end_stopped_distance_;
        const int min_run = std::max(1, static_cast<int>(threshold_));
        int runlen = 0;
        for (int d = 1; d <= dmax + 1; ++d) {
            const int px = x + d * dx;
            const int py = y + d * dy;
            if (px < 0 || px >= width_ || py < 0 || py >= height_) break;
            if (!is_recent(px, py, t)) break;
            ++runlen;
        }
        return runlen >= min_run && runlen <= dmax;
    }

    struct Detected { float x, y, strength; };

    void detect_and_track() {
        std::vector<Detected> detected;
        switch (mode_) {
            case Mode::EndStopped:
            case Mode::TypeCoincidence:
                detected.swap(event_detected_);
                break;
            case Mode::Harris:
                detect_harris(detected);
                break;
        }
        track(detected);
        // Reset accumulation frames after detection (Harris only needs this,
        // but resetting for all modes is harmless and keeps surfaces bounded).
        accum_.setTo(0.0F);
        on_.setTo(0.0F);
        off_.setTo(0.0F);
    }

    void detect_harris(std::vector<Detected>& out) {
        cv::Mat strength;
        cv::cornerHarris(accum_, strength, 3, 3, 0.04);
        cv::normalize(strength, strength, 0.0, 1.0, cv::NORM_MINMAX);
        collect_maxima(strength, out);
    }

    void collect_maxima(const cv::Mat& strength, std::vector<Detected>& out) {
        const double thr = threshold_;
        int ksize = std::min(15, std::max(3, track_radius_px_ * 2 + 1));
        if ((ksize & 1) == 0) ++ksize;  // ensure odd
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(ksize, ksize));
        cv::Mat dil;
        cv::dilate(strength, dil, kernel);
        cv::Mat mask = (strength == dil) & (strength > thr);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (mask.at<uchar>(y, x)) {
                    out.push_back(Detected{static_cast<float>(x),
                                           static_cast<float>(y),
                                           strength.at<float>(y, x)});
                }
            }
        }
    }

    struct Track {
        int id{-1};
        float x{0.0F}, y{0.0F};
        float strength{0.0F};
        std::vector<cv::Point2f> traj;
        int len{0};
        int miss{0};
        bool updated{false};
    };

    void track(const std::vector<Detected>& detected) {
        for (auto& t : tracks_) t.updated = false;
        const float r = static_cast<float>(track_radius_px_);
        const float r2 = r * r;
        for (const auto& d : detected) {
            int best = -1;
            float best_d2 = r2;
            for (int k = 0; k < static_cast<int>(tracks_.size()); ++k) {
                if (!tracks_[k].updated) {
                    const float dx = tracks_[k].x - d.x;
                    const float dy = tracks_[k].y - d.y;
                    const float d2 = dx * dx + dy * dy;
                    if (d2 < best_d2) { best_d2 = d2; best = k; }
                }
            }
            if (best < 0) {
                Track t;
                t.id = next_track_id_++;
                t.x = d.x;
                t.y = d.y;
                t.strength = d.strength;
                t.traj.push_back(cv::Point2f(d.x, d.y));
                t.len = 1;
                t.updated = true;
                tracks_.push_back(t);
            } else {
                Track& t = tracks_[best];
                const float a = 0.4F;
                t.x = t.x * (1.0F - a) + d.x * a;
                t.y = t.y * (1.0F - a) + d.y * a;
                t.strength = d.strength;
                t.traj.push_back(cv::Point2f(t.x, t.y));
                if (t.traj.size() > 200) t.traj.erase(t.traj.begin());
                ++t.len;
                t.updated = true;
                t.miss = 0;
            }
        }
        // Age and prune unmatched tracks.
        std::vector<Track> kept;
        kept.reserve(tracks_.size());
        for (auto& t : tracks_) {
            if (!t.updated) ++t.miss;
            if (t.miss <= 3) kept.push_back(std::move(t));
        }
        tracks_.swap(kept);
    }

    void emit_corners() {
        corners_.clear();
        for (const auto& t : tracks_) {
            if (t.len < min_track_len_) continue;
            Corner c;
            c.x = t.x;
            c.y = t.y;
            c.strength = t.strength;
            c.track_id = t.id;
            c.trajectory = t.traj;
            corners_.push_back(c);
        }
    }

    int width_;
    int height_;
    Mode mode_;
    double accumulation_ms_{10.0};
    double threshold_{0.1};
    int track_radius_px_{5};
    int min_track_len_{10};
    int output_hz_{100};

    // TypeCoincidence / EndStopped params (jAER ports).
    int coincidence_window_us_{10000};   // jAER minDtThreshold
    int neighborhood_radius_{1};         // jAER dist
    int num_orientations_{4};            // jAER NUM_TYPES
    int end_stopped_distance_{3};        // jAER endStoppedLength
    int max_age_us_{40000};              // jAER maxDtToUse
    int ori_time_window_us_{10000};      // PCA neighbour freshness window
    int ori_min_neighbors_{2};           // PCA min recent neighbours

    // Harris accumulation frames.
    cv::Mat accum_;
    cv::Mat on_;
    cv::Mat off_;

    // Event-native state.
    std::vector<Metavision::timestamp> ori_surface_;  // 2 * w * h (polarity)
    std::vector<int> last_ori_;                       // w * h, last bin or -1
    std::vector<Metavision::timestamp> last_ori_t_;   // w * h
    std::vector<Detected> event_detected_;            // per-window corners

    std::vector<Track> tracks_;
    std::vector<Corner> corners_;
    Metavision::timestamp last_event_t_{0};
    Metavision::timestamp last_frame_t_{0};
    Metavision::timestamp last_emit_t_{0};
    int next_track_id_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_CORNER_DETECTOR_H
