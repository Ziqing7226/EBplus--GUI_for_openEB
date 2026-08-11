// algo/cv/frequency_map.h — per-pixel flicker frequency/period estimation with
// frequency clustering and modulated-light source detection.
//
// Self-developed (design §4.3.x, vibration/flicker analysis): every pixel
// tracks the timestamps of its most recent ON and OFF events. A pixel's flicker
// period is estimated from consecutive SAME-polarity intervals — a full
// dark↔light cycle is ON-to-ON or OFF-to-OFF — and confirmed after
// frequency_filter_length stable periods (jitter tolerance
// period_diff_thresh_us, frequency band [min_freq_hz, max_freq_hz]). analyze() then clusters spatially adjacent pixels
// with similar frequency (8-neighbourhood BFS, tolerance
// max_cluster_frequency_diff) into light sources; clusters smaller than
// min_cluster_size are ignored. Frequencies older than stale_us are dropped so
// the map tracks the live scene.
//
// Output: per-pixel frequency map (CV_32F, Hz), cluster label map (CV_32S) and
// a light-source list (centroid, frequency, area). Header-only, no Qt.

#ifndef GUI_ALGO_CV_FREQUENCY_MAP_H
#define GUI_ALGO_CV_FREQUENCY_MAP_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Tunable parameters for FrequencyMap.
///
/// The defaults follow the estimation principles: the period jitter tolerance
/// is a small fraction of the shortest accepted period (1600 µs on an 8 Hz…
/// 120 Hz band), and the clustering tolerance keeps common AC-flicker
/// frequencies (50/60/100/120 Hz) separable.
struct FrequencyMapParams {
    /// Consecutive stable periods to confirm a frequency (median of the last
    /// N same-polarity intervals; N is clamped to [2, kMaxPeriods]).
    int frequency_filter_length{7};
    Metavision::timestamp period_diff_thresh_us{1600};  ///< Period jitter tolerance (µs).
    float min_freq_hz{8.0f};               ///< Minimum confirmed frequency (Hz).
    float max_freq_hz{120.0f};             ///< Maximum confirmed frequency (Hz).
    float max_cluster_frequency_diff{6.0f};  ///< Hz tolerance when clustering neighbours.
    int min_cluster_size{20};              ///< Min cluster pixels to report a source.
    Metavision::timestamp stale_us{200000}; ///< Drop frequencies with no events older than this.
};

/// @brief A detected flickering light source (frequency cluster).
struct FlickerSource {
    float x{0.0F};         ///< Centroid x (px)
    float y{0.0F};         ///< Centroid y (px)
    float frequency_hz{0.0F};  ///< Mean cluster frequency (Hz)
    int area{0};           ///< Cluster pixel count
    float radius_px{0.0F}; ///< sqrt(area / π)
};

/// @brief Per-pixel flicker frequency map + frequency clustering.
class FrequencyMap {
public:
    /// Maximum number of same-polarity intervals kept per pixel for the
    /// median confirmation (bounds per-pixel memory: 2×8 int32 = 64 B/px).
    static constexpr int kMaxPeriods = 8;

    FrequencyMap(int width, int height) : w_(width), h_(height) {
        const std::size_t n = static_cast<std::size_t>(w_) * h_;
        const std::array<int, kMaxPeriods> zero{};
        last_on_.assign(n, kNoTs);
        last_off_.assign(n, kNoTs);
        period_ring_on_.assign(n, zero);
        period_ring_off_.assign(n, zero);
        period_cnt_on_.assign(n, 0);
        period_cnt_off_.assign(n, 0);
        last_active_.assign(n, kNoTs);
        frequency_hz_ = cv::Mat_<float>::zeros(h_, w_);
        labels_ = cv::Mat_<int>::zeros(h_, w_);
    }

    void set_params(const FrequencyMapParams& p) { params_ = p; }
    const FrequencyMapParams& params() const { return params_; }

    /// @brief Updates per-pixel period estimation from a CD event batch.
    void process(const Event* begin, const Event* end) {
        if (!begin || !end || begin >= end) return;
        last_ts_ = (end - 1)->t;
        for (const Event* it = begin; it != end; ++it) {
            const int x = static_cast<int>(it->x);
            const int y = static_cast<int>(it->y);
            if (x < 0 || x >= w_ || y < 0 || y >= h_) continue;
            const std::size_t idx = static_cast<std::size_t>(y) * w_ + x;
            if (it->p != 0) {
                update_pixel(it->t, last_on_[idx], period_ring_on_[idx],
                             period_cnt_on_[idx], idx);
            } else {
                update_pixel(it->t, last_off_[idx], period_ring_off_[idx],
                             period_cnt_off_[idx], idx);
            }
            last_active_[idx] = it->t;
        }
    }

    /// @brief Recomputes frequency freshness and the clustered light sources.
    void analyze() {
        const Metavision::timestamp tmin =
            last_ts_ - params_.stale_us;
        // Drop stale frequencies so the map tracks the live scene.
        for (int y = 0; y < h_; ++y) {
            float* row = frequency_hz_.ptr<float>(y);
            const Metavision::timestamp* act = &last_active_[static_cast<std::size_t>(y) * w_];
            for (int x = 0; x < w_; ++x) {
                if (act[x] < tmin) row[x] = 0.0f;
            }
        }
        cluster_sources();
    }

    /// @brief Discards all temporal state.
    void reset() {
        const std::array<int, kMaxPeriods> zero{};
        std::fill(last_on_.begin(), last_on_.end(), kNoTs);
        std::fill(last_off_.begin(), last_off_.end(), kNoTs);
        std::fill(period_ring_on_.begin(), period_ring_on_.end(), zero);
        std::fill(period_ring_off_.begin(), period_ring_off_.end(), zero);
        std::fill(period_cnt_on_.begin(), period_cnt_on_.end(), 0);
        std::fill(period_cnt_off_.begin(), period_cnt_off_.end(), 0);
        std::fill(last_active_.begin(), last_active_.end(), kNoTs);
        frequency_hz_.setTo(0.0f);
        labels_.setTo(0);
        sources_.clear();
        last_ts_ = 0;
    }

    int width() const { return w_; }
    int height() const { return h_; }
    const cv::Mat& frequency_hz() const { return frequency_hz_; }
    const cv::Mat& cluster_labels() const { return labels_; }
    const std::vector<FlickerSource>& sources() const { return sources_; }

private:
    /// @brief Median of the first @p count intervals in @p ring. Plain-array
    /// insertion sort (n ≤ kMaxPeriods = 8) — std::sort on the std::array
    /// with a runtime end iterator trips -Werror=array-bounds.
    static double ring_median(const std::array<int, kMaxPeriods>& ring,
                              int count) {
        if (count <= 0) return 0.0;
        int tmp[kMaxPeriods];
        const int n = count < kMaxPeriods ? count : kMaxPeriods;
        for (int i = 0; i < n; ++i) tmp[i] = ring[i];
        for (int i = 1; i < n; ++i) {
            const int key = tmp[i];
            int j = i - 1;
            while (j >= 0 && tmp[j] > key) {
                tmp[j + 1] = tmp[j];
                --j;
            }
            tmp[j + 1] = key;
        }
        return static_cast<double>(tmp[n / 2]);
    }

    /// @brief Appends one same-polarity interval and confirms the frequency
    /// once the ring holds @c filter_length consistent intervals. The median
    /// (not the mean) is both the jitter reference and the confirmed period —
    /// a single glitched interval (e.g. a dropped event doubling the interval)
    /// cannot skew the estimate.
    void update_pixel(Metavision::timestamp t, Metavision::timestamp& last,
                      std::array<int, kMaxPeriods>& ring, int& ring_count,
                      std::size_t idx) {
        if (last == kNoTs) {
            last = t;
            return;
        }
        const double dt = static_cast<double>(t) - static_cast<double>(last);
        last = t;
        if (dt <= 0.0) return;
        const int filter_length =
            std::clamp(params_.frequency_filter_length, 2, kMaxPeriods);
        const double period = static_cast<double>(std::llround(dt));
        if (ring_count == 0) {
            ring[0] = static_cast<int>(period);
            ring_count = 1;
            return;
        }
        if (std::fabs(period - ring_median(ring, ring_count)) <=
            static_cast<double>(params_.period_diff_thresh_us)) {
            // Consistent with the ring: append (or slide the window if full).
            if (ring_count < kMaxPeriods) {
                ring[ring_count] = static_cast<int>(period);
                ++ring_count;
            } else {
                for (int i = 0; i < kMaxPeriods - 1; ++i) ring[i] = ring[i + 1];
                ring[kMaxPeriods - 1] = static_cast<int>(period);
            }
            if (ring_count >= filter_length) {
                // Confirmed: median period → frequency. Values outside
                // [min_freq, max_freq] are rejected (e.g. high-rate PWM
                // noise: its short intervals always fall within the absolute
                // µs tolerance) and the confirmation restarts.
                const double med = ring_median(ring, ring_count);
                const double freq = med > 0.0 ? 1e6 / med : 0.0;
                if (freq >= static_cast<double>(params_.min_freq_hz) &&
                    freq <= static_cast<double>(params_.max_freq_hz)) {
                    frequency_hz_.at<float>(static_cast<int>(idx / w_),
                                            static_cast<int>(idx % w_)) =
                        static_cast<float>(freq);
                }
                ring_count = 0;
            }
        } else {
            // Outlier interval: restart the confirmation from it.
            ring[0] = static_cast<int>(period);
            ring_count = 1;
        }
    }

    /// @brief BFS-clusters spatially adjacent, frequency-similar pixels.
    void cluster_sources() {
        labels_.setTo(0);
        sources_.clear();
        int next_label = 1;
        std::deque<std::pair<int, int>> queue;
        for (int y = 0; y < h_; ++y) {
            const float* frow = frequency_hz_.ptr<float>(y);
            for (int x = 0; x < w_; ++x) {
                const float f0 = frow[x];
                if (f0 <= 0.0f || labels_.at<int>(y, x) != 0) continue;
                // Seed a new cluster.
                const int label = next_label++;
                queue.clear();
                queue.emplace_back(x, y);
                labels_.at<int>(y, x) = label;
                double sum_x = 0, sum_y = 0, sum_f = 0;
                int count = 0;
                while (!queue.empty()) {
                    const auto [cx, cy] = queue.front();
                    queue.pop_front();
                    ++count;
                    sum_x += cx;
                    sum_y += cy;
                    sum_f += f0;  // seed frequency (uniform source)
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) continue;
                            const int nx = cx + dx, ny = cy + dy;
                            if (nx < 0 || nx >= w_ || ny < 0 || ny >= h_) continue;
                            if (labels_.at<int>(ny, nx) != 0) continue;
                            const float fn = frequency_hz_.at<float>(ny, nx);
                            if (fn <= 0.0f) continue;
                            if (std::fabs(fn - f0) > params_.max_cluster_frequency_diff) continue;
                            labels_.at<int>(ny, nx) = label;
                            queue.emplace_back(nx, ny);
                        }
                    }
                }
                if (count >= params_.min_cluster_size) {
                    FlickerSource src;
                    src.x = static_cast<float>(sum_x / count);
                    src.y = static_cast<float>(sum_y / count);
                    src.frequency_hz = static_cast<float>(sum_f / count);
                    src.area = count;
                    src.radius_px = static_cast<float>(std::sqrt(
                        static_cast<double>(count) / 3.14159265358979323846));
                    sources_.push_back(src);
                }
            }
        }
    }

    static constexpr Metavision::timestamp kNoTs =
        std::numeric_limits<Metavision::timestamp>::min();

    int w_{0};
    int h_{0};
    FrequencyMapParams params_;
    Metavision::timestamp last_ts_{0};

    std::vector<Metavision::timestamp> last_on_, last_off_;
    std::vector<std::array<int, kMaxPeriods>> period_ring_on_, period_ring_off_;
    std::vector<int> period_cnt_on_, period_cnt_off_;
    std::vector<Metavision::timestamp> last_active_;
    cv::Mat_<float> frequency_hz_;
    cv::Mat_<int> labels_;
    std::vector<FlickerSource> sources_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_FREQUENCY_MAP_H
