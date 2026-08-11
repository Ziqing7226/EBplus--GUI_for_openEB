// algo/analytics/freq_detector.h — Flickering light source frequency detection.
//
// Design §4.4.7. Detects relatively static flickering light sources (LEDs) by
// accumulating an event heatmap, clustering hot spots, then for each cluster
// gathering the event timestamps in a 3x3 region around the centroid, binning
// them, applying a Hann window, and running a DFT to find the dominant
// frequency peak (with parabolic interpolation + 2x harmonic confirmation).
// Frequency definition: event_freq = 2 * LED blink_freq. Inspired by the
// Lighthouse freq_analyzer tool. Header-only; uses a self-contained DFT (no
// external FFT library).

#ifndef GUI_ALGO_ANALYTICS_FREQ_DETECTOR_H
#define GUI_ALGO_ANALYTICS_FREQ_DETECTOR_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"

namespace gui_algo {

namespace freq_detail {
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxFFT = 8192;  // Cap on DFT input size to keep O(N*K) bounded.
}

/// @brief A detected flickering light source.
struct LightSource {
    int u{0};                 ///< Centroid column
    int v{0};                 ///< Centroid row
    float event_freq_hz{0.0f}; ///< Event frequency (2x blink frequency)
    float blink_freq_hz{0.0f}; ///< Physical LED blink frequency
};

/// @brief Flickering light source frequency detector (heatmap + DFT).
class FreqDetector {
public:
    /// @brief Constructs the detector.
    /// @param width,height Sensor dimensions.
    explicit FreqDetector(int width, int height)
        : width_(width), height_(height),
          heatmap_(static_cast<std::size_t>(width) * height, 0) {}

    /// @brief Accumulates a batch of events into the rolling buffer + heatmap.
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            buffer_.push_back(e);
            if (e.x < width_ && e.y < height_) {
                const std::size_t idx =
                    static_cast<std::size_t>(e.y) * width_ + e.x;
                ++heatmap_[idx];
            }
            if (e.t > latest_t_) latest_t_ = e.t;
        }
        prune();
    }

    /// @brief Runs the full detection pipeline and returns detected sources.
    /// Call periodically (respecting update_interval_s).
    std::vector<LightSource> analyze() {
        std::vector<LightSource> out;
        if (width_ <= 0 || height_ <= 0 || latest_t_ <= 0) return out;
        const Metavision::timestamp t_end = latest_t_;
        const Metavision::timestamp analysis_us =
            static_cast<Metavision::timestamp>(first_analysis_s_ * 1.0e6);
        const Metavision::timestamp max_us =
            static_cast<Metavision::timestamp>(max_duration_s_ * 1.0e6);
        const Metavision::timestamp t_start =
            (t_end > analysis_us) ? (t_end - analysis_us) : 0;
        const Metavision::timestamp window_lo =
            (t_end > max_us) ? (t_end - max_us) : 0;
        // Threshold the heatmap (restricted to the analysis window).
        cv::Mat mask(height_, width_, CV_8UC1, cv::Scalar(0));
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * width_ + x;
                if (heatmap_[idx] >= heatmap_threshold_) {
                    mask.at<std::uint8_t>(y, x) = 255;
                }
            }
        }
        cv::Mat labels, stats, centroids;
        const int n_labels =
            cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
        for (int i = 1; i < n_labels; ++i) {
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < min_cc_area_) continue;
            // Defensive cap: compute_frequency() scans the whole event buffer
            // per cluster (O(buffer)), so on busy scenes with many clusters the
            // analyze() would block the calling thread (SDK thread → GUI
            // freeze) for seconds. Report at most the kMaxSources largest
            // clusters.
            if (out.size() >= kMaxSources) break;
            const int u = static_cast<int>(centroids.at<double>(i, 0));
            const int v = static_cast<int>(centroids.at<double>(i, 1));
            LightSource src;
            src.u = u;
            src.v = v;
            compute_frequency(u, v, t_start, t_end, window_lo, src);
            out.push_back(src);
        }
        last_analyze_t_ = t_end;
        return out;
    }

    /// @brief Returns true if enough time has elapsed since the last analysis
    ///        for a new analysis to run (per update_interval_s).
    bool should_analyze() const {
        const Metavision::timestamp interval_us =
            static_cast<Metavision::timestamp>(update_interval_s_ * 1.0e6);
        return (latest_t_ - last_analyze_t_) >= interval_us;
    }

    /// @brief Renders the heatmap (Inferno colormap) as a CV_8UC3 image.
    cv::Mat render_heatmap() const {
        cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
        if (width_ <= 0 || height_ <= 0) return img;
        int max_c = heatmap_threshold_;
        for (const auto c : heatmap_) {
            if (c > max_c) max_c = c;
        }
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * width_ + x;
                const int cnt = heatmap_[idx];
                if (cnt < heatmap_threshold_) continue;
                const double v = max_c > heatmap_threshold_
                                     ? static_cast<double>(cnt - heatmap_threshold_) /
                                           static_cast<double>(max_c - heatmap_threshold_)
                                     : 0.0;
                img.at<cv::Vec3b>(y, x) = inferno_color(v);
            }
        }
        return img;
    }

    // Parameter setters / getters ----------------------------------------
    void set_f_min(float f) { f_min_ = clamp_fmin(f); }
    float f_min() const { return f_min_; }

    void set_f_max(float f) { f_max_ = clamp_fmax(f); }
    float f_max() const { return f_max_; }

    void set_bin_dt_us(float us) { bin_dt_us_ = clamp_bin_dt(us); }
    float bin_dt_us() const { return bin_dt_us_; }

    void set_heatmap_threshold(int t) {
        heatmap_threshold_ = (t < 1) ? 1 : (t > 1000 ? 1000 : t);
    }
    int heatmap_threshold() const { return heatmap_threshold_; }

    void set_min_cc_area(int a) {
        min_cc_area_ = (a < 1) ? 1 : (a > 100 ? 100 : a);
    }
    int min_cc_area() const { return min_cc_area_; }

    void set_region_radius(int r) {
        region_radius_ = (r < 0) ? 0 : (r > 5 ? 5 : r);
    }
    int region_radius() const { return region_radius_; }

    void set_peak_alpha(float a) {
        peak_alpha_ = (a < 1.0f) ? 1.0f : (a > 20.0f ? 20.0f : a);
    }
    float peak_alpha() const { return peak_alpha_; }

    void set_first_analysis_s(float s) {
        first_analysis_s_ = clamp_range(s, 0.5f, 10.0f);
    }
    float first_analysis_s() const { return first_analysis_s_; }

    void set_max_duration_s(float s) {
        max_duration_s_ = clamp_range(s, 5.0f, 120.0f);
    }
    float max_duration_s() const { return max_duration_s_; }

    void set_update_interval_s(float s) {
        update_interval_s_ = clamp_range(s, 0.1f, 5.0f);
    }
    float update_interval_s() const { return update_interval_s_; }

    /// @brief Clears the rolling buffer and heatmap.
    void reset() {
        buffer_.clear();
        std::fill(heatmap_.begin(), heatmap_.end(), 0);
        latest_t_ = 0;
        last_analyze_t_ = 0;
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    /// Max clusters reported per analyze() (defensive cost cap, see the
    /// cluster loop in analyze()).
    static constexpr std::size_t kMaxSources = 32;
    static float clamp_fmin(float f) {
        if (f < 10.0f) return 10.0f;
        if (f > 1000.0f) return 1000.0f;
        return f;
    }
    static float clamp_fmax(float f) {
        if (f < 1000.0f) return 1000.0f;
        if (f > 50000.0f) return 50000.0f;
        return f;
    }
    static float clamp_bin_dt(float us) {
        if (us < 10.0f) return 10.0f;
        if (us > 1000.0f) return 1000.0f;
        return us;
    }
    static float clamp_range(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static cv::Vec3b inferno_color(double v) {
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        const auto b8 = [](double x) {
            return static_cast<std::uint8_t>(x * 255.0 + 0.5);
        };
        double r = v * 2.0;
        double g = std::max(0.0, (v - 0.5) * 2.0);
        double b = std::max(0.0, 0.4 - 0.8 * v);
        if (r > 1.0) r = 1.0;
        return cv::Vec3b(b8(b), b8(g), b8(r));
    }

    void prune() {
        const Metavision::timestamp max_us =
            static_cast<Metavision::timestamp>(max_duration_s_ * 1.0e6);
        const Metavision::timestamp t_lo = latest_t_ - max_us;
        while (!buffer_.empty() && buffer_.front().t < t_lo) {
            const Event& e = buffer_.front();
            if (e.x < width_ && e.y < height_) {
                const std::size_t idx =
                    static_cast<std::size_t>(e.y) * width_ + e.x;
                if (heatmap_[idx] > 0) --heatmap_[idx];
            }
            buffer_.pop_front();
        }
    }

    /// @brief Computes the event frequency for one cluster via binned DFT.
    void compute_frequency(int u, int v,
                           Metavision::timestamp t_start,
                           Metavision::timestamp t_end,
                           Metavision::timestamp window_lo,
                           LightSource& out) const {
        // Gather timestamps from the region around the centroid.
        ts_.clear();
        auto& ts = ts_;
        ts_.reserve(64);
        for (const Event& e : buffer_) {
            if (e.t < window_lo) continue;
            if (std::abs(static_cast<int>(e.x) - u) <= region_radius_ &&
                std::abs(static_cast<int>(e.y) - v) <= region_radius_) {
                ts.push_back(e.t);
            }
        }
        if (ts.size() < 4) return;
        // Determine FFT window: most recent N bins (capped at kMaxFFT).
        const double bin_dt = static_cast<double>(bin_dt_us_);
        const double total_window =
            static_cast<double>(t_end - (t_start > 0 ? t_start : 0));
        int N = static_cast<int>(total_window / bin_dt);
        if (N < 4) return;
        if (N > freq_detail::kMaxFFT) N = freq_detail::kMaxFFT;
        const Metavision::timestamp t_fft_start =
            t_end - static_cast<Metavision::timestamp>(
                        static_cast<double>(N) * bin_dt);
        // Bin the timestamps.
        signal_.assign(N, 0.0);
        auto& signal = signal_;
        for (const Metavision::timestamp t : ts) {
            if (t < t_fft_start) continue;
            const int b = static_cast<int>(
                static_cast<double>(t - t_fft_start) / bin_dt);
            if (b >= 0 && b < N) signal[b] += 1.0;
        }
        // Apply Hann window.
        if (hann_N_ != N) {
            hann_N_ = N;
            hann_window_.resize(N > 0 ? N : 0);
            for (int n = 0; n < N; ++n) {
                hann_window_[n] = (N > 1)
                    ? 0.5 * (1.0 - std::cos(2.0 * freq_detail::kPi *
                                             static_cast<double>(n) /
                                             static_cast<double>(N - 1)))
                    : 1.0;
            }
        }
        for (int n = 0; n < N; ++n) {
            signal[n] *= hann_window_[n];
        }
        // DFT magnitude over the frequency range of interest.
        const double fs = 1.0e6 / bin_dt;  // sampling rate in Hz
        const int k_min = std::max(1, static_cast<int>(std::floor(
            static_cast<double>(f_min_) * N / fs)));
        const int k_max = std::min(N / 2, static_cast<int>(std::ceil(
            static_cast<double>(f_max_) * N / fs)));
        if (k_max <= k_min) return;
        int k_peak = -1;
        double mag_peak = 0.0;
        mag_.assign(k_max - k_min + 1, 0.0);
        auto& mag = mag_;
        std::vector<double> cos_tab(N), sin_tab(N);
        for (int n = 0; n < N; ++n) {
            const double phase = 2.0 * freq_detail::kPi * static_cast<double>(n) / static_cast<double>(N);
            cos_tab[n] = std::cos(phase);
            sin_tab[n] = std::sin(phase);
        }
        for (int k = k_min; k <= k_max; ++k) {
            double re = 0.0, im = 0.0;
            for (int n = 0; n < N; ++n) {
                const int idx = static_cast<int>((static_cast<std::size_t>(k) * static_cast<std::size_t>(n)) % static_cast<std::size_t>(N));
                re += signal[n] * cos_tab[idx];
                im -= signal[n] * sin_tab[idx];
            }
            const double m = std::sqrt(re * re + im * im);
            mag[k - k_min] = m;
            if (m > mag_peak) {
                mag_peak = m;
                k_peak = k;
            }
        }
        if (k_peak < 0 || mag_peak <= 0.0) return;
        // Parabolic interpolation around the peak.
        double k_interp = static_cast<double>(k_peak);
        if (k_peak > k_min && k_peak < k_max) {
            const double y0 = mag[k_peak - 1 - k_min];
            const double y1 = mag[k_peak - k_min];
            const double y2 = mag[k_peak + 1 - k_min];
            const double denom = y0 - 2.0 * y1 + y2;
            if (std::abs(denom) > 1e-12) {
                k_interp = static_cast<double>(k_peak) + 0.5 * (y0 - y2) / denom;
            }
        }
        double event_freq = k_interp * fs / static_cast<double>(N);
        // Harmonic confirmation: if there is a significant peak at half the
        // detected frequency, the detected peak was the 2nd harmonic of a
        // square wave -> the true fundamental is at half the frequency.
        const int k_half = k_peak / 2;
        if (k_half >= k_min) {
            const double m_half = mag[k_half - k_min];
            if (m_half > mag_peak / static_cast<double>(peak_alpha_)) {
                event_freq = 0.5 * event_freq;
            }
        }
        out.event_freq_hz = static_cast<float>(event_freq);
        out.blink_freq_hz = static_cast<float>(event_freq * 0.5);
    }

    int width_;
    int height_;
    // Tunable parameters (design defaults).
    float f_min_{100.0f};
    float f_max_{10000.0f};
    float bin_dt_us_{50.0f};
    int heatmap_threshold_{50};
    int min_cc_area_{3};
    int region_radius_{1};
    float peak_alpha_{5.0f};
    float first_analysis_s_{2.0f};
    float max_duration_s_{20.0f};
    float update_interval_s_{1.0f};

    std::deque<Event> buffer_;
    std::vector<int> heatmap_;
    Metavision::timestamp latest_t_{0};
    Metavision::timestamp last_analyze_t_{0};

    // --- Reusable temporary buffers (compute_frequency is const -> mutable) ---
    mutable std::vector<Metavision::timestamp> ts_;
    mutable std::vector<double> signal_, mag_;
    mutable int hann_N_{0};
    mutable std::vector<double> hann_window_;
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_FREQ_DETECTOR_H
