// algo/analytics/freq_detector.h — Flickering light source frequency detection.
//
// Design §4.4.7. Detects relatively static flickering light sources (LEDs) by
// accumulating an event heatmap, clustering hot spots, then for each cluster
// gathering the event timestamps in a small region around the centroid,
// binning them, applying a Hann window, and running a DFT to find the dominant
// frequency peak (with parabolic interpolation + 2x harmonic confirmation).
// Frequency definition: event_freq = 2 * LED blink_freq. Inspired by the
// Lighthouse freq_analyzer tool. Uses cv::dft (no external FFT library).
//
// Pipeline aligned with the reference tool:
//   - Initialization phase: no results until the stream has accumulated
//     first_analysis_s_ of events (default 5 s) AND at least kMinTotalEvents.
//   - Growing window: every analysis covers the whole retained buffer (capped
//     at max_duration_s_ by pruning), so frequency accuracy improves as the
//     window grows.
//   - Freeze: like the reference, analysis stops after max_duration_s_ of
//     stream — analyze() keeps returning the last computed result. Deviation
//     for continuous GUI use: the camera keeps running and events keep
//     accumulating (the reference stops collection outright).

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

/// @brief A detected flickering light source.
struct LightSource {
    int u{0};                 ///< Centroid column
    int v{0};                 ///< Centroid row
    int u0{0};                ///< Cluster bounding-box left
    int v0{0};                ///< Cluster bounding-box top
    int w{0};                 ///< Cluster bounding-box width
    int h{0};                 ///< Cluster bounding-box height
    float event_freq_hz{0.0f}; ///< Event frequency (2x blink frequency)
    float blink_freq_hz{0.0f}; ///< Physical LED blink frequency
};

/// @brief Flickering light source frequency detector (heatmap + DFT).
class FreqDetector {
public:
    /// @brief Detector lifecycle phase (drives the GUI init-phase hint).
    enum class Phase {
        Initializing,  ///< Accumulating the warm-up window, no results yet.
        Running,       ///< Warm-up done, results update every interval.
    };

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
            if (total_events_ == 0) first_t_ = e.t;
            ++total_events_;
            // Count safety valve: at full-sensor PWM-noise rates (~100 Mev/s)
            // a 20 s time window would retain BILLIONS of events (~25 GB —
            // the process gets OOM-killed). Cap the buffer; the analysis
            // window gracefully degrades to whatever is retained (at
            // kMaxBufferEvents the resolution still covers the 100 Hz–10 kHz
            // band comfortably).
            while (buffer_.size() > max_buffer_events_) drop_front();
        }
        prune();
    }

    /// @brief Hard cap on retained events (default kMaxBufferEvents).
    void set_max_buffer_events(std::size_t n) { max_buffer_events_ = n; }
    std::size_t max_buffer_events() const { return max_buffer_events_; }

    /// @brief Current lifecycle phase (see Phase).
    Phase phase() const {
        if (latest_t_ <= 0 || total_events_ < kMinTotalEvents) {
            return Phase::Initializing;
        }
        const Metavision::timestamp warmup_us =
            static_cast<Metavision::timestamp>(first_analysis_s_ * 1.0e6);
        return (latest_t_ - first_t_ >= warmup_us) ? Phase::Running
                                                   : Phase::Initializing;
    }

    /// @brief Seconds of warm-up remaining in the initialization phase
    ///        (0 once running).
    float init_remaining_s() const {
        if (phase() == Phase::Running) return 0.0f;
        const Metavision::timestamp warmup_us =
            static_cast<Metavision::timestamp>(first_analysis_s_ * 1.0e6);
        const Metavision::timestamp elapsed = latest_t_ - first_t_;
        const Metavision::timestamp left_us =
            elapsed < warmup_us ? warmup_us - elapsed : 0;
        return static_cast<float>(left_us) / 1.0e6f;
    }

    /// @brief Runs the full detection pipeline and returns detected sources.
    ///        Empty while the initialization phase has not completed; after
    ///        max_duration_s_ of stream the result is FROZEN: the FIRST
    ///        analyze() past the boundary computes one final result over the
    ///        full max_duration_s_ window, every later call returns it
    ///        unchanged.
    std::vector<LightSource> analyze() {
        if (width_ <= 0 || height_ <= 0) return {};
        if (phase() != Phase::Running) return {};
        const Metavision::timestamp t_end = latest_t_;
        const Metavision::timestamp max_us =
            static_cast<Metavision::timestamp>(max_duration_s_ * 1.0e6);
        const bool frozen = (t_end - first_t_ >= max_us);
        if (frozen && freeze_final_done_) return frozen_result_;
        std::vector<LightSource> out;
        const Metavision::timestamp window_lo =
            (t_end > max_us) ? (t_end - max_us) : 0;
        // Growing analysis window: the whole retained buffer (pruning keeps it
        // at max_duration_s_), so accuracy improves as the window grows. The
        // freeze-boundary analysis covers the FULL window by construction.
        const Metavision::timestamp t_start =
            buffer_.empty() ? t_end : std::max(buffer_.front().t, window_lo);
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
        // Collect the accepted cluster regions first...
        struct ClusterRegion {
            int label;  // connected-component label (for the bbox stats)
            int u, v;   // rounded centroid
        };
        std::vector<ClusterRegion> regions;
        for (int i = 1; i < n_labels; ++i) {
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < min_cc_area_) continue;
            // Defensive cap: report at most the kMaxSources largest clusters
            // (bounds the per-analysis DFT count).
            if (regions.size() >= kMaxSources) break;
            ClusterRegion cr;
            cr.label = i;
            cr.u = static_cast<int>(centroids.at<double>(i, 0));
            cr.v = static_cast<int>(centroids.at<double>(i, 1));
            regions.push_back(cr);
        }
        // ...then gather the region timestamps for ALL clusters in ONE pass
        // over the event buffer (a per-cluster full-buffer scan is O(buffer ×
        // clusters) and froze the pipeline at high event rates). region_map
        // holds cluster_index+1 inside each cluster's square region.
        std::vector<std::vector<Metavision::timestamp>> region_ts(regions.size());
        if (!regions.empty()) {
            region_map_.assign(static_cast<std::size_t>(width_) * height_, 0);
            for (std::size_t c = 0; c < regions.size(); ++c) {
                const int x0 = std::max(0, regions[c].u - region_radius_);
                const int x1 = std::min(width_ - 1, regions[c].u + region_radius_);
                const int y0 = std::max(0, regions[c].v - region_radius_);
                const int y1 = std::min(height_ - 1, regions[c].v + region_radius_);
                for (int y = y0; y <= y1; ++y) {
                    std::uint8_t* row =
                        region_map_.data() + static_cast<std::size_t>(y) * width_;
                    for (int x = x0; x <= x1; ++x) {
                        if (row[x] == 0) row[x] = static_cast<std::uint8_t>(c + 1);
                    }
                }
            }
            for (std::size_t c = 0; c < region_ts.size(); ++c) region_ts[c].reserve(256);
            for (const Event& e : buffer_) {
                if (e.t < window_lo) continue;
                if (e.x >= width_ || e.y >= height_) continue;
                const std::uint8_t c =
                    region_map_[static_cast<std::size_t>(e.y) * width_ + e.x];
                if (c != 0) region_ts[c - 1].push_back(e.t);
            }
        }
        for (std::size_t c = 0; c < regions.size(); ++c) {
            const int li = regions[c].label;
            LightSource src;
            src.u = regions[c].u;
            src.v = regions[c].v;
            src.u0 = stats.at<int>(li, cv::CC_STAT_LEFT);
            src.v0 = stats.at<int>(li, cv::CC_STAT_TOP);
            src.w = stats.at<int>(li, cv::CC_STAT_WIDTH);
            src.h = stats.at<int>(li, cv::CC_STAT_HEIGHT);
            compute_frequency(region_ts[c], t_start, t_end, src);
            out.push_back(src);
        }
        frozen_result_ = out;
        if (frozen) freeze_final_done_ = true;
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
        update_interval_s_ = clamp_range(s, 0.1f, 10.0f);
    }
    float update_interval_s() const { return update_interval_s_; }

    /// @brief Clears the rolling buffer and heatmap.
    void reset() {
        buffer_.clear();
        std::fill(heatmap_.begin(), heatmap_.end(), 0);
        latest_t_ = 0;
        first_t_ = 0;
        last_analyze_t_ = 0;
        total_events_ = 0;
        frozen_result_.clear();
        freeze_final_done_ = false;
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    /// Max clusters reported per analyze() (defensive cost cap, see the
    /// cluster loop in analyze()).
    static constexpr std::size_t kMaxSources = 32;
    /// Minimum total events before any analysis (reference guard).
    static constexpr std::size_t kMinTotalEvents = 100;
    /// Minimum timestamps in a cluster region to attempt the DFT.
    static constexpr std::size_t kMinRegionTs = 50;
    /// Minimum bin count to attempt the DFT.
    static constexpr int kMinBins = 16;

public:
    /// Default hard cap on retained events (~240 MB of Event payloads) —
    /// the OOM safety valve (see process()).
    static constexpr std::size_t kMaxBufferEvents = 20'000'000;

private:

    static constexpr double kPi = 3.14159265358979323846;

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

    /// @brief Drops the oldest buffered event, undoing its heatmap count.
    void drop_front() {
        if (buffer_.empty()) return;
        const Event& e = buffer_.front();
        if (e.x < width_ && e.y < height_) {
            const std::size_t idx =
                static_cast<std::size_t>(e.y) * width_ + e.x;
            if (heatmap_[idx] > 0) --heatmap_[idx];
        }
        buffer_.pop_front();
    }

    void prune() {
        const Metavision::timestamp max_us =
            static_cast<Metavision::timestamp>(max_duration_s_ * 1.0e6);
        const Metavision::timestamp t_lo = latest_t_ - max_us;
        while (!buffer_.empty() && buffer_.front().t < t_lo) drop_front();
    }

    /// @brief One detected spectral peak (frequency + DFT magnitude).
    struct SpectralPeak {
        double freq_hz{0.0};
        double magnitude{0.0};
    };

    /// @brief Reference harmonic confirmation: among the peaks sorted by
    ///        frequency, the fundamental is the LOWEST peak whose 2x harmonic
    ///        also exists within tol Hz; fallback = highest-magnitude peak.
    static double identify_event_frequency(
        const std::vector<SpectralPeak>& peaks, double tol_hz) {
        if (peaks.empty()) return 0.0;
        const SpectralPeak* fallback = &peaks[0];
        for (const auto& p : peaks) {
            if (p.magnitude > fallback->magnitude) fallback = &p;
        }
        for (const auto& p : peaks) {  // sorted by frequency ascending
            const double f2 = 2.0 * p.freq_hz;
            for (const auto& q : peaks) {
                if (q.freq_hz > p.freq_hz && std::abs(q.freq_hz - f2) <= tol_hz) {
                    return p.freq_hz;
                }
            }
        }
        return fallback->freq_hz;
    }

    /// @brief Computes the event frequency for one cluster via binned DFT
    ///        over the full growing window (cv::dft, zero-padded to a power
    ///        of two), with noise-threshold peak detection, parabolic
    ///        interpolation and harmonic confirmation.
    ///        @p ts holds the region's event timestamps (pre-gathered in a
    ///        single pass over the buffer by analyze()).
    void compute_frequency(const std::vector<Metavision::timestamp>& ts,
                           Metavision::timestamp t_start,
                           Metavision::timestamp t_end,
                           LightSource& out) const {
        if (ts.size() < kMinRegionTs) return;
        if (t_end <= t_start) return;
        const double bin_dt = static_cast<double>(bin_dt_us_);
        const int num_bins =
            static_cast<int>(static_cast<double>(t_end - t_start) / bin_dt) + 1;
        if (num_bins < kMinBins) return;
        // Zero-pad to the next power of two for cv::dft.
        int fft_size = 1;
        while (fft_size < num_bins) fft_size <<= 1;
        // Bin the timestamps into the analysis window, then Hann-window the
        // valid samples (padding stays zero).
        cv::Mat signal = cv::Mat::zeros(1, fft_size, CV_32F);
        auto* sig = signal.ptr<float>(0);
        for (const Metavision::timestamp t : ts) {
            if (t < t_start) continue;
            const int b = static_cast<int>(
                static_cast<double>(t - t_start) / bin_dt);
            if (b >= 0 && b < num_bins) sig[b] += 1.0f;
        }
        // Hann window over the valid samples (padding stays zero). Cached per
        // num_bins: all clusters in one analyze() share the same window, so
        // the cos() sweep runs once per analysis, not once per cluster.
        if (hann_bins_ != num_bins) {
            hann_bins_ = num_bins;
            hann_window_.resize(static_cast<std::size_t>(num_bins));
            for (int n = 0; n < num_bins; ++n) {
                hann_window_[static_cast<std::size_t>(n)] = (num_bins > 1)
                    ? 0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(n) /
                                             static_cast<double>(num_bins - 1)))
                    : 1.0;
            }
        }
        for (int n = 0; n < num_bins; ++n) {
            sig[n] = static_cast<float>(sig[n] *
                                        hann_window_[static_cast<std::size_t>(n)]);
        }
        // Real-input DFT (packed CCS output — ~2x faster than the complex
        // output and the magnitude loop below reads it directly):
        // packed[k] = [Re0, Re_{N/2}, Re1, Im1, Re2, Im2, ...], i.e. for
        // k >= 1: Re_k = p[2k], Im_k = p[2k+1].
        cv::Mat packed_spec;
        cv::dft(signal, packed_spec);
        const int half = fft_size / 2;
        std::vector<double> mag(static_cast<std::size_t>(half), 0.0);
        const auto* pk = packed_spec.ptr<float>(0);
        for (int k = 1; k < half; ++k) {
            const double re = pk[2 * k];
            const double im = pk[2 * k + 1];
            mag[static_cast<std::size_t>(k)] = std::sqrt(re * re + im * im);
        }
        // Noise threshold: mean + peak_alpha * std over the spectrum, skipping
        // DC and the first bin (like the reference).
        double sum = 0.0, sum_sq = 0.0;
        int n_stats = 0;
        for (int k = 2; k < half; ++k) {
            const double m = mag[static_cast<std::size_t>(k)];
            sum += m;
            sum_sq += m * m;
            ++n_stats;
        }
        if (n_stats < 2) return;
        const double mean = sum / n_stats;
        const double variance = std::max(0.0, sum_sq / n_stats - mean * mean);
        const double threshold =
            mean + static_cast<double>(peak_alpha_) * std::sqrt(variance);
        const double fs = 1.0e6 / bin_dt;  // sampling rate in Hz
        // Local maxima within [f_min, f_max] above the noise threshold, with
        // parabolic interpolation.
        std::vector<SpectralPeak> peaks;
        peaks.reserve(16);
        for (int k = 1; k < half - 1; ++k) {
            const double f = static_cast<double>(k) * fs / fft_size;
            if (f < f_min_ || f > f_max_) continue;
            const double m = mag[static_cast<std::size_t>(k)];
            if (m < threshold) continue;
            if (m > mag[static_cast<std::size_t>(k - 1)] &&
                m >= mag[static_cast<std::size_t>(k + 1)]) {
                double k_interp = static_cast<double>(k);
                const double y0 = mag[static_cast<std::size_t>(k - 1)];
                const double y2 = mag[static_cast<std::size_t>(k + 1)];
                const double denom = y0 - 2.0 * m + y2;
                if (std::abs(denom) > 1e-12) {
                    k_interp = static_cast<double>(k) + 0.5 * (y0 - y2) / denom;
                }
                SpectralPeak p;
                p.freq_hz = k_interp * fs / fft_size;
                p.magnitude = m;
                peaks.push_back(p);
            }
        }
        if (peaks.empty()) return;
        std::sort(peaks.begin(), peaks.end(),
                  [](const SpectralPeak& a, const SpectralPeak& b) {
                      return a.freq_hz < b.freq_hz;
                  });
        // Harmonic confirmation tolerance adapts to the window length: the
        // frequency resolution is fs/fft_size, so shorter windows need a
        // looser absolute tolerance (reference: max(1, 2/span) Hz).
        const double span_s = static_cast<double>(t_end - t_start) / 1.0e6;
        const double tol_hz = std::max(1.0, 2.0 / std::max(span_s, 1e-9));
        const double event_freq = identify_event_frequency(peaks, tol_hz);
        out.event_freq_hz = static_cast<float>(event_freq);
        out.blink_freq_hz = static_cast<float>(event_freq * 0.5);
    }

    int width_;
    int height_;
    // Tunable parameters (reference defaults).
    float f_min_{100.0f};
    float f_max_{10000.0f};
    float bin_dt_us_{50.0f};
    int heatmap_threshold_{50};
    int min_cc_area_{3};
    int region_radius_{1};
    float peak_alpha_{5.0f};
    float first_analysis_s_{5.0f};
    float max_duration_s_{20.0f};
    float update_interval_s_{2.0f};

    std::deque<Event> buffer_;
    std::size_t max_buffer_events_{kMaxBufferEvents};
    std::vector<int> heatmap_;
    Metavision::timestamp latest_t_{0};
    Metavision::timestamp first_t_{0};
    Metavision::timestamp last_analyze_t_{0};
    std::size_t total_events_{0};
    std::vector<LightSource> frozen_result_;  ///< Final result computed at the freeze boundary.
    bool freeze_final_done_{false};
    std::vector<std::uint8_t> region_map_;  ///< Reusable cluster-region label map (analyze).
    // Reusable Hann window (compute_frequency is const -> mutable). Shared by
    // all clusters of one analyze() pass (same num_bins).
    mutable int hann_bins_{0};
    mutable std::vector<double> hann_window_;
};

}  // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_FREQ_DETECTOR_H
