// algo/analytics/isi_analyzer.h — ISI (inter-spike interval) histogram analysis.
//
// Design §4.4.4. Computes the distribution of inter-event intervals per pixel
// (or globally) using histogram_ring_buffer, for scene motion-frequency and
// noise characterization. Inspired by jAER IntegrateAndFire / ISI tools.
// Header-only.

#ifndef GUI_ALGO_ANALYTICS_ISI_ANALYZER_H
#define GUI_ALGO_ANALYTICS_ISI_ANALYZER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/histogram_ring_buffer.h"

namespace gui_algo {

/// @brief Inter-spike-interval histogram analyzer.
class ISIAnalyzer {
public:
    /// @brief Constructs the analyzer.
    /// @param width,height Sensor dimensions.
    /// @param bin_count Number of histogram bins, [8, 256].
    /// @param max_isi_ms Maximum ISI covered by the histogram in ms, [1, 1000].
    /// @param min_isi_ms Minimum ISI covered by the histogram in ms (jAER
    ///        ISIHistogrammer minIsiUs band) — shorter intervals are
    ///        discarded so the X axis focuses on the band of interest.
    ISIAnalyzer(int width, int height,
                int bin_count = 32,
                float max_isi_ms = 100.0f,
                float min_isi_ms = 0.0f)
        : width_(width), height_(height),
          bin_count_(clamp_bins(bin_count)),
          max_isi_us_(static_cast<Metavision::timestamp>(
              clamp_isi(max_isi_ms) * 1000.0f)),
          min_isi_us_(clamp_min_isi_us(min_isi_ms, max_isi_us_)),
          hist_(kRingWindow, static_cast<std::size_t>(clamp_bins(bin_count)),
                static_cast<double>(clamp_min_isi_us(min_isi_ms, max_isi_us_)),
                static_cast<double>(max_isi_us_)),
          last_ts_pixel_(static_cast<std::size_t>(width) * height, -1) {}

    /// @brief Feeds a batch of events and updates the ISI histogram.
    /// Per-pixel only (jAER ISIHistogrammer is inherently per-channel): each
    /// pixel's inter-event interval is a sample. The former "global" mode
    /// (intervals of the pooled event stream) was deleted — it degenerates
    /// to a single static bin at high event rates (user decision).
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const std::size_t idx =
                static_cast<std::size_t>(e.y) * width_ + e.x;
            const Metavision::timestamp prev = last_ts_pixel_[idx];
            last_ts_pixel_[idx] = e.t;
            if (prev >= 0 && e.t > prev) {
                push_isi(e.t - prev);
            }
        }
    }

    /// @brief Renders the ISI histogram as a cv::Mat bar chart (CV_8UC3)
    /// with labelled axes and the full metric name (user request, Phase 2.6
    /// debug). X = ISI in ms over [min_isi, max_isi], Y = sample count.
    cv::Mat render(int img_w = 512, int img_h = 256) const {
        cv::Mat img(img_h, img_w, CV_8UC3, cv::Scalar(20, 20, 20));
        const std::vector<std::uint64_t>& counts = hist_.counts();
        const int ml = 52, mr = 10, mt = 24, mb = 34;
        const int plot_w = img.cols - ml - mr;
        const int plot_h = img.rows - mt - mb;
        cv::putText(img, "Inter-Spike Interval (ISI) histogram",
                    cv::Point(ml, 16), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(255, 255, 255), 1);
        cv::putText(img, "per-pixel",
                    cv::Point(img.cols - 80, 16), cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(140, 140, 140), 1);
        cv::rectangle(img, cv::Rect(ml, mt, plot_w, plot_h),
                      cv::Scalar(90, 90, 90), 1);
        if (!counts.empty()) {
            std::uint64_t max_c = 1;
            for (const auto c : counts) {
                if (c > max_c) max_c = c;
            }
            const int n = static_cast<int>(counts.size());
            const int bw = std::max(plot_w / n, 1);  // §四-低14: bw-1 must stay > 0
            for (int i = 0; i < n; ++i) {
                const int bh = static_cast<int>(
                    static_cast<double>(counts[i]) /
                    static_cast<double>(max_c) * plot_h);
                cv::rectangle(img,
                              cv::Rect(ml + i * bw, mt + plot_h - bh,
                                       std::max(bw - 1, 1), bh),
                              cv::Scalar(100, 200, 255), cv::FILLED);
            }
            // Y axis: max count + label.
            char buf[32];
            std::snprintf(buf, sizeof buf, "%llu",
                          static_cast<unsigned long long>(max_c));
            cv::putText(img, buf, cv::Point(2, mt + 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35,
                        cv::Scalar(180, 180, 180), 1);
            cv::putText(img, "count", cv::Point(2, mt + plot_h),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35,
                        cv::Scalar(180, 180, 180), 1);
        }
        // X axis: 5 ticks over [min_isi_us_, max_isi_us_], labelled in ms.
        for (int i = 0; i <= 4; ++i) {
            const double frac = i / 4.0;
            const double us = static_cast<double>(min_isi_us_) +
                frac * static_cast<double>(max_isi_us_ - min_isi_us_);
            const int tx = ml + static_cast<int>(frac * plot_w);
            cv::line(img, cv::Point(tx, mt + plot_h),
                     cv::Point(tx, mt + plot_h + 4), cv::Scalar(90, 90, 90), 1);
            char buf[32];
            std::snprintf(buf, sizeof buf, "%.0f", us / 1000.0);
            cv::putText(img, buf, cv::Point(tx - 8, mt + plot_h + 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35,
                        cv::Scalar(180, 180, 180), 1);
        }
        cv::putText(img, "ISI (ms)", cv::Point(ml + plot_w / 2 - 20, img.rows - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        return img;
    }

    // Summary statistics --------------------------------------------------
    double mean_us() const { return hist_.mean(); }
    double median_us() const { return hist_.percentile(50.0); }
    double p90_us() const { return hist_.percentile(90.0); }
    double std_dev_us() const { return hist_.std_dev(); }

    const std::vector<std::uint64_t>& counts() const { return hist_.counts(); }

    void set_bin_count(int b) {
        bin_count_ = clamp_bins(b);
        rebuild_hist();
    }
    int bin_count() const { return bin_count_; }

    void set_max_isi_ms(float ms) {
        max_isi_us_ = static_cast<Metavision::timestamp>(clamp_isi(ms) * 1000.0f);
        min_isi_us_ = clamp_min_isi_us(
            static_cast<float>(min_isi_us_) / 1000.0f, max_isi_us_);
        rebuild_hist();
    }
    float max_isi_ms() const {
        return static_cast<float>(max_isi_us_) / 1000.0f;
    }

    /// @brief Sets the minimum ISI covered by the histogram (jAER
    /// ISIHistogrammer minIsiUs band). Shorter intervals are discarded.
    void set_min_isi_ms(float ms) {
        min_isi_us_ = clamp_min_isi_us(ms, max_isi_us_);
        rebuild_hist();
    }
    float min_isi_ms() const {
        return static_cast<float>(min_isi_us_) / 1000.0f;
    }

    /// @brief Resets the analyzer state.
    void reset() {
        hist_.clear();
        std::fill(last_ts_pixel_.begin(), last_ts_pixel_.end(),
                  static_cast<Metavision::timestamp>(-1));
    }

private:
    static int clamp_bins(int b) {
        if (b < 8) return 8;
        if (b > 256) return 256;
        return b;
    }
    static float clamp_isi(float ms) {
        if (ms < 1.0f) return 1.0f;
        if (ms > 1000.0f) return 1000.0f;
        return ms;
    }
    /// min ISI in µs, clamped to [0, max_isi_us).
    static Metavision::timestamp clamp_min_isi_us(
        float ms, Metavision::timestamp max_isi_us) {
        if (ms < 0.0f) ms = 0.0f;
        auto us = static_cast<Metavision::timestamp>(ms * 1000.0f);
        if (us >= max_isi_us) us = max_isi_us > 1000 ? max_isi_us - 1000 : 0;
        return us;
    }

    void rebuild_hist() {
        // ISI histogram in native µs over [min_isi_us_, max_isi_us_]
        // (clamped), so the _us accessors are correct without conversion.
        hist_ = HistogramRingBuffer(kRingWindow,
                                    static_cast<std::size_t>(bin_count_),
                                    static_cast<double>(min_isi_us_),
                                    static_cast<double>(max_isi_us_));
    }

    void push_isi(Metavision::timestamp isi_us) {
        // jAER ISIHistogrammer band: ISIs outside [min, max] are discarded.
        if (isi_us < min_isi_us_ || isi_us >= max_isi_us_) return;
        hist_.push(static_cast<double>(isi_us));
    }

    static constexpr std::size_t kRingWindow = 8192;

    int width_;
    int height_;
    int bin_count_;
    Metavision::timestamp max_isi_us_;
    Metavision::timestamp min_isi_us_{0};
    HistogramRingBuffer hist_;
    std::vector<Metavision::timestamp> last_ts_pixel_;
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_ISI_ANALYZER_H
