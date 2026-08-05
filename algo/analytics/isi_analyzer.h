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
    /// @param width,height Sensor dimensions (used in per-pixel mode).
    /// @param bin_count Number of histogram bins, [8, 256].
    /// @param max_isi_ms Maximum ISI covered by the histogram in ms, [1, 1000].
    /// @param per_pixel If true, ISI is computed per pixel; otherwise globally.
    ISIAnalyzer(int width, int height,
                int bin_count = 32,
                float max_isi_ms = 100.0f,
                bool per_pixel = false)
        : width_(width), height_(height),
          bin_count_(clamp_bins(bin_count)),
          max_isi_us_(static_cast<Metavision::timestamp>(
              clamp_isi(max_isi_ms) * 1000.0f)),
          per_pixel_(per_pixel),
          // ISI histogram in native µs over [0, max_isi_us_] (clamped), so the
          // _us accessors below are correct without unit conversion.
          hist_(kRingWindow, static_cast<std::size_t>(clamp_bins(bin_count)),
                0.0, static_cast<double>(max_isi_us_)),
          last_ts_pixel_(static_cast<std::size_t>(width) * height, -1) {}

    /// @brief Feeds a batch of events and updates the ISI histogram.
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (per_pixel_) {
                if (e.x >= width_ || e.y >= height_) continue;
                const std::size_t idx =
                    static_cast<std::size_t>(e.y) * width_ + e.x;
                const Metavision::timestamp prev = last_ts_pixel_[idx];
                last_ts_pixel_[idx] = e.t;
                if (prev >= 0 && e.t > prev) {
                    push_isi(e.t - prev);
                }
            } else {
                if (last_ts_global_ >= 0 && e.t > last_ts_global_) {
                    push_isi(e.t - last_ts_global_);
                }
                last_ts_global_ = e.t;
            }
        }
    }

    /// @brief Renders the ISI histogram as a cv::Mat bar chart (CV_8UC3).
    cv::Mat render(int img_w = 512, int img_h = 256) const {
        cv::Mat img(img_h, img_w, CV_8UC3, cv::Scalar(20, 20, 20));
        const std::vector<std::uint64_t>& counts = hist_.counts();
        if (counts.empty()) return img;
        std::uint64_t max_c = 1;
        for (const auto c : counts) {
            if (c > max_c) max_c = c;
        }
        const int n = static_cast<int>(counts.size());
        const int pad = 30;
        const int w = img.cols - 2 * pad;
        const int hh = img.rows - 2 * pad;
        const int bw = std::max(w / n, 1);  // §四-低14: bw-1 must stay > 0
        for (int i = 0; i < n; ++i) {
            const int bh = static_cast<int>(
                static_cast<double>(counts[i]) / static_cast<double>(max_c) * hh);
            const int x = pad + i * bw;
            const int y = img.rows - pad - bh;
            cv::rectangle(img, cv::Rect(x, y, std::max(bw - 1, 1), bh),
                          cv::Scalar(100, 200, 255), cv::FILLED);
        }
        cv::putText(img, "ISI histogram", cv::Point(pad, pad / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        return img;
    }

    // Summary statistics --------------------------------------------------
    double mean_us() const { return hist_.mean(); }
    double median_us() const { return hist_.percentile(50.0); }
    double p90_us() const { return hist_.percentile(90.0); }
    double std_dev_us() const { return hist_.std_dev(); }

    const std::vector<std::uint64_t>& counts() const { return hist_.counts(); }

    void set_per_pixel(bool p) { per_pixel_ = p; }
    bool per_pixel() const { return per_pixel_; }

    void set_bin_count(int b) {
        bin_count_ = clamp_bins(b);
        hist_ = HistogramRingBuffer(kRingWindow,
                                    static_cast<std::size_t>(bin_count_),
                                    0.0,
                                    static_cast<double>(max_isi_us_));
    }
    int bin_count() const { return bin_count_; }

    void set_max_isi_ms(float ms) {
        max_isi_us_ = static_cast<Metavision::timestamp>(clamp_isi(ms) * 1000.0f);
        hist_ = HistogramRingBuffer(kRingWindow,
                                    static_cast<std::size_t>(bin_count_),
                                    0.0,
                                    static_cast<double>(max_isi_us_));
    }
    float max_isi_ms() const {
        return static_cast<float>(max_isi_us_) / 1000.0f;
    }

    /// @brief Resets the analyzer state.
    void reset() {
        hist_.clear();
        std::fill(last_ts_pixel_.begin(), last_ts_pixel_.end(),
                  static_cast<Metavision::timestamp>(-1));
        last_ts_global_ = -1;
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

    void push_isi(Metavision::timestamp isi_us) {
        // Push raw µs; the histogram range is [0, max_isi_us_] µs so the _us
        // accessors (mean/median/p90/std_dev) are returned in native µs.
        hist_.push(static_cast<double>(isi_us));
    }

    static constexpr std::size_t kRingWindow = 8192;

    int width_;
    int height_;
    int bin_count_;
    Metavision::timestamp max_isi_us_;
    bool per_pixel_;
    HistogramRingBuffer hist_;
    std::vector<Metavision::timestamp> last_ts_pixel_;
    Metavision::timestamp last_ts_global_{-1};
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_ISI_ANALYZER_H
