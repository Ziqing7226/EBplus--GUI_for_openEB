// algo/analytics/flow_statistics.h — Optical flow quality evaluation.
//
// Design §4.4.3. Evaluates optical-flow algorithm output (4.3.9) against ground
// truth (synthetic motion or manual annotation) using EPE (endpoint error),
// PE (percentage error) and angular error. Reports histograms plus mean /
// median / 90th percentile. Inspired by jAER NoiseTesterFilter (TP/FP/TN/FN
// framework). Header-only.

#ifndef GUI_ALGO_ANALYTICS_FLOW_STATISTICS_H
#define GUI_ALGO_ANALYTICS_FLOW_STATISTICS_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algo/common/histogram_ring_buffer.h"

namespace gui_algo {

namespace detail {
constexpr double kPi = 3.14159265358979323846;
}

/// @brief A single estimated vs ground-truth flow sample.
struct FlowSample {
    float u_est{0.0f};
    float v_est{0.0f};
    float u_gt{0.0f};
    float v_gt{0.0f};
};

/// @brief Computes optical-flow error metrics against ground truth.
class FlowStatistics {
public:
    enum class Source {
        Synthetic,  ///< Ground truth from synthetic event simulation.
        Annotated,  ///< Ground truth from manual annotation.
    };

    /// @brief Constructs the evaluator.
    /// @param source Ground-truth source.
    /// @param output_hz Statistics reporting rate in Hz, [1, 30].
    explicit FlowStatistics(Source source = Source::Synthetic, int output_hz = 5)
        : source_(source),
          output_hz_(clamp_hz(output_hz)),
          epe_(kWindow, kBins, 0.0, 5.0),       // EPE in [0, 5] px
          pe_(kWindow, kBins, 0.0, 100.0),      // PE in [0, 100] %
          ae_(kWindow, kBins, 0.0, 90.0) {}      // angular error in [0, 90] deg

    /// @brief Adds a batch of estimated vs ground-truth flow samples.
    void add_samples(const FlowSample* samples, std::size_t n) {
        if (samples == nullptr || n == 0) return;
        // jAER MotionFlowStatistics guards: skip the sample entirely when the
        // ground-truth magnitude is (near) zero; for the angular error also
        // skip when the estimated magnitude is zero.
        constexpr double kEps = 1e-6;
        for (std::size_t i = 0; i < n; ++i) {
            const FlowSample& s = samples[i];
            const double u_est = s.u_est, v_est = s.v_est;
            const double u_gt = s.u_gt, v_gt = s.v_gt;
            const double mag_gt = std::sqrt(u_gt * u_gt + v_gt * v_gt);
            const double mag_est = std::sqrt(u_est * u_est + v_est * v_est);
            if (mag_gt < kEps) continue;
            // Endpoint error.
            const double dx = u_est - u_gt;
            const double dy = v_est - v_gt;
            const double epe = std::sqrt(dx * dx + dy * dy);
            epe_.push(epe);
            // Percentage error: relative endpoint error EPE/|v_gt|*100 (jAER
            // EndpointErrorRel), not the relative magnitude error.
            pe_.push(epe * 100.0 / mag_gt);
            // Angular error: 2D vector angle between est and gt (jAER
            // AngularError). Skip when est magnitude is zero.
            if (mag_est >= kEps) {
                double cos_a = (u_est * u_gt + v_est * v_gt) / (mag_est * mag_gt);
                if (cos_a > 1.0) cos_a = 1.0;
                if (cos_a < -1.0) cos_a = -1.0;
                ae_.push(std::acos(cos_a) * 180.0 / detail::kPi);
            }
        }
    }

    /// @brief Renders the EPE histogram as a cv::Mat bar chart (CV_8UC3).
    cv::Mat render(int width = 512, int height = 256) const {
        cv::Mat img(height, width, CV_8UC3, cv::Scalar(20, 20, 20));
        draw_histogram(img, epe_, cv::Scalar(0, 200, 255), "EPE");
        return img;
    }

    /// @brief Renders the PE histogram as a cv::Mat bar chart (CV_8UC3).
    cv::Mat render_pe(int width = 512, int height = 256) const {
        cv::Mat img(height, width, CV_8UC3, cv::Scalar(20, 20, 20));
        draw_histogram(img, pe_, cv::Scalar(0, 255, 0), "PE");
        return img;
    }

    /// @brief Renders the angular-error histogram as a cv::Mat bar chart.
    cv::Mat render_angular(int width = 512, int height = 256) const {
        cv::Mat img(height, width, CV_8UC3, cv::Scalar(20, 20, 20));
        draw_histogram(img, ae_, cv::Scalar(255, 100, 100), "AE");
        return img;
    }

    // Summary statistics --------------------------------------------------
    double epe_mean() const { return epe_.mean(); }
    double epe_median() const { return epe_.percentile(50.0); }
    double epe_p90() const { return epe_.percentile(90.0); }

    double pe_mean() const { return pe_.mean(); }
    double pe_median() const { return pe_.percentile(50.0); }
    double pe_p90() const { return pe_.percentile(90.0); }

    double ae_mean() const { return ae_.mean(); }
    double ae_median() const { return ae_.percentile(50.0); }
    double ae_p90() const { return ae_.percentile(90.0); }

    void set_source(Source s) { source_ = s; }
    Source source() const { return source_; }

    void set_output_hz(int hz) { output_hz_ = clamp_hz(hz); }
    int output_hz() const { return output_hz_; }

    /// @brief Minimum interval between reports in us.
    Metavision::timestamp report_interval_us() const {
        return static_cast<Metavision::timestamp>(1.0e6 / output_hz_);
    }

    void reset() {
        epe_.clear();
        pe_.clear();
        ae_.clear();
    }

private:
    static int clamp_hz(int hz) {
        if (hz < 1) return 1;
        if (hz > 30) return 30;
        return hz;
    }

    static void draw_histogram(cv::Mat& img, const HistogramRingBuffer& h,
                               const cv::Scalar& color, const char* label) {
        const std::vector<std::uint64_t>& counts = h.counts();
        if (counts.empty()) return;
        std::uint64_t max_c = 1;
        for (const auto c : counts) {
            if (c > max_c) max_c = c;
        }
        const int n = static_cast<int>(counts.size());
        const int pad = 20;
        const int w = img.cols - 2 * pad;
        const int hh = img.rows - 2 * pad;
        const int bw = w / n;
        for (int i = 0; i < n; ++i) {
            const int bh = static_cast<int>(
                static_cast<double>(counts[i]) / static_cast<double>(max_c) * hh);
            const int x = pad + i * bw;
            const int y = img.rows - pad - bh;
            cv::rectangle(img, cv::Rect(x, y, bw - 1, bh), color, cv::FILLED);
        }
        cv::putText(img, label, cv::Point(pad, pad),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    static constexpr std::size_t kWindow = 4096;
    static constexpr std::size_t kBins = 32;

    Source source_;
    int output_hz_;
    HistogramRingBuffer epe_;
    HistogramRingBuffer pe_;
    HistogramRingBuffer ae_;
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_FLOW_STATISTICS_H
