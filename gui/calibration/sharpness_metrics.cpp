// gui/calibration/sharpness_metrics.cpp

#include "sharpness_metrics.h"

#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace gui {

cv::Mat remove_isolated_pixels(const cv::Mat& count_image, float max_count) {
    if (count_image.empty()) return cv::Mat();

    cv::Mat src;
    count_image.convertTo(src, CV_32F);
    cv::Mat dst = src.clone();

    const int rows = src.rows;
    const int cols = src.cols;
    for (int y = 0; y < rows; ++y) {
        const float* srow = src.ptr<float>(y);
        float* drow = dst.ptr<float>(y);
        for (int x = 0; x < cols; ++x) {
            const float v = srow[x];
            if (v <= 0.0f || v > max_count) continue;
            // A hot pixel is a lone speck: none of its 8 neighbours (read from
            // the ORIGINAL image, so the pass is order-independent) fired.
            bool has_neighbour = false;
            for (int dy = -1; dy <= 1 && !has_neighbour; ++dy) {
                const int ny = y + dy;
                if (ny < 0 || ny >= rows) continue;
                const float* nrow = src.ptr<float>(ny);
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = x + dx;
                    if (nx < 0 || nx >= cols) continue;
                    if (nrow[nx] > 0.0f) { has_neighbour = true; break; }
                }
            }
            if (!has_neighbour) drow[x] = 0.0f;
        }
    }
    return dst;
}

SharpnessMetrics compute_sharpness_metrics(const cv::Mat& count_image,
                                           double window_s) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    SharpnessMetrics m{nan, nan, 0.0, false};

    if (count_image.empty() || count_image.rows < 1 || count_image.cols < 1) {
        return m;
    }

    // S2: denoise first — isolated hot pixels must not feed the metrics (R1).
    const cv::Mat img = remove_isolated_pixels(count_image);

    const double total = cv::sum(img)[0];
    if (total <= 0.0) return m;  // zero events -> caller shows "—"

    m.valid = true;
    m.event_rate = (window_s > 0.0) ? total / window_s : 0.0;

    // S3: normalized contrast = σ²(I)/μ(I)². Dividing by μ² makes the metric
    // first-order invariant to event count / window length (R2): scaling all
    // counts by k scales σ² by k² and μ² by k².
    cv::Scalar mean, stddev;
    cv::meanStdDev(img, mean, stddev);
    const double mu = mean[0];
    m.contrast = (mu > 0.0) ? (stddev[0] * stddev[0]) / (mu * mu) : nan;

    // S4: mean line width. Binarize (count > 0 -> edge pixel), distance-
    // transform the inverse image, then average the distance at edge pixels
    // and double it (distance to nearest background on both sides of the
    // stroke). Defocus widens event trails -> width grows; good focus
    // concentrates events on fewer pixels -> width shrinks.
    cv::Mat binary;   // CV_8U, 255 where img > 0
    cv::compare(img, 0.0, binary, cv::CMP_GT);
    const int edge_pixels = cv::countNonZero(binary);
    if (edge_pixels == 0) {
        m.mean_line_width = nan;
        return m;
    }
    // distanceTransform must run on the edge image itself (non-zero edge
    // pixels get their distance to the nearest zero/background pixel). The
    // audit text says "distance transform of the inverse", but running it on
    // the inverse leaves every edge pixel at distance 0 — verified by test.
    cv::Mat dist;
    cv::distanceTransform(binary, dist, cv::DIST_L2, 3);

    double dist_sum = 0.0;
    for (int y = 0; y < binary.rows; ++y) {
        const uchar* brow = binary.ptr<uchar>(y);
        const float* frow = dist.ptr<float>(y);
        for (int x = 0; x < binary.cols; ++x) {
            if (brow[x]) dist_sum += static_cast<double>(frow[x]);
        }
    }
    m.mean_line_width = 2.0 * dist_sum / static_cast<double>(edge_pixels);
    return m;
}

} // namespace gui
