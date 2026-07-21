// gui/calibration/sharpness_metrics.cpp

#include "sharpness_metrics.h"

#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace gui {

cv::Mat remove_isolated_pixels(const cv::Mat& count_image, float max_count) {
    if (count_image.empty()) return cv::Mat();

    // §12.2-A #4: replace the hand-written 8-neighbour double loop (~44M
    // memory reads/tick at 1280×720) with SIMD-vectorised OpenCV ops.
    // Semantics are IDENTICAL: a pixel is removed iff 0 < v <= max_count AND
    // none of its 8 neighbours is non-zero. (cv::medianBlur was considered
    // but rejected — it would also thin/remove legitimate 2-px-wide lines,
    // breaking the IsolatedNoiseDoesNotDistortMetrics test.)
    //
    // Strategy:
    //   1. candidates  = (src > 0) & (src <= max_count)   — pixels that COULD
    //      be removed (single-event specks, not saturated/structural pixels).
    //   2. has_neighbour = dilate(binary, ring_kernel)    — 1 iff at least
    //      one of the 8 neighbours is non-zero. The ring kernel (0 at center,
    //      1 elsewhere) excludes the center pixel itself.
    //   3. isolated   = candidates & ~has_neighbour        — removal mask.
    //   4. result     = src.clone(); result.setTo(0, isolated).
    cv::Mat src;
    count_image.convertTo(src, CV_32F);

    cv::Mat binary;
    cv::compare(src, 0.0, binary, cv::CMP_GT);  // CV_8U: 255 where src > 0

    // Ring kernel: 1 at all 8 neighbours, 0 at center — so dilation looks at
    // neighbours only, not the pixel itself.
    const cv::Mat ring_kernel = (cv::Mat_<uchar>(3, 3) <<
        1, 1, 1,
        1, 0, 1,
        1, 1, 1);
    cv::Mat neighbour;
    cv::dilate(binary, neighbour, ring_kernel, cv::Point(-1, -1), 1,
               cv::BORDER_CONSTANT, 0);

    // candidates: 0 < v <= max_count. has_neighbour: any 8-neighbour > 0.
    cv::Mat candidates, has_neighbour;
    cv::inRange(src, cv::Scalar(1e-6f), cv::Scalar(max_count), candidates);
    cv::compare(neighbour, 0, has_neighbour, cv::CMP_GT);  // 255 where neighbour > 0

    // isolated = candidates AND NOT has_neighbour
    cv::Mat isolated;
    cv::bitwise_and(candidates, ~has_neighbour, isolated);

    cv::Mat dst = src.clone();
    dst.setTo(0.0f, isolated);
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
