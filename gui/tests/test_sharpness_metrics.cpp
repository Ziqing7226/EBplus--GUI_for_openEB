// gui/tests/test_sharpness_metrics.cpp — unit tests for the count-image
// sharpness metrics (systematic_audit §9.3 validation).
//
// Covers the audit's R1/R2 failure modes directly:
//   - thin vs thick synthetic lines: contrast is higher and mean line width
//     lower for the sharper (thinner) line;
//   - isolated hot-pixel noise (R1): denoising keeps both metrics within
//     10 % of the noise-free values;
//   - event density x2 (R2): normalized contrast stays within 20 %;
//   - empty image: no crash, metrics flagged invalid so the UI can show "—".
//
// Pure OpenCV — no Qt widgets or QApplication needed, so this test links
// GTest::Main (no custom main) and needs no offscreen platform.

#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/core.hpp>

#include "calibration/sharpness_metrics.h"

using gui::SharpnessMetrics;
using gui::compute_sharpness_metrics;

namespace {

constexpr int kRows = 64;
constexpr int kCols = 64;
constexpr int kLineY = 30;      // horizontal line row (top row of the stroke)
constexpr float kLineCount = 4.0f;
constexpr double kWindowS = 0.1;

/// @brief Count image with a single horizontal line of the given thickness,
/// every line pixel holding @p count events.
cv::Mat make_line_image(int thickness, float count = kLineCount) {
    cv::Mat img = cv::Mat::zeros(kRows, kCols, CV_32F);
    for (int dy = 0; dy < thickness; ++dy) {
        img.row(kLineY + dy).setTo(count);
    }
    return img;
}

} // namespace

// S3/S4 sanity: a sharper (1 px) line must score higher contrast and smaller
// line width than the same line blurred to 3 px.
TEST(SharpnessMetrics, ThinLineScoresBetterThanThickLine) {
    const SharpnessMetrics thin = compute_sharpness_metrics(
        make_line_image(1), kWindowS);
    const SharpnessMetrics thick = compute_sharpness_metrics(
        make_line_image(3), kWindowS);

    ASSERT_TRUE(thin.valid);
    ASSERT_TRUE(thick.valid);
    EXPECT_GT(thin.contrast, thick.contrast);
    EXPECT_LT(thin.mean_line_width, thick.mean_line_width);
}

// R1 fix: sprinkle isolated count-1 hot pixels (grid spacing 4, so none has
// a non-empty 8-neighbour and none touches the line). After denoising, both
// metrics must stay within 10 % of the noise-free values.
TEST(SharpnessMetrics, IsolatedNoiseDoesNotDistortMetrics) {
    const cv::Mat clean = make_line_image(2);
    cv::Mat noisy = clean.clone();
    for (int y = 4; y < kRows; y += 4) {
        for (int x = 4; x < kCols; x += 4) {
            if (noisy.at<float>(y, x) == 0.0f) noisy.at<float>(y, x) = 1.0f;
        }
    }

    const SharpnessMetrics base = compute_sharpness_metrics(clean, kWindowS);
    const SharpnessMetrics with_noise =
        compute_sharpness_metrics(noisy, kWindowS);

    ASSERT_TRUE(base.valid);
    ASSERT_TRUE(with_noise.valid);
    EXPECT_NEAR(with_noise.contrast, base.contrast,
                0.10 * std::fabs(base.contrast));
    EXPECT_NEAR(with_noise.mean_line_width, base.mean_line_width,
                0.10 * std::fabs(base.mean_line_width));
}

// R2 fix: doubling every pixel's event count (same pattern, twice the
// density) must leave the normalized contrast essentially unchanged (< 20 %).
TEST(SharpnessMetrics, ContrastInvariantToEventDensity) {
    const cv::Mat img = make_line_image(2);
    const cv::Mat dense = img * 2.0f;

    const SharpnessMetrics sparse = compute_sharpness_metrics(img, kWindowS);
    const SharpnessMetrics doubled = compute_sharpness_metrics(dense, kWindowS);

    ASSERT_TRUE(sparse.valid);
    ASSERT_TRUE(doubled.valid);
    EXPECT_NEAR(doubled.contrast, sparse.contrast,
                0.20 * std::fabs(sparse.contrast));
    // The binary edge mask is unchanged, so the width must match too.
    EXPECT_DOUBLE_EQ(doubled.mean_line_width, sparse.mean_line_width);
    // Event rate doubles: it reports density, not sharpness (reference only).
    EXPECT_DOUBLE_EQ(doubled.event_rate, 2.0 * sparse.event_rate);
}

// Empty / all-zero images must not crash and must be flagged invalid so the
// dialog shows "—" instead of garbage.
TEST(SharpnessMetrics, EmptyImageIsInvalid) {
    const SharpnessMetrics from_empty =
        compute_sharpness_metrics(cv::Mat(), kWindowS);
    EXPECT_FALSE(from_empty.valid);
    EXPECT_TRUE(std::isnan(from_empty.contrast));
    EXPECT_TRUE(std::isnan(from_empty.mean_line_width));
    EXPECT_DOUBLE_EQ(from_empty.event_rate, 0.0);

    const cv::Mat zeros = cv::Mat::zeros(kRows, kCols, CV_32F);
    const SharpnessMetrics from_zeros =
        compute_sharpness_metrics(zeros, kWindowS);
    EXPECT_FALSE(from_zeros.valid);
    EXPECT_TRUE(std::isnan(from_zeros.contrast));
    EXPECT_TRUE(std::isnan(from_zeros.mean_line_width));
}

// Pure noise (only isolated hot pixels, no real edges): denoising removes
// everything, so the result is invalid rather than a bogus "sharp" reading —
// this is the exact R1 failure direction of the old Laplacian metric.
TEST(SharpnessMetrics, PureNoiseYieldsNoSharpnessReading) {
    cv::Mat noise = cv::Mat::zeros(kRows, kCols, CV_32F);
    for (int y = 2; y < kRows; y += 4) {
        for (int x = 2; x < kCols; x += 4) {
            noise.at<float>(y, x) = 1.0f;
        }
    }
    const SharpnessMetrics m = compute_sharpness_metrics(noise, kWindowS);
    EXPECT_FALSE(m.valid);
}
