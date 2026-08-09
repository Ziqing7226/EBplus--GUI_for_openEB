// algo/tests/test_intrinsic.cpp — unit tests for IntrinsicCalibration (Phase 4).
//
// Locks the Phase 4 contract: the AsymmetricCircles object-point formula, the
// detect_only/accept split (detect does not accumulate), the add_frame
// convenience wrapper, and the duplicate-pose rejection used by the wizard.

#include <gtest/gtest.h>

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algo/calibration/intrinsic.h"

using gui_algo::CalibrationPattern;
using gui_algo::IntrinsicCalibration;

namespace {

// Synthesizes a cols×rows asymmetric circle-grid image matching the
// object-point formula x=(2c+(r&1))*spacing, y=r*spacing. BLACK filled
// circles on a WHITE background — the polarity cv::findCirclesGrid's default
// blob detector expects (dark blobs on a light field). The on-screen
// CircleGridDisplay is the inverse (white-on-black, high contrast for the
// camera); the rendered event capture frame is also dark-on-light for
// detection.
cv::Mat make_asymmetric_grid(int cols, int rows, int spacing, int dot_r,
                             int margin) {
    const int footprint_w = 2 * cols - 1;
    const int footprint_h = rows - 1;
    const int w = footprint_w * spacing + 2 * margin;
    const int h = footprint_h * spacing + 2 * margin;
    cv::Mat img(h, w, CV_8UC1, cv::Scalar(255));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const cv::Point center(margin + (2 * c + (r & 1)) * spacing,
                                   margin + r * spacing);
            cv::circle(img, center, dot_r, cv::Scalar(0), -1);
        }
    }
    return img;
}

} // namespace

TEST(IntrinsicCalibration, AsymmetricObjectGridFormula) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::AsymmetricCircles, 4, 11, 5.0f);
    const auto g = cal.object_grid();
    ASSERT_EQ(g.size(), 44u);
    // Row-major order (index = r*cols + c). Spot-check the asymmetric offset:
    // even rows start at x=0, odd rows offset by one cell (x=spacing).
    EXPECT_EQ(g[0],  cv::Point3f(0,  0,  0));   // r=0,c=0
    EXPECT_EQ(g[1],  cv::Point3f(10, 0,  0));   // r=0,c=1 -> (2*5,0)
    EXPECT_EQ(g[4],  cv::Point3f(5,  5,  0));   // r=1,c=0 -> ((2*0+1)*5, 1*5)
    EXPECT_EQ(g[5],  cv::Point3f(15, 5,  0));   // r=1,c=1 -> ((2*1+1)*5, 5)
    EXPECT_EQ(g[40], cv::Point3f(0,  50, 0));   // r=10,c=0 (even) -> (0, 10*5)
    EXPECT_EQ(g[43], cv::Point3f(30, 50, 0));   // r=10,c=3 -> (2*3*5, 50)
}

TEST(IntrinsicCalibration, DetectOnlyDoesNotAccumulate) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::AsymmetricCircles, 4, 11, 5.0f);
    cv::Mat img = make_asymmetric_grid(4, 11, 30, 10, 40);
    auto res = cal.detect_only(img, false);
    EXPECT_TRUE(res.found);
    ASSERT_EQ(res.points.size(), 44u);
    EXPECT_EQ(cal.frame_count(), 0u);  // detect_only must not accumulate
}

TEST(IntrinsicCalibration, AcceptAccumulatesAndDuplicateDetected) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::AsymmetricCircles, 4, 11, 5.0f);
    cv::Mat img = make_asymmetric_grid(4, 11, 30, 10, 40);
    auto res = cal.detect_only(img, false);
    ASSERT_TRUE(res.found);
    cal.accept(res.points);
    EXPECT_EQ(cal.frame_count(), 1u);

    // Identical points -> duplicate of the stored pose.
    EXPECT_TRUE(cal.is_duplicate_pose(res.points, 10.0));
    // Translated points -> a different pose.
    std::vector<cv::Point2f> shifted = res.points;
    for (auto& p : shifted) { p.x += 50; p.y += 50; }
    EXPECT_FALSE(cal.is_duplicate_pose(shifted, 10.0));
}

TEST(IntrinsicCalibration, AddFrameAccumulatesOnFound) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::AsymmetricCircles, 4, 11, 5.0f);
    cv::Mat img = make_asymmetric_grid(4, 11, 30, 10, 40);
    auto res = cal.add_frame(img, false);
    EXPECT_TRUE(res.found);
    EXPECT_EQ(cal.frame_count(), 1u);  // add_frame = detect_only + accept
}

TEST(IntrinsicCalibration, RejectsEmptyFrame) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::AsymmetricCircles, 4, 11, 5.0f);
    auto res = cal.detect_only(cv::Mat(), false);
    EXPECT_FALSE(res.found);
    EXPECT_TRUE(res.points.empty());
}
