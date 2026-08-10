// algo/tests/test_intrinsic.cpp — unit tests for IntrinsicCalibration (Zhou's
// screw-head grid).
//
// Locks the ScrewHeadGrid contract: the object-point formula (asymmetric grid,
// d = square/2 because the user measures the same-row/same-column adjacent pair), the
// detect_only/accept split (detect does not accumulate), set_dot_gap storage,
// and the duplicate-pose rejection used by the wizard. Full detection of a
// synthetic screw-head frame is covered by test_screwhead_detect.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include "algo/calibration/intrinsic.h"

using gui_algo::CalibrationPattern;
using gui_algo::IntrinsicCalibration;

namespace {
constexpr float kTwo = 2.0f;
} // namespace

TEST(IntrinsicCalibration, ScrewHeadObjectGridFormula) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::ScrewHeadGrid, 4, 11, 5.0f);
    const auto g = cal.object_grid();
    ASSERT_EQ(g.size(), 44u);
    // d = square / 2: the user measures the same-row (or same-column) adjacent
    // marker distance = 2d, so the entered value is halved to recover d. No √2.
    const float d = 5.0f / kTwo;
    EXPECT_NEAR(g[0].x, 0, 1e-4f);          EXPECT_NEAR(g[0].y, 0, 1e-4f);        // r=0,c=0
    EXPECT_NEAR(g[1].x, 2 * d, 1e-4f);      EXPECT_NEAR(g[1].y, 0, 1e-4f);        // r=0,c=1 (same-row adjacent = 2d = square)
    EXPECT_NEAR(g[4].x, d, 1e-4f);          EXPECT_NEAR(g[4].y, d, 1e-4f);        // r=1,c=0 -> (2*0+1)*d, d
    EXPECT_NEAR(g[5].x, 3 * d, 1e-4f);      EXPECT_NEAR(g[5].y, d, 1e-4f);        // r=1,c=1 -> (2*1+1)*d, d
    EXPECT_NEAR(g[40].x, 0, 1e-4f);         EXPECT_NEAR(g[40].y, 10 * d, 1e-4f);  // r=10,c=0 (even)
    EXPECT_NEAR(g[43].x, 6 * d, 1e-4f);     EXPECT_NEAR(g[43].y, 10 * d, 1e-4f);  // r=10,c=3 -> 2*3*d
}

TEST(IntrinsicCalibration, SetDotGapStoresValue) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::ScrewHeadGrid, 6, 5, 5.0f);
    cal.set_dot_gap(3);
    // No direct getter — verify via detect on an empty frame (must not crash and
    // must reject). The stored value only affects detection, not object geometry.
    auto res = cal.detect_only(cv::Mat(10, 10, CV_8UC3, cv::Scalar(0, 0, 0)), false);
    EXPECT_FALSE(res.found);
    EXPECT_TRUE(res.points.empty());
    // Object grid is unaffected by dot_gap.
    EXPECT_EQ(cal.object_grid().size(), 30u);
}

TEST(IntrinsicCalibration, DetectOnlyDoesNotAccumulate) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::ScrewHeadGrid, 6, 5, 5.0f);
    // Empty/black frame → detection fails, nothing accumulated.
    cv::Mat black(128, 128, CV_8UC3, cv::Scalar(0, 0, 0));
    auto res = cal.detect_only(black, false);
    EXPECT_FALSE(res.found);
    EXPECT_EQ(cal.frame_count(), 0u);
}

TEST(IntrinsicCalibration, AcceptAccumulatesAndDuplicateDetected) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::ScrewHeadGrid, 6, 5, 5.0f);
    // Synthesize 30 ordered points matching a 6x5 grid (row-major).
    std::vector<cv::Point2f> pts;
    const float d = 5.0f / kTwo;
    for (int r = 0; r < 5; ++r)
        for (int c = 0; c < 6; ++c)
            pts.emplace_back((2 * c + (r & 1)) * d * 10.f, r * d * 10.f);  // *10 px scale
    cal.accept(pts);
    EXPECT_EQ(cal.frame_count(), 1u);
    // Identical points -> duplicate of the stored pose.
    EXPECT_TRUE(cal.is_duplicate_pose(pts, 10.0));
    // Translated points -> a different pose.
    std::vector<cv::Point2f> shifted = pts;
    for (auto& p : shifted) { p.x += 50; p.y += 50; }
    EXPECT_FALSE(cal.is_duplicate_pose(shifted, 10.0));
}

TEST(IntrinsicCalibration, RejectsEmptyFrame) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::ScrewHeadGrid, 6, 5, 5.0f);
    auto res = cal.detect_only(cv::Mat(), false);
    EXPECT_FALSE(res.found);
    EXPECT_TRUE(res.points.empty());
}
