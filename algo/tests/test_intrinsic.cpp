// algo/tests/test_intrinsic.cpp — unit tests for IntrinsicCalibration (Zhang
// two-pass method).
//
// Locks: the chessboard object-point formula, the detect_only/accept split
// (detect does not accumulate), the duplicate-pose rejection used by the
// wizard, and the two-pass calibration with per-view outlier rejection
// (bad views dropped, per-view RMS reported). Full blink-frame detection is
// covered by test_blinking_detect.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "algo/calibration/intrinsic.h"

using gui_algo::CalibrationPattern;
using gui_algo::IntrinsicCalibration;

namespace {

// Synthesizes N views of a chessboard (inner corners 9×6, 20 mm squares)
// projected through a known pinhole camera, with optional per-view corruption
// (a large offset on selected views simulates bad detections).
struct SyntheticViews {
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<cv::Point3f> object_grid;
    cv::Mat K;
};

SyntheticViews make_views(std::size_t n_views,
                          const std::vector<bool>& corrupt = {},
                          double noise_px = 0.3) {
    constexpr int kCols = 9, kRows = 6;
    constexpr float kSquareMm = 20.0f;
    const double fx = 500.0, fy = 500.0, cx = 320.0, cy = 240.0;

    SyntheticViews sv;
    sv.K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Object grid: 9×6 inner corners at (c·20, r·20, 0) mm.
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
            sv.object_grid.emplace_back(c * kSquareMm, r * kSquareMm, 0.0f);

    std::mt19937 rng(1234);
    std::normal_distribution<double> noise(0.0, noise_px);
    const cv::Mat no_dist = cv::Mat::zeros(1, 5, CV_64F);

    for (std::size_t i = 0; i < n_views; ++i) {
        // Varied, plausible poses (small rotations, camera at ~500 mm distance).
        const cv::Vec3d rvec(0.15 + 0.25 * static_cast<double>(i),
                             -0.05 * static_cast<double>(i),
                             0.02 * static_cast<double>(i));
        const cv::Vec3d tvec(-40.0 * static_cast<double>(i % 3),
                             25.0 * static_cast<double>((i / 3) % 2),
                             500.0);

        std::vector<cv::Point2f> pts;
        cv::projectPoints(sv.object_grid, rvec, tvec, sv.K, no_dist, pts);
        for (auto& p : pts) {
            p.x += static_cast<float>(noise(rng));
            p.y += static_cast<float>(noise(rng));
        }
        // Corrupt the view if requested (large coherent offset → huge RMS).
        if (i < corrupt.size() && corrupt[i]) {
            for (auto& p : pts) { p.x += 200.0f; p.y -= 200.0f; }
        }
        sv.image_points.push_back(std::move(pts));
    }
    return sv;
}

void feed(IntrinsicCalibration& cal, const SyntheticViews& sv) {
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    // detect_only records image_size_ on first call (mirrors the wizard, which
    // always detects before accumulating); the blank frame must not be found.
    auto res = cal.detect_only(cv::Mat(480, 640, CV_8UC1, cv::Scalar(0)), false);
    ASSERT_FALSE(res.found);
    for (const auto& pts : sv.image_points) cal.accept(pts);
}

} // namespace

TEST(IntrinsicCalibration, ChessboardObjectGridFormula) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    const auto g = cal.object_grid();
    ASSERT_EQ(g.size(), 54u);
    EXPECT_NEAR(g[0].x, 0, 1e-4f);           EXPECT_NEAR(g[0].y, 0, 1e-4f);
    EXPECT_NEAR(g[1].x, 20, 1e-4f);          EXPECT_NEAR(g[1].y, 0, 1e-4f);
    EXPECT_NEAR(g[8].x, 160, 1e-4f);         EXPECT_NEAR(g[8].y, 0, 1e-4f);
    EXPECT_NEAR(g[9].x, 0, 1e-4f);           EXPECT_NEAR(g[9].y, 20, 1e-4f);
    EXPECT_NEAR(g[53].x, 160, 1e-4f);        EXPECT_NEAR(g[53].y, 100, 1e-4f);
}

TEST(IntrinsicCalibration, DetectOnlyDoesNotAccumulate) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    cv::Mat black(128, 128, CV_8UC1, cv::Scalar(0));
    auto res = cal.detect_only(black, false);
    EXPECT_FALSE(res.found);
    EXPECT_EQ(cal.frame_count(), 0u);
}

TEST(IntrinsicCalibration, AcceptAccumulatesAndDuplicateDetected) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    // Synthesize 54 ordered points matching the 9×6 grid (row-major, *10 px scale).
    std::vector<cv::Point2f> pts;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 9; ++c)
            pts.emplace_back(c * 20.f * 10.f, r * 20.f * 10.f);
    cal.accept(pts);
    EXPECT_EQ(cal.frame_count(), 1u);
    // Identical points → duplicate of the stored pose.
    EXPECT_TRUE(cal.is_duplicate_pose(pts, 10.0));
    // Translated points → a different pose.
    std::vector<cv::Point2f> shifted = pts;
    for (auto& p : shifted) { p.x += 50; p.y += 50; }
    EXPECT_FALSE(cal.is_duplicate_pose(shifted, 10.0));
}

TEST(IntrinsicCalibration, RejectsEmptyFrame) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    auto res = cal.detect_only(cv::Mat(), false);
    EXPECT_FALSE(res.found);
    EXPECT_TRUE(res.points.empty());
}

TEST(IntrinsicCalibration, TwoPassCalibrationRecoversIntrinsics) {
    SyntheticViews sv = make_views(8);
    IntrinsicCalibration cal;
    feed(cal, sv);

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.kept_frames, 8u);
    EXPECT_EQ(r.removed_frames, 0u);
    EXPECT_EQ(r.frames_used, 8u);
    EXPECT_EQ(r.per_view_rms.size(), 8u);
    // 0.3 px noise → RMS well under 2 px.
    EXPECT_LT(r.rms, 2.0);
    // Camera matrix recovered near ground truth (fx = fy = 500, cx = 320, cy = 240).
    EXPECT_NEAR(r.K.at<double>(0, 0), 500.0, 30.0);
    EXPECT_NEAR(r.K.at<double>(1, 1), 500.0, 30.0);
    EXPECT_NEAR(r.K.at<double>(0, 2), 320.0, 30.0);
    EXPECT_NEAR(r.K.at<double>(1, 2), 240.0, 30.0);
}

TEST(IntrinsicCalibration, OutlierViewIsRemoved) {
    // One corrupted view (of 8) must be dropped by the default mean+2σ rule.
    const std::vector<bool> corrupt = {false, false, false, true, false, false, false, false};
    SyntheticViews sv = make_views(8, corrupt);
    IntrinsicCalibration cal;
    feed(cal, sv);

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.removed_frames, 1u);
    EXPECT_EQ(r.kept_frames, 7u);
    EXPECT_EQ(r.frames_used, 7u);
    EXPECT_EQ(r.per_view_rms.size(), 7u);
    // The corrupted view is flagged in selected_views (index 3).
    ASSERT_EQ(r.selected_views.size(), 8u);
    EXPECT_FALSE(r.selected_views[3]);
    EXPECT_LT(r.rms, 2.0);
}

TEST(IntrinsicCalibration, NegativeOutlierThresholdKeepsAll) {
    const std::vector<bool> corrupt = {false, false, false, true, false, false};
    SyntheticViews sv = make_views(6, corrupt);
    IntrinsicCalibration cal;
    feed(cal, sv);
    cal.set_outlier_threshold(-1.0);  // keep all views

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.removed_frames, 0u);
    EXPECT_EQ(r.kept_frames, 6u);
}

TEST(IntrinsicCalibration, TooFewViewsAfterRejectionFails) {
    // 6 views, 4 of them heavily corrupted; an aggressive outlier threshold
    // (0.1σ) rejects them → only 2 survive → run() errors.
    const std::vector<bool> corrupt = {false, true, true, true, true, false};
    SyntheticViews sv = make_views(6, corrupt);
    IntrinsicCalibration cal;
    feed(cal, sv);
    cal.set_outlier_threshold(0.1);

    const auto r = cal.run();
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.error.empty());
    EXPECT_LT(r.kept_frames, 3u);
}

TEST(IntrinsicCalibration, FewerThanThreeFramesFails) {
    SyntheticViews sv = make_views(2);
    IntrinsicCalibration cal;
    feed(cal, sv);

    const auto r = cal.run();
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.error.empty());
}
