// gui/tests/test_coverage_hull.cpp — compute_coverage_overlay unit tests.

#include <gtest/gtest.h>

#include <vector>

#include <opencv2/core.hpp>

#include "calibration/coverage_hull.h"

namespace {

using gui::compute_coverage_overlay;

// No views / degenerate sensor → empty hull, zero coverage.
TEST(CoverageHull, EmptyInput) {
    const auto r = compute_coverage_overlay({}, 1000, 500);
    EXPECT_TRUE(r.hull.empty());
    EXPECT_DOUBLE_EQ(r.coverage, 0.0);
    const auto bad = compute_coverage_overlay({{{0, 0}, {10, 0}, {10, 10}}}, 0, 500);
    EXPECT_TRUE(bad.hull.empty());
    EXPECT_DOUBLE_EQ(bad.coverage, 0.0);
}

// Fewer than 3 points in total → no hull.
TEST(CoverageHull, TooFewPoints) {
    const auto r = compute_coverage_overlay({{{0, 0}, {10, 0}}}, 1000, 500);
    EXPECT_TRUE(r.hull.empty());
    EXPECT_DOUBLE_EQ(r.coverage, 0.0);
}

// A single view spanning a 100×50 rectangle on a 1000×500 sensor →
// hull is that rectangle, coverage = 5000 / 500000 = 1%.
TEST(CoverageHull, SingleViewRect) {
    const std::vector<std::vector<cv::Point2f>> views = {
        {{100, 100}, {200, 100}, {200, 150}, {100, 150}, {150, 125}},  // interior pt
    };
    const auto r = compute_coverage_overlay(views, 1000, 500);
    ASSERT_EQ(r.hull.size(), 4u);  // the interior point is dropped by the hull
    EXPECT_NEAR(r.coverage, 0.01, 1e-6);
}

// Two views at opposite corners → the hull covers the union's extremes.
TEST(CoverageHull, TwoViewUnion) {
    const std::vector<std::vector<cv::Point2f>> views = {
        {{0, 0}, {100, 0}, {0, 100}},
        {{900, 400}, {1000, 400}, {1000, 500}},
    };
    const auto r = compute_coverage_overlay(views, 1000, 500);
    ASSERT_GE(r.hull.size(), 4u);
    // The hull spans both extremes (area 120000 of 500000 ≈ 24%): well above
    // either triangle alone, inside the sensor.
    EXPECT_GT(r.coverage, 0.2);
    EXPECT_LE(r.coverage, 1.0);
}

} // namespace
