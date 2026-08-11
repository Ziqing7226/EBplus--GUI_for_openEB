// algo/tests/test_screwhead_detect.cpp — unit tests for the Zhou's Screw-Head
// Grid detector (algo/calibration/screwhead_detect.cpp).
//
// Renders synthetic three-valued colour frames (black bg / gold=ON / white=OFF)
// that mimic what CalibrationWizard::render_event_frame produces from a short
// event window, then exercises the full four-stage pipeline:
//   1. Clean grid      → all 30 markers detected, positions match.
//   2. Grid + noise    → still detected (robustness).
//   3. Empty frame     → not found.
//   4. Partial grid    → not found (all 30 required).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algo/calibration/screwhead_detect.h"

using gui_algo::detect_screwheads;

namespace {

// Colours must match CalibrationWizard::render_event_frame and
// screwhead_detect.cpp exactly (BGR ordering).
const cv::Vec3b kGold(0, 215, 255);    // ON polarity
const cv::Vec3b kWhite(255, 255, 255); // OFF polarity

// Grid geometry constants — mirror CircleGridDisplay defaults.
constexpr int kCols = 6;
constexpr int kRows = 5;
constexpr int kDotGap = 2;
constexpr double kMarkerRadiusFrac = 0.20;

// Renders a synthetic screw-head grid into a three-valued BGR frame.
//
// The grid uses the asymmetric layout (2c+(r&1), r)·spacing. Each marker is a
// solid ring (annulus R_in..R_out) whose right semicircle (dx > 0) is gold (ON,
// simulating the leading edge under +x translation) and left semicircle (dx ≤ 0)
// is white (OFF, trailing edge), plus a dashed cross (1px gold dots at period
// 1+dot_gap) inside the ring. This is exactly the polarity structure the real
// camera produces and the detector expects.
//
// @param spacing_px  Grid half-cell spacing in pixels (d).
// @param origin      Top-left grid origin in pixel coordinates.
// @param dot_gap     Dashed-cross dot gap (1/2/3).
// @param frame_size  Output frame size; must be large enough for the grid.
cv::Mat render_screwhead_grid(int spacing_px, cv::Point origin,
                              int dot_gap, cv::Size frame_size) {
    cv::Mat frame(frame_size, CV_8UC3, cv::Scalar(0, 0, 0));

    const double R = kMarkerRadiusFrac * spacing_px;
    const int g = dot_gap;
    const double R_out = R + g * 0.5;
    const double R_in = std::max(1.0, R - g * 0.5);
    const double cross_limit = std::max(1.0, R_in - 1.0);
    const int period = 1 + g;
    const int kmax = std::max(1, static_cast<int>(cross_limit / period));

    for (int r = 0; r < kRows; ++r) {
        for (int c = 0; c < kCols; ++c) {
            const int cx = origin.x + (2 * c + (r & 1)) * spacing_px;
            const int cy = origin.y + r * spacing_px;

            // Solid ring with polarity: right half = gold (ON), left half = white (OFF).
            const int ro = static_cast<int>(std::floor(R_out + 1));
            for (int dy = -ro; dy <= ro; ++dy) {
                for (int dx = -ro; dx <= ro; ++dx) {
                    const double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < R_in || dist > R_out) continue;
                    const int px = cx + dx, py = cy + dy;
                    if (px < 0 || px >= frame.cols || py < 0 || py >= frame.rows) continue;
                    frame.at<cv::Vec3b>(py, px) = (dx > 0) ? kGold : kWhite;
                }
            }

            // Dashed cross: 1px gold dots along x and y arms.
            for (int k = -kmax; k <= kmax; ++k) {
                const int off = k * period;
                if (std::abs(off) > static_cast<int>(cross_limit)) continue;
                // Horizontal arm.
                const int hx = cx + off, hy = cy;
                if (hx >= 0 && hx < frame.cols && hy >= 0 && hy < frame.rows)
                    frame.at<cv::Vec3b>(hy, hx) = kGold;
                // Vertical arm (skip centre — already drawn by horizontal).
                if (k != 0) {
                    const int vx = cx, vy = cy + off;
                    if (vx >= 0 && vx < frame.cols && vy >= 0 && vy < frame.rows)
                        frame.at<cv::Vec3b>(vy, vx) = kGold;
                }
            }
        }
    }
    return frame;
}

// Expected marker centres for the asymmetric grid (row-major).
std::vector<cv::Point2f> expected_centers(int spacing_px, cv::Point origin) {
    std::vector<cv::Point2f> pts;
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
            pts.emplace_back(
                static_cast<float>(origin.x + (2 * c + (r & 1)) * spacing_px),
                static_cast<float>(origin.y + r * spacing_px));
    return pts;
}

} // namespace

// --- Clean grid: the happy path ------------------------------------------------

TEST(ScrewHeadDetect, CleanGridAllThirtyMarkers) {
    const int spacing = 30;
    const cv::Point origin(60, 60);
    const cv::Size frame_size(450, 240);

    cv::Mat frame = render_screwhead_grid(spacing, origin, kDotGap, frame_size);
    auto res = detect_screwheads(frame, kCols, kRows, kDotGap, false);

    ASSERT_TRUE(res.found) << "Detection should succeed on a clean synthetic grid";
    ASSERT_EQ(res.points.size(), static_cast<size_t>(kCols * kRows));

    // Each detected point must be close to its expected centre (within ~30% of
    // the grid spacing — the density-peak localisation is sub-pixel-ish but the
    // grid-fit match tolerance is 0.35*d).
    const auto expected = expected_centers(spacing, origin);
    const double tol = 0.35 * spacing;
    for (size_t i = 0; i < expected.size(); ++i) {
        const double dx = res.points[i].x - expected[i].x;
        const double dy = res.points[i].y - expected[i].y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        EXPECT_LT(dist, tol)
            << "Point " << i << " at (" << res.points[i].x << "," << res.points[i].y
            << ") expected (" << expected[i].x << "," << expected[i].y
            << ") dist=" << dist << " tol=" << tol;
    }
}

// --- Grid + sparse noise: robustness ------------------------------------------

TEST(ScrewHeadDetect, GridWithSparseNoise) {
    const int spacing = 30;
    const cv::Point origin(60, 60);
    const cv::Size frame_size(450, 240);

    cv::Mat frame = render_screwhead_grid(spacing, origin, kDotGap, frame_size);

    // Add ~200 random coloured noise pixels (less than one per ~500 px).
    cv::RNG rng(42);  // deterministic seed
    for (int i = 0; i < 200; ++i) {
        const int x = rng.uniform(0, frame.cols);
        const int y = rng.uniform(0, frame.rows);
        const int pol = rng.uniform(0, 2);
        frame.at<cv::Vec3b>(y, x) = pol ? kGold : kWhite;
    }

    auto res = detect_screwheads(frame, kCols, kRows, kDotGap, false);
    EXPECT_TRUE(res.found);
    EXPECT_EQ(res.points.size(), static_cast<size_t>(kCols * kRows));
}

// --- Empty frame: rejection ---------------------------------------------------

TEST(ScrewHeadDetect, EmptyFrameRejected) {
    cv::Mat black(240, 450, CV_8UC3, cv::Scalar(0, 0, 0));
    auto res = detect_screwheads(black, kCols, kRows, kDotGap, false);
    EXPECT_FALSE(res.found);
    EXPECT_TRUE(res.points.empty());
}

// --- Partial grid (only 20 of 30 markers): rejection --------------------------

TEST(ScrewHeadDetect, PartialGridRejected) {
    const int spacing = 30;
    const cv::Point origin(60, 60);
    const cv::Size frame_size(450, 240);

    cv::Mat frame = render_screwhead_grid(spacing, origin, kDotGap, frame_size);

    // Erase the last 10 markers (rows 3-4) by painting black over their area.
    const double R = kMarkerRadiusFrac * spacing;
    const double R_out = R + kDotGap * 0.5 + 2;  // a bit of margin
    for (int r = 3; r < kRows; ++r) {
        for (int c = 0; c < kCols; ++c) {
            const int cx = origin.x + (2 * c + (r & 1)) * spacing;
            const int cy = origin.y + r * spacing;
            cv::circle(frame, cv::Point(cx, cy), static_cast<int>(R_out),
                       cv::Scalar(0, 0, 0), -1);
        }
    }

    auto res = detect_screwheads(frame, kCols, kRows, kDotGap, false);
    EXPECT_FALSE(res.found) << "A grid missing 10 of 30 markers must be rejected";
}

// --- Annotate flag: produces a non-empty annotated image ----------------------

TEST(ScrewHeadDetect, AnnotateProducesImage) {
    const int spacing = 30;
    const cv::Point origin(60, 60);
    const cv::Size frame_size(450, 240);

    cv::Mat frame = render_screwhead_grid(spacing, origin, kDotGap, frame_size);
    auto res = detect_screwheads(frame, kCols, kRows, kDotGap, true);

    ASSERT_TRUE(res.found);
    EXPECT_FALSE(res.image.empty());
    EXPECT_EQ(res.image.type(), frame.type());
}

// --- Capture-review features: crosses and rings are reported ------------------

TEST(ScrewHeadDetect, ReviewFeaturesPopulated) {
    const int spacing = 30;
    const cv::Point origin(60, 60);
    const cv::Size frame_size(450, 240);

    cv::Mat frame = render_screwhead_grid(spacing, origin, kDotGap, frame_size);
    auto res = detect_screwheads(frame, kCols, kRows, kDotGap, false);

    ASSERT_TRUE(res.found);
    // Every marker yields a cross centre (the density peak); a clean synthetic
    // grid also yields a ring for every marker (red circles in the review
    // dialog), with a positive radius.
    EXPECT_GE(res.cross_centers.size(), static_cast<size_t>(kCols * kRows));
    EXPECT_GE(res.ring_centers.size(), static_cast<size_t>(kCols * kRows));
    EXPECT_EQ(res.ring_centers.size(), res.ring_radii.size());
    for (float rad : res.ring_radii) {
        EXPECT_GT(rad, 0.f);
    }
}

// --- Ring parameters are plumbed: a strict setting kills ring detection ------

TEST(ScrewHeadDetect, RingParamsGateRings) {
    const int spacing = 30;
    const cv::Point origin(60, 60);
    const cv::Size frame_size(450, 240);
    cv::Mat frame = render_screwhead_grid(spacing, origin, kDotGap, frame_size);

    // Default parameters find rings for every marker.
    auto res_default = detect_screwheads(frame, kCols, kRows, kDotGap, false);
    ASSERT_GE(res_default.ring_centers.size(), static_cast<size_t>(kCols * kRows));

    // An absurd min-pixel count rejects every ring (the radial-histogram peak
    // can never reach it) — but crosses are still detected.
    gui_algo::RingParams strict;
    strict.min_pixels = 1 << 20;
    auto res_minpx = detect_screwheads(frame, kCols, kRows, kDotGap, false, strict);
    EXPECT_TRUE(res_minpx.ring_centers.empty());
    EXPECT_GE(res_minpx.cross_centers.size(), static_cast<size_t>(kCols * kRows));

    // An impossible coverage threshold (> 1.0) rejects every ring too.
    gui_algo::RingParams no_cover;
    no_cover.cover_frac = 2.0f;
    auto res_cover = detect_screwheads(frame, kCols, kRows, kDotGap, false, no_cover);
    EXPECT_TRUE(res_cover.ring_centers.empty());
}
