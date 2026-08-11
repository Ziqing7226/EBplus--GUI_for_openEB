// algo/tests/test_blinking_detect.cpp — unit tests for the blinking-chessboard
// calibration support (algo/calibration/blinking_detect.h + the Chessboard
// branch of IntrinsicCalibration::detect_only).
//
// Locks: blink-frame construction (both-polarity mask), the validity gate
// (min blinking pixels + single-polarity ratios), and end-to-end chessboard
// detection on a synthetic blink frame.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo/calibration/blinking_detect.h"
#include "algo/calibration/intrinsic.h"

using gui_algo::BlinkParams;
using gui_algo::BlinkFrame;
using gui_algo::CalibrationPattern;
using gui_algo::IntrinsicCalibration;
using gui_algo::accumulate_blink_counts;
using gui_algo::accumulate_blink_masks;
using gui_algo::build_blink_frame;
using gui_algo::build_blink_frame_from_counts;
using gui_algo::make_blink_frame;

namespace {

// Builds an EventCD at (x, y) with the given polarity and timestamp.
Metavision::EventCD ev(int x, int y, short p, Metavision::timestamp t = 0) {
    Metavision::EventCD e;
    e.x = x; e.y = y; e.p = p; e.t = t;
    return e;
}

} // namespace

TEST(BlinkFrame, BothPolarityPixelsBecomeBlackOnWhite) {
    // A 3×2 region: (0,0) and (2,1) see BOTH polarities; (1,0) sees only ON;
    // (0,1) sees only OFF; the rest nothing. The blink frame is INVERTED:
    // both-polarity pixels are black (0), everything else white (255).
    // Median blur is disabled so isolated single-pixel dots are not erased.
    BlinkParams params;
    params.median_blur_diameter = 0;
    std::vector<Metavision::EventCD> events = {
        ev(0, 0, 1), ev(0, 0, 0),
        ev(2, 1, 1), ev(2, 1, 0),
        ev(1, 0, 1),
        ev(0, 1, 0),
    };
    const BlinkFrame bf = make_blink_frame(events.data(), events.data() + events.size(),
                                           4, 4, params);
    ASSERT_EQ(bf.both, 2);
    ASSERT_EQ(bf.only_on, 1);
    ASSERT_EQ(bf.only_off, 1);
    ASSERT_EQ(bf.frame.type(), CV_8UC1);
    ASSERT_EQ(bf.frame.at<uchar>(0, 0), 0);   // both polarities → black
    ASSERT_EQ(bf.frame.at<uchar>(1, 2), 0);   // both polarities → black
    ASSERT_EQ(bf.frame.at<uchar>(0, 1), 255); // only ON → not blinking → white
    ASSERT_EQ(bf.frame.at<uchar>(1, 0), 255); // only OFF → not blinking → white
}

TEST(BlinkFrame, GateRejectsTooFewBlinkingPixels) {
    // Only one both-polarity pixel, min = 100 → invalid.
    std::vector<Metavision::EventCD> events = { ev(0, 0, 1), ev(0, 0, 0) };
    BlinkParams params;
    params.min_blink_pixels = 100;
    const BlinkFrame bf = make_blink_frame(events.data(), events.data() + events.size(),
                                           8, 8, params);
    EXPECT_FALSE(bf.valid);
}

TEST(BlinkFrame, GateRejectsPolarityImbalancedFrame) {
    // Many both pixels but also many only-ON pixels → ratio_on exceeded.
    std::vector<Metavision::EventCD> events;
    for (int i = 0; i < 200; ++i) {
        const int x = i % 20, y = i / 20;  // 20×10 block of both-polarity pixels
        events.push_back(ev(x, y, 1));
        events.push_back(ev(x, y, 0));
    }
    for (int i = 0; i < 60; ++i) {  // 60 only-ON over 200 both = 0.30 > 0.15
        events.push_back(ev(20 + i, 0, 1));
    }
    const BlinkFrame bf = make_blink_frame(events.data(), events.data() + events.size(),
                                           80, 12, BlinkParams{});
    ASSERT_EQ(bf.both, 200);
    ASSERT_EQ(bf.only_on, 60);
    EXPECT_FALSE(bf.valid);
}

TEST(BlinkFrame, RebuildFromMasksWithRelaxedParams) {
    // Masks are the durable capture representation: build once with strict
    // params (invalid), then rebuild the frame from the SAME masks with
    // relaxed params (valid) — mirrors the wizard's Re-detect flow.
    std::vector<Metavision::EventCD> events;
    for (int i = 0; i < 200; ++i) {
        const int x = i % 20, y = i / 20;
        events.push_back(ev(x, y, 1));
        events.push_back(ev(x, y, 0));
    }
    cv::Mat on, off;
    accumulate_blink_masks(events.data(), events.data() + events.size(), 32, 16, on, off);

    BlinkParams strict;
    strict.min_blink_pixels = 1000;
    EXPECT_FALSE(build_blink_frame(on, off, strict).valid);

    BlinkParams relaxed;
    relaxed.min_blink_pixels = 100;
    const BlinkFrame bf = build_blink_frame(on, off, relaxed);
    EXPECT_TRUE(bf.valid);
    EXPECT_EQ(bf.both, 200);
    // Both-polarity block (x<20, y<10) is black; outside it the frame is white.
    // at<uchar>(row=y, col=x).
    EXPECT_EQ(bf.frame.at<uchar>(0, 0), 0);
    EXPECT_EQ(bf.frame.at<uchar>(15, 25), 255);
}

TEST(BlinkFrame, EmptyInputInvalid) {
    const BlinkFrame bf = make_blink_frame(nullptr, nullptr, 8, 8, BlinkParams{});
    EXPECT_FALSE(bf.valid);
    // A full-white frame is still produced (no blinking pixels → all white);
    // only the validity gate fails.
    EXPECT_FALSE(bf.frame.empty());
    EXPECT_EQ(bf.frame.at<uchar>(0, 0), 255);
}

TEST(BlinkFrame, ChessboardDetectionOnSyntheticBlinkFrame) {
    // Synthesize a 10×7 checkerboard (9×6 inner corners) as both-polarity
    // events on the black squares → blink frame is a filled checkerboard →
    // cv::findChessboardCorners must find the 9×6 inner corners.
    constexpr int kCols = 9, kRows = 6;
    constexpr int kSquarePx = 40;
    constexpr int kImgW = kCols * kSquarePx + 2 * kSquarePx;  // 10×7 board + margin
    constexpr int kImgH = kRows * kSquarePx + 2 * kSquarePx;
    const int sq_cols = kCols + 1, sq_rows = kRows + 1;

    std::vector<Metavision::EventCD> events;
    events.reserve(sq_cols * sq_rows * kSquarePx * kSquarePx * 2);
    for (int r = 0; r < sq_rows; ++r) {
        for (int c = 0; c < sq_cols; ++c) {
            if (((r + c) & 1) != 0) continue;  // black squares blink
            for (int dy = 0; dy < kSquarePx; ++dy) {
                for (int dx = 0; dx < kSquarePx; ++dx) {
                    const int x = c * kSquarePx + dx;
                    const int y = r * kSquarePx + dy;
                    events.push_back(ev(x, y, 1));
                    events.push_back(ev(x, y, 0));
                }
            }
        }
    }

    const BlinkFrame bf = make_blink_frame(events.data(), events.data() + events.size(),
                                           kImgW, kImgH, BlinkParams{});
    ASSERT_TRUE(bf.valid);
    ASSERT_GT(bf.both, 0);

    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, kCols, kRows, 20.0f);
    const auto res = cal.detect_only(bf.frame, false);
    ASSERT_TRUE(res.found) << "findChessboardCorners failed on the synthetic blink frame";
    EXPECT_EQ(res.points.size(), static_cast<std::size_t>(kCols * kRows));
}

// Regression: on a real LCD the backlight PWM gives EVERY on-screen pixel both
// polarities, so the binary both-mask is a solid block and detection fails;
// the black squares stand out only by a higher event COUNT. The count-based
// frame must recover the checkerboard where the mask-based one cannot.
TEST(BlinkFrame, CountBasedFrameSurvivesPolarityImbalancedNoise) {
    constexpr int kCols = 9, kRows = 6;
    constexpr int kSquarePx = 40;
    constexpr int kImgW = kCols * kSquarePx + 2 * kSquarePx;  // 10×7 board + margin
    constexpr int kImgH = kRows * kSquarePx + 2 * kSquarePx;
    const int sq_cols = kCols + 1, sq_rows = kRows + 1;

    // Black squares: 60 ON + 60 OFF events per pixel (strong blink).
    // White squares AND background: 8 ON + 8 OFF per pixel (PWM noise —
    // both polarities, so the binary mask sees them as "blinking" too).
    constexpr int kBlackCount = 60;
    constexpr int kNoiseCount = 8;
    std::vector<Metavision::EventCD> events;
    for (int r = 0; r < sq_rows; ++r) {
        for (int c = 0; c < sq_cols; ++c) {
            const bool black = ((r + c) & 1) == 0;
            for (int dy = 0; dy < kSquarePx; ++dy) {
                for (int dx = 0; dx < kSquarePx; ++dx) {
                    const int x = c * kSquarePx + dx;
                    const int y = r * kSquarePx + dy;
                    const int cnt = black ? kBlackCount : kNoiseCount;
                    for (int k = 0; k < cnt; ++k) {
                        events.push_back(ev(x, y, 1));
                        events.push_back(ev(x, y, 0));
                    }
                }
            }
        }
    }

    // The binary both-mask frame is a solid block → detection fails.
    {
        cv::Mat on, off;
        accumulate_blink_masks(events.data(), events.data() + events.size(),
                               kImgW, kImgH, on, off);
        const BlinkFrame bf = build_blink_frame(on, off, BlinkParams{});
        ASSERT_TRUE(bf.valid);  // gate passes — that is the trap
        IntrinsicCalibration cal;
        cal.set_pattern(CalibrationPattern::Chessboard, kCols, kRows, 20.0f);
        const auto res = cal.detect_only(bf.frame, false);
        EXPECT_FALSE(res.found) << "binary both-mask must NOT detect under "
                                   "polarity-imbalanced noise";
    }

    // The count-based frame recovers the checkerboard.
    {
        cv::Mat on, off;
        accumulate_blink_counts(events.data(), events.data() + events.size(),
                                kImgW, kImgH, on, off);
        const BlinkFrame bf = build_blink_frame_from_counts(on, off, BlinkParams{});
        ASSERT_TRUE(bf.valid);
        IntrinsicCalibration cal;
        cal.set_pattern(CalibrationPattern::Chessboard, kCols, kRows, 20.0f);
        const auto res = cal.detect_only(bf.frame, false);
        ASSERT_TRUE(res.found) << "count-based blink frame must detect the "
                                  "checkerboard under polarity-imbalanced noise";
        EXPECT_EQ(res.points.size(), static_cast<std::size_t>(kCols * kRows));
    }
}
