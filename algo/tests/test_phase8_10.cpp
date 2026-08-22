// algo/tests/test_phase8_10.cpp — unit tests for Phase 8-10 modules.
//
// Covers Phase 8 (algo/cv/ §4.3.13–4.3.23), Phase 9 (algo/cv/ §4.3.24–4.3.27),
// and Phase 10 (algo/analytics/ §4.4.1–4.4.7). Compiled with -Wall -Wextra -Werror.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/cv/line_segment_detector.h"
#include "algo/cv/hough_line_tracker.h"
#include "algo/cv/hough_circle_tracker.h"
#include "algo/cv/orientation_cluster.h"
#include "algo/cv/cluster_lif.h"
#include "algo/cv/background_mask_filter.h"
#include "algo/cv/optical_gyro.h"
#include "algo/cv/xyt_visualizer.h"
#include "algo/cv/time_surface.h"
#include "algo/cv/dense_optical_flow.h"
#include "algo/cv/frequency_map.h"
#include "algo/analytics/event_to_video.h"
#include "algo/analytics/e2vid/event_voxel_grid.h"
#include "algo/analytics/e2vid/intensity_rescaler.h"
#include "algo/analytics/e2vid/unsharp_mask.h"
#include "algo/analytics/e2vid/e2vid_inference.h"
#include "algo/analytics/auto_bias_controller.h"
#include "algo/analytics/freq_detector.h"
#include "algo/analytics/sensor_self_test.h"

using gui_algo::Event;
using gui_algo::EventPacket;
using gui_algo::MutableEventPacket;
using gui_algo::LineSegmentDetector;
using gui_algo::HoughLineTracker;
using gui_algo::HoughCircleTracker;
using gui_algo::OrientationCluster;
using gui_algo::ClusterLIF;
using gui_algo::BackgroundMaskFilter;
using gui_algo::OpticalGyro;
using gui_algo::XYTVisualizer;
using gui_algo::TimeSurface;
using gui_algo::DenseOpticalFlow;
using gui_algo::FrequencyMap;
using gui_algo::FrequencyMapParams;
using gui_algo::EventToVideo;
using gui_algo::AutoBiasController;
using gui_algo::FreqDetector;
using gui_algo::SensorSelfTest;

static std::vector<Event> make_events(int w, int h, int count, int t0 = 0) {
    std::vector<Event> ev;
    ev.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const uint16_t x = static_cast<uint16_t>(i % w);
        const uint16_t y = static_cast<uint16_t>((i / w) % h);
        ev.emplace_back(x, y, i & 1, t0 + i * 100);
    }
    return ev;
}

static EventPacket make_packet(const std::vector<Event>& v) {
    return EventPacket(v.data(), v.size());
}

// =========================================================================
// Phase 8: algo/cv/ §4.3.13–4.3.23
// =========================================================================

// --- 4.3.13 LineSegmentDetector ---
TEST(LineSegmentDetectorTest, Construction) {
    LineSegmentDetector d(64, 48);
    (void)d;
    SUCCEED();
}
TEST(LineSegmentDetectorTest, Params) {
    LineSegmentDetector d(32, 32);
    d.set_min_line_length_px(50);
    EXPECT_EQ(d.min_line_length_px(), 50);
    d.set_max_line_gap_px(10);
    EXPECT_EQ(d.max_line_gap_px(), 10);
}
TEST(LineSegmentDetectorTest, ProcessEmpty) {
    LineSegmentDetector d(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    auto result = d.process(pkt);
    EXPECT_TRUE(result.empty());
}
TEST(LineSegmentDetectorTest, ElisedParams) {
    LineSegmentDetector d(32, 32);
    d.set_max_age_us(50000);
    EXPECT_EQ(d.max_age_us(), 50000);
    d.set_num_orientations(8);
    EXPECT_EQ(d.num_orientations(), 8);
}
// ELiSeD port: a horizontal line of ON events at y=16 with temporal contrast
// supplied by neighbouring rows (y=15 older, y=17 newer) must produce a
// roughly horizontal segment whose length meets the minimum threshold.
TEST(LineSegmentDetectorTest, DetectsHorizontalLine) {
    LineSegmentDetector d(48, 48);
    d.set_min_line_length_px(10);
    std::vector<Event> ev;
    // Pre-fill rows 15 (older) and 17 (newer) for timestamp-contrast.
    for (int x = 4; x <= 43; ++x) {
        ev.emplace_back(static_cast<uint16_t>(x), 15, 1, 1000);
        ev.emplace_back(static_cast<uint16_t>(x), 17, 1, 5000);
    }
    // Line row, emitted last so neighbours are already populated.
    for (int x = 4; x <= 43; ++x) {
        ev.emplace_back(static_cast<uint16_t>(x), 16, 1, 3000);
    }
    auto pkt = make_packet(ev);
    auto result = d.process(pkt);
    ASSERT_GE(result.size(), 1u);
    // Segment should be roughly horizontal (angle within [0,180) and near 0).
    EXPECT_GE(result[0].angle, 0.0f);
    EXPECT_LT(result[0].angle, 180.0f);
    const float dx = result[0].end.x - result[0].start.x;
    const float dy = result[0].end.y - result[0].start.y;
    EXPECT_GT(dx * dx, dy * dy);  // horizontal extent dominates
    EXPECT_GE(result[0].track_id, 0);
}

// --- 4.3.14 HoughLineTracker ---
TEST(HoughLineTrackerTest, Construction) {
    HoughLineTracker t(64, 48);
    (void)t;
    SUCCEED();
}
TEST(HoughLineTrackerTest, Params) {
    HoughLineTracker t(32, 32);
    t.set_threshold(100);
    EXPECT_EQ(t.threshold(), 100);
    t.set_num_theta_bins(45);
    EXPECT_EQ(t.num_theta_bins(), 45);
    t.set_hough_decay_factor(0.5F);
    EXPECT_FLOAT_EQ(t.hough_decay_factor(), 0.5F);
}
TEST(HoughLineTrackerTest, ProcessEmpty) {
    HoughLineTracker t(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    auto result = t.process(pkt);
    EXPECT_TRUE(result.empty());
}

// --- 4.3.15 HoughCircleTracker ---
TEST(HoughCircleTrackerTest, Construction) {
    HoughCircleTracker t(64, 48);
    (void)t;
    SUCCEED();
}
TEST(HoughCircleTrackerTest, Params) {
    HoughCircleTracker t(32, 32);
    t.set_max_radius_px(100);
    EXPECT_EQ(t.max_radius_px(), 100);
    t.set_threshold(50);
    EXPECT_EQ(t.threshold(), 50);
}
TEST(HoughCircleTrackerTest, ProcessEmpty) {
    HoughCircleTracker t(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    auto result = t.process(pkt);
    EXPECT_TRUE(result.empty());
}

TEST(HoughCircleTrackerTest, SmallDtPacketsDoNotAmplifyAccumulator) {
    // Regression: the jAER decay formula 1/(0.0001*decay*dt) is > 1 for
    // dt < 10 ms. jAER never hits that branch (render-cycle packets), but
    // the GUI feeds SDK batches (~1-5 ms) — the accumulator was amplified
    // every packet until all cells saturated to the same value, and the
    // scan-order tie-break then reported a phantom circle at the bottom-
    // right interior cell, persisted by maxCoordinate. The factor is now
    // jAER-exact for dt >= T and exp((dt-T)/T) below T.
    HoughCircleTracker t(64, 48);
    t.set_max_radius_px(8);
    // Many small-dt packets, one event each at the center.
    for (int i = 0; i < 500; ++i) {
        std::vector<Event> ev;
        ev.emplace_back(32, 24, 1, 1000 + i * 1000);  // 1 ms apart
        auto pkt = make_packet(ev);
        t.accumulate_only(pkt);
    }
    const auto& accum = t.accum();
    const float mx = *std::max_element(accum.begin(), accum.end());
    ASSERT_TRUE(std::isfinite(mx));
    // No phantom detection at the bottom-right interior cell (62,46) —
    // the saturated-tiebreak artifact. (A legitimate detection near the
    // center is fine.)
    for (const auto& c : t.find_peaks()) {
        EXPECT_FALSE(std::abs(c.center.x - 62.0F) <= 1.0F &&
                     std::abs(c.center.y - 46.0F) <= 1.0F)
            << "phantom bottom-right circle detected";
    }
}

TEST(HoughCircleTrackerTest, DecayAppliesAtSmallPacketCadence) {
    // The earlier clamp-to-1 fix stalled decay below T (factor == 1 for
    // dt < 10 ms): votes never expired and every persistent structure
    // became a false-positive circle. The exp continuation must really
    // shrink the accumulator at small packet cadence.
    HoughCircleTracker t(64, 48);
    t.set_max_radius_px(8);
    std::vector<Event> ev;
    ev.emplace_back(32, 24, 1, 1000);
    auto pkt = make_packet(ev);
    t.accumulate_only(pkt);
    const float v0 = *std::max_element(t.accum().begin(), t.accum().end());
    ASSERT_GT(v0, 0.0f);
    for (int i = 0; i < 20; ++i) {  // +1 ms per empty packet
        std::vector<Event> empty;
        auto ep = make_packet(empty);
        t.accumulate_only(ep, 1000 + (i + 1) * 1000);
    }
    const float v1 = *std::max_element(t.accum().begin(), t.accum().end());
    EXPECT_LT(v1, v0 * 0.5f);
}

TEST(HoughCircleTrackerTest, DecayMatchesJaeRAtRenderCadence) {
    // jAER parity must be exact in jAER's operating range (dt >= T):
    // decay=1, dt=30 ms → factor 1/(0.0001*1*30000) = 1/3.
    HoughCircleTracker t(64, 48);
    t.set_max_radius_px(8);
    std::vector<Event> ev;
    ev.emplace_back(32, 24, 1, 1000);
    auto pkt = make_packet(ev);
    t.accumulate_only(pkt);
    const float v0 = *std::max_element(t.accum().begin(), t.accum().end());
    ASSERT_GT(v0, 0.0f);
    std::vector<Event> empty;
    auto ep = make_packet(empty);
    t.accumulate_only(ep, 31000);  // dt = 30000 us
    const float v1 = *std::max_element(t.accum().begin(), t.accum().end());
    EXPECT_NEAR(v1 / v0, 1.0f / 3.0f, 0.01f);
}

// P9 regression: the decay is CADENCE-INVARIANT — 30 x 1 ms packets must
// decay the accumulator by the same total factor as a single 30 ms packet
// (the former exp((dt-T)/T) small-dt branch wiped ~0.4 per 1 ms packet,
// 25x faster than jAER's cadence, killing live detection).
TEST(HoughCircleTrackerTest, DecayIsCadenceInvariant) {
    auto prime = [](HoughCircleTracker& t) {
        std::vector<Event> ev;
        ev.emplace_back(32, 24, 1, 1000);
        auto pkt = make_packet(ev);
        t.accumulate_only(pkt);
        return *std::max_element(t.accum().begin(), t.accum().end());
    };
    HoughCircleTracker a(64, 48), b(64, 48);
    a.set_max_radius_px(8);
    b.set_max_radius_px(8);
    const float v0a = prime(a);
    const float v0b = prime(b);
    ASSERT_GT(v0a, 0.0F);
    std::vector<Event> empty;
    auto ep = make_packet(empty);
    for (int i = 1; i <= 30; ++i) a.accumulate_only(ep, 1000 + i * 1000);  // 30x1ms
    b.accumulate_only(ep, 31000);                                           // 1x30ms
    const float va = *std::max_element(a.accum().begin(), a.accum().end());
    const float vb = *std::max_element(b.accum().begin(), b.accum().end());
    EXPECT_NEAR(va / v0a, vb / v0b, 0.01f);
    // And the total factor matches jAER's accumulated 1/3 per 30 ms.
    EXPECT_NEAR(va / v0a, 1.0F / 3.0F, 0.02F);
}

// --- 4.3.17 OrientationCluster ---
TEST(OrientationClusterTest, Construction) {
    OrientationCluster c(64, 48);
    (void)c;
    SUCCEED();
}
TEST(OrientationClusterTest, ProcessEmpty) {
    OrientationCluster c(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    auto result = c.process(pkt);
    EXPECT_TRUE(result.empty());
}

// --- 4.3.18 ClusterLIF ---
TEST(ClusterLIFTest, Construction) {
    ClusterLIF c(64, 48);
    (void)c;
    SUCCEED();
}
TEST(ClusterLIFTest, Params) {
    ClusterLIF c(32, 32);
    c.set_tau_ms(20.0f);
    EXPECT_FLOAT_EQ(c.tau_ms(), 20.0f);
    c.set_threshold(2.0f);
    EXPECT_FLOAT_EQ(c.threshold(), 2.0f);
}
TEST(ClusterLIFTest, ProcessEmpty) {
    ClusterLIF c(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    auto result = c.process(pkt);
    EXPECT_TRUE(result.empty());
}

// --- 4.3.19 BackgroundMaskFilter ---
TEST(BackgroundMaskFilterTest, Construction) {
    BackgroundMaskFilter f(64, 48);
    (void)f;
    SUCCEED();
}
TEST(BackgroundMaskFilterTest, Params) {
    BackgroundMaskFilter f(32, 32);
    f.set_learning_window_s(10.0f);
    EXPECT_FLOAT_EQ(f.learning_window_s(), 10.0f);
    f.set_background_rate_threshold_hz(5.0f);
    EXPECT_FLOAT_EQ(f.background_rate_threshold_hz(), 5.0f);
}
TEST(BackgroundMaskFilterTest, ProcessEmpty) {
    BackgroundMaskFilter f(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    const auto& mask = f.process(pkt);
    EXPECT_FALSE(mask.empty());
}


// --- 4.3.23 OpticalGyro ---
TEST(OpticalGyroTest, Construction) {
    OpticalGyro g(64, 48);
    auto m = g.total_motion();
    EXPECT_FLOAT_EQ(m.dx, 0.0f);
    EXPECT_FLOAT_EQ(m.dy, 0.0f);
}
TEST(OpticalGyroTest, Params) {
    OpticalGyro g(32, 32);
    g.set_stabilization_strength(0.5f);
    EXPECT_FLOAT_EQ(g.stabilization_strength(), 0.5f);
    g.set_smoothing_window_ms(200.0f);
    EXPECT_FLOAT_EQ(g.smoothing_window_ms(), 200.0f);
    // Rotation estimation toggle (jAER opticalGyroRotationEnabled default=false)
    EXPECT_FALSE(g.rotation_enabled());
    g.set_rotation_enabled(true);
    EXPECT_TRUE(g.rotation_enabled());
}

// =========================================================================
// Phase 9: algo/cv/ §4.3.24–4.3.27
// =========================================================================

// --- 4.3.25 XYTVisualizer ---
TEST(XYTVisualizerTest, Construction) {
    XYTVisualizer v;
    EXPECT_FLOAT_EQ(v.time_window_ms(), 50.0f);
}
TEST(XYTVisualizerTest, Params) {
    XYTVisualizer v;
    v.set_time_window_ms(500.0f);
    EXPECT_FLOAT_EQ(v.time_window_ms(), 500.0f);
    v.set_point_size(5.0f);
    EXPECT_FLOAT_EQ(v.point_size(), 5.0f);
}
TEST(XYTVisualizerTest, Process) {
    XYTVisualizer v;
    auto ev = make_events(32, 32, 50);
    v.process(ev.data(), ev.size());
    SUCCEED();
}

// --- 4.3.27 TimeSurface ---
TEST(TimeSurfaceTest, Construction) {
    TimeSurface ts(64, 48);
    EXPECT_EQ(ts.width(), 64);
    EXPECT_EQ(ts.height(), 48);
}
TEST(TimeSurfaceTest, Params) {
    TimeSurface ts(32, 32);
    ts.set_decay_time_us(200000);
    EXPECT_EQ(ts.decay_time_us(), 200000);
    ts.set_refresh_rate_hz(60);
    EXPECT_EQ(ts.refresh_rate_hz(), 60);
}
TEST(TimeSurfaceTest, ProcessAndRender) {
    TimeSurface ts(32, 32);
    auto ev = make_events(32, 32, 50);
    ts.process(ev.data(), ev.size());
    cv::Mat img = ts.render();
    EXPECT_FALSE(img.empty());
    EXPECT_EQ(img.rows, 32);
    EXPECT_EQ(img.cols, 32);
}
TEST(TimeSurfaceTest, ExponentialDecay) {
    // dv Accumulator EXPONENTIAL (accumulator.hpp:119-154): per-event
    // decay + contribute. Defaults: eventContribution=0.15, neutral=0,
    // [min,max]=[0,1]. A pixel hit once at t=0, rendered tau later:
    //   display = 0.15 * exp(-1) ≈ 0.0552 → gray ≈ 14.
    TimeSurface ts(32, 32, TimeSurface::Channels::Merged, 100000,
                   TimeSurface::Palette::Gray, 30,
                   TimeSurface::Decay::Exponential, 100000);
    EXPECT_EQ(ts.decay(), TimeSurface::Decay::Exponential);
    EXPECT_EQ(ts.tau_us(), 100000);
    std::vector<Event> ev;
    ev.emplace_back(5, 5, 1, 0);         // reference pixel at t=0
    ev.emplace_back(10, 10, 1, 100000);  // advances current_t_ by exactly tau
    ts.process(ev.data(), ev.size());
    cv::Mat img = ts.render();
    const cv::Vec3b old_px = img.at<cv::Vec3b>(5, 5);
    const cv::Vec3b new_px = img.at<cv::Vec3b>(10, 10);
    // pot=0.15; display = 0.15 * exp(-tau/tau) = 0.15*exp(-1) ≈ 14.1
    const double expect_old = 255.0 * 0.15 * std::exp(-1.0);
    EXPECT_NEAR(old_px[0], expect_old, 2.0);
    EXPECT_EQ(old_px[0], old_px[1]);  // Gray palette: all channels equal
    EXPECT_EQ(old_px[1], old_px[2]);
    // pot=0.15; dt=0 → display = 0.15 → gray ≈ 38
    const double expect_new = 255.0 * 0.15;
    EXPECT_NEAR(new_px[0], expect_new, 2.0);
    EXPECT_EQ(img.at<cv::Vec3b>(0, 0)[0], 0);  // never-hit pixel stays black
    // tau_us setter round-trip.
    ts.set_tau_us(50000);
    EXPECT_EQ(ts.tau_us(), 50000);
}

TEST(TimeSurfaceTest, ExponentialAccumulation) {
    // dv Accumulator: multiple events at the same pixel accumulate
    // (contribute), saturating to max_potential (1.0). With
    // eventContribution=0.15 and tau=1s, 10 events 100us apart accumulate
    // well past 1.0 → clamped to 1.0 → gray 255 (dt≈0 at render).
    TimeSurface ts(32, 32, TimeSurface::Channels::Merged, 100000,
                   TimeSurface::Palette::Gray, 30,
                   TimeSurface::Decay::Exponential, 1000000);
    std::vector<Event> ev;
    for (int i = 0; i < 10; ++i)
        ev.emplace_back(5, 5, 1, i * 100);  // 100us apart, << tau=1s
    ev.emplace_back(10, 10, 1, 1000);       // single event, advances current_t_
    ts.process(ev.data(), ev.size());
    cv::Mat img = ts.render();
    // 10 events saturate to 1.0; dt=100us → exp(-0.0001)≈1.0 → gray 255.
    EXPECT_EQ(img.at<cv::Vec3b>(5, 5)[0], 255);
    // Single-event pixel: pot=0.15, dt=0 → gray ≈ 38.
    EXPECT_NEAR(img.at<cv::Vec3b>(10, 10)[0], 255.0 * 0.15, 2.0);
}
TEST(TimeSurfaceTest, LinearDecayUnchangedByDefault) {
    // Default decay stays Linear: hard cut to 0 at the window tail.
    TimeSurface ts(32, 32, TimeSurface::Channels::Merged, 100000,
                   TimeSurface::Palette::Gray);
    EXPECT_EQ(ts.decay(), TimeSurface::Decay::Linear);
    std::vector<Event> ev;
    ev.emplace_back(5, 5, 1, 0);
    ev.emplace_back(10, 10, 1, 100000);  // dt == decay_time_us -> cut to 0
    ts.process(ev.data(), ev.size());
    cv::Mat img = ts.render();
    EXPECT_EQ(img.at<cv::Vec3b>(5, 5)[0], 0);
    EXPECT_EQ(img.at<cv::Vec3b>(10, 10)[0], 255);
}

// =========================================================================
// Phase 10: algo/analytics/ §4.4.1–4.4.7
// =========================================================================

// --- 4.4.2 EventToVideo ---
TEST(EventToVideoTest, Construction) {
    EventToVideo v(64, 48);
    EXPECT_EQ(v.width(), 64);
    EXPECT_EQ(v.height(), 48);
}
TEST(EventToVideoTest, ModeSwitching) {
    EventToVideo v(32, 32);
    v.set_mode(EventToVideo::Mode::InteractingMaps);
    EXPECT_EQ(v.mode(), EventToVideo::Mode::InteractingMaps);
    v.set_mode(EventToVideo::Mode::E2VID);
    EXPECT_EQ(v.mode(), EventToVideo::Mode::E2VID);
}
TEST(EventToVideoTest, ProcessAndGetFrame) {
    EventToVideo v(32, 32, EventToVideo::Mode::BardowVariational);
    auto ev = make_events(32, 32, 100);
    v.process(ev.data(), ev.size());
    cv::Mat frame = v.get_frame();
    EXPECT_FALSE(frame.empty());
}

// Diagnostic: BardowVariational with downsample should produce non-flat output.
TEST(EventToVideoTest, BardowVariationalNotFlat) {
    EventToVideo v(128, 128, EventToVideo::Mode::BardowVariational);
    v.set_downsample(true);
    auto ev = make_events(128, 128, 500);
    v.process(ev.data(), ev.size());
    cv::Mat frame = v.get_frame();
    EXPECT_FALSE(frame.empty());
    EXPECT_EQ(frame.rows, 128);
    EXPECT_EQ(frame.cols, 128);
    // Check that the frame is not uniformly gray (128).
    double min_val, max_val;
    cv::minMaxLoc(frame, &min_val, &max_val);
    EXPECT_LT(min_val, 100.0) << "min=" << min_val << " max=" << max_val;
    EXPECT_GT(max_val, 150.0) << "min=" << min_val << " max=" << max_val;
}

// Diagnostic: BardowVariational without downsample should produce non-flat output.
TEST(EventToVideoTest, BardowVariationalNotFlatNoDownsample) {
    EventToVideo v(128, 128, EventToVideo::Mode::BardowVariational);
    v.set_downsample(false);
    auto ev = make_events(128, 128, 500);
    v.process(ev.data(), ev.size());
    cv::Mat frame = v.get_frame();
    EXPECT_FALSE(frame.empty());
    double min_val, max_val;
    cv::minMaxLoc(frame, &min_val, &max_val);
    EXPECT_LT(min_val, 100.0) << "min=" << min_val << " max=" << max_val;
    EXPECT_GT(max_val, 150.0) << "min=" << min_val << " max=" << max_val;
}

// Diagnostic: InteractingMaps should produce non-flat output (warm-start fix).
TEST(EventToVideoTest, InteractingMapsNotFlat) {
    EventToVideo v(128, 128, EventToVideo::Mode::InteractingMaps);
    v.set_downsample(true);
    auto ev = make_events(128, 128, 500);
    v.process(ev.data(), ev.size());
    cv::Mat frame = v.get_frame();
    EXPECT_FALSE(frame.empty());
    EXPECT_EQ(frame.rows, 128);
    EXPECT_EQ(frame.cols, 128);
    double min_val, max_val;
    cv::minMaxLoc(frame, &min_val, &max_val);
    EXPECT_LT(min_val, 100.0) << "min=" << min_val << " max=" << max_val;
    EXPECT_GT(max_val, 150.0) << "min=" << min_val << " max=" << max_val;
}
TEST(EventToVideoTest, E2VIDModeHeuristic) {
    // E2VID without model -> heuristic fallback (always available).
    EventToVideo v(32, 32, EventToVideo::Mode::E2VID);
    EXPECT_FALSE(v.e2vid_model_loaded());
    auto ev = make_events(32, 32, 200);
    v.process(ev.data(), ev.size());
    cv::Mat frame = v.get_frame();
    EXPECT_FALSE(frame.empty());
    EXPECT_EQ(frame.type(), CV_8UC1);
    EXPECT_EQ(frame.rows, 32);
    EXPECT_EQ(frame.cols, 32);
}
TEST(EventToVideoTest, E2VIDParams) {
    EventToVideo v(32, 32, EventToVideo::Mode::E2VID);
    v.set_e2vid_num_bins(10);
    EXPECT_EQ(v.e2vid_num_bins(), 10);
    v.set_e2vid_auto_hdr(true);
    EXPECT_TRUE(v.e2vid_auto_hdr());
    v.set_unsharp_amount(0.5f);
    EXPECT_FLOAT_EQ(v.unsharp_amount(), 0.5f);
    v.set_unsharp_sigma(2.0f);
    EXPECT_FLOAT_EQ(v.unsharp_sigma(), 2.0f);
    v.set_bilateral_sigma(1.0f);
    EXPECT_FLOAT_EQ(v.bilateral_sigma(), 1.0f);
}
TEST(EventToVideoTest, E2VIDModelLoadFailure) {
    // Loading a nonexistent model path should fail gracefully.
    EventToVideo v(32, 32, EventToVideo::Mode::E2VID);
    v.set_model_path("/nonexistent/model.onnx");
    EXPECT_FALSE(v.e2vid_model_loaded());
}

// --- E2VID submodule tests ---
TEST(EventVoxelGridTest, Construction) {
    gui_algo::EventVoxelGrid g(64, 48, 5);
    EXPECT_EQ(g.width(), 64);
    EXPECT_EQ(g.height(), 48);
    EXPECT_EQ(g.num_bins(), 5);
    EXPECT_EQ(g.size(), static_cast<std::size_t>(5 * 64 * 48));
}
TEST(EventVoxelGridTest, BuildAndNormalize) {
    gui_algo::EventVoxelGrid g(32, 32, 5);
    auto ev = make_events(32, 32, 100);
    const auto& grid = g.build(ev.data(), ev.size());
    EXPECT_EQ(grid.size(), static_cast<std::size_t>(5 * 32 * 32));
    g.normalize();  // should not crash
    SUCCEED();
}
TEST(EventVoxelGridTest, RenderPreview) {
    gui_algo::EventVoxelGrid g(32, 32, 5);
    auto ev = make_events(32, 32, 50);
    g.build(ev.data(), ev.size());
    cv::Mat preview = g.render_preview();
    EXPECT_EQ(preview.type(), CV_8UC3);
    EXPECT_EQ(preview.rows, 32);
    EXPECT_EQ(preview.cols, 32);
}
TEST(IntensityRescalerTest, Construction) {
    gui_algo::IntensityRescaler r;
    EXPECT_FALSE(r.auto_hdr());
    EXPECT_FLOAT_EQ(r.imin(), 0.0f);
    EXPECT_FLOAT_EQ(r.imax(), 1.0f);
}
TEST(IntensityRescalerTest, AutoHDR) {
    gui_algo::IntensityRescaler r(true, 5);
    EXPECT_TRUE(r.auto_hdr());
    cv::Mat img(32, 32, CV_32FC1, cv::Scalar(0.5));
    cv::Mat out = r(img);
    EXPECT_EQ(out.type(), CV_8UC1);
    EXPECT_EQ(out.rows, 32);
}
TEST(IntensityRescalerTest, ResetClearsBounds) {
    // NIT 2 regression: reset() should clear imin_/imax_ to defaults.
    gui_algo::IntensityRescaler r(true, 3);
    cv::Mat img(32, 32, CV_32FC1, cv::Scalar(0.5));
    r(img);
    r.reset();
    EXPECT_FLOAT_EQ(r.imin(), 0.0f);
    EXPECT_FLOAT_EQ(r.imax(), 1.0f);
}
TEST(UnsharpMaskTest, Construction) {
    gui_algo::UnsharpMaskFilter f(0.3f, 1.0f);
    EXPECT_FLOAT_EQ(f.amount(), 0.3f);
    EXPECT_FLOAT_EQ(f.sigma(), 1.0f);
}
TEST(UnsharpMaskTest, Apply) {
    gui_algo::UnsharpMaskFilter f(0.3f, 1.0f);
    cv::Mat img(32, 32, CV_32FC1, cv::Scalar(0.5));
    cv::Mat out = f(img);
    EXPECT_EQ(out.type(), CV_32FC1);
    EXPECT_EQ(out.rows, 32);
}
TEST(BilateralFilterTest, Apply) {
    gui_algo::BilateralImageFilter f(1.0f);
    cv::Mat img(32, 32, CV_8UC1, cv::Scalar(128));
    cv::Mat out = f(img);
    EXPECT_EQ(out.type(), CV_8UC1);
    EXPECT_EQ(out.rows, 32);
}
TEST(E2VIDInferenceTest, Construction) {
    gui_algo::E2VIDInference e(64, 48, 5);
    EXPECT_EQ(e.width(), 64);
    EXPECT_EQ(e.height(), 48);
    EXPECT_EQ(e.num_bins(), 5);
    EXPECT_FALSE(e.is_model_loaded());
}
TEST(E2VIDInferenceTest, HeuristicInference) {
    gui_algo::E2VIDInference e(32, 32, 5);
    auto ev = make_events(32, 32, 200);
    cv::Mat frame = e.infer(ev.data(), ev.size());
    EXPECT_FALSE(frame.empty());
    EXPECT_EQ(frame.type(), CV_8UC1);
    EXPECT_EQ(frame.rows, 32);
    EXPECT_EQ(frame.cols, 32);
}
TEST(E2VIDInferenceTest, ModelLoadFailure) {
    gui_algo::E2VIDInference e(32, 32, 5);
    EXPECT_FALSE(e.load_model("/nonexistent/model.onnx"));
    EXPECT_FALSE(e.is_model_loaded());
}
TEST(E2VIDInferenceTest, NumBinsClamp) {
    // BUG 1 regression: num_bins must be clamped to [1, 20].
    gui_algo::E2VIDInference e(32, 32, 100);
    EXPECT_EQ(e.num_bins(), 20);
    e.set_num_bins(0);
    EXPECT_EQ(e.num_bins(), 1);
    e.set_num_bins(10);
    EXPECT_EQ(e.num_bins(), 10);
}
TEST(E2VIDInferenceTest, HotPixelMaskPreservedAcrossNumBins) {
    // BUG 7 regression: set_num_bins must not drop the hot-pixel mask.
    gui_algo::E2VIDInference e(32, 32, 5);
    std::vector<std::uint8_t> mask(32 * 32, 1);
    mask[0] = 0;  // mark (0,0) as hot
    e.set_hot_pixel_mask(mask);
    e.set_num_bins(10);
    // After rebuilding the voxel grid, the mask should still be active.
    // Verify by checking that infer still works (no crash).
    auto ev = make_events(32, 32, 100);
    cv::Mat frame = e.infer(ev.data(), ev.size());
    EXPECT_FALSE(frame.empty());
}
TEST(E2VIDInferenceTest, CropToSensor) {
    // BUG 2 regression: crop_to_sensor should be a no-op for sensor-sized images.
    gui_algo::E2VIDInference e(32, 32, 5);
    cv::Mat sensor_sized(32, 32, CV_8UC1, cv::Scalar(128));
    cv::Mat out1 = e.crop_to_sensor(sensor_sized);
    EXPECT_EQ(out1.rows, 32);
    EXPECT_EQ(out1.cols, 32);
    // A larger image should be cropped back.
    cv::Mat padded(64, 64, CV_8UC1, cv::Scalar(200));
    cv::Mat out2 = e.crop_to_sensor(padded);
    EXPECT_EQ(out2.rows, 32);
    EXPECT_EQ(out2.cols, 32);
}

// --- 4.4.6 AutoBiasController (dual loop: rate band + ON/OFF balance) ---
namespace {
// Feed a constant-rate stream for the given duration; returns the CORRECTION
// commands emitted along the way (home commands excluded — see the homing
// tests below).
std::vector<gui_algo::BiasCommand> feed(AutoBiasController& c,
                                        double on_mev, double off_mev,
                                        std::int64_t duration_us) {
    std::vector<gui_algo::BiasCommand> cmds;
    const std::int64_t step = 1000;  // 1 ms batches
    const std::uint32_t n_on = static_cast<std::uint32_t>(on_mev * step);
    const std::uint32_t n_off = static_cast<std::uint32_t>(off_mev * step);
    std::int64_t t = 0;
    while (t < duration_us) {
        t += step;
        c.accumulate(t, n_on, n_off);
        const auto cmd = c.update(t);
        if (cmd.active && !cmd.home) cmds.push_back(cmd);
    }
    return cmds;
}
} // namespace

TEST(AutoBiasControllerTest, RateBandValidation) {
    AutoBiasController c;  // default band 1..50
    EXPECT_FLOAT_EQ(c.rate_min_mev(), 1.0f);
    EXPECT_FLOAT_EQ(c.rate_max_mev(), 50.0f);
    EXPECT_EQ(c.max_step(), 32);  // fast-convergence default
    EXPECT_FALSE(c.set_rate_bounds(5.0f, 5.0f));   // lo == hi rejected
    EXPECT_FALSE(c.set_rate_bounds(8.0f, 4.0f));   // inverted rejected
    EXPECT_FLOAT_EQ(c.rate_min_mev(), 1.0f);       // previous band kept
    EXPECT_TRUE(c.set_rate_bounds(2.0f, 20.0f));
    EXPECT_FLOAT_EQ(c.rate_min_mev(), 2.0f);
    EXPECT_FLOAT_EQ(c.rate_max_mev(), 20.0f);
    EXPECT_TRUE(c.set_rate_bounds(0.0f, 5000.0f)); // no artificial ceiling
    EXPECT_FLOAT_EQ(c.rate_min_mev(), 0.05f);      // min floored, max kept
    EXPECT_FLOAT_EQ(c.rate_max_mev(), 5000.0f);
}
TEST(AutoBiasControllerTest, DeadbandIsIdle) {
    AutoBiasController c;  // band 1..50
    const auto cmds = feed(c, 3.0, 3.0, 1'000'000);  // in band, balanced
    EXPECT_TRUE(cmds.empty());
    EXPECT_NEAR(c.rate_on_mev(), 3.0, 0.2);
    EXPECT_NEAR(c.rate_off_mev(), 3.0, 0.2);
}
TEST(AutoBiasControllerTest, HomingWhenSatisfied) {
    AutoBiasController c;  // band 1..50
    // In band + balanced → home commands, no correction deltas, spaced by
    // kHomeIntervalTicks (~1 s at the 30 Hz tick; the test ticks every ms,
    // so ~30 updates apart).
    int home_cmds = 0;
    int last_home_at = -1000;
    const std::int64_t step = 1000;
    for (std::int64_t t = step; t <= 2'000'000; t += step) {
        c.accumulate(t, 3000, 3000);
        const auto cmd = c.update(t);
        if (cmd.home) {
            EXPECT_TRUE(cmd.active);
            EXPECT_EQ(cmd.delta_on, 0);
            EXPECT_EQ(cmd.delta_off, 0);
            ++home_cmds;
            EXPECT_GT(static_cast<int>(t / step) - last_home_at, 25);
            last_home_at = static_cast<int>(t / step);
        }
    }
    EXPECT_GE(home_cmds, 50);  // ~1 per second over 2 s
}
TEST(AutoBiasControllerTest, HomingSuspendsOnViolation) {
    AutoBiasController c;  // band 1..50
    // In-band phase homes; the out-of-band phase must NOT home once the
    // measurement window reflects the new rate (during the first window
    // span after the transition the window still holds in-band samples —
    // a home command there is measurement lag, not a logic error).
    bool homed_after_settled = false;
    const std::int64_t step = 1000;
    for (std::int64_t t = step; t <= 3'000'000; t += step) {
        const bool violating = t > 1'000'000;
        c.accumulate(t, violating ? 60000 : 3000, violating ? 60000 : 3000);
        const auto cmd = c.update(t);
        if (violating && t > 1'500'000 && cmd.home) homed_after_settled = true;
    }
    EXPECT_FALSE(homed_after_settled);
}
TEST(AutoBiasControllerTest, HighRateRaisesBoth) {
    AutoBiasController c;  // band 1..50
    const auto cmds = feed(c, 40.0, 40.0, 1'000'000);  // 80 Mev/s >> max
    ASSERT_FALSE(cmds.empty());
    for (const auto& cmd : cmds) {
        EXPECT_GT(cmd.delta_on, 0);   // raise both = less sensitive
        EXPECT_GT(cmd.delta_off, 0);
    }
}
TEST(AutoBiasControllerTest, LowRateLowersBoth) {
    AutoBiasController c;  // band 1..10
    const auto cmds = feed(c, 0.05, 0.05, 1'000'000);  // 0.1 Mev/s < min
    ASSERT_FALSE(cmds.empty());
    for (const auto& cmd : cmds) {
        EXPECT_LT(cmd.delta_on, 0);   // lower both = more sensitive
        EXPECT_LT(cmd.delta_off, 0);
    }
}
TEST(AutoBiasControllerTest, BalanceRaisesOnlyDominantPolarity) {
    AutoBiasController c;  // band 1..10
    // Total 6 Mev/s (in band), heavily ON-skewed: b = 0.67 > tol.
    const auto cmds = feed(c, 5.0, 1.0, 1'000'000);
    ASSERT_FALSE(cmds.empty());
    for (const auto& cmd : cmds) {
        EXPECT_GT(cmd.delta_on, 0);    // gate the dominant ON side
        EXPECT_EQ(cmd.delta_off, 0);   // leave the OFF side untouched
    }
}
TEST(AutoBiasControllerTest, HoldAfterAction) {
    AutoBiasController c;  // band 1..50
    const auto cmds = feed(c, 40.0, 40.0, 600'000);  // 80 Mev/s >> max
    ASSERT_GE(cmds.size(), 1u);
    // Each action flushes the measurement window (all pre-action samples
    // are stale), so consecutive commands are spaced by the 200 ms refill
    // at minimum — over 600 ms at most ~2 actions, never back-to-back.
    EXPECT_LE(cmds.size(), 3u);
}
TEST(AutoBiasControllerTest, IntegerDeltasAndStepCap) {
    AutoBiasController c(1.0f, 10.0f, 4);  // max_step 4
    const auto cmds = feed(c, 50.0, 50.0, 500'000);
    ASSERT_FALSE(cmds.empty());
    for (const auto& cmd : cmds) {
        EXPECT_LE(cmd.delta_on, 4);
        EXPECT_LE(cmd.delta_off, 4);
    }
}
TEST(AutoBiasControllerTest, ColdWindowIsIdle) {
    AutoBiasController c;
    c.accumulate(1000, 100000, 100000);  // single batch, window not ready
    EXPECT_FALSE(c.window_ready());
    EXPECT_FALSE(c.update(1000).active);
}
TEST(AutoBiasControllerTest, Reset) {
    AutoBiasController c;
    feed(c, 20.0, 20.0, 400'000);
    c.reset();
    EXPECT_TRUE(c.rate_on_mev() == 0.0 && c.rate_off_mev() == 0.0);
    EXPECT_FALSE(c.window_ready());
}

// --- 4.4.7 FreqDetector ---
TEST(FreqDetectorTest, Construction) {
    FreqDetector d(64, 48);
    EXPECT_EQ(d.width(), 64);
    EXPECT_EQ(d.height(), 48);
}
TEST(FreqDetectorTest, Params) {
    FreqDetector d(32, 32);
    d.set_f_min(200.0f);
    EXPECT_FLOAT_EQ(d.f_min(), 200.0f);
    d.set_f_max(5000.0f);
    EXPECT_FLOAT_EQ(d.f_max(), 5000.0f);
    d.set_heatmap_threshold(30);
    EXPECT_EQ(d.heatmap_threshold(), 30);
}
TEST(FreqDetectorTest, AnalyzeEmpty) {
    FreqDetector d(32, 32);
    auto sources = d.analyze();
    EXPECT_TRUE(sources.empty());
}
TEST(FreqDetectorTest, ProcessAndAnalyze) {
    FreqDetector d(32, 32);
    auto ev = make_events(32, 32, 100);
    d.process(ev.data(), ev.size());
    auto sources = d.analyze();
    // Still in the initialization phase -> no results yet.
    EXPECT_TRUE(sources.empty());
}

// Blinking LED covering the 3x3 block around (16, 16) (a real LED spans
// several pixels, and clusters below min_cc_area=3 are skipped): one event
// every 5000 us -> event frequency 200 Hz, physical blink frequency 100 Hz.
static std::vector<Event> make_blinking_led(std::size_t count,
                                            Metavision::timestamp dt_us = 5000,
                                            Metavision::timestamp t0 = 0) {
    std::vector<Event> ev;
    ev.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const uint16_t x = static_cast<uint16_t>(15 + i % 3);
        const uint16_t y = static_cast<uint16_t>(15 + (i / 3) % 3);
        ev.emplace_back(x, y, static_cast<uint8_t>(i & 1),
                        t0 + static_cast<Metavision::timestamp>(i) * dt_us);
    }
    return ev;
}

TEST(FreqDetectorTest, InitPhaseGatesAnalyze) {
    FreqDetector d(32, 32);
    // 3 s of stream (600 events at 5 ms): below first_analysis_s (5 s).
    auto ev = make_blinking_led(600, 5000, 0);
    d.process(ev.data(), ev.size());
    EXPECT_EQ(d.phase(), FreqDetector::Phase::Initializing);
    EXPECT_GT(d.init_remaining_s(), 0.0f);
    EXPECT_TRUE(d.analyze().empty());
    // Cross the warm-up threshold (5 s).
    auto ev2 = make_blinking_led(500, 5000, 3000000);
    d.process(ev2.data(), ev2.size());
    EXPECT_EQ(d.phase(), FreqDetector::Phase::Running);
    EXPECT_FLOAT_EQ(d.init_remaining_s(), 0.0f);
    auto sources = d.analyze();
    ASSERT_EQ(sources.size(), 1u);
    // Bounding box covers the blinking pixel.
    EXPECT_LE(sources[0].u0, 16);
    EXPECT_LE(sources[0].v0, 16);
    EXPECT_GT(sources[0].u0 + sources[0].w, 16);
    EXPECT_GT(sources[0].v0 + sources[0].h, 16);
    // Event frequency 200 Hz (blink 100 Hz), within the DFT resolution.
    EXPECT_NEAR(sources[0].event_freq_hz, 200.0f, 10.0f);
    EXPECT_NEAR(sources[0].blink_freq_hz, 100.0f, 5.0f);
}

TEST(FreqDetectorTest, FreezesAfterMaxDuration) {
    FreqDetector d(32, 32);
    // Run past max_duration_s (20 s): the last computed result is frozen.
    auto ev = make_blinking_led(4300, 5000, 0);  // spans 0..21.5 s
    d.process(ev.data(), ev.size());
    ASSERT_EQ(d.phase(), FreqDetector::Phase::Running);
    auto frozen = d.analyze();
    ASSERT_EQ(frozen.size(), 1u);
    // Feed a different pattern after the freeze: the result must not change.
    std::vector<Event> other;
    for (int i = 0; i < 500; ++i) {
        other.emplace_back(2, 2, 1, 22000000 + i * 2500);
    }
    d.process(other.data(), other.size());
    auto after = d.analyze();
    ASSERT_EQ(after.size(), frozen.size());
    EXPECT_EQ(after[0].u0, frozen[0].u0);
    EXPECT_NEAR(after[0].blink_freq_hz, frozen[0].blink_freq_hz, 1e-3f);
}

TEST(FreqDetectorTest, MinTotalEventsGate) {
    FreqDetector d(32, 32);
    // 6 s of stream but only 50 events: below the 100-event guard.
    auto ev = make_blinking_led(50, 120000, 0);
    d.process(ev.data(), ev.size());
    EXPECT_EQ(d.phase(), FreqDetector::Phase::Initializing);
    EXPECT_TRUE(d.analyze().empty());
}

TEST(FreqDetectorTest, BufferCapEvictsOldestAndUndoesHeatmap) {
    FreqDetector d(32, 32);
    d.set_first_analysis_s(0.5f);   // min clamp
    d.set_heatmap_threshold(1);
    d.set_min_cc_area(3);
    // Tiny cap: at most 150 of the 400 events below are retained.
    d.set_max_buffer_events(150);
    // LED A around (5..7): 200 events over 0..1 s, then LED B around
    // (20..22): 200 events over 1..2 s. The cap keeps only the tail -> all
    // of A is evicted (its heatmap counts must be undone) and only B forms
    // a cluster.
    std::vector<Event> ev;
    for (int i = 0; i < 200; ++i)
        ev.emplace_back(5 + i % 3, 5 + (i / 3) % 3, static_cast<uint8_t>(i & 1),
                        static_cast<Metavision::timestamp>(i * 5000));
    for (int i = 0; i < 200; ++i)
        ev.emplace_back(20 + i % 3, 20 + (i / 3) % 3, static_cast<uint8_t>(i & 1),
                        static_cast<Metavision::timestamp>(1000000 + i * 5000));
    d.process(ev.data(), ev.size());
    ASSERT_EQ(d.phase(), FreqDetector::Phase::Running);
    auto sources = d.analyze();
    ASSERT_EQ(sources.size(), 1u);
    EXPECT_GE(sources[0].u0, 19);
    EXPECT_LE(sources[0].u0, 23);
}

TEST(FreqDetectorTest, ResetRestartsInitPhase) {
    FreqDetector d(32, 32);
    auto ev = make_blinking_led(1200, 5000, 0);  // 6 s > 5 s warm-up
    d.process(ev.data(), ev.size());
    ASSERT_EQ(d.phase(), FreqDetector::Phase::Running);
    d.reset();
    EXPECT_EQ(d.phase(), FreqDetector::Phase::Initializing);
    EXPECT_TRUE(d.analyze().empty());
}

// --- 4.4.8 SensorSelfTest ---

TEST(SensorSelfTestTest, Construction) {
    SensorSelfTest s(64, 48);
    EXPECT_EQ(s.width(), 64);
    EXPECT_EQ(s.height(), 48);
}

TEST(SensorSelfTestTest, NoEventsAllBadPixels) {
    // With no events fed, every pixel is a suspected bad pixel.
    SensorSelfTest s(8, 4);
    auto stats = s.compute_stats();
    EXPECT_EQ(stats.total_pixels, 32u);
    EXPECT_EQ(stats.triggered_pixels, 0u);
    EXPECT_EQ(stats.measured_pixels, 0u);
    EXPECT_EQ(stats.bad_pixels, 32u);
    auto coords = s.bad_pixel_coords();
    EXPECT_EQ(coords.size(), 32u);
}

TEST(SensorSelfTestTest, SingleEventNoInterval) {
    // A pixel with only one event has no interval (measured_pixels == 0).
    SensorSelfTest s(4, 4);
    Event ev[1] = {{2, 2, 1, 1000}};
    s.process(ev, 1);
    auto stats = s.compute_stats();
    EXPECT_EQ(stats.triggered_pixels, 1u);
    EXPECT_EQ(stats.measured_pixels, 0u);
    EXPECT_EQ(stats.bad_pixels, 15u);
}

TEST(SensorSelfTestTest, MinIntervalTracked) {
    // Feed three events at the same pixel with intervals 500us and 200us.
    // The per-pixel min interval should be 200us (the shorter of the two).
    // Stats operate on per-pixel minimums, so min=max=mean=200 for one pixel.
    SensorSelfTest s(4, 4);
    Event ev[3] = {{2, 2, 1, 1000}, {2, 2, 1, 1500}, {2, 2, 1, 1700}};
    s.process(ev, 3);
    auto stats = s.compute_stats();
    EXPECT_EQ(stats.triggered_pixels, 1u);
    EXPECT_EQ(stats.measured_pixels, 1u);
    EXPECT_EQ(stats.min_us, 200);
    EXPECT_EQ(stats.max_us, 200);
    EXPECT_EQ(stats.mean_us, 200.0);
}

TEST(SensorSelfTestTest, MinIntervalUpdatedOnShorter) {
    // First interval = 1000us, then 500us → min should be 500us.
    SensorSelfTest s(4, 4);
    Event ev1[2] = {{0, 0, 1, 0}, {0, 0, 1, 1000}};
    s.process(ev1, 2);
    Event ev2[2] = {{0, 0, 1, 2000}, {0, 0, 1, 2500}};
    s.process(ev2, 2);
    auto stats = s.compute_stats();
    EXPECT_EQ(stats.min_us, 500);
    EXPECT_EQ(stats.measured_pixels, 1u);
}

TEST(SensorSelfTestTest, OutOfBoundsEventsIgnored) {
    SensorSelfTest s(4, 4);
    Event ev[2] = {{10, 10, 1, 100}, {3, 3, 1, 200}};
    s.process(ev, 2);
    auto stats = s.compute_stats();
    EXPECT_EQ(stats.triggered_pixels, 1u);  // only (3,3) is in bounds
}

TEST(SensorSelfTestTest, ResetClearsState) {
    SensorSelfTest s(4, 4);
    Event ev[2] = {{0, 0, 1, 0}, {0, 0, 1, 500}};
    s.process(ev, 2);
    EXPECT_EQ(s.compute_stats().triggered_pixels, 1u);
    s.reset();
    auto stats = s.compute_stats();
    EXPECT_EQ(stats.triggered_pixels, 0u);
    EXPECT_EQ(stats.bad_pixels, 16u);
}

TEST(SensorSelfTestTest, RenderProducesCorrectSize) {
    SensorSelfTest s(16, 8);
    cv::Mat img = s.render();
    ASSERT_FALSE(img.empty());
    EXPECT_EQ(img.cols, 16);
    EXPECT_EQ(img.rows, 8);
    EXPECT_EQ(img.type(), CV_8UC3);
}

TEST(SensorSelfTestTest, RenderBadPixelIsRed) {
    // With no events, all pixels should be red (BGR 0,0,255).
    SensorSelfTest s(4, 2);
    cv::Mat img = s.render();
    for (int y = 0; y < 2; ++y) {
        for (int x = 0; x < 4; ++x) {
            const auto& px = img.at<cv::Vec3b>(y, x);
            EXPECT_EQ(px[0], 0);    // B
            EXPECT_EQ(px[1], 0);    // G
            EXPECT_EQ(px[2], 255);  // R
        }
    }
}

TEST(SensorSelfTestTest, RenderTriggeredPixelIsGrayscale) {
    // A pixel with two events (interval=1us) should render bright (non-red,
    // non-black). Bad pixels remain red.
    SensorSelfTest s(4, 4);
    Event ev[2] = {{0, 0, 1, 100}, {0, 0, 1, 101}};
    s.process(ev, 2);
    cv::Mat img = s.render();
    const auto& triggered = img.at<cv::Vec3b>(0, 0);
    // Grayscale: R == G == B, and bright (interval=1us → ~255).
    EXPECT_EQ(triggered[0], triggered[1]);
    EXPECT_EQ(triggered[1], triggered[2]);
    EXPECT_GT(triggered[0], 200);
    // An untriggered pixel is still red.
    const auto& bad = img.at<cv::Vec3b>(1, 1);
    EXPECT_EQ(bad[2], 255);
    EXPECT_EQ(bad[0], 0);
}

TEST(SensorSelfTestTest, ReportNotEmpty) {
    SensorSelfTest s(4, 4);
    Event ev[3] = {{0, 0, 1, 0}, {0, 0, 1, 100}, {0, 0, 1, 150}};
    s.process(ev, 3);
    const std::string r = s.report();
    EXPECT_FALSE(r.empty());
    EXPECT_NE(r.find("Sensor Self-Test Report"), std::string::npos);
    EXPECT_NE(r.find("bad"), std::string::npos);
}

TEST(SensorSelfTestTest, MultiplePixelsStats) {
    // Two pixels: one with min interval 100us, one with 200us.
    SensorSelfTest s(4, 4);
    Event ev[4] = {{0, 0, 1, 0}, {0, 0, 1, 100},
                   {1, 1, 1, 0}, {1, 1, 1, 200}};
    s.process(ev, 4);
    auto stats = s.compute_stats();
    EXPECT_EQ(stats.measured_pixels, 2u);
    EXPECT_EQ(stats.min_us, 100);
    EXPECT_EQ(stats.max_us, 200);
    EXPECT_EQ(stats.mean_us, 150.0);
    // Sorted intervals: [100, 200]. median = intervals[1] = 200.
    EXPECT_EQ(stats.median_us, 200.0);
    EXPECT_EQ(stats.bad_pixels, 14u);  // 16 - 2 triggered
}

// ---------------------------------------------------------------------------
// DenseOpticalFlow (design §4.3.9b) — synthetic moving-line / moving-point
// scenes verify the three dense flow modes recover the known velocity.
// ---------------------------------------------------------------------------

namespace {

// A vertical line (height h) moving right at vx px/s: one event per pixel of
// the line every step_us µs, for n_steps steps. Returns sorted-by-t events.
std::vector<gui_algo::Event> make_moving_line(int start_x, int h,
                                              double vx_px_s,
                                              int n_steps, double step_us) {
    std::vector<gui_algo::Event> evs;
    const double vx_us = vx_px_s * 1e-6;  // px/µs
    for (int k = 0; k < n_steps; ++k) {
        const Metavision::timestamp t = static_cast<Metavision::timestamp>(
            std::llround(k * step_us));
        const int x = static_cast<int>(std::lround(start_x + vx_us * t));
        for (int y = 0; y < h; ++y) {
            gui_algo::Event e;
            e.x = x; e.y = y; e.p = 1; e.t = t;
            evs.push_back(e);
        }
    }
    return evs;
}

// A single point moving at (vx, vy) px/s: one event every step_us µs.
std::vector<gui_algo::Event> make_moving_point(double start_x, double start_y,
                                               double vx_px_s, double vy_px_s,
                                               int n_steps, double step_us) {
    std::vector<gui_algo::Event> evs;
    const double vx_us = vx_px_s * 1e-6, vy_us = vy_px_s * 1e-6;
    for (int k = 0; k < n_steps; ++k) {
        const Metavision::timestamp t = static_cast<Metavision::timestamp>(
            std::llround(k * step_us));
        gui_algo::Event e;
        e.x = static_cast<int>(std::lround(start_x + vx_us * t));
        e.y = static_cast<int>(std::lround(start_y + vy_us * t));
        e.p = 1; e.t = t;
        evs.push_back(e);
    }
    return evs;
}

} // namespace

TEST(DenseOpticalFlowTest, Construction) {
    gui_algo::DenseOpticalFlow f(64, 48);
    EXPECT_EQ(f.mode(), gui_algo::DenseOpticalFlow::Mode::PlaneFitting);
    EXPECT_EQ(f.width(), 64);
    EXPECT_EQ(f.height(), 48);
    f.set_mode(gui_algo::DenseOpticalFlow::Mode::TripletMatching);
    EXPECT_EQ(f.mode(), gui_algo::DenseOpticalFlow::Mode::TripletMatching);
}

TEST(DenseOpticalFlowTest, NoEventsProducesEmptyFlow) {
    gui_algo::DenseOpticalFlow f(32, 32);
    cv::Mat flow, conf;
    f.get_flow(flow, conf);
    EXPECT_EQ(flow.rows, 32);
    EXPECT_EQ(flow.cols, 32);
    EXPECT_EQ(cv::countNonZero(conf), 0);
}

TEST(DenseOpticalFlowTest, PlaneFittingRecoversHorizontalMotion) {
    constexpr int W = 64, H = 32;
    gui_algo::DenseOpticalFlow f(W, H, gui_algo::DenseOpticalFlow::Mode::PlaneFitting);
    f.set_time_window_us(10000);
    f.set_max_events(200000);
    f.set_max_velocity_px_s(10000);
    // Vertical line at x=8 → 28 over 10 ms, 2000 px/s.
    auto evs = make_moving_line(8, H - 4, 2000.0, 2000, 5.0);
    f.process(evs.data(), evs.data() + evs.size());

    cv::Mat flow, conf;
    f.get_flow(flow, conf);
    // Interior cell of the trajectory (x=18, y=16).
    const cv::Vec2f v = flow.at<cv::Vec2f>(16, 18);
    EXPECT_GT(conf.at<float>(16, 18), 0.0f);
    EXPECT_NEAR(v[0], 2000.0, 400.0) << "normal flow should recover vx";
    EXPECT_NEAR(v[1], 0.0, 200.0) << "no vertical motion expected";
}

TEST(DenseOpticalFlowTest, TimeGradientRecoversHorizontalMotion) {
    constexpr int W = 64, H = 32;
    gui_algo::DenseOpticalFlow f(W, H, gui_algo::DenseOpticalFlow::Mode::TimeGradient);
    f.set_time_window_us(10000);
    f.set_max_events(200000);
    f.set_max_velocity_px_s(10000);
    auto evs = make_moving_line(8, H - 4, 2000.0, 2000, 5.0);
    f.process(evs.data(), evs.data() + evs.size());

    cv::Mat flow, conf;
    f.get_flow(flow, conf);
    const cv::Vec2f v = flow.at<cv::Vec2f>(16, 18);
    EXPECT_GT(conf.at<float>(16, 18), 0.0f);
    EXPECT_NEAR(v[0], 2000.0, 600.0);
    EXPECT_NEAR(v[1], 0.0, 300.0);
}

TEST(DenseOpticalFlowTest, TripletMatchingRecoversDiagonalMotion) {
    constexpr int W = 128, H = 64;
    gui_algo::DenseOpticalFlow f(W, H, gui_algo::DenseOpticalFlow::Mode::TripletMatching);
    f.set_time_window_us(10000);
    f.set_max_velocity_px_s(20000);
    // Point from (30,20) moving (5000, 3000) px/s, event every 100 µs (100
    // events over 10 ms → well-separated triplets, exact velocity).
    auto evs = make_moving_point(30.0, 20.0, 5000.0, 3000.0, 100, 100.0);
    f.process(evs.data(), evs.data() + evs.size());

    cv::Mat flow, conf;
    f.get_flow(flow, conf);
    // A cell on the trajectory (near the middle).
    const cv::Vec2f v = flow.at<cv::Vec2f>(35, 55);
    EXPECT_GT(conf.at<float>(35, 55), 0.0f);
    EXPECT_NEAR(v[0], 5000.0, 1500.0) << "triplet flow should recover vx";
    EXPECT_NEAR(v[1], 3000.0, 1200.0) << "triplet flow should recover vy";
}

TEST(DenseOpticalFlowTest, ResetClearsState) {
    gui_algo::DenseOpticalFlow f(32, 32);
    auto evs = make_moving_line(4, 8, 1000.0, 200, 5.0);
    f.process(evs.data(), evs.data() + evs.size());
    f.reset();
    cv::Mat flow, conf;
    f.get_flow(flow, conf);
    EXPECT_EQ(cv::countNonZero(conf), 0);
}

// ---------------------------------------------------------------------------
// FrequencyMap (design §4.3.x) — synthetic blinking blobs verify per-pixel
// frequency estimation and frequency clustering.
// ---------------------------------------------------------------------------

namespace {

// Appends `n_cycles` ON/OFF pairs for an (x0,y0)-(x0+sw-1,y0+sh-1) blob that
// blinks at `freq_hz` (full period = 1/freq, half period between polarities).
// Events are emitted in non-decreasing timestamp order.
void add_blinking_blob(std::vector<gui_algo::Event>& evs,
                       int x0, int y0, int sw, int sh,
                       double freq_hz, int n_cycles) {
    const double half_us = 0.5e6 / freq_hz;
    for (int k = 0; k < n_cycles; ++k) {
        const Metavision::timestamp t_on = static_cast<Metavision::timestamp>(
            std::llround(k * 2.0 * half_us));
        const Metavision::timestamp t_off = static_cast<Metavision::timestamp>(
            std::llround(t_on + half_us));
        for (int dy = 0; dy < sh; ++dy) {
            for (int dx = 0; dx < sw; ++dx) {
                gui_algo::Event on;
                on.x = x0 + dx; on.y = y0 + dy; on.p = 1; on.t = t_on;
                evs.push_back(on);
                gui_algo::Event off;
                off.x = x0 + dx; off.y = y0 + dy; off.p = 0; off.t = t_off;
                evs.push_back(off);
            }
        }
    }
}

} // namespace

TEST(FrequencyMapTest, Construction) {
    gui_algo::FrequencyMap fm(32, 32);
    EXPECT_EQ(fm.width(), 32);
    EXPECT_EQ(fm.height(), 32);
    EXPECT_TRUE(fm.sources().empty());
}

TEST(FrequencyMapTest, DetectsSingleBlinkingBlob) {
    constexpr int W = 64, H = 64;
    gui_algo::FrequencyMap fm(W, H);
    gui_algo::FrequencyMapParams p;
    p.frequency_filter_length = 7;
    p.min_cluster_size = 20;
    fm.set_params(p);

    std::vector<gui_algo::Event> evs;
    add_blinking_blob(evs, 20, 20, 5, 5, 100.0, 20);  // 5×5 @ 100 Hz
    fm.process(evs.data(), evs.data() + evs.size());
    fm.analyze();

    // Per-pixel frequency at the blob centre ≈ 100 Hz.
    EXPECT_NEAR(fm.frequency_hz().at<float>(22, 22), 100.0f, 5.0f);
    // One clustered source with the expected centroid / frequency / area.
    ASSERT_EQ(fm.sources().size(), 1u);
    EXPECT_NEAR(fm.sources()[0].x, 22.0f, 1.0f);
    EXPECT_NEAR(fm.sources()[0].y, 22.0f, 1.0f);
    EXPECT_NEAR(fm.sources()[0].frequency_hz, 100.0f, 5.0f);
    EXPECT_EQ(fm.sources()[0].area, 25);
}

TEST(FrequencyMapTest, ClustersTwoDistinctFrequencies) {
    constexpr int W = 128, H = 64;
    gui_algo::FrequencyMap fm(W, H);
    gui_algo::FrequencyMapParams p;
    p.frequency_filter_length = 7;
    p.min_cluster_size = 10;
    p.max_cluster_frequency_diff = 10.0f;
    fm.set_params(p);

    std::vector<gui_algo::Event> evs;
    add_blinking_blob(evs, 20, 20, 4, 4, 80.0, 20);   // 80 Hz
    add_blinking_blob(evs, 40, 20, 4, 4, 100.0, 20);  // 100 Hz (gap ≥ 2 px)
    fm.process(evs.data(), evs.data() + evs.size());
    fm.analyze();

    ASSERT_EQ(fm.sources().size(), 2u);
    // 80 Hz source first (lower x), 100 Hz second.
    EXPECT_NEAR(fm.sources()[0].frequency_hz, 80.0f, 5.0f);
    EXPECT_NEAR(fm.sources()[1].frequency_hz, 100.0f, 5.0f);
}

TEST(FrequencyMapTest, NoiseProducesNoSource) {
    constexpr int W = 64, H = 64;
    gui_algo::FrequencyMap fm(W, H);
    gui_algo::FrequencyMapParams p;
    p.frequency_filter_length = 7;
    fm.set_params(p);

    // Random single events: no pixel repeats at a stable period.
    std::vector<gui_algo::Event> evs;
    unsigned seed = 7;
    for (int k = 0; k < 2000; ++k) {
        seed = seed * 1103515245u + 12345u;
        gui_algo::Event e;
        e.x = static_cast<int>((seed >> 16) % W);
        seed = seed * 1103515245u + 12345u;
        e.y = static_cast<int>((seed >> 16) % H);
        e.p = static_cast<short>((seed >> 16) & 1);
        e.t = k * 100;  // 100 µs apart, random pixel/polarity
        evs.push_back(e);
    }
    fm.process(evs.data(), evs.data() + evs.size());
    fm.analyze();
    EXPECT_TRUE(fm.sources().empty());
}

TEST(FrequencyMapTest, StaleFrequenciesAreDropped) {
    constexpr int W = 64, H = 64;
    gui_algo::FrequencyMap fm(W, H);
    gui_algo::FrequencyMapParams p;
    p.frequency_filter_length = 7;
    p.min_cluster_size = 20;
    p.stale_us = 100000;  // 100 ms
    fm.set_params(p);

    std::vector<gui_algo::Event> evs;
    add_blinking_blob(evs, 20, 20, 5, 5, 100.0, 10);  // 100 ms of blinking
    fm.process(evs.data(), evs.data() + evs.size());
    fm.analyze();
    EXPECT_NEAR(fm.frequency_hz().at<float>(22, 22), 100.0f, 5.0f);

    // Advance the stream past the stale threshold with events elsewhere.
    const Metavision::timestamp later = evs.back().t + p.stale_us + 1000;
    gui_algo::Event far;
    far.x = 0; far.y = 0; far.p = 1; far.t = later;
    fm.process(&far, &far + 1);
    fm.analyze();
    EXPECT_EQ(fm.frequency_hz().at<float>(22, 22), 0.0f);
    EXPECT_TRUE(fm.sources().empty());
}
