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
#include "algo/cv/perspective_undistort.h"
#include "algo/cv/trigger_synced_filter.h"
#include "algo/cv/bandpass_filter.h"
#include "algo/cv/optical_gyro.h"
#include "algo/cv/ultra_slow_motion.h"
#include "algo/cv/xyt_visualizer.h"
#include "algo/cv/overlay.h"
#include "algo/cv/time_surface.h"
#include "algo/analytics/active_marker.h"
#include "algo/analytics/event_to_video.h"
#include "algo/analytics/e2vid/event_voxel_grid.h"
#include "algo/analytics/e2vid/intensity_rescaler.h"
#include "algo/analytics/e2vid/unsharp_mask.h"
#include "algo/analytics/e2vid/e2vid_inference.h"
#include "algo/analytics/flow_statistics.h"
#include "algo/analytics/isi_analyzer.h"
#include "algo/analytics/particle_counter.h"
#include "algo/analytics/auto_bias_controller.h"
#include "algo/analytics/freq_detector.h"

using gui_algo::Event;
using gui_algo::EventPacket;
using gui_algo::MutableEventPacket;
using gui_algo::LineSegmentDetector;
using gui_algo::HoughLineTracker;
using gui_algo::HoughCircleTracker;
using gui_algo::OrientationCluster;
using gui_algo::ClusterLIF;
using gui_algo::BackgroundMaskFilter;
using gui_algo::PerspectiveUndistort;
using gui_algo::TriggerSyncedFilter;
using gui_algo::BandpassFilter;
using gui_algo::OpticalGyro;
using gui_algo::UltraSlowMotion;
using gui_algo::XYTVisualizer;
using gui_algo::Overlay;
using gui_algo::TimeSurface;
using gui_algo::ActiveMarker;
using gui_algo::EventToVideo;
using gui_algo::FlowStatistics;
using gui_algo::ISIAnalyzer;
using gui_algo::ParticleCounter;
using gui_algo::AutoBiasController;
using gui_algo::FreqDetector;

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
    t.set_accumulator_decay_us(50000);
    EXPECT_EQ(t.accumulator_decay_us(), 50000);
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
    t.set_min_radius_px(10);
    EXPECT_EQ(t.min_radius_px(), 10);
    t.set_max_radius_px(100);
    EXPECT_EQ(t.max_radius_px(), 100);
    t.set_threshold(50);
    EXPECT_EQ(t.threshold(), 50);
    t.set_accumulator_decay_us(50000);
    EXPECT_EQ(t.accumulator_decay_us(), 50000);
}
TEST(HoughCircleTrackerTest, ProcessEmpty) {
    HoughCircleTracker t(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    auto result = t.process(pkt);
    EXPECT_TRUE(result.empty());
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

// --- 4.3.20 PerspectiveUndistort ---
TEST(PerspectiveUndistortTest, Construction) {
    PerspectiveUndistort u;
    EXPECT_TRUE(u.use_lut());
    EXPECT_TRUE(u.undistort());
}
TEST(PerspectiveUndistortTest, Params) {
    PerspectiveUndistort u;
    u.set_undistort(false);
    EXPECT_FALSE(u.undistort());
    u.set_rectify(true);
    EXPECT_TRUE(u.rectify());
}

// --- 4.3.21 TriggerSyncedFilter ---
TEST(TriggerSyncedFilterTest, Construction) {
    TriggerSyncedFilter f;
    EXPECT_EQ(f.trigger_window_us(), 100000);
    EXPECT_EQ(f.trigger_channel(), 0);
}
TEST(TriggerSyncedFilterTest, Params) {
    TriggerSyncedFilter f;
    f.set_trigger_window_us(50000);
    EXPECT_EQ(f.trigger_window_us(), 50000);
    f.set_trigger_channel(3);
    EXPECT_EQ(f.trigger_channel(), 3);
}
TEST(TriggerSyncedFilterTest, ProcessEmpty) {
    TriggerSyncedFilter f;
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    auto result = f.process(pkt);
    EXPECT_TRUE(result.empty());
}

// --- 4.3.22 BandpassFilter ---
TEST(BandpassFilterTest, Construction) {
    BandpassFilter f;
    EXPECT_DOUBLE_EQ(f.value(), 0.0);
}
TEST(BandpassFilterTest, Params) {
    BandpassFilter f(2.0f, 20.0f, 2, 0.01);
    f.set_cutoffs(5.0, 50.0);
    f.set_sample_dt(0.02);
    SUCCEED();
}
TEST(BandpassFilterTest, ProcessScalar) {
    BandpassFilter f;
    double y = f.process(100.0);
    EXPECT_TRUE(std::isfinite(y));
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
}

// =========================================================================
// Phase 9: algo/cv/ §4.3.24–4.3.27
// =========================================================================

// --- 4.3.24 UltraSlowMotion ---
TEST(UltraSlowMotionTest, Construction) {
    UltraSlowMotion m;
    EXPECT_FLOAT_EQ(m.dilation_factor(), 10.0f);
    EXPECT_EQ(m.min_accumulation_us(), 5);
}
TEST(UltraSlowMotionTest, Params) {
    UltraSlowMotion m;
    m.set_dilation_factor(100.0f);
    EXPECT_FLOAT_EQ(m.dilation_factor(), 100.0f);
    m.set_min_accumulation_us(10);
    EXPECT_EQ(m.min_accumulation_us(), 10);
}
TEST(UltraSlowMotionTest, EquivalentFps) {
    UltraSlowMotion m;
    // Default min_accumulation_us=5 -> 1e6/5 = 200000 fps
    EXPECT_DOUBLE_EQ(m.equivalent_fps(), 200000.0);
}
TEST(UltraSlowMotionTest, Process) {
    UltraSlowMotion m(10.0f, 5);
    auto ev = make_events(32, 32, 10);
    auto out = m.process(ev.data(), ev.size());
    EXPECT_EQ(out.size(), ev.size());
    // Timestamps should be dilated.
    EXPECT_GT(out.back().t, ev.front().t);
}

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

// --- 4.3.26 Overlay ---
TEST(OverlayTest, Construction) {
    Overlay o;
    (void)o;
    SUCCEED();
}
TEST(OverlayTest, DrawOnBlankImage) {
    Overlay o;
    cv::Mat img(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<gui_algo::FlowVector> flows;
    flows.push_back(gui_algo::FlowVector{10.0f, 10.0f, 5.0f, 0.0f});
    o.draw_flow_arrows(img, flows);
    EXPECT_EQ(img.rows, 100);
}
TEST(OverlayTest, DrawBboxes) {
    Overlay o;
    cv::Mat img(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<gui_algo::TrackedObject> objs;
    gui_algo::TrackedObject obj;
    obj.id = 1;
    obj.x = 10.0f;
    obj.y = 10.0f;
    obj.w = 30.0f;
    obj.h = 30.0f;
    objs.push_back(obj);
    o.draw_bboxes(img, objs);
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

// =========================================================================
// Phase 10: algo/analytics/ §4.4.1–4.4.7
// =========================================================================

// --- 4.4.1 ActiveMarker ---
TEST(ActiveMarkerTest, Construction) {
    ActiveMarker m(64, 48);
    EXPECT_EQ(m.width(), 64);
    EXPECT_EQ(m.height(), 48);
}
TEST(ActiveMarkerTest, Params) {
    ActiveMarker m(32, 32);
    m.set_window_ms(50.0f);
    EXPECT_FLOAT_EQ(m.window_ms(), 50.0f);
    m.set_heatmap_threshold(100);
    EXPECT_EQ(m.heatmap_threshold(), 100);
    m.set_enable_freq_detect(true);
    EXPECT_TRUE(m.enable_freq_detect());
}
TEST(ActiveMarkerTest, Process) {
    ActiveMarker m(32, 32);
    auto ev = make_events(32, 32, 100);
    m.process(ev.data(), ev.size());
    SUCCEED();
}

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

// --- 4.4.3 FlowStatistics ---
TEST(FlowStatisticsTest, Construction) {
    FlowStatistics fs;
    EXPECT_EQ(fs.output_hz(), 5);
}
TEST(FlowStatisticsTest, Params) {
    FlowStatistics fs;
    fs.set_output_hz(10);
    EXPECT_EQ(fs.output_hz(), 10);
    fs.set_source(FlowStatistics::Source::Annotated);
    EXPECT_EQ(fs.source(), FlowStatistics::Source::Annotated);
}
TEST(FlowStatisticsTest, AddSamples) {
    FlowStatistics fs;
    std::vector<gui_algo::FlowSample> samples;
    samples.push_back(gui_algo::FlowSample{1.0f, 0.0f, 1.0f, 0.0f}); // perfect match
    samples.push_back(gui_algo::FlowSample{2.0f, 0.0f, 1.0f, 0.0f}); // off by 1
    fs.add_samples(samples.data(), samples.size());
    EXPECT_GT(fs.epe_mean(), 0.0);
    EXPECT_GE(fs.epe_median(), 0.0);
}

// --- 4.4.4 ISIAnalyzer ---
TEST(ISIAnalyzerTest, Construction) {
    ISIAnalyzer a(64, 48);
    EXPECT_EQ(a.bin_count(), 32);  // default bin count
}
TEST(ISIAnalyzerTest, Params) {
    ISIAnalyzer a(32, 32);
    a.set_bin_count(64);
    EXPECT_EQ(a.bin_count(), 64);
    a.set_per_pixel(true);
    EXPECT_TRUE(a.per_pixel());
}
TEST(ISIAnalyzerTest, Process) {
    ISIAnalyzer a(32, 32);
    auto ev = make_events(32, 32, 50);
    a.process(ev.data(), ev.size());
    SUCCEED();
}

// --- 4.4.5 ParticleCounter ---
TEST(ParticleCounterTest, Construction) {
    ParticleCounter c(64, 48);
    EXPECT_EQ(c.width(), 64);
    EXPECT_EQ(c.height(), 48);
}
TEST(ParticleCounterTest, Params) {
    ParticleCounter c(32, 32);
    c.set_min_particle_size_px(10);
    EXPECT_EQ(c.min_particle_size_px(), 10);
    c.set_max_particle_size_px(200);
    EXPECT_EQ(c.max_particle_size_px(), 200);
}
TEST(ParticleCounterTest, InitialCount) {
    ParticleCounter c(32, 32);
    EXPECT_EQ(c.cumulative_count(), 0u);
}

// --- 4.4.6 AutoBiasController ---
TEST(AutoBiasControllerTest, Construction) {
    AutoBiasController c;
    EXPECT_FLOAT_EQ(c.target_event_rate_mev(), 5.0f);
    EXPECT_FLOAT_EQ(c.kp(), 0.5f);
}
TEST(AutoBiasControllerTest, Params) {
    AutoBiasController c;
    c.set_target_event_rate_mev(10.0f);
    EXPECT_FLOAT_EQ(c.target_event_rate_mev(), 10.0f);
    c.set_gains(0.3f, 0.02f, 0.01f);
    EXPECT_FLOAT_EQ(c.kp(), 0.3f);
    EXPECT_FLOAT_EQ(c.ki(), 0.02f);
    EXPECT_FLOAT_EQ(c.kd(), 0.01f);
}
TEST(AutoBiasControllerTest, Update) {
    AutoBiasController c(5.0f);
    auto cmd = c.update(3.0, 1000000); // measured 3 Mev/s, target 5
    EXPECT_NE(cmd.delta_diff, 0.0f); // should have some correction
}
TEST(AutoBiasControllerTest, Reset) {
    AutoBiasController c(5.0f);
    c.update(3.0, 1000000);
    c.reset();
    EXPECT_DOUBLE_EQ(c.integral(), 0.0);
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
    // May or may not find light sources.
    SUCCEED();
}
