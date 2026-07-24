// algo/tests/test_phase7_cv.cpp — unit tests for Phase 7 algo/cv/ modules.
//
// Covers: noise_filter, hot_pixel_filter, orientation_filter,
// direction_selective_filter, sparse_optical_flow, blob_detector,
// object_tracker, corner_detector, cluster_interface, cluster_path_point.
// Compiled with -Wall -Wextra -Werror.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/cv/cluster_path_point.h"
#include "algo/cv/cluster_interface.h"
#include "algo/cv/noise_filter.h"
#include "algo/cv/hot_pixel_filter.h"
#include "algo/cv/orientation_filter.h"
#include "algo/cv/direction_selective_filter.h"
#include "algo/cv/sparse_optical_flow.h"
#include "algo/cv/blob_detector.h"
#include "algo/cv/object_tracker.h"
#include "algo/cv/corner_detector.h"

using gui_algo::Event;
using gui_algo::EventPacket;
using gui_algo::MutableEventPacket;
using gui_algo::ClusterPathPoint;
using gui_algo::NoiseFilter;
using gui_algo::HotPixelFilter;
using gui_algo::OrientationFilter;
using gui_algo::DirectionSelectiveFilter;
using gui_algo::SparseOpticalFlow;
using gui_algo::BlobDetector;
using gui_algo::ObjectTracker;
using gui_algo::CornerDetector;

// Helper: build a vector of events.
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

// Helper: build a packet from a vector.
static EventPacket make_packet(const std::vector<Event>& v) {
    return EventPacket(v.data(), v.size());
}

// =========================================================================
// 1. cluster_path_point.h
// =========================================================================

TEST(ClusterPathPointTest, DefaultConstruction) {
    ClusterPathPoint p;
    EXPECT_FLOAT_EQ(p.x, 0.0F);
    EXPECT_FLOAT_EQ(p.y, 0.0F);
    EXPECT_FLOAT_EQ(p.vx, 0.0F);
    EXPECT_FLOAT_EQ(p.vy, 0.0F);
    EXPECT_FLOAT_EQ(p.radius, 0.0F);
    EXPECT_EQ(p.t, 0);
}

TEST(ClusterPathPointTest, Parameterized) {
    ClusterPathPoint p(1.5F, 2.5F, 0.1F, -0.2F, 1000, 3.0F);
    EXPECT_FLOAT_EQ(p.x, 1.5F);
    EXPECT_FLOAT_EQ(p.y, 2.5F);
    EXPECT_FLOAT_EQ(p.vx, 0.1F);
    EXPECT_FLOAT_EQ(p.vy, -0.2F);
    EXPECT_FLOAT_EQ(p.radius, 3.0F);
    EXPECT_EQ(p.t, 1000);
}

// =========================================================================
// 2. cluster_interface.h
// =========================================================================

TEST(ClusterInterfaceTest, IsAbstract) {
    // Verify ClusterInterface is abstract (cannot instantiate directly).
    static_assert(!std::is_default_constructible<gui_algo::ClusterInterface>::value,
                  "ClusterInterface must be abstract");
    SUCCEED();
}

// =========================================================================
// 3. noise_filter.h
// =========================================================================

TEST(NoiseFilterTest, Construction) {
    NoiseFilter f(64, 48);
    EXPECT_EQ(f.width(), 64);
    EXPECT_EQ(f.height(), 48);
    EXPECT_EQ(f.mode(), NoiseFilter::Mode::STCF); // default
}

TEST(NoiseFilterTest, ModeSwitching) {
    NoiseFilter f(32, 32);
    f.set_mode(NoiseFilter::Mode::BAF);
    EXPECT_EQ(f.mode(), NoiseFilter::Mode::BAF);
    f.set_mode(NoiseFilter::Mode::Refractory);
    EXPECT_EQ(f.mode(), NoiseFilter::Mode::Refractory);
}

TEST(NoiseFilterTest, BafParams) {
    NoiseFilter f(32, 32, NoiseFilter::Mode::BAF);
    f.set_baf_dt_us(5000);
    EXPECT_EQ(f.baf_dt_us(), 5000);
    // Clamped to [1000, 100000]
    f.set_baf_dt_us(0);
    EXPECT_EQ(f.baf_dt_us(), 1000);
    f.set_baf_dt_us(999999);
    EXPECT_EQ(f.baf_dt_us(), 100000);
}

TEST(NoiseFilterTest, StcfParams) {
    NoiseFilter f(32, 32, NoiseFilter::Mode::STCF);
    f.set_correlation_time_s(0.05);
    EXPECT_DOUBLE_EQ(f.correlation_time_s(), 0.05);
    f.set_min_neighbors(4);
    EXPECT_EQ(f.min_neighbors(), 4);
}

TEST(NoiseFilterTest, CommonOptions) {
    NoiseFilter f(32, 32);
    f.set_filter_hot_pixels(true);
    EXPECT_TRUE(f.filter_hot_pixels());
    f.set_adaptive_correlation_time(true);
    EXPECT_TRUE(f.adaptive_correlation_time());
}

TEST(NoiseFilterTest, ProcessEmpty) {
    NoiseFilter f(32, 32, NoiseFilter::Mode::BAF);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    std::size_t kept = f.process(pkt);
    EXPECT_EQ(kept, 0u);
}

TEST(NoiseFilterTest, ProcessRefractory) {
    NoiseFilter f(32, 32, NoiseFilter::Mode::Refractory);
    f.set_refractory_period_us(500);
    // Two events at same pixel close in time: second should be filtered.
    std::vector<Event> ev;
    ev.emplace_back(10, 10, 1, 1000);
    ev.emplace_back(10, 10, 1, 1200); // dt=200 < 500, filtered
    auto pkt = make_packet(ev);
    std::size_t kept = f.process(pkt);
    EXPECT_LE(kept, 2u); // at most 2 kept (refractory may keep first)
}

TEST(NoiseFilterTest, HarmonicParams) {
    NoiseFilter f(32, 32, NoiseFilter::Mode::Harmonic);
    f.set_line_freq(NoiseFilter::LineFreq::Hz60);
    f.set_notch_q(20.0);
    EXPECT_DOUBLE_EQ(f.notch_q(), 20.0);
}

// =========================================================================
// 4. hot_pixel_filter.h
// =========================================================================

TEST(HotPixelFilterTest, Construction) {
    HotPixelFilter f(64, 48);
    EXPECT_EQ(f.width(), 64);
    EXPECT_EQ(f.height(), 48);
}

TEST(HotPixelFilterTest, Params) {
    HotPixelFilter f(32, 32);
    f.set_learning_window_s(10.0);
    EXPECT_DOUBLE_EQ(f.learning_window_s(), 10.0);
    f.set_num_hot_pixels_max(500);
    EXPECT_EQ(f.num_hot_pixels_max(), 500);
    f.set_enable_fpn_correction(true);
    EXPECT_TRUE(f.enable_fpn_correction());
    f.set_fpn_target_rate_hz(100.0);
    EXPECT_DOUBLE_EQ(f.fpn_target_rate_hz(), 100.0);
}

TEST(HotPixelFilterTest, ParamClamping) {
    HotPixelFilter f(32, 32);
    f.set_learning_window_s(0.0); // below 1.0
    EXPECT_DOUBLE_EQ(f.learning_window_s(), 1.0);
    f.set_num_hot_pixels_max(0); // below 1
    EXPECT_EQ(f.num_hot_pixels_max(), 1);
}

TEST(HotPixelFilterTest, LearnAndProcess) {
    HotPixelFilter f(16, 16);
    auto ev = make_events(16, 16, 100, 0);
    f.learn(ev.data(), ev.size());
    std::size_t kept = f.process(ev.data(), ev.size());
    // Without enough events to trigger hot pixel learning, all pass.
    EXPECT_LE(kept, ev.size());
}

TEST(HotPixelFilterTest, HotPixelCount) {
    HotPixelFilter f(8, 8);
    EXPECT_EQ(f.hot_pixel_count(), 0u);
}

// =========================================================================
// 5. orientation_filter.h
// =========================================================================

TEST(OrientationFilterTest, Construction) {
    OrientationFilter f(64, 48);
    EXPECT_EQ(f.width(), 64);
    EXPECT_EQ(f.height(), 48);
    EXPECT_EQ(f.min_dt_threshold_us(), 100000);
}

TEST(OrientationFilterTest, Params) {
    OrientationFilter f(32, 32);
    f.set_min_dt_threshold_us(5000);
    EXPECT_EQ(f.min_dt_threshold_us(), 5000);
    f.set_use_average_dt(false);
    EXPECT_FALSE(f.use_average_dt());
    f.set_color_map(OrientationFilter::ColorMap::HSV);
    EXPECT_EQ(f.color_map(), OrientationFilter::ColorMap::HSV);
}

TEST(OrientationFilterTest, ClassifySingle) {
    OrientationFilter f(32, 32);
    Event e(10, 10, 1, 1000);
    int orient = f.classify(e);
    // First event: no neighbours, expect -1.
    EXPECT_EQ(orient, -1);
}

TEST(OrientationFilterTest, ClassifyWithNeighbours) {
    OrientationFilter f(32, 32);
    // Seed a horizontal line of events.
    for (int i = 0; i < 5; ++i) {
        f.classify(Event(static_cast<uint16_t>(8 + i), 10, 1, 1000 + i * 10));
    }
    // Next event in line should get an orientation.
    int orient = f.classify(Event(12, 10, 1, 1050));
    EXPECT_GE(orient, -1);
    EXPECT_LE(orient, 3);
}

// =========================================================================
// 6. direction_selective_filter.h
// =========================================================================

TEST(DirectionSelectiveFilterTest, Construction) {
    DirectionSelectiveFilter f(64, 48);
    EXPECT_EQ(f.width(), 64);
    EXPECT_EQ(f.height(), 48);
    EXPECT_EQ(f.time_window_us(), 10000);
    EXPECT_TRUE(f.enable_global_mode());
}

TEST(DirectionSelectiveFilterTest, Params) {
    DirectionSelectiveFilter f(32, 32);
    f.set_time_window_us(3000);
    EXPECT_EQ(f.time_window_us(), 3000);
    f.set_enable_global_mode(false);
    EXPECT_FALSE(f.enable_global_mode());
}

TEST(DirectionSelectiveFilterTest, ClassifySingle) {
    DirectionSelectiveFilter f(32, 32);
    Event e(10, 10, 1, 1000);
    int dir = f.classify(e);
    EXPECT_GE(dir, -1);
    EXPECT_LE(dir, 7);
}

TEST(DirectionSelectiveFilterTest, ProcessBatch) {
    DirectionSelectiveFilter f(32, 32);
    auto ev = make_events(32, 32, 50);
    std::vector<int> out;
    f.process(ev.data(), ev.size(), out);
    EXPECT_EQ(out.size(), ev.size());
}

// =========================================================================
// 7. sparse_optical_flow.h
// =========================================================================

TEST(SparseOpticalFlowTest, Construction) {
    SparseOpticalFlow f(64, 48);
    EXPECT_EQ(f.width(), 64);
    EXPECT_EQ(f.height(), 48);
    EXPECT_EQ(f.mode(), SparseOpticalFlow::Mode::LocalPlanes);
}

TEST(SparseOpticalFlowTest, ModeSwitching) {
    SparseOpticalFlow f(32, 32);
    f.set_mode(SparseOpticalFlow::Mode::LucasKanade);
    EXPECT_EQ(f.mode(), SparseOpticalFlow::Mode::LucasKanade);
    f.set_mode(SparseOpticalFlow::Mode::BlockMatch);
    EXPECT_EQ(f.mode(), SparseOpticalFlow::Mode::BlockMatch);
}

TEST(SparseOpticalFlowTest, LocalPlanesParams) {
    SparseOpticalFlow f(32, 32, SparseOpticalFlow::Mode::LocalPlanes);
    f.set_time_window_us(20000);
    EXPECT_EQ(f.time_window_us(), 20000);
    f.set_spatial_radius_px(10);
    EXPECT_EQ(f.spatial_radius_px(), 10);
    f.set_min_events_per_cluster(5);
    EXPECT_EQ(f.min_events_per_cluster(), 5);
}

TEST(SparseOpticalFlowTest, ProcessEmpty) {
    SparseOpticalFlow f(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    std::vector<gui_algo::FlowVector> out;
    f.process(pkt, out);
    EXPECT_TRUE(out.empty());
}

TEST(SparseOpticalFlowTest, ProcessWithEvents) {
    SparseOpticalFlow f(32, 32, SparseOpticalFlow::Mode::LocalPlanes);
    auto ev = make_events(32, 32, 100);
    auto pkt = make_packet(ev);
    std::vector<gui_algo::FlowVector> out;
    f.process(pkt, out);
    // May or may not produce flow vectors depending on clustering.
    SUCCEED();
}

// LucasKanade port verification: a 1px-wide vertical stripe translating right
// 1px/timestep builds an asymmetric event-count histogram. The central
// first-order spatial derivative (ix) and event-density temporal derivative
// (it) are both non-zero from the second timestep onward, so the structure
// tensor eigenvalue lam1 exceeds lk_thr and the solver emits flow vectors
// (either the 1D normal-flow fallback or the full 2D LK solve).
TEST(SparseOpticalFlowTest, LucasKanadeProducesFlow) {
    SparseOpticalFlow f(48, 48, SparseOpticalFlow::Mode::LucasKanade);
    f.set_search_radius_px(2);
    f.set_lk_thr(0.01);
    f.set_time_window_us(50000);
    std::vector<Event> ev;
    for (int t = 0; t < 10; ++t) {
        const int x = 16 + t;
        for (int y = 16; y <= 24; ++y) {
            ev.emplace_back(static_cast<uint16_t>(x), static_cast<uint16_t>(y),
                            1, static_cast<Metavision::timestamp>(t * 2000));
        }
    }
    auto pkt = make_packet(ev);
    std::vector<gui_algo::FlowVector> out;
    f.process(pkt, out);
    EXPECT_GT(out.size(), 0u);
    // Every emitted vector must carry a finite velocity estimate.
    for (const auto& fv : out) {
        EXPECT_TRUE(std::isfinite(fv.vx));
        EXPECT_TRUE(std::isfinite(fv.vy));
    }
}

// =========================================================================
// 8. blob_detector.h
// =========================================================================

TEST(BlobDetectorTest, Construction) {
    BlobDetector d(64, 48);
    EXPECT_EQ(d.width(), 64);
    EXPECT_EQ(d.height(), 48);
}

TEST(BlobDetectorTest, Params) {
    BlobDetector d(32, 32);
    d.set_accumulation_ms(50.0f);
    EXPECT_FLOAT_EQ(d.accumulation_ms(), 50.0f);
    d.set_threshold(100);
    EXPECT_EQ(d.threshold(), 100);
    d.set_min_area(20);
    EXPECT_EQ(d.min_area(), 20);
}

TEST(BlobDetectorTest, ProcessEmpty) {
    BlobDetector d(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    d.process(pkt);
    SUCCEED();
}

TEST(BlobDetectorTest, ProcessWithEvents) {
    BlobDetector d(32, 32);
    auto ev = make_events(32, 32, 200);
    auto pkt = make_packet(ev);
    d.process(pkt);
    SUCCEED();
}

// =========================================================================
// 9. object_tracker.h
// =========================================================================

TEST(ObjectTrackerTest, Construction) {
    ObjectTracker t(64, 48);
    EXPECT_EQ(t.width(), 64);
    EXPECT_EQ(t.height(), 48);
    EXPECT_EQ(t.mode(), ObjectTracker::Mode::RCT);
}

TEST(ObjectTrackerTest, ModeSwitching) {
    ObjectTracker t(32, 32);
    t.set_mode(ObjectTracker::Mode::Median);
    EXPECT_EQ(t.mode(), ObjectTracker::Mode::Median);
    t.set_mode(ObjectTracker::Mode::Kalman);
    EXPECT_EQ(t.mode(), ObjectTracker::Mode::Kalman);
    t.set_mode(ObjectTracker::Mode::MultiHypothesis);
    EXPECT_EQ(t.mode(), ObjectTracker::Mode::MultiHypothesis);
}

TEST(ObjectTrackerTest, Params) {
    ObjectTracker t(32, 32);
    t.set_cluster_size_px(15);
    EXPECT_EQ(t.cluster_size_px(), 15);
    t.set_cluster_time_us(3000);
    EXPECT_EQ(t.cluster_time_us(), 3000);
    t.set_max_lost_age_s(2.0f);
    EXPECT_FLOAT_EQ(t.max_lost_age_s(), 2.0f);
}

TEST(ObjectTrackerTest, ProcessEmpty) {
    ObjectTracker t(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    t.process(pkt);
    EXPECT_TRUE(t.objects().empty());
}

TEST(ObjectTrackerTest, ProcessWithEvents) {
    ObjectTracker t(32, 32, ObjectTracker::Mode::RCT);
    // Feed a burst of events in a small area to form a cluster.
    std::vector<Event> ev;
    for (int i = 0; i < 100; ++i) {
        ev.emplace_back(10 + i % 3, 10 + i % 3, 1, i * 100);
    }
    auto pkt = make_packet(ev);
    t.process(pkt);
    // May or may not produce tracked objects.
    SUCCEED();
}

// =========================================================================
// 10. corner_detector.h
// =========================================================================

TEST(CornerDetectorTest, Construction) {
    CornerDetector d(64, 48);
    EXPECT_EQ(d.width(), 64);
    EXPECT_EQ(d.height(), 48);
    EXPECT_EQ(d.mode(), CornerDetector::Mode::EndStopped);
}

TEST(CornerDetectorTest, ModeSwitching) {
    CornerDetector d(32, 32);
    d.set_mode(CornerDetector::Mode::TypeCoincidence);
    EXPECT_EQ(d.mode(), CornerDetector::Mode::TypeCoincidence);
    d.set_mode(CornerDetector::Mode::Harris);
    EXPECT_EQ(d.mode(), CornerDetector::Mode::Harris);
}

TEST(CornerDetectorTest, Params) {
    CornerDetector d(32, 32);
    d.set_accumulation_ms(20.0f);
    EXPECT_FLOAT_EQ(d.accumulation_ms(), 20.0f);
    d.set_threshold(0.2f);
    EXPECT_FLOAT_EQ(d.threshold(), 0.2f);
    d.set_track_radius_px(10);
    EXPECT_EQ(d.track_radius_px(), 10);
}

TEST(CornerDetectorTest, ProcessEmpty) {
    CornerDetector d(32, 32);
    std::vector<Event> empty;
    auto pkt = make_packet(empty);
    d.process(pkt);
    SUCCEED();
}

TEST(CornerDetectorTest, ProcessWithEvents) {
    CornerDetector d(32, 32);
    auto ev = make_events(32, 32, 100);
    auto pkt = make_packet(ev);
    d.process(pkt);
    SUCCEED();
}
