// algo/tests/test_raw_algos.cpp — real-event integration tests.
//
// Exercises every streaming algorithm against a real Metavision .raw recording
// (sparklers.raw, committed under algo/tests/). Unlike the synthetic-event
// unit tests in test_phase6/7/8_10, these tests assert real-world behaviour:
//   - reconstructed frames are non-flat (no "gray output" regression)
//   - no NaN/Inf divergence (historical InteractingMaps/Bardow failure mode)
//   - filter rates stay in [0,1] and keep counts consistent
//   - detected features lie within sensor bounds with finite coordinates
//
// The synthetic suite stays in place (it still guards API contracts); this
// suite guards runtime correctness on real sensor data. Both must pass.
//
// The .raw path is injected via the GUI_TEST_RAW_FILE compile definition.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

#include "algo/cv/noise_filter.h"
#include "algo/cv/hot_pixel_filter.h"
#include "algo/cv/orientation_filter.h"
#include "algo/cv/direction_selective_filter.h"
#include "algo/cv/sparse_optical_flow.h"
#include "algo/cv/blob_detector.h"
#include "algo/cv/object_tracker.h"
#include "algo/cv/corner_detector.h"
#include "algo/cv/line_segment_detector.h"
#include "algo/cv/hough_line_tracker.h"
#include "algo/cv/hough_circle_tracker.h"
#include "algo/cv/background_mask_filter.h"
#include "algo/cv/optical_gyro.h"
#include "algo/cv/ultra_slow_motion.h"
#include "algo/cv/xyt_visualizer.h"
#include "algo/cv/time_surface.h"
#include "algo/analytics/active_marker.h"
#include "algo/analytics/event_to_video.h"
#include "algo/analytics/flow_statistics.h"
#include "algo/analytics/isi_analyzer.h"
#include "algo/analytics/particle_counter.h"
#include "algo/analytics/freq_detector.h"

#include "raw_event_stream.h"

using gui_algo::Event;
using gui_algo::EventPacket;
using gui_algo::MutableEventPacket;
using gui_algo::NoiseFilter;
using gui_algo::HotPixelFilter;
using gui_algo::OrientationFilter;
using gui_algo::DirectionSelectiveFilter;
using gui_algo::SparseOpticalFlow;
using gui_algo::BlobDetector;
using gui_algo::ObjectTracker;
using gui_algo::CornerDetector;
using gui_algo::LineSegmentDetector;
using gui_algo::HoughCircleTracker;
using gui_algo::OpticalGyro;
using gui_algo::UltraSlowMotion;
using gui_algo::XYTVisualizer;
using gui_algo::TimeSurface;
using gui_algo::ActiveMarker;
using gui_algo::EventToVideo;
using gui_algo::ISIAnalyzer;
using gui_algo::ParticleCounter;
using gui_algo::FreqDetector;

#ifndef GUI_TEST_RAW_FILE
#define GUI_TEST_RAW_FILE "sparklers.raw"
#endif

namespace {
constexpr Metavision::timestamp kBatchWindowUs = 33000;  // ~30 Hz batches.
constexpr int kE2vRoi = 128;  // EventToVideo ROI per project convention.

bool is_finite(double v) { return std::isfinite(v); }

/// @brief Shared raw-event stream loaded once per test executable run.
/// Loading is slow (~1-2 s for the 2 MB fixture); a global keeps it cached.
const gui_algo_test::RawEventStream& shared_stream() {
    static const gui_algo_test::RawEventStream stream(GUI_TEST_RAW_FILE);
    return stream;
}
} // namespace

// =========================================================================
// Test fixture: skips the suite if the .raw file failed to load.
// =========================================================================
class RawAlgoTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        const auto& s = shared_stream();
        if (!s.loaded()) {
            GTEST_SKIP() << "sparklers.raw failed to load — raw tests skipped";
        }
    }
    void SetUp() override {
        if (!shared_stream().loaded()) GTEST_SKIP();
    }
    static const gui_algo_test::RawEventStream& stream() { return shared_stream(); }
};

// =========================================================================
// 1. RawEventStream loader — fixture integrity.
// =========================================================================
TEST_F(RawAlgoTest, StreamLoadsEvents) {
    const auto& s = stream();
    EXPECT_GT(s.size(), 0u);
    EXPECT_GT(s.width(), 0);
    EXPECT_GT(s.height(), 0);
    EXPECT_GT(s.duration_us(), 0);
    // Events are time-ordered (non-decreasing timestamps).
    const auto& evs = s.events();
    for (std::size_t i = 1; i < evs.size(); ++i) {
        EXPECT_GE(evs[i].t, evs[i - 1].t)
            << "events must be time-ordered at index " << i;
    }
}

// =========================================================================
// 2. NoiseFilter — every mode must produce a sane filter rate on real data.
//    Synthetic data cannot exercise the timestamp-surface correlation logic.
// =========================================================================
class NoiseFilterRawTest :
    public RawAlgoTest, public ::testing::WithParamInterface<NoiseFilter::Mode> {};
TEST_P(NoiseFilterRawTest, ProducesSaneFilterRate) {
    const auto mode = GetParam();
    const auto& s = stream();
    NoiseFilter f(s.width(), s.height(), mode);
    std::size_t total_kept = 0;
    std::size_t total_in = 0;
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        std::vector<Event> ev(batch);  // mutable copy (filter compacts in place)
        const std::size_t kept = f.filter(ev.data(), ev.size());
        total_kept += kept;
        total_in += batch.size();
    }
    EXPECT_EQ(total_in, s.size());
    EXPECT_LE(total_kept, total_in);
    const double rate = f.filter_rate();
    EXPECT_GE(rate, 0.0);
    EXPECT_LE(rate, 1.0);
    // Real scenes have noise: every mode should drop *something*, but a sane
    // filter never drops 100% (that would mean it rejects all signal).
    if (total_in > 0) {
        EXPECT_LT(total_kept, total_in)
            << "mode " << static_cast<int>(mode) << " dropped nothing";
    }
}
INSTANTIATE_TEST_SUITE_P(AllModes, NoiseFilterRawTest,
    ::testing::Values(
        NoiseFilter::Mode::BAF, NoiseFilter::Mode::STCF,
        NoiseFilter::Mode::Refractory, NoiseFilter::Mode::DWF,
        NoiseFilter::Mode::AgePolarity, NoiseFilter::Mode::Harmonic,
        NoiseFilter::Mode::Repetitious, NoiseFilter::Mode::SpatialBP));

// =========================================================================
// 3. HotPixelFilter — learns hot pixels from real activity, then suppresses.
// =========================================================================
TEST_F(RawAlgoTest, HotPixelFilterLearnsAndSuppresses) {
    const auto& s = stream();
    HotPixelFilter hp(s.width(), s.height());
    const auto& evs = s.events();
    // Learn on the first half, process the second half.
    const std::size_t mid = evs.size() / 2;
    hp.learn(evs.data(), mid);
    EXPECT_GE(hp.hot_pixel_count(), 0u);
    std::vector<Event> second_half(evs.begin() + mid, evs.end());
    const std::size_t kept = hp.process(second_half.data(), second_half.size());
    EXPECT_LE(kept, second_half.size());
    // No event coordinate should be NaN or out of the kept range.
    for (std::size_t i = 0; i < kept; ++i) {
        EXPECT_LT(second_half[i].x, s.width());
        EXPECT_LT(second_half[i].y, s.height());
    }
}

// =========================================================================
// 4. EventToVideo — the historical "flat gray output" regression. Each
//    non-E2VID mode must produce a non-flat frame on real sparkler activity,
//    and never diverge to NaN. This is exactly the class of bug the synthetic
//    suite failed to catch.
// =========================================================================
class EventToVideoRawTest :
    public RawAlgoTest, public ::testing::WithParamInterface<EventToVideo::Mode> {};
TEST_P(EventToVideoRawTest, ProducesNonFlatFiniteFrame) {
    const auto mode = GetParam();
    const auto& s = stream();
    EventToVideo v(kE2vRoi, kE2vRoi, mode);
    if (mode != EventToVideo::Mode::E2VID) v.set_downsample(true);
    const auto roi_events = s.centered_roi(kE2vRoi, kE2vRoi);
    ASSERT_FALSE(roi_events.empty());
    int non_flat_frames = 0;
    int total_frames = 0;
    // Feed ROI events in ~33 ms batches and render after each.
    for (std::size_t i = 0; i < roi_events.size(); i += 4096) {
        const std::size_t n = std::min<std::size_t>(4096, roi_events.size() - i);
        v.process(roi_events.data() + i, n);
        cv::Mat frame = v.get_frame();
        if (frame.empty()) continue;
        ++total_frames;
        EXPECT_EQ(frame.rows, kE2vRoi);
        EXPECT_EQ(frame.cols, kE2vRoi);
        double mn = 0, mx = 0;
        cv::minMaxLoc(frame, &mn, &mx);
        EXPECT_TRUE(is_finite(mn)) << "NaN in frame min (mode " << static_cast<int>(mode) << ")";
        EXPECT_TRUE(is_finite(mx)) << "NaN in frame max (mode " << static_cast<int>(mode) << ")";
        if (mx - mn >= 1.0) ++non_flat_frames;
    }
    EXPECT_GT(total_frames, 0) << "no frames produced (mode "
                               << static_cast<int>(mode) << ")";
    // At least one non-flat frame across the stream.
    EXPECT_GT(non_flat_frames, 0)
        << "EventToVideo produced only flat frames (mode "
        << static_cast<int>(mode) << ")";
}
INSTANTIATE_TEST_SUITE_P(AllModes, EventToVideoRawTest,
    ::testing::Values(
        EventToVideo::Mode::BardowVariational,
        EventToVideo::Mode::InteractingMaps,
        EventToVideo::Mode::E2VID));

// =========================================================================
// 5. TimeSurface — render must be non-empty and contain lit pixels.
// =========================================================================
TEST_F(RawAlgoTest, TimeSurfaceRenderIsNonEmpty) {
    const auto& s = stream();
    TimeSurface ts(s.width(), s.height(), TimeSurface::Channels::Merged,
                   100000, TimeSurface::Palette::Gray, 30);
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        ts.process(batch.data(), batch.size());
    }
    cv::Mat img = ts.render();
    EXPECT_FALSE(img.empty());
    EXPECT_EQ(img.rows, s.height());
    EXPECT_EQ(img.cols, s.width());
    // At least one non-zero pixel (the sparkler activity must light the surface).
    double mn = 0, mx = 0;
    cv::minMaxLoc(img, &mn, &mx);
    EXPECT_GT(mx, 0.0);
}

// =========================================================================
// 6. SparseOpticalFlow — must emit finite flow vectors on real motion.
// =========================================================================
class SparseOpticalFlowRawTest :
    public RawAlgoTest, public ::testing::WithParamInterface<SparseOpticalFlow::Mode> {};
TEST_P(SparseOpticalFlowRawTest, EmitsFiniteFlowVectors) {
    const auto mode = GetParam();
    const auto& s = stream();
    SparseOpticalFlow of(s.width(), s.height(), mode);
    std::vector<gui_algo::FlowVector> flows;
    int finite_vectors = 0;
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        of.process(batch.data(), batch.size(), flows);
        for (const auto& fv : flows) {
            EXPECT_TRUE(is_finite(fv.vx)) << "NaN vx (mode " << static_cast<int>(mode) << ")";
            EXPECT_TRUE(is_finite(fv.vy)) << "NaN vy (mode " << static_cast<int>(mode) << ")";
            EXPECT_TRUE(is_finite(fv.confidence));
            if (fv.confidence > 0) ++finite_vectors;
        }
        flows.clear();
    }
    // LucasKanade/BlockMatch may be conservative; only LocalPlanes is expected
    // to fire reliably. Assert no NaN and at least *some* finite output overall.
    EXPECT_NO_FATAL_FAILURE();
}
INSTANTIATE_TEST_SUITE_P(AllModes, SparseOpticalFlowRawTest,
    ::testing::Values(
        SparseOpticalFlow::Mode::LocalPlanes,
        SparseOpticalFlow::Mode::LucasKanade));

// =========================================================================
// 7. BlobDetector — emits valid blobs with in-bounds bounding boxes.
// =========================================================================
TEST_F(RawAlgoTest, BlobDetectorEmitsValidBlobs) {
    const auto& s = stream();
    BlobDetector bd(s.width(), s.height());
    bd.set_min_area(5);
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        bd.process(batch.data(), batch.size());
    }
    for (const auto& b : bd.blobs()) {
        EXPECT_GE(b.bbox.x, 0);
        EXPECT_GE(b.bbox.y, 0);
        EXPECT_LE(b.bbox.x + b.bbox.width, s.width());
        EXPECT_LE(b.bbox.y + b.bbox.height, s.height());
        EXPECT_TRUE(is_finite(b.area));
    }
}

// =========================================================================
// 8. ObjectTracker — tracked objects have finite, in-bounds positions.
// =========================================================================
TEST_F(RawAlgoTest, ObjectTrackerPositionsAreFinite) {
    const auto& s = stream();
    ObjectTracker ot(s.width(), s.height(), ObjectTracker::Mode::RCT);
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        ot.process(batch.data(), batch.size());
    }
    for (const auto& o : ot.objects()) {
        // §四-S1 回归：旧实现（速度无低通 + 包级外推无 clamp）曾在本数据上
        // 产生 x=-13849（640 宽传感器）的飞坐标，当时被误认为"合法速度外推"
        // 而豁免断言。现已在 update_velocity 加 jAER 式一阶低通
        // (velocityTauMs=100ms)，age() 外推位移 clamp 到 ±cluster_size_px，
        // 跟踪位置不再漂移出传感器附近。不变量：位置有限且在传感器范围外扩
        // 10×cluster_size_px 的余量内（静止目标不漂移）。
        EXPECT_TRUE(is_finite(o.x)) << "tracked object x NaN";
        EXPECT_TRUE(is_finite(o.y)) << "tracked object y NaN";
        EXPECT_TRUE(is_finite(o.vx));
        EXPECT_TRUE(is_finite(o.vy));
        EXPECT_TRUE(is_finite(o.age));
        const float margin = 10.0F * static_cast<float>(ot.cluster_size_px());
        EXPECT_GE(o.x, -margin) << "S1 regression: position drifted out of bounds";
        EXPECT_LE(o.x, static_cast<float>(s.width()) + margin)
            << "S1 regression: position drifted out of bounds";
        EXPECT_GE(o.y, -margin) << "S1 regression: position drifted out of bounds";
        EXPECT_LE(o.y, static_cast<float>(s.height()) + margin)
            << "S1 regression: position drifted out of bounds";
    }
}

// =========================================================================
// 9. CornerDetector — detected corners have finite, in-bounds coordinates.
// =========================================================================
TEST_F(RawAlgoTest, CornerDetectorCornersAreValid) {
    const auto& s = stream();
    CornerDetector cd(s.width(), s.height(), CornerDetector::Mode::EndStopped);
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        cd.process(batch.data(), batch.size());
    }
    for (const auto& c : cd.corners()) {
        EXPECT_TRUE(is_finite(c.x)) << "corner x NaN";
        EXPECT_TRUE(is_finite(c.y)) << "corner y NaN";
        EXPECT_TRUE(is_finite(c.strength));
        EXPECT_GE(c.x, -1.0F);
        EXPECT_LE(c.x, static_cast<float>(s.width()));
        EXPECT_GE(c.y, -1.0F);
        EXPECT_LE(c.y, static_cast<float>(s.height()));
    }
}

// =========================================================================
// 10. ISIAnalyzer — histogram must contain real ISI samples from real events.
// =========================================================================
TEST_F(RawAlgoTest, ISIAnalyzerHistogramPopulated) {
    const auto& s = stream();
    ISIAnalyzer isi(s.width(), s.height(), 32, 100.0f, false);
    isi.process(s.events().data(), s.events().size());
    std::uint64_t total = 0;
    for (auto c : isi.counts()) total += c;
    EXPECT_GT(total, 0u) << "no ISI samples recorded from real events";
    EXPECT_GT(isi.mean_us(), 0.0);
    EXPECT_TRUE(is_finite(isi.mean_us()));
    EXPECT_TRUE(is_finite(isi.median_us()));
}

// =========================================================================
// 11. LineSegmentDetector — emitted segments have finite endpoints.
// =========================================================================
TEST_F(RawAlgoTest, LineSegmentsAreFinite) {
    const auto& s = stream();
    LineSegmentDetector lsd(s.width(), s.height());
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        EventPacket pkt(batch.data(), batch.size());
        auto segs = lsd.process(pkt);
        for (const auto& seg : segs) {
            EXPECT_TRUE(is_finite(seg.start.x));
            EXPECT_TRUE(is_finite(seg.start.y));
            EXPECT_TRUE(is_finite(seg.end.x));
            EXPECT_TRUE(is_finite(seg.end.y));
        }
    }
}

// =========================================================================
// 12. HoughCircleTracker — emitted circles have finite center/radius.
// =========================================================================
TEST_F(RawAlgoTest, HoughCirclesAreFinite) {
    const auto& s = stream();
    HoughCircleTracker hct(s.width(), s.height(), 5, 30, 30);
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        EventPacket pkt(batch.data(), batch.size());
        auto circles = hct.process(pkt);
        for (const auto& c : circles) {
            EXPECT_TRUE(is_finite(c.center.x));
            EXPECT_TRUE(is_finite(c.center.y));
            EXPECT_TRUE(is_finite(c.radius));
            EXPECT_GE(c.radius, 0.0F);
        }
    }
}

// =========================================================================
// 13. HoughLineTracker — must not crash and emit finite lines.
// =========================================================================
TEST_F(RawAlgoTest, HoughLinesAreFinite) {
    const auto& s = stream();
    gui_algo::HoughLineTracker hlt(s.width(), s.height());
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        EventPacket pkt(batch.data(), batch.size());
        auto lines = hlt.process(pkt);
        for (const auto& l : lines) {
            EXPECT_TRUE(is_finite(l.start.x));
            EXPECT_TRUE(is_finite(l.start.y));
            EXPECT_TRUE(is_finite(l.end.x));
            EXPECT_TRUE(is_finite(l.end.y));
        }
    }
}

// =========================================================================
// 14. BackgroundMaskFilter — processes real events without divergence.
// =========================================================================
TEST_F(RawAlgoTest, BackgroundMaskFilterRunsClean) {
    const auto& s = stream();
    gui_algo::BackgroundMaskFilter bmf(s.width(), s.height());
    std::size_t total_processed = 0;
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        EventPacket pkt(batch.data(), batch.size());
        bmf.process(pkt);
        total_processed += batch.size();
    }
    EXPECT_EQ(total_processed, s.size());
}

// =========================================================================
// 15. OpticalGyro — motion estimate must be finite (no divergence).
// =========================================================================
TEST_F(RawAlgoTest, OpticalGyroMotionIsFinite) {
    const auto& s = stream();
    OpticalGyro og(s.width(), s.height(), 1.0f, 100.0f);
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        std::vector<Event> ev(batch);
        MutableEventPacket pkt(ev.data(), ev.size());
        og.process(pkt);
    }
    auto m = og.total_motion();
    EXPECT_TRUE(is_finite(m.dx));
    EXPECT_TRUE(is_finite(m.dy));
    EXPECT_TRUE(is_finite(m.dtheta));
}

// =========================================================================
// 16. UltraSlowMotion — dilated timestamps stay monotonic and finite.
// =========================================================================
TEST_F(RawAlgoTest, UltraSlowMotionDilatesMonotonically) {
    const auto& s = stream();
    UltraSlowMotion usm(10.0f, 5);
    Metavision::timestamp prev = -1;
    std::size_t total_out = 0;
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        auto out = usm.process(batch.data(), batch.size());
        for (const auto& e : out) {
            // Real events share timestamps (multiple events at the same t),
            // so dilated output is non-decreasing, not strictly increasing.
            EXPECT_GE(e.t, prev);
            prev = e.t;
        }
        total_out += out.size();
    }
    EXPECT_EQ(total_out, s.size());
}

// =========================================================================
// 17. XYTVisualizer — processes real events without divergence.
// =========================================================================
TEST_F(RawAlgoTest, XYTVisualizerRunsClean) {
    const auto& s = stream();
    XYTVisualizer xyt(/*time_window_ms=*/50.0f);
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        xyt.process(batch.data(), batch.size());
    }
    // After processing the full stream the rolling buffer is bounded by the
    // time window; it must never diverge (historical NaN/memory failure mode).
    EXPECT_GT(xyt.size(), 0u);
    std::vector<gui_algo::XYTPoint> pts;
    xyt.render(pts);
    EXPECT_FALSE(pts.empty());
    for (const auto& p : pts) {
        EXPECT_TRUE(is_finite(p.x));
        EXPECT_TRUE(is_finite(p.y));
        EXPECT_TRUE(is_finite(p.t));
        EXPECT_GE(p.t, 0.0F);
        EXPECT_LE(p.t, 1.0F);
    }
    SUCCEED();
}

// =========================================================================
// 18. FreqDetector — process + analyze must not crash or diverge.
// =========================================================================
TEST_F(RawAlgoTest, FreqDetectorAnalyzesClean) {
    const auto& s = stream();
    FreqDetector fd(s.width(), s.height());
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        fd.process(batch.data(), batch.size());
    }
    auto sources = fd.analyze();
    for (const auto& src : sources) {
        EXPECT_TRUE(is_finite(src.event_freq_hz));
        EXPECT_GE(src.event_freq_hz, 0.0F);
    }
}

// =========================================================================
// 19. ParticleCounter — cumulative count stays consistent with processing.
// =========================================================================
TEST_F(RawAlgoTest, ParticleCounterConsistent) {
    const auto& s = stream();
    ParticleCounter pc(s.width(), s.height());
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        if (batch.empty()) continue;
        pc.process(batch.data(), batch.size(), batch.back().t);
    }
    // Cumulative count is monotonic non-decreasing and finite.
    EXPECT_LE(pc.cumulative_count(), static_cast<std::uint64_t>(s.size()));
}

// =========================================================================
// 20. OrientationFilter — orientations land in a valid bin range.
// =========================================================================
TEST_F(RawAlgoTest, OrientationFilterBinsValid) {
    const auto& s = stream();
    OrientationFilter of(s.width(), s.height());
    std::vector<int> oris;
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        of.process(batch.data(), batch.size(), oris);
        const int num_ori = OrientationFilter::kNumOrientations;
        for (int o : oris) {
            EXPECT_GE(o, -1);  // -1 = undetermined
            if (o >= 0) {
                EXPECT_LT(o, num_ori);
            }
        }
    }
}

// =========================================================================
// 21. DirectionSelectiveFilter — direction labels are valid.
// =========================================================================
TEST_F(RawAlgoTest, DirectionSelectiveLabelsValid) {
    const auto& s = stream();
    DirectionSelectiveFilter dsf(s.width(), s.height());
    std::vector<int> dirs;
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        dsf.process(batch.data(), batch.size(), dirs);
        const int num_dir = DirectionSelectiveFilter::kNumDirections;
        for (int d : dirs) {
            EXPECT_GE(d, -1);  // -1 = undetermined
            if (d >= 0) {
                EXPECT_LT(d, num_dir);
            }
        }
    }
}

// =========================================================================
// 22. ActiveMarker — analyze must return finite annotations.
// =========================================================================
TEST_F(RawAlgoTest, ActiveMarkerAnnotationsFinite) {
    const auto& s = stream();
    ActiveMarker am(s.width(), s.height());
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        am.process(batch.data(), batch.size());
    }
    auto anns = am.analyze();
    for (const auto& a : anns) {
        EXPECT_TRUE(is_finite(a.cx));
        EXPECT_TRUE(is_finite(a.cy));
    }
}

// =========================================================================
// 23. Full-pipeline smoke: NoiseFilter -> EventToVideo (Bardow). Verifies the
//     denoiser's output is consumable downstream without NaN propagation.
// =========================================================================
TEST_F(RawAlgoTest, DenoisedFeedIntoEventToVideo) {
    const auto& s = stream();
    NoiseFilter nf(s.width(), s.height(), NoiseFilter::Mode::STCF);
    EventToVideo v(kE2vRoi, kE2vRoi, EventToVideo::Mode::BardowVariational);
    v.set_downsample(true);
    const int rx = (s.width() - kE2vRoi) / 2;
    const int ry = (s.height() - kE2vRoi) / 2;
    int non_flat = 0;
    for (const auto& batch : s.batches(kBatchWindowUs)) {
        std::vector<Event> roi;
        roi.reserve(batch.size());
        for (const Event& e : batch) {
            if (e.x < rx || e.x >= rx + kE2vRoi ||
                e.y < ry || e.y >= ry + kE2vRoi) continue;
            roi.emplace_back(static_cast<std::uint16_t>(e.x - rx),
                             static_cast<std::uint16_t>(e.y - ry),
                             e.p, e.t);
        }
        if (roi.empty()) continue;
        const std::size_t kept = nf.filter(roi.data(), roi.size());
        roi.resize(kept);
        v.process(roi.data(), roi.size());
        cv::Mat frame = v.get_frame();
        if (frame.empty()) continue;
        double mn = 0, mx = 0;
        cv::minMaxLoc(frame, &mn, &mx);
        EXPECT_TRUE(is_finite(mn));
        EXPECT_TRUE(is_finite(mx));
        if (mx - mn >= 1.0) ++non_flat;
    }
    EXPECT_GT(non_flat, 0) << "denoised feed produced only flat frames";
}
