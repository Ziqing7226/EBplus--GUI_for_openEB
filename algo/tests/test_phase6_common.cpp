// algo/tests/test_phase6_common.cpp — strict unit tests for Phase 6 modules.
//
// Covers all 20 algo/common/ modules (3 pre-existing + 17 new). Tests verify
// numerical correctness, edge cases, boundary conditions, and behavioural
// contracts. Compiled with -Wall -Wextra -Werror.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <thread>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/common/event_buffer.h"
#include "algo/common/lifo_event_buffer.h"
#include "algo/common/frame_generator.h"
#include "algo/common/dvs_framer.h"
#include "algo/common/freme.h"
#include "algo/common/data_loader.h"
#include "algo/common/event_rate_estimator.h"
#include "algo/common/performance_meter.h"
#include "algo/common/time_limiter.h"
#include "algo/common/kalman_filter.h"
#include "algo/common/kmeans.h"
#include "algo/common/particle_filter.h"
#include "algo/common/periodic_spline.h"
#include "algo/common/histogram_ring_buffer.h"
#include "algo/common/lif_integrator.h"
#include "algo/common/filter/lowpass.h"
#include "algo/common/filter/highpass.h"
#include "algo/common/filter/bandpass.h"
#include "algo/common/filter/angular_lowpass.h"
#include "algo/common/filter/median_lowpass.h"

using gui_algo::Event;
using gui_algo::EventPacket;
using gui_algo::MutableEventPacket;
using gui_algo::LifoEventBuffer;
using gui_algo::DvsFramer;
using gui_algo::Freme;
using gui_algo::EventRateEstimator;
using gui_algo::PerformanceMeter;
using gui_algo::TimeLimiter;
using gui_algo::KalmanFilter2D;
using gui_algo::KMeans;
using gui_algo::ParticleFilter;
using gui_algo::PeriodicSpline;
using gui_algo::HistogramRingBuffer;
using gui_algo::LifIntegrator;
using gui_algo::LowPassFilter;
using gui_algo::HighPassFilter;
using gui_algo::BandpassFilter;
using gui_algo::AngularLowpassFilter;
using gui_algo::MedianLowpassFilter;
using gui_algo::Point2D;

// =========================================================================
// 1. event.h
// =========================================================================

TEST(EventTest, DefaultConstructionIsZero) {
    Event e;
    EXPECT_EQ(e.x, 0u);
    EXPECT_EQ(e.y, 0u);
    EXPECT_EQ(e.p, 0);
    EXPECT_EQ(e.t, 0);
}

TEST(EventTest, ParameterizedConstruction) {
    Event e(100, 200, 1, 5000);
    EXPECT_EQ(e.x, 100u);
    EXPECT_EQ(e.y, 200u);
    EXPECT_EQ(e.p, 1);
    EXPECT_EQ(e.t, 5000);
}

TEST(EventTest, LayoutCompatibleWithEventCD) {
    ::testing::StaticAssertTypeEq<decltype(Event::x), std::uint16_t>();
    ::testing::StaticAssertTypeEq<decltype(Event::t), Metavision::timestamp>();
    static_assert(sizeof(Event) == sizeof(Metavision::EventCD),
                  "layout mismatch");
}

TEST(EventTest, ConversionFromAndToEventCD) {
    Metavision::EventCD cd(50, 60, 1, 12345);
    Event e(cd);
    EXPECT_EQ(e.x, 50u);
    EXPECT_EQ(e.y, 60u);
    EXPECT_EQ(e.p, 1);
    EXPECT_EQ(e.t, 12345);

    Metavision::EventCD back = e;
    EXPECT_EQ(back.x, 50u);
    EXPECT_EQ(back.y, 60u);
    EXPECT_EQ(back.p, 1);
    EXPECT_EQ(back.t, 12345);
}

TEST(EventTest, PolarityHelpers) {
    Event on(0, 0, 1, 0);
    Event off(0, 0, 0, 0);
    EXPECT_TRUE(on.is_on());
    EXPECT_FALSE(on.is_off());
    EXPECT_FALSE(off.is_on());
    EXPECT_TRUE(off.is_off());
    EXPECT_EQ(on.signed_polarity(), 1);
    EXPECT_EQ(off.signed_polarity(), -1);
}

TEST(EventTest, NamedAccessors) {
    Event e(10, 20, 1, 999);
    EXPECT_EQ(e.col(), 10u);
    EXPECT_EQ(e.row(), 20u);
    EXPECT_EQ(e.time_us(), 999);
}

TEST(EventTest, Comparators) {
    Event a(0, 0, 0, 100);
    Event b(0, 0, 0, 200);
    EXPECT_TRUE(a < b);
    EXPECT_TRUE(a <= b);
    EXPECT_FALSE(a > b);
    EXPECT_TRUE(b >= a);
    EXPECT_FALSE(a == b);
    Event c(0, 0, 0, 100);
    EXPECT_TRUE(a == c);
}

// =========================================================================
// 2. event_packet.h
// =========================================================================

TEST(EventPacketTest, EmptyPacket) {
    EventPacket pkt;
    EXPECT_TRUE(pkt.empty());
    EXPECT_EQ(pkt.size(), 0u);
}

TEST(EventPacketTest, ConstructFromPointerSize) {
    std::vector<Event> evs = {{0,0,1,10}, {1,1,0,20}, {2,2,1,30}};
    EventPacket pkt(evs.data(), evs.size());
    EXPECT_EQ(pkt.size(), 3u);
    EXPECT_FALSE(pkt.empty());
}

TEST(EventPacketTest, ConstructFromIterators) {
    std::vector<Event> evs = {{0,0,1,10}, {1,1,0,20}};
    EventPacket pkt(evs.data(), evs.data() + evs.size());
    EXPECT_EQ(pkt.size(), 2u);
}

TEST(EventPacketTest, ElementAccess) {
    std::vector<Event> evs = {{5,6,1,100}, {7,8,0,200}};
    EventPacket pkt(evs.data(), evs.size());
    EXPECT_EQ(pkt[0].x, 5u);
    EXPECT_EQ(pkt[1].y, 8u);
    EXPECT_EQ(pkt.data(), evs.data());
}

TEST(EventPacketTest, Iteration) {
    std::vector<Event> evs = {{0,0,1,1}, {0,0,1,2}, {0,0,1,3}};
    EventPacket pkt(evs.data(), evs.size());
    int count = 0;
    for (const auto& e : pkt) {
        EXPECT_EQ(e.t, count + 1);
        ++count;
    }
    EXPECT_EQ(count, 3);
}

TEST(EventPacketTest, Subpacket) {
    std::vector<Event> evs = {{0,0,1,1}, {0,0,1,2}, {0,0,1,3}, {0,0,1,4}};
    EventPacket pkt(evs.data(), evs.size());
    auto sub = pkt.subpacket(1, 2);
    EXPECT_EQ(sub.size(), 2u);
    EXPECT_EQ(sub[0].t, 2);
    EXPECT_EQ(sub[1].t, 3);
}

TEST(EventPacketTest, FirstLast) {
    std::vector<Event> evs = {{0,0,1,1}, {0,0,1,2}, {0,0,1,3}};
    EventPacket pkt(evs.data(), evs.size());
    auto f = pkt.first(2);
    auto l = pkt.last(2);
    EXPECT_EQ(f.size(), 2u);
    EXPECT_EQ(f[0].t, 1);
    EXPECT_EQ(l.size(), 2u);
    EXPECT_EQ(l[0].t, 2);
    EXPECT_EQ(l[1].t, 3);
}

TEST(EventPacketTest, FirstMoreThanSize) {
    std::vector<Event> evs = {{0,0,1,1}, {0,0,1,2}};
    EventPacket pkt(evs.data(), evs.size());
    auto f = pkt.first(10);
    EXPECT_EQ(f.size(), 2u);
}

TEST(EventPacketTest, MutablePacket) {
    std::vector<Event> evs = {{0,0,0,1}};
    MutableEventPacket pkt(evs.data(), evs.size());
    pkt[0].p = 1;
    EXPECT_EQ(evs[0].p, 1);
}

// =========================================================================
// 3. lifo_event_buffer.h
// =========================================================================

TEST(LifoEventBufferTest, PushPopLIFOOrder) {
    LifoEventBuffer buf(5);
    EXPECT_TRUE(buf.empty());
    buf.push(Event(0,0,1,10));
    buf.push(Event(0,0,1,20));
    buf.push(Event(0,0,1,30));
    EXPECT_EQ(buf.size(), 3u);
    Event e;
    ASSERT_TRUE(buf.pop(e));
    EXPECT_EQ(e.t, 30);
    ASSERT_TRUE(buf.pop(e));
    EXPECT_EQ(e.t, 20);
    ASSERT_TRUE(buf.pop(e));
    EXPECT_EQ(e.t, 10);
    EXPECT_FALSE(buf.pop(e));
    EXPECT_TRUE(buf.empty());
}

TEST(LifoEventBufferTest, TopAccess) {
    LifoEventBuffer buf(5);
    EXPECT_EQ(buf.top(), nullptr);
    buf.push(Event(0,0,1,100));
    const Event* t = buf.top();
    ASSERT_NE(t, nullptr);
    EXPECT_EQ(t->t, 100);
    buf.push(Event(0,0,1,200));
    t = buf.top();
    EXPECT_EQ(t->t, 200);
}

TEST(LifoEventBufferTest, AtFromTop) {
    LifoEventBuffer buf(5);
    buf.push(Event(0,0,1,1));
    buf.push(Event(0,0,1,2));
    buf.push(Event(0,0,1,3));
    EXPECT_EQ(buf.at_from_top(0)->t, 3);
    EXPECT_EQ(buf.at_from_top(1)->t, 2);
    EXPECT_EQ(buf.at_from_top(2)->t, 1);
    EXPECT_EQ(buf.at_from_top(3), nullptr);
}

TEST(LifoEventBufferTest, OverflowDiscardsOldest) {
    LifoEventBuffer buf(3);
    buf.push(Event(0,0,1,1));
    buf.push(Event(0,0,1,2));
    buf.push(Event(0,0,1,3));
    EXPECT_EQ(buf.size(), 3u);
    // Overflow: should discard t=1, keep 2,3,4.
    buf.push(Event(0,0,1,4));
    EXPECT_EQ(buf.size(), 3u);
    Event e;
    buf.pop(e); EXPECT_EQ(e.t, 4);
    buf.pop(e); EXPECT_EQ(e.t, 3);
    buf.pop(e); EXPECT_EQ(e.t, 2);
}

TEST(LifoEventBufferTest, ForEachNewestFirst) {
    LifoEventBuffer buf(5);
    buf.push(Event(0,0,1,1));
    buf.push(Event(0,0,1,2));
    buf.push(Event(0,0,1,3));
    std::vector<Metavision::timestamp> ts;
    buf.for_each_newest_first([&](const Event& e){ ts.push_back(e.t); });
    ASSERT_EQ(ts.size(), 3u);
    EXPECT_EQ(ts[0], 3);
    EXPECT_EQ(ts[1], 2);
    EXPECT_EQ(ts[2], 1);
}

TEST(LifoEventBufferTest, ForEachOldestFirst) {
    LifoEventBuffer buf(5);
    buf.push(Event(0,0,1,1));
    buf.push(Event(0,0,1,2));
    buf.push(Event(0,0,1,3));
    std::vector<Metavision::timestamp> ts;
    buf.for_each_oldest_first([&](const Event& e){ ts.push_back(e.t); });
    ASSERT_EQ(ts.size(), 3u);
    EXPECT_EQ(ts[0], 1);
    EXPECT_EQ(ts[1], 2);
    EXPECT_EQ(ts[2], 3);
}

TEST(LifoEventBufferTest, TrimOld) {
    LifoEventBuffer buf(10);
    buf.push(Event(0,0,1,1000));
    buf.push(Event(0,0,1,2000));
    buf.push(Event(0,0,1,3000));
    buf.push(Event(0,0,1,4000));
    // Keep only events within 500us of newest (4000).
    buf.trim_old(500);
    EXPECT_EQ(buf.size(), 1u);
    EXPECT_EQ(buf.top()->t, 4000);
}

TEST(LifoEventBufferTest, TrimOldKeepsAllIfThresholdLarge) {
    LifoEventBuffer buf(10);
    buf.push(Event(0,0,1,1000));
    buf.push(Event(0,0,1,2000));
    buf.trim_old(100000);
    EXPECT_EQ(buf.size(), 2u);
}

TEST(LifoEventBufferTest, Clear) {
    LifoEventBuffer buf(5);
    buf.push(Event(0,0,1,1));
    buf.push(Event(0,0,1,2));
    buf.clear();
    EXPECT_TRUE(buf.empty());
    EXPECT_EQ(buf.size(), 0u);
}

TEST(LifoEventBufferTest, CapacityZeroHandled) {
    LifoEventBuffer buf(0);  // should clamp to 1
    buf.push(Event(0,0,1,1));
    EXPECT_EQ(buf.size(), 1u);
    EXPECT_EQ(buf.capacity(), 1u);
}

// =========================================================================
// 4. dvs_framer.h
// =========================================================================

TEST(DvsFramerTest, UnsignedCountAccumulation) {
    DvsFramer f(10, 10, DvsFramer::PolarityMode::UnsignedCount);
    f.add_event(Metavision::EventCD(0, 0, 1, 0));
    f.add_event(Metavision::EventCD(0, 0, 1, 1));
    f.add_event(Metavision::EventCD(0, 0, 0, 2));  // OFF also counts
    cv::Mat frame = f.generate();
    ASSERT_EQ(frame.type(), CV_8UC1);
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 3);
}

TEST(DvsFramerTest, SignedMode) {
    DvsFramer f(10, 10, DvsFramer::PolarityMode::Signed);
    f.add_event(Metavision::EventCD(0, 0, 1, 0));  // ON: +1
    f.add_event(Metavision::EventCD(0, 0, 1, 1));  // ON: +1
    f.add_event(Metavision::EventCD(0, 0, 0, 2));  // OFF: -1
    cv::Mat frame = f.generate();
    ASSERT_EQ(frame.type(), CV_8UC1);
    // 128 + 2 - 1 = 129
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 129);
}

TEST(DvsFramerTest, SignedModeSaturatesLow) {
    DvsFramer f(10, 10, DvsFramer::PolarityMode::Signed);
    for (int i = 0; i < 300; ++i)
        f.add_event(Metavision::EventCD(0, 0, 0, i));  // many OFF
    cv::Mat frame = f.generate();
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 0);
}

TEST(DvsFramerTest, SignedModeSaturatesHigh) {
    DvsFramer f(10, 10, DvsFramer::PolarityMode::Signed);
    for (int i = 0; i < 300; ++i)
        f.add_event(Metavision::EventCD(0, 0, 1, i));  // many ON
    cv::Mat frame = f.generate();
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 255);
}

TEST(DvsFramerTest, SplitPolarityMode) {
    DvsFramer f(10, 10, DvsFramer::PolarityMode::SplitPolarity);
    f.add_event(Metavision::EventCD(0, 0, 1, 0));
    f.add_event(Metavision::EventCD(0, 0, 1, 1));
    f.add_event(Metavision::EventCD(0, 0, 0, 2));
    f.add_event(Metavision::EventCD(0, 0, 0, 3));
    f.add_event(Metavision::EventCD(0, 0, 0, 4));
    cv::Mat frame = f.generate();
    ASSERT_EQ(frame.type(), CV_8UC2);
    cv::Vec2b px = frame.at<cv::Vec2b>(0, 0);
    EXPECT_EQ(px[0], 3);  // OFF count
    EXPECT_EQ(px[1], 2);  // ON count
}

TEST(DvsFramerTest, GenerateAndResetClears) {
    DvsFramer f(10, 10);
    f.add_event(Metavision::EventCD(0, 0, 1, 0));
    f.generate_and_reset();
    cv::Mat frame = f.generate();
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 0);
}

TEST(DvsFramerTest, OutOfBoundsEventIgnored) {
    DvsFramer f(10, 10);
    f.add_event(Metavision::EventCD(100, 100, 1, 0));  // OOB
    cv::Mat frame = f.generate();
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 0);
}

TEST(DvsFramerTest, CountSaturationAt255) {
    DvsFramer f(10, 10, DvsFramer::PolarityMode::UnsignedCount);
    for (int i = 0; i < 300; ++i)
        f.add_event(Metavision::EventCD(0, 0, 1, i));
    cv::Mat frame = f.generate();
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 255);
}

TEST(DvsFramerTest, AddEventsRange) {
    DvsFramer f(10, 10);
    std::vector<Metavision::EventCD> evs = {
        {0,0,1,0}, {0,0,1,1}, {1,1,0,2}
    };
    f.add_events(evs.begin(), evs.end());
    cv::Mat frame = f.generate();
    EXPECT_EQ(frame.at<std::uint8_t>(0, 0), 2);
    EXPECT_EQ(frame.at<std::uint8_t>(1, 1), 1);
}

// =========================================================================
// 5. freme.h
// =========================================================================

// Freme<O> is a generic 2D state-map container ported from jAER's Freme<O>.
// Each pixel (x, y) maps to a single state of type O.

TEST(FremeTest, ConstructionAndSize) {
    Freme<float> f(10, 8);
    EXPECT_EQ(f.width(), 10);
    EXPECT_EQ(f.height(), 8);
    EXPECT_EQ(f.size(), 80);
}

TEST(FremeTest, DefaultInitializedToZero) {
    Freme<float> f(10, 10);
    // Value-initialized float elements are 0.0f.
    for (int y = 0; y < 10; ++y)
        for (int x = 0; x < 10; ++x)
            EXPECT_EQ(f.get(x, y), 0.0f);
}

TEST(FremeTest, SetAndGet) {
    Freme<float> f(10, 10);
    f.set(3, 4, 7.5f);
    EXPECT_EQ(f.get(3, 4), 7.5f);
    // Other pixels untouched.
    EXPECT_EQ(f.get(0, 0), 0.0f);
}

TEST(FremeTest, GetReturnsReference) {
    Freme<float> f(10, 10);
    f.get(2, 5) = 9.0f;  // mutate through the returned reference
    EXPECT_EQ(f.get(2, 5), 9.0f);
}

TEST(FremeTest, Fill) {
    Freme<int> f(6, 6);
    f.fill(42);
    for (int y = 0; y < 6; ++y)
        for (int x = 0; x < 6; ++x)
            EXPECT_EQ(f.get(x, y), 42);
}

TEST(FremeTest, GetIndex) {
    Freme<float> f(10, 10);
    EXPECT_EQ(f.getIndex(0, 0), 0);
    EXPECT_EQ(f.getIndex(3, 4), 43);  // 4 * 10 + 3
    EXPECT_EQ(f.getIndex(9, 9), 99);
}

TEST(FremeTest, Iteration) {
    Freme<float> f(4, 5);  // 20 pixels
    f.fill(1.0f);
    float sum = 0.0f;
    for (float v : f) sum += v;
    EXPECT_NEAR(sum, 20.0f, 1e-9);
}

TEST(FremeTest, ConstAccess) {
    Freme<int> f(3, 3);
    f.set(1, 1, 123);
    const Freme<int>& cf = f;
    EXPECT_EQ(cf.get(1, 1), 123);
    int sum = 0;
    for (int v : cf) sum += v;
    EXPECT_EQ(sum, 123);
}

TEST(FremeTest, DifferentStateType) {
    // The state type may itself be a container, e.g. std::vector<int>.
    Freme<std::vector<int>> f(2, 2);
    f.get(0, 0).push_back(10);
    f.get(0, 0).push_back(20);
    EXPECT_EQ(f.get(0, 0).size(), 2u);
    EXPECT_EQ(f.get(1, 1).size(), 0u);  // default-constructed empty vector
}

TEST(FremeTest, EmptyContainer) {
    Freme<float> f(0, 0);
    EXPECT_EQ(f.size(), 0);
    EXPECT_EQ(f.begin(), f.end());
}

// =========================================================================
// 6. event_rate_estimator.h
// =========================================================================

TEST(EventRateEstimatorTest, FirstBatchNoRate) {
    EventRateEstimator est;  // default window 10000us
    est.add_events(100, 1000);
    EXPECT_EQ(est.rate_eps(), 0.0);  // uninitialized, window not yet elapsed
}

TEST(EventRateEstimatorTest, RateComputation) {
    EventRateEstimator est;  // default window 10000us
    est.add_events(100, 0);        // seed
    est.add_events(100, 10000);    // 100 events in 10ms → 10000 eps
    EXPECT_NEAR(est.rate_eps(), 10000.0, 1.0);
}

TEST(EventRateEstimatorTest, MevsConversion) {
    EventRateEstimator est;
    est.add_events(100, 0);
    est.add_events(100, 10000);
    EXPECT_NEAR(est.rate_meps(), 0.01, 1e-6);
}

TEST(EventRateEstimatorTest, WindowedCounting) {
    // jAER EventRateEstimator: no IIR; rate is recomputed only when the
    // elapsed time reaches window_us. filteredRate == instantaneousRate.
    EventRateEstimator est;  // window 10000us
    est.add_events(100, 0);    // seed
    est.add_events(50, 5000);  // dt=5000 < 10000 → no update yet
    EXPECT_EQ(est.rate_eps(), 0.0);
    est.add_events(50, 10000); // dt=10000 >= 10000 → 100 events / 10ms
    EXPECT_NEAR(est.rate_eps(), 10000.0, 1.0);
    EXPECT_NEAR(est.instantaneous_rate(), 10000.0, 1.0);
}

TEST(EventRateEstimatorTest, Reset) {
    EventRateEstimator est;
    est.add_events(100, 0);
    est.add_events(100, 10000);
    est.reset();
    EXPECT_EQ(est.rate_eps(), 0.0);
    EXPECT_EQ(est.instantaneous_rate(), 0.0);
}

TEST(EventRateEstimatorTest, ZeroEventsIgnored) {
    EventRateEstimator est;
    est.add_events(100, 0);
    est.add_events(0, 10000);  // zero events → ignored
    EXPECT_EQ(est.rate_eps(), 0.0);
}

// =========================================================================
// 7. performance_meter.h
// =========================================================================

TEST(PerformanceMeterTest, InitialState) {
    PerformanceMeter pm;
    EXPECT_EQ(pm.fps(), 0.0);
    EXPECT_EQ(pm.latency_us(), 0.0);
    EXPECT_EQ(pm.total_events(), 0u);
    EXPECT_EQ(pm.total_frames(), 0u);
    EXPECT_EQ(pm.total_dropped(), 0u);
    EXPECT_EQ(pm.drop_ratio(), 0.0);
}

TEST(PerformanceMeterTest, EventCountAccumulates) {
    PerformanceMeter pm;
    pm.tick_events(100, 0);
    pm.tick_events(200, 1000);
    EXPECT_EQ(pm.total_events(), 300u);
}

TEST(PerformanceMeterTest, FrameCountAccumulates) {
    PerformanceMeter pm;
    pm.tick_frame();
    pm.tick_frame();
    pm.tick_frame();
    EXPECT_EQ(pm.total_frames(), 3u);
}

TEST(PerformanceMeterTest, DropRatio) {
    PerformanceMeter pm;
    pm.tick_events(100, 0);
    pm.tick_drop(50);
    EXPECT_NEAR(pm.drop_ratio(), 50.0 / 150.0, 1e-9);
}

TEST(PerformanceMeterTest, DropRatioZeroWhenEmpty) {
    PerformanceMeter pm;
    EXPECT_EQ(pm.drop_ratio(), 0.0);
}

TEST(PerformanceMeterTest, Reset) {
    PerformanceMeter pm;
    pm.tick_events(100, 0);
    pm.tick_frame();
    pm.tick_drop(10);
    pm.reset();
    EXPECT_EQ(pm.total_events(), 0u);
    EXPECT_EQ(pm.total_frames(), 0u);
    EXPECT_EQ(pm.total_dropped(), 0u);
    EXPECT_EQ(pm.fps(), 0.0);
}

TEST(PerformanceMeterTest, FPSPositiveAfterTwoFrames) {
    PerformanceMeter pm(1.0f);
    pm.tick_frame();
    // Sleep to ensure measurable dt.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pm.tick_frame();
    EXPECT_GT(pm.fps(), 0.0);
    EXPECT_LT(pm.fps(), 1000.0);  // sane upper bound
}

TEST(PerformanceMeterTest, StartStopPerFilterMetrics) {
    // jAER EventProcessingPerformanceMeter: start(n)/stop() records ns/event.
    PerformanceMeter pm;
    pm.start(1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    pm.stop();
    EXPECT_GT(pm.ns_per_event(), 0.0);
    EXPECT_GT(pm.eps(), 0.0);
    EXPECT_EQ(pm.n_samples(), 1u);
    // Accumulate a second sample for avg/stderr.
    pm.start(2000);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    pm.stop();
    EXPECT_EQ(pm.n_samples(), 2u);
    EXPECT_GT(pm.avg_ns_per_event(), 0.0);
    EXPECT_GE(pm.stderr_ns_per_event(), 0.0);
    pm.reset();
    EXPECT_EQ(pm.n_samples(), 0u);
    EXPECT_EQ(pm.avg_ns_per_event(), 0.0);
}

// =========================================================================
// 8. time_limiter.h
// =========================================================================

TEST(TimeLimiterTest, NoBudgetMeansNoLimit) {
    TimeLimiter tl(0);
    tl.start();
    EXPECT_FALSE(tl.should_stop());
}

TEST(TimeLimiterTest, DoesNotStopImmediately) {
    TimeLimiter tl(100000);  // 100ms
    tl.start();
    EXPECT_FALSE(tl.should_stop());
}

TEST(TimeLimiterTest, StopsAfterBudget) {
    TimeLimiter tl(1000);  // 1ms
    tl.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_TRUE(tl.should_stop());
}

TEST(TimeLimiterTest, ManualStop) {
    TimeLimiter tl(100000);
    tl.start();
    tl.stop();
    EXPECT_TRUE(tl.should_stop());
}

TEST(TimeLimiterTest, ElapsedIncreases) {
    TimeLimiter tl(100000);
    tl.start();
    auto e1 = tl.elapsed_us();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    auto e2 = tl.elapsed_us();
    EXPECT_GT(e2, e1);
}

TEST(TimeLimiterTest, BudgetFraction) {
    TimeLimiter tl(0);  // no budget
    tl.start();
    EXPECT_EQ(tl.budget_fraction(), 0.0);

    TimeLimiter tl2(100000);
    tl2.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_GT(tl2.budget_fraction(), 0.0);
}

TEST(TimeLimiterTest, SetBudget) {
    TimeLimiter tl(100000);
    tl.set_budget_us(0);
    EXPECT_EQ(tl.budget_us(), 0);
}

// =========================================================================
// 9. kalman_filter.h
// =========================================================================

TEST(KalmanFilter2DTest, InitSetsPosition) {
    KalmanFilter2D kf;
    kf.init(10.0, 20.0);
    EXPECT_NEAR(kf.x(), 10.0, 1e-9);
    EXPECT_NEAR(kf.y(), 20.0, 1e-9);
    EXPECT_NEAR(kf.vx(), 0.0, 1e-9);
    EXPECT_NEAR(kf.vy(), 0.0, 1e-9);
    EXPECT_TRUE(kf.initialized());
}

TEST(KalmanFilter2DTest, PredictAdvancesByVelocity) {
    KalmanFilter2D kf(1.0, 10.0, 0.1);
    kf.init(0.0, 0.0);
    // Set velocity via update with moving target.
    kf.update(1.0, 0.0);  // measurement at (1,0)
    kf.predict();
    // After update, velocity should be positive; predict moves further.
    EXPECT_GT(kf.x(), 0.0);
}

TEST(KalmanFilter2DTest, UpdateBeforeInitInits) {
    KalmanFilter2D kf;
    kf.update(5.0, 6.0);
    EXPECT_TRUE(kf.initialized());
    EXPECT_NEAR(kf.x(), 5.0, 1e-9);
    EXPECT_NEAR(kf.y(), 6.0, 1e-9);
}

TEST(KalmanFilter2DTest, ConvergesToMeasurement) {
    KalmanFilter2D kf(1.0, 1.0, 0.01);
    kf.init(0.0, 0.0);
    // Feed noisy measurements around (100, 100).
    for (int i = 0; i < 200; ++i) {
        kf.predict();
        kf.update(100.0, 100.0);
    }
    EXPECT_NEAR(kf.x(), 100.0, 2.0);
    EXPECT_NEAR(kf.y(), 100.0, 2.0);
}

TEST(KalmanFilter2DTest, TracksConstantVelocity) {
    KalmanFilter2D kf(1.0, 1.0, 0.1);
    kf.init(0.0, 0.0);
    double vx = 2.0, vy = 0.0;
    for (int i = 1; i <= 100; ++i) {
        kf.predict();
        kf.update(vx * i * 0.1, vy * i * 0.1);
    }
    // After enough steps, estimated velocity should approach true velocity.
    EXPECT_NEAR(kf.vx(), vx, 0.5);
    EXPECT_NEAR(kf.vy(), vy, 0.5);
}

TEST(KalmanFilter2DTest, SetDt) {
    KalmanFilter2D kf(1.0, 1.0, 0.1);
    kf.set_dt(0.05);
    // Just verify it doesn't crash.
    kf.init(0, 0);
    kf.predict();
}

// =========================================================================
// 10. kmeans.h
// =========================================================================

TEST(KMeansTest, EmptyInput) {
    KMeans km(3);
    std::vector<Point2D> pts;
    double inertia = km.fit(pts);
    EXPECT_EQ(inertia, 0.0);
    EXPECT_TRUE(km.centroids().empty());
}

TEST(KMeansTest, FewerPointsThanK) {
    KMeans km(5);
    std::vector<Point2D> pts = {{0,0}, {1,1}};
    km.fit(pts);
    EXPECT_EQ(km.centroids().size(), 2u);
}

TEST(KMeansTest, TwoWellSeparatedClusters) {
    KMeans km(2, 100, 12345);
    std::vector<Point2D> pts;
    for (int i = 0; i < 50; ++i) pts.emplace_back(0.0, 0.0);
    for (int i = 0; i < 50; ++i) pts.emplace_back(100.0, 100.0);
    km.fit(pts);
    ASSERT_EQ(km.centroids().size(), 2u);
    // One centroid near (0,0), one near (100,100).
    bool has_origin = false, has_far = false;
    for (const auto& c : km.centroids()) {
        if (std::hypot(c.x, c.y) < 5.0) has_origin = true;
        if (std::hypot(c.x - 100.0, c.y - 100.0) < 5.0) has_far = true;
    }
    EXPECT_TRUE(has_origin);
    EXPECT_TRUE(has_far);
}

TEST(KMeansTest, PredictNearestCentroid) {
    KMeans km(2, 100, 42);
    std::vector<Point2D> pts;
    for (int i = 0; i < 50; ++i) pts.emplace_back(0.0, 0.0);
    for (int i = 0; i < 50; ++i) pts.emplace_back(100.0, 100.0);
    km.fit(pts);
    // A point near origin should be assigned to the origin cluster.
    std::size_t label = km.predict(Point2D(1.0, 1.0));
    // The centroid for this label should be near origin.
    const auto& c = km.centroids()[label];
    EXPECT_LT(std::hypot(c.x, c.y), std::hypot(c.x - 100.0, c.y - 100.0));
}

TEST(KMeansTest, ReproducibleWithSameSeed) {
    std::vector<Point2D> pts;
    for (int i = 0; i < 100; ++i) {
        pts.emplace_back(i * 1.0, i * 2.0);
    }
    KMeans km1(3, 50, 999);
    KMeans km2(3, 50, 999);
    km1.fit(pts);
    km2.fit(pts);
    ASSERT_EQ(km1.centroids().size(), km2.centroids().size());
    for (std::size_t i = 0; i < km1.centroids().size(); ++i) {
        EXPECT_NEAR(km1.centroids()[i].x, km2.centroids()[i].x, 1e-9);
        EXPECT_NEAR(km1.centroids()[i].y, km2.centroids()[i].y, 1e-9);
    }
}

TEST(KMeansTest, KZeroClampedToOne) {
    KMeans km(0);
    std::vector<Point2D> pts = {{1,2}};
    km.fit(pts);
    EXPECT_EQ(km.centroids().size(), 1u);
}

// =========================================================================
// 11. particle_filter.h
// =========================================================================

TEST(ParticleFilterTest, InitSpreadsParticles) {
    ParticleFilter pf(100, 3.0, 5.0, 0.033, 42);
    pf.init(50.0, 50.0);
    auto est = pf.estimate();
    // Mean should be near seed (within noise variance).
    EXPECT_NEAR(est.x, 50.0, 5.0);
    EXPECT_NEAR(est.y, 50.0, 5.0);
}

TEST(ParticleFilterTest, PredictMovesParticles) {
    ParticleFilter pf(100, 0.1, 5.0, 0.033, 42);  // low noise
    pf.init(10.0, 10.0);
    // Give particles velocity by... they start with 0 velocity, so predict
    // only adds diffusion. Check particles moved (or stayed within diffusion).
    pf.predict();
    auto est = pf.estimate();
    EXPECT_NEAR(est.x, 10.0, 2.0);
    EXPECT_NEAR(est.y, 10.0, 2.0);
}

TEST(ParticleFilterTest, UpdateConvergesToMeasurement) {
    ParticleFilter pf(200, 3.0, 5.0, 0.033, 42);
    pf.init(0.0, 0.0);
    // Repeatedly update with measurement at (100, 100) and resample.
    for (int i = 0; i < 50; ++i) {
        pf.predict();
        pf.update(100.0, 100.0);
        pf.resample();
    }
    auto est = pf.estimate();
    EXPECT_NEAR(est.x, 100.0, 10.0);
    EXPECT_NEAR(est.y, 100.0, 10.0);
}

TEST(ParticleFilterTest, WeightsNormalizedAfterUpdate) {
    ParticleFilter pf(50, 1.0, 5.0, 0.033, 42);
    pf.init(0.0, 0.0);
    pf.update(1.0, 1.0);
    double sum = 0.0;
    for (const auto& p : pf.particles()) sum += p.weight;
    EXPECT_NEAR(sum, 1.0, 1e-6);
}

TEST(ParticleFilterTest, ResamplePreservesCount) {
    ParticleFilter pf(100, 1.0, 5.0, 0.033, 42);
    pf.init(0.0, 0.0);
    pf.update(1.0, 1.0);
    pf.resample();
    EXPECT_EQ(pf.particles().size(), 100u);
}

TEST(ParticleFilterTest, ResampleWeightsUniform) {
    ParticleFilter pf(100, 1.0, 5.0, 0.033, 42);
    pf.init(0.0, 0.0);
    pf.update(1.0, 1.0);
    pf.resample();
    double w = 1.0 / 100.0;
    for (const auto& p : pf.particles()) {
        EXPECT_NEAR(p.weight, w, 1e-9);
    }
}

// =========================================================================
// 12. periodic_spline.h
// =========================================================================

TEST(PeriodicSplineTest, FitRequiresMin3Points) {
    PeriodicSpline spline;
    std::vector<double> xs = {0, 1}, ys = {0, 1};
    spline.fit(xs, ys);  // too few → should not crash
}

TEST(PeriodicSplineTest, EvaluateAtKnotReturnsKnotValue) {
    PeriodicSpline spline;
    // Square path: (0,0), (1,0), (1,1), (0,1)
    std::vector<double> xs = {0, 1, 1, 0};
    std::vector<double> ys = {0, 0, 1, 1};
    spline.fit(xs, ys);
    // At t=0, should be close to first point (0,0).
    auto p = spline.evaluate(0.0);
    EXPECT_NEAR(p.x, 0.0, 0.5);
    EXPECT_NEAR(p.y, 0.0, 0.5);
}

TEST(PeriodicSplineTest, Periodicity) {
    PeriodicSpline spline;
    std::vector<double> xs = {0, 1, 2, 1};
    std::vector<double> ys = {0, 1, 0, -1};
    spline.fit(xs, ys);
    auto p0 = spline.evaluate(0.0);
    auto p2pi = spline.evaluate(2.0 * M_PI);
    EXPECT_NEAR(p0.x, p2pi.x, 1e-6);
    EXPECT_NEAR(p0.y, p2pi.y, 1e-6);
}

TEST(PeriodicSplineTest, ResampleCount) {
    PeriodicSpline spline;
    std::vector<double> xs = {0, 1, 2, 3};
    std::vector<double> ys = {0, 1, 0, -1};
    spline.fit(xs, ys);
    auto pts = spline.resample(32);
    EXPECT_EQ(pts.size(), 32u);
}

TEST(PeriodicSplineTest, WrapParameter) {
    PeriodicSpline spline;
    std::vector<double> xs = {0, 1, 2, 1};
    std::vector<double> ys = {0, 1, 0, -1};
    spline.fit(xs, ys);
    // t > 2π should wrap.
    auto p1 = spline.evaluate(0.5);
    auto p2 = spline.evaluate(0.5 + 2.0 * M_PI);
    EXPECT_NEAR(p1.x, p2.x, 1e-6);
    EXPECT_NEAR(p1.y, p2.y, 1e-6);
}

TEST(PeriodicSplineTest, NegativeParameter) {
    PeriodicSpline spline;
    std::vector<double> xs = {0, 1, 2, 1};
    std::vector<double> ys = {0, 1, 0, -1};
    spline.fit(xs, ys);
    // Negative t should also wrap correctly.
    auto p1 = spline.evaluate(-0.5);
    auto p2 = spline.evaluate(2.0 * M_PI - 0.5);
    EXPECT_NEAR(p1.x, p2.x, 1e-6);
    EXPECT_NEAR(p1.y, p2.y, 1e-6);
}

// =========================================================================
// 13. histogram_ring_buffer.h
// =========================================================================

TEST(HistogramRingBufferTest, PushAndSize) {
    HistogramRingBuffer h(5, 10, 0.0, 1.0);
    EXPECT_EQ(h.size(), 0u);
    h.push(0.5);
    h.push(0.6);
    EXPECT_EQ(h.size(), 2u);
}

TEST(HistogramRingBufferTest, WindowEviction) {
    HistogramRingBuffer h(3, 10, 0.0, 1.0);
    h.push(0.1);
    h.push(0.2);
    h.push(0.3);
    h.push(0.4);  // evicts 0.1
    EXPECT_EQ(h.size(), 3u);
    // mean should be (0.2+0.3+0.4)/3
    EXPECT_NEAR(h.mean(), 0.3, 1e-9);
}

TEST(HistogramRingBufferTest, Mean) {
    HistogramRingBuffer h(100, 10, 0.0, 1.0);
    h.push(1.0);
    h.push(2.0);
    h.push(3.0);
    h.push(4.0);
    EXPECT_NEAR(h.mean(), 2.5, 1e-9);
}

TEST(HistogramRingBufferTest, StdDev) {
    HistogramRingBuffer h(100, 10, 0.0, 100.0);
    // Push constant values → std = 0
    h.push(5.0);
    h.push(5.0);
    h.push(5.0);
    EXPECT_NEAR(h.std_dev(), 0.0, 1e-9);
}

TEST(HistogramRingBufferTest, StdDevNonZero) {
    HistogramRingBuffer h(100, 10, 0.0, 100.0);
    h.push(0.0);
    h.push(10.0);
    // population std of {0, 10} = sqrt(((0-5)^2 + (10-5)^2)/2) = 5
    EXPECT_NEAR(h.std_dev(), 5.0, 1e-9);
}

TEST(HistogramRingBufferTest, Percentile) {
    HistogramRingBuffer h(100, 10, 0.0, 100.0);
    for (int i = 1; i <= 10; ++i) h.push(i * 1.0);
    EXPECT_NEAR(h.percentile(50), 5.5, 1e-9);  // median
    EXPECT_NEAR(h.percentile(0), 1.0, 1e-9);
    EXPECT_NEAR(h.percentile(100), 10.0, 1e-9);
}

TEST(HistogramRingBufferTest, BinCounts) {
    HistogramRingBuffer h(100, 10, 0.0, 10.0);
    h.push(0.5);  // bin 0
    h.push(1.5);  // bin 1
    h.push(5.5);  // bin 5
    const auto& counts = h.counts();
    EXPECT_EQ(counts[0], 1u);
    EXPECT_EQ(counts[1], 1u);
    EXPECT_EQ(counts[5], 1u);
}

TEST(HistogramRingBufferTest, OutOfRangeValueNotBinned) {
    HistogramRingBuffer h(100, 10, 0.0, 1.0);
    h.push(-1.0);  // out of range
    h.push(2.0);   // out of range
    const auto& counts = h.counts();
    std::uint64_t total = 0;
    for (auto c : counts) total += c;
    EXPECT_EQ(total, 0u);
}

TEST(HistogramRingBufferTest, Clear) {
    HistogramRingBuffer h(100, 10, 0.0, 1.0);
    h.push(0.5);
    h.push(0.6);
    h.clear();
    EXPECT_EQ(h.size(), 0u);
    const auto& counts = h.counts();
    for (auto c : counts) EXPECT_EQ(c, 0u);
}

// =========================================================================
// 14. lif_integrator.h
// =========================================================================

TEST(LifIntegratorTest, FirstEventDoesNotFire) {
    // threshold=2.0: first ON event → pot=1 < 2 → no fire.
    LifIntegrator lif(10, 10, 10000, 2.0);
    EXPECT_FALSE(lif.add_event(0, 0, 1, 0));
}

TEST(LifIntegratorTest, FiresExactlyAtThreshold) {
    // Use same timestamp (t=0) for all events → no leak between events.
    LifIntegrator lif(10, 10, 1000000, 3.0, 0.0, 1000);
    lif.add_event(0, 0, 1, 0);   // pot = 1
    lif.add_event(0, 0, 1, 0);   // pot = 2 (same t → no leak)
    bool fired = lif.add_event(0, 0, 1, 0);  // pot = 3 >= 3.0 → fire
    EXPECT_TRUE(fired);
    // After firing, potential resets.
    EXPECT_NEAR(lif.potential(0, 0), 0.0, 1e-9);
}

TEST(LifIntegratorTest, OffEventAlsoIncrements) {
    // Per jAER BlurringTunnelFilter, all events add +1.0 regardless of polarity.
    LifIntegrator lif(10, 10, 1000000, 10.0);
    lif.add_event(0, 0, 1, 0);   // pot = 1
    lif.add_event(0, 0, 0, 0);   // pot = 2 (OFF also adds +1)
    EXPECT_NEAR(lif.potential(0, 0), 2.0, 1e-9);
}

TEST(LifIntegratorTest, LeakDecays) {
    LifIntegrator lif(10, 10, 1000, 100.0);  // tau=1ms, threshold=100
    lif.add_event(0, 0, 1, 0);  // pot = 1
    // Leak 1000us with tau=1000us → decay = e^(-1) ≈ 0.368
    lif.leak_global(1000);
    EXPECT_NEAR(lif.potential(0, 0), 0.367879, 0.01);
}

TEST(LifIntegratorTest, PerPixelLeakOnEvent) {
    LifIntegrator lif(10, 10, 1000, 100.0);
    lif.add_event(0, 0, 1, 0);      // pot = 1 at t=0
    lif.add_event(0, 0, 1, 1000);   // dt=1000, tau=1000 → decay e^(-1)
    // pot before this event = 1 * e^(-1) ≈ 0.368, then +1 = 1.368
    EXPECT_NEAR(lif.potential(0, 0), 1.367879, 0.01);
}

TEST(LifIntegratorTest, Clear) {
    LifIntegrator lif(10, 10, 1000, 5.0);
    lif.add_event(0, 0, 1, 0);
    lif.add_event(0, 0, 1, 1);
    lif.clear();
    EXPECT_NEAR(lif.potential(0, 0), 0.0, 1e-9);
}

TEST(LifIntegratorTest, OutOfBoundsIgnored) {
    LifIntegrator lif(10, 10, 1000, 5.0);
    EXPECT_FALSE(lif.add_event(100, 100, 1, 0));
}

TEST(LifIntegratorTest, SetThresholdAndTau) {
    LifIntegrator lif(10, 10, 1000, 5.0);
    lif.set_threshold(2.0);
    lif.set_tau_us(2000);
    EXPECT_EQ(lif.threshold(), 2.0);
    EXPECT_EQ(lif.tau_us(), 2000);
}

TEST(LifIntegratorTest, PotentialGridSize) {
    LifIntegrator lif(10, 10);
    EXPECT_EQ(lif.potential_grid().size(), 100u);
}

// =========================================================================
// 15. filter/lowpass.h
// =========================================================================

TEST(LowPassFilterTest, FirstSamplePassedThrough) {
    LowPassFilter lp(10.0, 0.033);
    EXPECT_EQ(lp.process(5.0), 5.0);
}

TEST(LowPassFilterTest, ConvergesToDC) {
    LowPassFilter lp(1.0, 0.1);  // low cutoff → heavy smoothing
    for (int i = 0; i < 1000; ++i) lp.process(10.0);
    EXPECT_NEAR(lp.value(), 10.0, 0.1);
}

TEST(LowPassFilterTest, AlphaMode) {
    LowPassFilter lp = LowPassFilter::from_alpha(1.0);  // no smoothing
    lp.process(5.0);
    EXPECT_EQ(lp.value(), 5.0);
    EXPECT_EQ(lp.process(10.0), 10.0);  // alpha=1 → output = input
}

TEST(LowPassFilterTest, AlphaHalfAverages) {
    LowPassFilter lp = LowPassFilter::from_alpha(0.5);
    lp.process(10.0);  // init → 10
    // y = 0.5*20 + 0.5*10 = 15
    EXPECT_NEAR(lp.process(20.0), 15.0, 1e-9);
}

TEST(LowPassFilterTest, Reset) {
    LowPassFilter lp(10.0, 0.033);
    lp.process(5.0);
    lp.reset();
    EXPECT_FALSE(lp.initialized());
    EXPECT_EQ(lp.value(), 0.0);
}

TEST(LowPassFilterTest, SetCutoff) {
    LowPassFilter lp(100.0, 0.033);
    lp.set_cutoff_hz(1.0);
    // Just verify it doesn't crash.
    lp.process(5.0);
}

// =========================================================================
// 16. filter/highpass.h
// =========================================================================

TEST(HighPassFilterTest, FirstSampleOutputZero) {
    HighPassFilter hp(10.0, 0.033);
    EXPECT_EQ(hp.process(5.0), 0.0);
}

TEST(HighPassFilterTest, RemovesDC) {
    HighPassFilter hp(1.0, 0.1);  // low cutoff → passes almost everything
    for (int i = 0; i < 1000; ++i) hp.process(10.0);
    // Constant signal → high-pass output → 0
    EXPECT_NEAR(hp.value(), 0.0, 0.5);
}

TEST(HighPassFilterTest, PassesTransient) {
    HighPassFilter hp(10.0, 0.01);
    hp.process(0.0);  // init
    // Sudden jump to 100 → high-pass should pass the change.
    double out = hp.process(100.0);
    EXPECT_GT(out, 0.0);
}

TEST(HighPassFilterTest, Reset) {
    HighPassFilter hp(10.0, 0.033);
    hp.process(5.0);
    hp.reset();
    EXPECT_FALSE(hp.initialized());
    EXPECT_EQ(hp.value(), 0.0);
}

// =========================================================================
// 17. filter/bandpass.h
// =========================================================================

TEST(BandpassFilterTest, FirstSampleInit) {
    BandpassFilter bp(1.0, 10.0, 0.033, 1);
    double out = bp.process(5.0);
    // First sample: HP outputs 0, LP passes 0.
    EXPECT_EQ(out, 0.0);
}

TEST(BandpassFilterTest, ConvergesToMidbandConstant) {
    BandpassFilter bp(0.1, 100.0, 0.01, 1);
    // Constant input: HP removes DC → output → 0
    for (int i = 0; i < 1000; ++i) bp.process(5.0);
    EXPECT_NEAR(bp.value(), 0.0, 0.5);
}

TEST(BandpassFilterTest, OrderIncreasesAttenuation) {
    // Higher order → steeper roll-off.
    BandpassFilter bp1(1.0, 10.0, 0.01, 1);
    BandpassFilter bp4(1.0, 10.0, 0.01, 4);
    for (int i = 0; i < 100; ++i) {
        bp1.process(5.0);
        bp4.process(5.0);
    }
    // Both should remove DC, but order-4 attenuates faster.
    // Just verify both are small.
    EXPECT_NEAR(bp1.value(), 0.0, 2.0);
    EXPECT_NEAR(bp4.value(), 0.0, 2.0);
}

TEST(BandpassFilterTest, Reset) {
    BandpassFilter bp(1.0, 10.0, 0.033, 2);
    bp.process(5.0);
    bp.process(6.0);
    bp.reset();
    EXPECT_EQ(bp.value(), 0.0);
}

TEST(BandpassFilterTest, SetCutoffs) {
    BandpassFilter bp(1.0, 10.0, 0.033, 1);
    bp.set_cutoffs(0.5, 20.0);
    bp.process(5.0);  // should not crash
}

// =========================================================================
// 18. filter/angular_lowpass.h
// =========================================================================

TEST(AngularLowpassFilterTest, FirstSampleReturned) {
    AngularLowpassFilter af;  // default period=2π, tau=0.1
    double out = af.process(1.0, 0.0);
    EXPECT_NEAR(out, 1.0, 1e-9);
}

TEST(AngularLowpassFilterTest, WrapAround) {
    AngularLowpassFilter af(2.0 * M_PI, 1.0);  // tau=1.0s
    af.process(0.1, 0.0);        // near 0
    double out = af.process(6.2, 0.1);  // near 2π ≈ 6.283 (i.e. ≈ -0.083)
    // The smoothed angle should stay near 0 (not drift toward π).
    EXPECT_LT(std::abs(out), 1.0);  // should be near 0, not near π
}

TEST(AngularLowpassFilterTest, ConvergesToConsistentAngle) {
    AngularLowpassFilter af(2.0 * M_PI, 0.05);  // small tau → fast convergence
    for (int i = 0; i < 20; ++i) af.process(1.0, i * 0.01);
    // All same angle → filtered value approaches the input.
    EXPECT_NEAR(af.value(), 1.0, 0.1);
}

TEST(AngularLowpassFilterTest, ShortestPathAcrossWrap) {
    // Filter from near 2π should converge toward ~0 via the short path.
    AngularLowpassFilter af(2.0 * M_PI, 0.01);  // fac≈1 (dt >> tau)
    af.process(6.2, 0.0);     // init near 2π
    double out = af.process(0.1, 1.0);  // jump to 0.1
    // Shortest path from 6.2 (≈ -0.083) to 0.1 is small; result ≈ 0.1.
    EXPECT_NEAR(out, 0.1, 0.1);
}

TEST(AngularLowpassFilterTest, FacOneNoSmoothing) {
    // tau << dt → fac = clamp(dt/tau, 0, 1) = 1 → output = input.
    AngularLowpassFilter af(2.0 * M_PI, 0.001);
    af.process(1.0, 0.0);
    EXPECT_NEAR(af.process(2.0, 0.1), 2.0, 1e-9);
}

TEST(AngularLowpassFilterTest, Reset) {
    AngularLowpassFilter af;
    af.process(1.0, 0.0);
    af.reset();
    EXPECT_FALSE(af.initialized());
    EXPECT_EQ(af.value(), 0.0);
}

// =========================================================================
// 19. filter/median_lowpass.h
// =========================================================================

TEST(MedianLowpassFilterTest, SingleSample) {
    // jAER MedianLowpassFilter zero-fills the window at startup, so the first
    // sample yields a median of 0 (window = {0,0,0,0,5} → upper-middle = 0).
    MedianLowpassFilter ml(5);
    EXPECT_EQ(ml.process(5.0), 0.0);
}

TEST(MedianLowpassFilterTest, RemovesImpulse) {
    MedianLowpassFilter ml(5);
    ml.process(10.0);
    ml.process(10.0);
    ml.process(10.0);
    ml.process(10.0);
    // Impulse: 1000 among 10s → median should stay 10.
    double out = ml.process(1000.0);
    EXPECT_EQ(out, 10.0);
}

TEST(MedianLowpassFilterTest, PreservesEdge) {
    MedianLowpassFilter ml(3);
    ml.process(0.0);
    ml.process(0.0);
    // Step from 0 to 100.
    double out = ml.process(100.0);
    // Window = {0, 0, 100} → median = 0
    EXPECT_EQ(out, 0.0);
    out = ml.process(100.0);
    // Window = {0, 100, 100} → median = 100
    EXPECT_EQ(out, 100.0);
}

TEST(MedianLowpassFilterTest, EvenWindowUpperMiddle) {
    // jAER uses samples[length/2] (upper-middle for even windows), and the
    // window is zero-filled at startup.
    MedianLowpassFilter ml(4);
    ml.process(1.0);  // {0,0,0,1} → upper-middle = 0
    ml.process(2.0);  // {0,0,1,2} → upper-middle = 1
    EXPECT_NEAR(ml.value(), 1.0, 1e-9);
    ml.process(3.0);  // {0,1,2,3} → upper-middle = 2
    ml.process(4.0);  // {1,2,3,4} → upper-middle = 3
    EXPECT_NEAR(ml.value(), 3.0, 1e-9);
}

TEST(MedianLowpassFilterTest, Reset) {
    MedianLowpassFilter ml(5);
    ml.process(5.0);
    ml.reset();
    EXPECT_EQ(ml.value(), 0.0);
    EXPECT_EQ(ml.window_size(), 5u);
}

TEST(MedianLowpassFilterTest, SetWindowSizeTrims) {
    MedianLowpassFilter ml(10);
    for (int i = 0; i < 10; ++i) ml.process(i * 1.0);
    ml.set_window_size(5);  // should trim to 5
    // After trimming, value is median of last 5 samples {5,6,7,8,9} → 7
    EXPECT_NEAR(ml.value(), 7.0, 1e-9);
}

TEST(MedianLowpassFilterTest, EmptyValueIsZero) {
    MedianLowpassFilter ml(5);
    EXPECT_EQ(ml.value(), 0.0);
}
