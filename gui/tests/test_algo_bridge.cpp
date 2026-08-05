// gui/tests/test_algo_bridge.cpp — AlgoBridge unit tests (design §3.11.2).
//
// Covers: registry completeness, find_or_create idempotency, create()
// freshness, find_live() visibility, the flood guard (4 consecutive capped
// batches auto-disable the instance), and set_param/get_param round-trip.
// All events are synthetic — no real camera or file I/O.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo_bridge/algo_bridge.h"

using gui::AlgoBridge;
using gui::AlgoInstance;
using Metavision::EventCD;

namespace {

// Builds a synthetic batch of @p n EventCD events with monotonically
// increasing timestamps and in-bounds coordinates for a 1280x720 sensor.
std::vector<EventCD> make_events(std::size_t n, int w = 1280, int h = 720) {
    std::vector<EventCD> evs(n);
    for (std::size_t i = 0; i < n; ++i) {
        evs[i].x = static_cast<uint16_t>(i % w);
        evs[i].y = static_cast<uint16_t>((i / w) % h);
        evs[i].p = (i % 2) ? 1 : 0;
        evs[i].t = static_cast<Metavision::timestamp>(i);
    }
    return evs;
}

} // namespace

// ---------------------------------------------------------------------------
// Registry completeness (§3.11.2: 28 self + 8 openEB). noise_filter was
// removed in v1.0.9 (now a stackable preprocessing stage); sensor_self_test
// was added in §4.4.8; intrinsic_calibration was removed (now a Tools-menu
// wizard, not a registered algo); perspective_undistort was removed (audit
// §三-B8, superseded by the preproc undistort stage); the 22 unreachable
// OpenEB frame/preproc/util/roi_mask/adaptive_rate_split registrations were
// removed (audit §三-B6/7), leaving the 8 FilterChain event-transform stages
// (roi_filter, polarity_filter, polarity_invert, flip_x, flip_y, rotate,
// transpose, rescale). Live registry: 28 self-developed + 8 OpenEB = 36.
// ---------------------------------------------------------------------------
TEST(AlgoBridgeRegistry, ListsAllRegisteredAlgos) {
    AlgoBridge bridge;
    const auto algos = bridge.list_algos();
    EXPECT_EQ(algos.size(), 36u);

    std::size_t self_count = 0, openeb_count = 0;
    for (const auto& a : algos) {
        if (a.source == "self") ++self_count;
        else if (a.source == "openeb") ++openeb_count;
    }
    EXPECT_EQ(self_count, 28u);
    EXPECT_EQ(openeb_count, 8u);
}

TEST(AlgoBridgeRegistry, KeyNamesPresent) {
    AlgoBridge bridge;
    // Self-developed key algorithms.
    EXPECT_NE(bridge.find("hot_pixel_filter"), nullptr);
    EXPECT_NE(bridge.find("event_to_video"), nullptr);
    EXPECT_NE(bridge.find("object_tracker"), nullptr);
    EXPECT_NE(bridge.find("hough_line"), nullptr);
    EXPECT_NE(bridge.find("time_surface"), nullptr);
    EXPECT_NE(bridge.find("blob_detector"), nullptr);
    EXPECT_NE(bridge.find("sensor_self_test"), nullptr);
    // OpenEB event-transform stages (handled by FilterChain).
    EXPECT_NE(bridge.find("roi_filter"), nullptr);
    EXPECT_NE(bridge.find("flip_x"), nullptr);
    EXPECT_NE(bridge.find("polarity_filter"), nullptr);
}

TEST(AlgoBridgeRegistry, RemovedRegistrationsAreGone) {
    AlgoBridge bridge;
    // Audit §三-B6/7: unreachable OpenEB backends removed.
    EXPECT_EQ(bridge.find("frame_integration"), nullptr);
    EXPECT_EQ(bridge.find("preproc_diff"), nullptr);
    EXPECT_EQ(bridge.find("preproc_hw_diff"), nullptr);
    EXPECT_EQ(bridge.find("util_rate_estimator"), nullptr);
    EXPECT_EQ(bridge.find("roi_mask"), nullptr);
    EXPECT_EQ(bridge.find("adaptive_rate_split"), nullptr);
    // Audit §三-B8: superseded by the preproc undistort stage.
    EXPECT_EQ(bridge.find("perspective_undistort"), nullptr);
}

TEST(AlgoBridgeRegistry, NoiseFilterRemovedInV1_0_9) {
    AlgoBridge bridge;
    // noise_filter is now a stackable preprocessing stage, not a standalone
    // algorithm, so it must not be in the registry.
    EXPECT_EQ(bridge.find("noise_filter"), nullptr);
    // Unknown name returns nullptr.
    EXPECT_EQ(bridge.find("does_not_exist_algo"), nullptr);
}

TEST(AlgoBridgeRegistry, IntrinsicCalibrationRemovedIsNowToolsWizard) {
    // intrinsic_calibration is now a Tools-menu wizard (CalibrationWizard),
    // not a registered algorithm, so it must not be in the registry.
    // Undistortion is available as a stackable preprocessing checkbox in the
    // Algorithms panel (preproc_undistort_enabled / preproc_undistort_path).
    AlgoBridge bridge;
    EXPECT_EQ(bridge.find("intrinsic_calibration"), nullptr);
}

TEST(AlgoBridgeRegistry, EventToVideoIsRegistered) {
    AlgoBridge bridge;
    const auto* info = bridge.find("event_to_video");
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(info->source, "self");
    EXPECT_FALSE(info->params.empty());
}

// ---------------------------------------------------------------------------
// find_or_create() idempotency / create() freshness / find_live() visibility.
// ---------------------------------------------------------------------------
TEST(AlgoBridgeInstances, FindOrCreateIsIdempotent) {
    AlgoBridge bridge;
    auto a = bridge.find_or_create("hot_pixel_filter");
    auto b = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a.get(), b.get());  // same underlying instance
    EXPECT_EQ(a, b);              // shared_ptr equality
}

TEST(AlgoBridgeInstances, CreateReturnsFreshInstanceEachCall) {
    AlgoBridge bridge;
    auto a = bridge.create("hot_pixel_filter");
    auto b = bridge.create("hot_pixel_filter");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_NE(a.get(), b.get());  // distinct instances
}

TEST(AlgoBridgeInstances, FindLiveBeforeAndAfterCreate) {
    AlgoBridge bridge;
    EXPECT_EQ(bridge.find_live("hot_pixel_filter"), nullptr);
    auto inst = bridge.create("hot_pixel_filter");
    (void)inst;
    EXPECT_NE(bridge.find_live("hot_pixel_filter"), nullptr);
    EXPECT_EQ(bridge.find_live("object_tracker"), nullptr);
}

TEST(AlgoBridgeInstances, ListLiveReturnsAllLiveInstances) {
    AlgoBridge bridge;
    // Hold the shared_ptrs alive — AlgoBridge stores weak_ptrs, so the live
    // instances expire if the returned shared_ptrs are discarded.
    auto a = bridge.create("hot_pixel_filter");
    auto b = bridge.create("object_tracker");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    auto live = bridge.list_live();
    EXPECT_EQ(live.size(), 2u);
}

TEST(AlgoBridgeInstances, FindLivePrunesExpiredEntries) {
    AlgoBridge bridge;
    {
        auto inst = bridge.create("hot_pixel_filter");
        (void)inst;
        EXPECT_NE(bridge.find_live("hot_pixel_filter"), nullptr);
    }  // inst destroyed here
    // After the shared_ptr is released, find_live returns nullptr and prunes.
    EXPECT_EQ(bridge.find_live("hot_pixel_filter"), nullptr);
}

TEST(AlgoBridgeInstances, CreateUnknownReturnsNull) {
    AlgoBridge bridge;
    EXPECT_EQ(bridge.create("not_a_real_algo"), nullptr);
    EXPECT_EQ(bridge.find_or_create("not_a_real_algo"), nullptr);
}

// ---------------------------------------------------------------------------
// set_param / get_param round-trip.
// ---------------------------------------------------------------------------
TEST(AlgoBridgeInstances, ParamRoundTrip) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    inst->set_param("fpn_target_rate_hz", "250");
    inst->set_param("learning_window_s", "10.0");
    EXPECT_EQ(inst->get_param("fpn_target_rate_hz"), "250");
    EXPECT_EQ(inst->get_param("learning_window_s"), "10.0");
    // Unknown key returns an empty string.
    EXPECT_EQ(inst->get_param("no_such_key"), "");
}

TEST(AlgoBridgeInstances, DefaultsAppliedAtConstruction) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    // The default fpn_target_rate_hz declared in the registry is "100".
    EXPECT_EQ(inst->get_param("fpn_target_rate_hz"), "100");
    // n_sigma was removed from the registry (algo marks it unused, audit
    // §三-31) — it is now an unknown key and returns an empty string.
    EXPECT_EQ(inst->get_param("n_sigma"), "");
}

TEST(AlgoBridgeInstances, EnableDisableState) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    EXPECT_FALSE(inst->is_enabled());
    inst->set_enabled(true);
    EXPECT_TRUE(inst->is_enabled());
    inst->set_enabled(false);
    EXPECT_FALSE(inst->is_enabled());
}

// ---------------------------------------------------------------------------
// Flood guard (design §5.6.7): kMaxBatchEvents = 50000, kFloodStrikes = 4.
// Four consecutive batches each exceeding the cap auto-disable the instance.
// ---------------------------------------------------------------------------
TEST(AlgoBridgeFloodGuard, OverloadsAfterFourConsecutiveCappedBatches) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);
    ASSERT_FALSE(inst->is_overloaded());

    // Each batch exceeds kMaxBatchEvents (50000) → capped + strike counted.
    auto batch = make_events(60000);
    EventCD* data = batch.data();
    for (int i = 0; i < 3; ++i) {
        inst->push_events(data, data + batch.size());
        EXPECT_FALSE(inst->is_overloaded())
            << "should not be overloaded after strike " << (i + 1);
    }
    // The 4th consecutive capped batch trips the guard.
    inst->push_events(data, data + batch.size());
    EXPECT_TRUE(inst->is_overloaded());
    EXPECT_FALSE(inst->is_enabled());  // overloaded implies disabled

    // clear_overload() resets the state.
    inst->clear_overload();
    EXPECT_FALSE(inst->is_overloaded());
}

TEST(AlgoBridgeFloodGuard, NonCappedBatchResetsStrikeCounter) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    auto big = make_events(60000);   // capped (strike)
    auto small = make_events(1000);  // not capped (resets strikes)
    EventCD* d = big.data();
    EventCD* s = small.data();

    inst->push_events(d, d + big.size());    // strike 1
    inst->push_events(d, d + big.size());    // strike 2
    inst->push_events(s, s + small.size());  // reset
    inst->push_events(d, d + big.size());    // strike 1 again
    EXPECT_FALSE(inst->is_overloaded());     // only 1 strike since reset
}

TEST(AlgoBridgeFloodGuard, OverloadedInstanceIgnoresFurtherEvents) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    auto batch = make_events(60000);
    EventCD* d = batch.data();
    for (int i = 0; i < 4; ++i) {
        inst->push_events(d, d + batch.size());
    }
    ASSERT_TRUE(inst->is_overloaded());
    // Pushing more events must be a no-op (no crash, still overloaded).
    inst->push_events(d, d + batch.size());
    EXPECT_TRUE(inst->is_overloaded());
}
