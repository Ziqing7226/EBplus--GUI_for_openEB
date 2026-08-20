// gui/tests/test_preproc_routing.cpp — conditioning parameter routing
// regression tests.
//
// Since the 2026-08-19 rework, conditioning (noise filter / 1/4 thin /
// undistort) runs ONCE per event source in the shared StreamConditioner and
// every consumer — display + all algorithm instances — gets the same output
// span. What remains per instance is the coordinate HALVING convention
// (halving backends) plus round-trip storage of the conditioner keys so
// per-algorithm configs keep capturing the global state.
//
// These tests pin:
//   1. AlgoBridge::apply_global_preproc still forwards + replays the keys
//      into instances (round-trip storage, BUG-R4 cache replay);
//   2. StreamConditioner accepts the panel keys and activates;
//   3. the slim per-consumer Preprocessor halves coordinates only — events
//      keep passing through un-thinned when downsample is off, and halving
//      applies to every (pre-thinned, even-even) event.
// All events are synthetic — no camera, no display.

#include <gtest/gtest.h>

#include <memory>

#include "algo_bridge/algo_bridge.h"
#include "algo_bridge/backends/backend_common.h"
#include "app/frame_pipeline.h"
#include "app/stream_conditioner.h"

using gui::AlgoBridge;
using gui::FramePipeline;
using gui::StreamConditioner;
using gui::backend_detail::Preprocessor;
using gui::backend_detail::as_events;

// Route 1a: apply_global_preproc forwards to an already-live self-developed
// instance (the slim Preprocessor answers get_param from its round-trip
// storage).
TEST(PreprocRouting, BridgeForwardsToLiveInstances) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("time_surface");
    ASSERT_NE(inst, nullptr);

    bridge.apply_global_preproc("preproc_filter_enabled", "true");
    EXPECT_EQ(inst->get_param("preproc_filter_enabled"), "true");

    bridge.apply_global_preproc("preproc_downsample", "true");
    EXPECT_EQ(inst->get_param("preproc_downsample"), "true");
}

// Route 1b: instances created AFTER the global setting inherit it from
// preproc_cache_ (BUG-R4 — the cache replay in create_with_info).
TEST(PreprocRouting, BridgeCachedParamsInheritedByLaterInstances) {
    AlgoBridge bridge;
    bridge.apply_global_preproc("preproc_filter_enabled", "true");
    bridge.apply_global_preproc("preproc_downsample", "true");

    auto inst = bridge.find_or_create("time_surface");
    ASSERT_NE(inst, nullptr);
    EXPECT_EQ(inst->get_param("preproc_filter_enabled"), "true");
    EXPECT_EQ(inst->get_param("preproc_downsample"), "true");
}

// Route 2: the panel keys activate the SHARED conditioner (the display path
// is this same object now — FramePipeline no longer owns a Preprocessor).
TEST(PreprocRouting, ConditionerReceivesPanelParams) {
    StreamConditioner c;
    c.init(64, 48);
    EXPECT_FALSE(c.active());

    EXPECT_TRUE(c.set_param("preproc_filter_enabled", "true"));
    EXPECT_TRUE(c.active());

    EXPECT_TRUE(c.set_param("preproc_filter_enabled", "false"));
    EXPECT_FALSE(c.active());

    EXPECT_TRUE(c.set_param("preproc_downsample", "true"));
    EXPECT_TRUE(c.active());
}

// The per-consumer residual: halving only, and only when the consumer halves.
TEST(PreprocRouting, SlimPreprocessorHalvesOnly) {
    std::vector<Metavision::EventCD> batch;
    for (int y = 0; y < 8; y += 2) {          // pre-thinned (even-even)
        for (int x = 0; x < 8; x += 2) {
            batch.push_back(Metavision::EventCD(static_cast<std::uint16_t>(x),
                                                static_cast<std::uint16_t>(y),
                                                1, 1000));
        }
    }
    // Non-halving consumer: downsample on → still a passthrough (the shared
    // conditioner already thinned; nothing left to compute).
    {
        Preprocessor p;
        p.set_param("preproc_downsample", "true");
        EXPECT_FALSE(p.active());
        auto [out, n] = p.apply(as_events(batch.data()), batch.size());
        EXPECT_EQ(n, batch.size());
        EXPECT_EQ(out[0].x, 0);
    }
    // Halving consumer: coordinates shift right by one, every event kept.
    {
        Preprocessor p;
        p.halve_coords_ = true;
        p.set_param("preproc_downsample", "true");
        EXPECT_TRUE(p.active());
        auto [out, n] = p.apply(as_events(batch.data()), batch.size());
        EXPECT_EQ(n, batch.size());
        EXPECT_EQ(out[0].x, 0);
        EXPECT_EQ(out[1].x, 1);  // (2,0) -> (1,0)
        EXPECT_EQ(out[4].y, 1);  // (0,2) -> (0,1)
    }
    // Downsample off: passthrough even for halving consumers (factor 1 —
    // backends size themselves via preproc.factor()).
    {
        Preprocessor p;
        p.halve_coords_ = true;
        EXPECT_FALSE(p.active());
        EXPECT_EQ(p.factor(), 1);
        auto [out, n] = p.apply(as_events(batch.data()), batch.size());
        EXPECT_EQ(n, batch.size());
        EXPECT_EQ(out[1].x, 2);
    }
}
