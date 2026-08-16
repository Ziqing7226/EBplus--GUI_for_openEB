// gui/tests/test_preproc_routing.cpp — preprocessing parameter routing
// regression tests.
//
// The three preprocessing stages (filter / downsample / undistort, one
// shared implementation gui::backend_detail::Preprocessor) must reach BOTH
// consumers from the single AlgorithmsPanel control:
//   1. every live self-developed algorithm instance, via
//      AlgoBridge::apply_global_preproc (forwarded immediately AND replayed
//      from preproc_cache_ into instances created later — BUG-R4);
//   2. the display stream, via FramePipeline::set_display_preproc_param
//      (observable through display_preproc_active()).
// These tests pin both routes so a future refactor cannot silently break
// one of them. All events are synthetic — no camera, no display.

#include <gtest/gtest.h>

#include <memory>

#include "algo_bridge/algo_bridge.h"
#include "app/frame_pipeline.h"

using gui::AlgoBridge;
using gui::FramePipeline;

// Route 1a: apply_global_preproc forwards to an already-live self-developed
// instance (the backend's Preprocessor answers get_param).
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

// Route 2: display-path parameters activate the FramePipeline's own
// Preprocessor (same keys, same semantics as the algorithm instances).
TEST(PreprocRouting, DisplayPipelineReceivesSameParams) {
    FramePipeline fp;
    EXPECT_FALSE(fp.display_preproc_active());

    fp.set_display_preproc_param("preproc_filter_enabled", "true");
    EXPECT_TRUE(fp.display_preproc_active());

    fp.set_display_preproc_param("preproc_filter_enabled", "false");
    EXPECT_FALSE(fp.display_preproc_active());

    fp.set_display_preproc_param("preproc_downsample", "true");
    EXPECT_TRUE(fp.display_preproc_active());
}
