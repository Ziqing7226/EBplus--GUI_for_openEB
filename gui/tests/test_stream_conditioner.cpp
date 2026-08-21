// gui/tests/test_stream_conditioner.cpp — unit tests for the shared
// conditioning pipeline (ROI → value stages → noise filter → thin →
// undistort → flips). Uses synthetic batches; no camera required.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <vector>

#include <opencv2/core.hpp>

#include "algo_bridge/filter_chain.h"
#include "app/stream_conditioner.h"

namespace {

using gui::FilterChain;
using gui::StreamConditioner;
using Ev = Metavision::EventCD;

std::vector<Ev> make_batch(int x0, int y0, int x1, int y1,
                           Metavision::timestamp t0, int stride_us = 10) {
    std::vector<Ev> out;
    Metavision::timestamp t = t0;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            out.push_back(Ev(static_cast<std::uint16_t>(x),
                             static_cast<std::uint16_t>(y), 1, t));
            t += stride_us;
        }
    }
    return out;
}

TEST(StreamConditionerTest, InactivePassthroughIsZeroCopy) {
    StreamConditioner c;
    c.init(64, 48);
    auto batch = make_batch(0, 0, 64, 48, 1000);
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    EXPECT_EQ(p, batch.data());  // same span, no copy
    EXPECT_EQ(n, batch.size());
    EXPECT_FALSE(c.active());
    EXPECT_EQ(c.out_width(), 64);
    EXPECT_EQ(c.out_height(), 48);
}

TEST(StreamConditionerTest, RoiModeCropsAndShifts) {
    StreamConditioner c;
    c.init(64, 48);
    c.set_roi(true, 10, 8, 42, 24, /*roni=*/false);
    ASSERT_TRUE(c.active());
    EXPECT_EQ(c.out_width(), 32);
    EXPECT_EQ(c.out_height(), 16);
    auto batch = make_batch(0, 0, 64, 48, 1000);
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    ASSERT_EQ(n, 32u * 16u);
    for (std::size_t i = 0; i < n; ++i) {
        EXPECT_LT(p[i].x, 32) << "x not ROI-relative";
        EXPECT_LT(p[i].y, 16) << "y not ROI-relative";
    }
}

TEST(StreamConditionerTest, RoniDropsInsideKeepsAbsolute) {
    StreamConditioner c;
    c.init(64, 48);
    c.set_roi(true, 10, 8, 42, 24, /*roni=*/true);
    ASSERT_TRUE(c.active());
    // RONI keeps the full-frame coordinate system.
    EXPECT_EQ(c.out_width(), 64);
    EXPECT_EQ(c.out_height(), 48);
    auto batch = make_batch(0, 0, 64, 48, 1000);
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    EXPECT_EQ(n, batch.size() - 32u * 16u);
    bool saw_outside = false;
    for (std::size_t i = 0; i < n; ++i) {
        const bool inside = p[i].x >= 10 && p[i].x < 42 && p[i].y >= 8 &&
                            p[i].y < 24;
        EXPECT_FALSE(inside);
        if (p[i].x >= 42 || p[i].y >= 24) saw_outside = true;
    }
    EXPECT_TRUE(saw_outside);  // absolute coords preserved
}

TEST(StreamConditionerTest, NoiseFilterAppliedOnce) {
    StreamConditioner c;
    c.init(64, 48);
    // STCF default drops isolated events (no spatio-temporal support).
    c.set_param("preproc_filter_mode", "1");
    ASSERT_TRUE(c.set_param("preproc_filter_enabled", "true"));
    auto batch = make_batch(0, 0, 64, 48, 1000, /*stride_us=*/1000);
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    EXPECT_EQ(n, 0u);  // every event isolated (1s apart) → all dropped
}

TEST(StreamConditionerTest, DownsampleThinsOddCoordinates) {
    StreamConditioner c;
    c.init(64, 48);
    ASSERT_TRUE(c.set_param("preproc_downsample", "true"));
    auto batch = make_batch(0, 0, 64, 48, 1000);
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    EXPECT_EQ(n, 32u * 24u);  // only even-even survive
    for (std::size_t i = 0; i < n; ++i) {
        EXPECT_EQ(p[i].x % 2, 0);
        EXPECT_EQ(p[i].y % 2, 0);
        // coordinates NOT halved — halving is per-consumer
    }
}

TEST(StreamConditionerTest, ValueStagesRunBeforeFilter) {
    StreamConditioner c;
    FilterChain fc;
    c.init(64, 48);
    c.set_filter_chain(&fc);
    // Polarity filter keeps ON(1) only — the OFF events must never reach the
    // noise filter (it would otherwise see them).
    fc.set_stage_enabled("polarity_filter", true);
    fc.set_stage_param("polarity_filter", "polarity", "1");
    c.set_param("preproc_filter_mode", "1");  // STCF
    ASSERT_TRUE(c.set_param("preproc_filter_enabled", "true"));

    // Correlated OFF pairs (support each other) + isolated ON events: if the
    // polarity stage ran AFTER the filter, the OFF pairs would survive STCF
    // and then be dropped by polarity → 0 out. With value stages first, the
    // OFF pairs never enter the filter; the isolated ONs are dropped by it.
    std::vector<Ev> batch;
    batch.push_back(Ev(10, 10, 0, 2000));
    batch.push_back(Ev(11, 10, 0, 2050));
    batch.push_back(Ev(30, 30, 1, 5000));
    batch.push_back(Ev(50, 50, 1, 6000));
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    EXPECT_EQ(n, 0u);
    EXPECT_TRUE(fc.has_enabled(FilterChain::Group::ValueOnly));
    EXPECT_FALSE(fc.has_enabled(FilterChain::Group::GeometryOnly));
}

TEST(StreamConditionerTest, FlipsMirrorWithinOutputGeometry) {
    StreamConditioner c;
    FilterChain fc;
    c.init(64, 48);
    c.set_filter_chain(&fc);
    fc.set_stage_enabled("flip_x", true);

    // Full frame: mirror within sensor dims.
    std::vector<Ev> batch;
    batch.push_back(Ev(3, 7, 1, 1000));
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    ASSERT_EQ(n, 1u);
    EXPECT_EQ(p[0].x, 63 - 3);  // mirrored within 64
    EXPECT_EQ(p[0].y, 7);

    // ROI mode: mirror within ROI dims (geometry follows the coordinate
    // system — the RONI/flip mismatch class of bug).
    c.set_roi(true, 8, 4, 40, 28, /*roni=*/false);
    std::vector<Ev> batch2;
    batch2.push_back(Ev(8, 4, 1, 100'000));   // ROI-relative → (0, 0)
    batch2.push_back(Ev(15, 9, 1, 100'010));  // ROI-relative → (7, 5)
    auto [p2, n2] = c.apply(batch2.data(), batch2.data() + batch2.size());
    ASSERT_EQ(n2, 2u);
    EXPECT_EQ(p2[0].x, 31);  // mirrored within the 32-wide ROI frame
    EXPECT_EQ(p2[1].x, 31 - 7);
    EXPECT_EQ(p2[1].y, 5);   // flip_x only

    // RONI: absolute coordinates → mirror within sensor dims again.
    c.set_roi(true, 8, 4, 40, 28, /*roni=*/true);
    std::vector<Ev> batch3;
    batch3.push_back(Ev(3, 7, 1, 200'000));  // outside the rect → kept
    auto [p3, n3] = c.apply(batch3.data(), batch3.data() + batch3.size());
    ASSERT_EQ(n3, 1u);
    EXPECT_EQ(p3[0].x, 63 - 3);  // mirrored within 64, not 32
}

// Writes a minimal calibration YAML so the undistort LUT builds. Zero
// distortion → identity map; nonzero k1 → real correction.
std::string write_calib_yml(int w, int h, double k1) {
    const std::string path = ::testing::TempDir() + "sc_identity.yml";
    std::ofstream f(path);
    f << "%YAML:1.0\n---\n";
    f << "image_width: " << w << "\nimage_height: " << h << "\n";
    f << "camera_matrix:\n   rows: 3\n   cols: 3\n   dt: d\n";
    f << "   data: [" << w / 2.0 << ", 0., " << w / 2.0 << ",\n";
    f << "           0., " << h / 2.0 << ", " << h / 2.0 << ",\n";
    f << "           0., 0., 1.]\n";
    f << "distortion_coefficients:\n   rows: 1\n   cols: 5\n   dt: d\n";
    f << "   data: [" << k1 << ", 0., 0., 0., 0.]\n";
    return path;
}

std::string write_identity_yml(int w, int h) { return write_calib_yml(w, h, 0.0); }

TEST(StreamConditionerTest, UndistortIdentityKeepsCoordinates) {
    StreamConditioner c;
    c.init(64, 48);
    ASSERT_TRUE(c.set_param("preproc_undistort_path", write_identity_yml(64, 48)));
    ASSERT_TRUE(c.set_param("preproc_undistort_enabled", "true"));
    ASSERT_TRUE(c.active());
    auto batch = make_batch(0, 0, 64, 48, 1000);
    auto [p, n] = c.apply(batch.data(), batch.data() + batch.size());
    EXPECT_EQ(n, batch.size());
    for (std::size_t i = 0; i < n; ++i) {
        EXPECT_EQ(p[i].x, batch[i].x);  // identity distortion → unchanged
        EXPECT_EQ(p[i].y, batch[i].y);
    }
}

// Regression (review 2026-08-21): RONI keeps ABSOLUTE coordinates, so the
// undistort LUT must use the unadjusted sensor K — with the rect origin
// subtracted from cx/cy the corrected positions were systematically shifted.
TEST(StreamConditionerTest, RoniUndistortUsesSensorK) {
    const std::string yml = write_calib_yml(64, 48, -0.25);
    std::vector<Ev> batch{Ev(36, 28, 1, 1000)};  // near center (survives the
    // distortion correction), outside the RONI rect below

    // Reference: no ROI at all.
    StreamConditioner plain;
    plain.init(64, 48);
    plain.set_param("preproc_undistort_path", yml);
    plain.set_param("preproc_undistort_enabled", "true");
    auto [rp, rn] = plain.apply(batch.data(), batch.data() + batch.size());
    ASSERT_EQ(rn, 1u);

    // RONI (rect far from the event, event kept at absolute coords): the
    // corrected position must be IDENTICAL to the no-ROI reference.
    StreamConditioner roni;
    roni.init(64, 48);
    roni.set_param("preproc_undistort_path", yml);
    roni.set_param("preproc_undistort_enabled", "true");
    roni.set_roi(true, 10, 8, 30, 20, /*roni=*/true);
    auto [np, nn] = roni.apply(batch.data(), batch.data() + batch.size());
    ASSERT_EQ(nn, 1u);
    EXPECT_EQ(np[0].x, rp[0].x);
    EXPECT_EQ(np[0].y, rp[0].y);
}

TEST(StreamConditionerTest, ResetTemporalAndParamRoundTrip) {
    StreamConditioner c;
    c.init(64, 48);
    EXPECT_EQ(c.get_param("preproc_filter_mode"), "");  // not set yet
    ASSERT_TRUE(c.set_param("preproc_filter_mode", "8"));
    ASSERT_TRUE(c.set_param("preproc_filter_enabled", "true"));
    ASSERT_TRUE(c.set_param("preproc_downsample", "true"));
    EXPECT_EQ(c.get_param("preproc_filter_mode"), "8");
    EXPECT_EQ(c.get_param("preproc_filter_enabled"), "true");
    EXPECT_EQ(c.get_param("preproc_downsample"), "true");
    EXPECT_EQ(c.get_param("preproc_bogus"), "");
    EXPECT_FALSE(c.set_param("not_a_preproc_key", "1"));
    c.reset_temporal();  // must not crash / reset params
    EXPECT_EQ(c.get_param("preproc_filter_mode"), "8");
}

} // namespace
