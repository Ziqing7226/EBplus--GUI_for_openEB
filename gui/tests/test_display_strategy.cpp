// gui/tests/test_display_strategy.cpp — display strategy apply() correctness
// (design §3.11.2).
//
// Exercises each of the four IDisplayStrategy implementations through the
// public AlgoInstance::apply_strategy() entry point with synthetic AlgoResult
// data. Strategies are tested in isolation: Passive leaves the frame untouched,
// Overlay draws primitives onto it, Replace substitutes it with the algo frame,
// Standalone routes elsewhere (no window => frame unchanged). All data is
// synthetic; no real camera or widgets beyond a bare QApplication are needed.

#include <gtest/gtest.h>

#include <QApplication>
#include <QHash>
#include <QImage>
#include <QPointer>

#include <opencv2/core.hpp>

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/algo_bridge.h"
#include "display/display_strategy.h"
#include "display/frame_annotator.h"
#include "widgets/algo_window.h"

using gui::AlgoBridge;
using gui::AlgoResult;
using gui::DisplayContext;
using gui::FrameAnnotator;
using gui::OverlayBox;
using gui::OverlayLine;
using gui::OverlayPoint;

namespace {

DisplayContext make_ctx(FrameAnnotator& annotator,
                        QHash<std::string, QPointer<gui::AlgoWindow>>& windows) {
    DisplayContext ctx;
    ctx.annotator = &annotator;
    ctx.algo_windows = &windows;
    ctx.xyt_display = nullptr;
    ctx.window = nullptr;
    ctx.camera = nullptr;
    ctx.instance = nullptr;
    return ctx;
}

} // namespace

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// Passive: no drawing. With no AlgoWindow open the strategy returns early and
// the frame is left byte-for-byte unchanged.
TEST(DisplayStrategyPassive, LeavesFrameUnchanged) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");  // Passive
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    QImage frame(40, 40, QImage::Format_RGB888);
    frame.fill(Qt::black);
    const QImage before = frame.copy();

    AlgoResult r;
    r.status = "passive status";

    QHash<std::string, QPointer<gui::AlgoWindow>> windows;
    FrameAnnotator annotator;
    auto ctx = make_ctx(annotator, windows);

    inst->apply_strategy(frame, r, ctx);
    EXPECT_EQ(frame, before);
}

// Overlay: boxes / lines / points are drawn onto the main frame.
TEST(DisplayStrategyOverlay, DrawsPrimitivesOntoFrame) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("object_tracker");  // Overlay
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    QImage frame(120, 120, QImage::Format_RGB888);
    frame.fill(Qt::black);
    const QImage before = frame.copy();

    AlgoResult r;
    OverlayBox box;
    box.x = 10; box.y = 10; box.w = 40; box.h = 40; box.id = 1;
    r.boxes.push_back(box);
    OverlayLine line;
    line.x1 = 0; line.y1 = 0; line.x2 = 100; line.y2 = 100;
    r.lines.push_back(line);
    OverlayPoint point;
    point.x = 60; point.y = 60;
    r.points.push_back(point);

    QHash<std::string, QPointer<gui::AlgoWindow>> windows;
    FrameAnnotator annotator;
    auto ctx = make_ctx(annotator, windows);

    inst->apply_strategy(frame, r, ctx);

    // The frame must have been modified.
    EXPECT_NE(frame, before);
    // A pixel on the box's left edge (x=10) should no longer be black.
    EXPECT_NE(frame.pixel(10, 30), before.pixel(10, 30));
    // A pixel on the diagonal line should have changed.
    EXPECT_NE(frame.pixel(50, 50), before.pixel(50, 50));
}

// Regression (P0, 2026-08-22): colored_events must actually paint on the
// display pipeline's Format_RGB888 frames. QImage::setPixel only supports
// 32-bit/indexed formats and silently no-ops on 24-bit RGB888 — orientation
// / direction coloring was therefore never visible. The annotator now
// writes RGB888 through the scanline pointer.
TEST(DisplayStrategyOverlay, ColoredEventsPaintOnRgb888) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("object_tracker");  // Overlay
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    QImage frame(80, 80, QImage::Format_RGB888);
    frame.fill(Qt::black);
    const QImage before = frame.copy();

    AlgoResult r;
    gui::ColoredEvent ce;
    ce.event.x = 40; ce.event.y = 40;
    ce.event.p = 1;
    ce.r = 255; ce.g = 0; ce.b = 0;
    r.colored_events.push_back(ce);

    QHash<std::string, QPointer<gui::AlgoWindow>> windows;
    FrameAnnotator annotator;
    auto ctx = make_ctx(annotator, windows);

    inst->apply_strategy(frame, r, ctx);
    EXPECT_NE(frame, before);
    // (40,40) must now be red (qRgb(255,0,0) = 0xFFFF0000).
    EXPECT_EQ(frame.pixel(40, 40), 0xFFFF0000);
}

// Overlay with no primitives and ROI off: frame unchanged (nothing to draw).
TEST(DisplayStrategyOverlay, EmptyResultLeavesFrameUnchanged) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("object_tracker");
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    QImage frame(60, 60, QImage::Format_RGB888);
    frame.fill(Qt::white);
    const QImage before = frame.copy();

    AlgoResult r;  // no boxes/lines/points
    QHash<std::string, QPointer<gui::AlgoWindow>> windows;
    FrameAnnotator annotator;
    auto ctx = make_ctx(annotator, windows);

    inst->apply_strategy(frame, r, ctx);
    EXPECT_EQ(frame, before);
}

// Replace: the main frame is substituted with the algorithm's output cv::Mat.
TEST(DisplayStrategyReplace, SubstitutesFrameWithAlgoOutput) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("background_mask");  // Replace
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    QImage frame(64, 64, QImage::Format_RGB888);
    frame.fill(Qt::black);

    // Distinct 3-channel BGR frame (gray 200). mat_to_qimage converts BGR->RGB.
    cv::Mat mat(32, 48, CV_8UC3, cv::Scalar(200, 200, 200));
    AlgoResult r;
    r.has_frame = true;
    r.frame = mat;

    QHash<std::string, QPointer<gui::AlgoWindow>> windows;
    FrameAnnotator annotator;
    auto ctx = make_ctx(annotator, windows);

    inst->apply_strategy(frame, r, ctx);

    EXPECT_EQ(frame.width(), 48);
    EXPECT_EQ(frame.height(), 32);
    // Gray 200 is unchanged by BGR->RGB; luminance should be 200.
    EXPECT_EQ(qGray(frame.pixel(10, 10)), 200);
}

// Replace with no frame: the main frame is left untouched.
TEST(DisplayStrategyReplace, NoFrameLeavesFrameUnchanged) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("background_mask");
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    QImage frame(50, 50, QImage::Format_RGB888);
    frame.fill(Qt::blue);
    const QImage before = frame.copy();

    AlgoResult r;  // has_frame == false
    QHash<std::string, QPointer<gui::AlgoWindow>> windows;
    FrameAnnotator annotator;
    auto ctx = make_ctx(annotator, windows);

    inst->apply_strategy(frame, r, ctx);
    EXPECT_EQ(frame, before);
}

// Standalone: routes to an AlgoWindow. With no window open it returns early,
// leaving the main frame unchanged and not crashing.
TEST(DisplayStrategyStandalone, NoWindowLeavesFrameUnchanged) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("time_surface");  // Standalone
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    QImage frame(40, 40, QImage::Format_RGB888);
    frame.fill(Qt::black);
    const QImage before = frame.copy();

    AlgoResult r;
    r.has_frame = true;
    r.frame = cv::Mat(20, 20, CV_8UC3, cv::Scalar(0, 255, 0));
    r.status = "standalone";

    QHash<std::string, QPointer<gui::AlgoWindow>> windows;  // empty
    FrameAnnotator annotator;
    auto ctx = make_ctx(annotator, windows);

    inst->apply_strategy(frame, r, ctx);
    EXPECT_EQ(frame, before);
}

// mat_to_qimage: 1-channel gray and 3-channel BGR round-trip correctly.
TEST(DisplayStrategyHelpers, MatToQImageGrayAndBgr) {
    cv::Mat gray(10, 12, CV_8UC1, cv::Scalar(128));
    QImage qg = gui::mat_to_qimage(gray);
    EXPECT_EQ(qg.width(), 12);
    EXPECT_EQ(qg.height(), 10);
    EXPECT_EQ(qGray(qg.pixel(0, 0)), 128);

    cv::Mat bgr(5, 5, CV_8UC3, cv::Scalar(0, 0, 255));  // BGR red
    QImage qb = gui::mat_to_qimage(bgr);
    EXPECT_EQ(qb.width(), 5);
    // BGR(0,0,255) -> RGB(255,0,0): red channel high.
    EXPECT_EQ(qRed(qb.pixel(0, 0)), 255);
    EXPECT_EQ(qGreen(qb.pixel(0, 0)), 0);
    EXPECT_EQ(qBlue(qb.pixel(0, 0)), 0);
}

TEST(DisplayStrategyHelpers, MatToQimageEmptyMatReturnsNull) {
    cv::Mat empty;
    QImage q = gui::mat_to_qimage(empty);
    EXPECT_TRUE(q.isNull());
}
