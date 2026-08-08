// gui/display/display_strategy.cpp — concrete display strategies (design §3.5).
//
// The per-mode logic here is copied verbatim from the former switch branches
// of MainWindow::process_algo_results() (only the dispatch changed: member
// references annotator_/algo_windows_/camera_ and the `this` invokeMethod
// context are now routed through DisplayContext). All QMetaObject::invokeMethod
// queued calls are preserved exactly so widget touches stay marshalled to the
// GUI thread identically to the prior implementation.

#include "display/display_strategy.h"

#include <atomic>

#include <QColor>
#include <QLineF>
#include <QMetaObject>
#include <QPoint>
#include <QPointF>
#include <QPointer>
#include <QRect>
#include <QString>

#include <algorithm>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "algo_bridge/algo_backend.h"   // AlgoResult + Overlay* structs
#include "algo_bridge/algo_bridge.h"    // AlgoInstance, AlgoInfo
#include "app/camera_controller.h"      // CameraController
#include "display/event_display_widget.h"  // EventDisplayWidget
#include "display/frame_annotator.h"    // FrameAnnotator
#include "main_window.h"                // MainWindow (QObject invokeMethod context)
#include "widgets/algo_window.h"        // AlgoWindow

namespace gui {

// ---------------------------------------------------------------------------
// mat_to_qimage (moved from main_window.cpp anonymous namespace)
// ---------------------------------------------------------------------------

QImage mat_to_qimage(const cv::Mat& mat) {
    if (mat.empty()) return QImage();
    cv::Mat rgb;
    if (mat.channels() == 1) {
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
    } else if (mat.channels() == 3) {
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    } else {
        // Audit §五-G5: don't fail silently — a Replace-mode algorithm
        // producing an unsupported channel count would otherwise show a
        // black screen with no hint as to why. Throttled: first occurrence,
        // then every 300th, so a persistently-misbehaving algorithm doesn't
        // flood the log at display rate.
        static std::atomic<int> bad_channels{0};
        const int c = ++bad_channels;
        if (c == 1 || c % 300 == 0) {
            qWarning("mat_to_qimage: unsupported channel count %d (need 1 or 3) (x%d)",
                     mat.channels(), c);
        }
        return QImage();
    }
    return QImage(rgb.data, rgb.cols, rgb.rows,
                  static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
}

// ---------------------------------------------------------------------------
// PassiveStrategy
// ---------------------------------------------------------------------------

void PassiveStrategy::apply(QImage& /*frame*/, AlgoResult& result,
                            const AlgoInfo& info, DisplayContext& ctx) {
    // Passive algorithms (in-place event filters like noise_filter /
    // hot_pixel_filter) don't draw overlays or replace the frame, but if the
    // user opened an AlgoWindow for one we still update the status text so the
    // window doesn't stay stuck on "Waiting for events..." forever. The result
    // has already been pulled by the caller.
    auto wit = ctx.algo_windows->find(info.name);
    if (wit == ctx.algo_windows->end() || !wit.value()) return;
    if (!result.status.empty()) {
        QString text = QString::fromStdString(result.status);
        for (const auto& t : result.texts) {
            text += QStringLiteral("\n  ");
            text += QString::fromStdString(t.text);
        }
        QPointer<AlgoWindow> w = wit.value();
        QMetaObject::invokeMethod(ctx.window, [w, text]() {
            if (w) w->set_status_text(text);
        }, Qt::QueuedConnection);
    }
}

// ---------------------------------------------------------------------------
// OverlayStrategy
// ---------------------------------------------------------------------------

void OverlayStrategy::apply(QImage& frame, AlgoResult& r,
                            const AlgoInfo& info, DisplayContext& ctx) {
    // Convert AlgoResult overlay primitives into FrameAnnotator calls.
    // Boxes: tracked-object boxes with optional id.
    if (!r.boxes.empty()) {
        std::vector<FrameAnnotator::Box> boxes;
        boxes.reserve(r.boxes.size());
        for (const auto& b : r.boxes) {
            FrameAnnotator::Box box;
            box.rect = QRect(b.x, b.y, b.w, b.h);
            box.id = b.id;
            boxes.push_back(std::move(box));
        }
        ctx.annotator->draw_boxes(frame, boxes);
    }
    // Lines.
    if (!r.lines.empty()) {
        std::vector<QLineF> lines;
        lines.reserve(r.lines.size());
        for (const auto& l : r.lines) {
            lines.emplace_back(QPointF(l.x1, l.y1), QPointF(l.x2, l.y2));
        }
        ctx.annotator->draw_lines(frame, lines);
    }
    // Points.
    if (!r.points.empty()) {
        std::vector<QPointF> pts;
        pts.reserve(r.points.size());
        for (const auto& p : r.points) {
            pts.emplace_back(p.x, p.y);
        }
        ctx.annotator->draw_points(frame, pts);
    }
    // Colored points (optical-flow HSV visualization).
    if (!r.colored_points.empty()) {
        std::vector<std::pair<QPointF, QColor>> pts;
        pts.reserve(r.colored_points.size());
        for (const auto& p : r.colored_points) {
            pts.emplace_back(QPointF(p.x, p.y),
                             QColor(p.r, p.g, p.b));
        }
        ctx.annotator->draw_colored_points(frame, pts, 3.0);
    }
    // Circles.
    if (!r.circles.empty()) {
        std::vector<std::pair<QPointF, double>> circs;
        circs.reserve(r.circles.size());
        for (const auto& c : r.circles) {
            circs.emplace_back(QPointF(c.cx, c.cy), c.r);
        }
        ctx.annotator->draw_circles(frame, circs);
    }
    // Text labels.
    if (!r.texts.empty()) {
        for (const auto& t : r.texts) {
            ctx.annotator->draw_text(frame, QString::fromStdString(t.text),
                                     QPoint(t.x, t.y));
        }
    }
    // Colored events (orientation/direction per-event coloring).
    if (!r.colored_events.empty()) {
        std::vector<std::tuple<int, int, QColor>> cevs;
        cevs.reserve(r.colored_events.size());
        for (const auto& ce : r.colored_events) {
            cevs.emplace_back(ce.event.x, ce.event.y,
                              QColor(ce.r, ce.g, ce.b));
        }
        ctx.annotator->draw_colored_events(frame, cevs);
    }
    // Trajectories (cluster history paths).
    if (!r.trajectories.empty()) {
        std::vector<std::pair<int, std::vector<QPointF>>> trajs;
        trajs.reserve(r.trajectories.size());
        for (const auto& t : r.trajectories) {
            std::vector<QPointF> pts;
            pts.reserve(t.points.size());
            for (const auto& pt : t.points) {
                pts.emplace_back(pt.x, pt.y);
            }
            trajs.emplace_back(t.id, std::move(pts));
        }
        ctx.annotator->draw_trajectories(frame, trajs, QColor(0, 255, 0));
    }
    // Aux frame (Hough θ-ρ / per-pixel accumulator space).
    // Routed to the AlgoWindow's display widget if one is open, so the user
    // can see the accumulator state alongside the overlay.
    if (r.has_aux_frame && !r.aux_frame.empty()) {
        auto wit = ctx.algo_windows->find(info.name);
        if (wit != ctx.algo_windows->end() && wit.value()) {
            QPointer<EventDisplayWidget> disp = wit.value()->frame_display();
            if (disp) {
                QImage q = mat_to_qimage(r.aux_frame);
                QMetaObject::invokeMethod(ctx.window, [disp, q]() {
                    if (disp) disp->set_frame(q);
                }, Qt::QueuedConnection);
            }
        }
    }
    // ROI zoom view (design §5.6.6, Phase 2.6 re-drive): if the UNIFIED ROI
    // is enabled and an AlgoWindow with an EventDisplayWidget is open, crop
    // the ROI region from the annotated main frame and push it to the
    // window. This gives the user a zoomed-in view of just the ROI region
    // (with the algorithm's overlay drawn on it), which is otherwise hard to
    // inspect on the sensor-scale main display. The rect comes from
    // CameraController::unified_roi (already clamped/computed on both the
    // live hardware-ROI and file software-crop paths).
    bool roi_on = false;
    int ax = 0, ay = 0, ax1 = 0, ay1 = 0;
    if (ctx.camera) {
        ctx.camera->unified_roi(roi_on, ax, ay, ax1, ay1);
    }
    const int aw = ax1 - ax;
    const int ah = ay1 - ay;
    if (roi_on && aw > 0 && ah > 0) {
        auto wit = ctx.algo_windows->find(info.name);
        if (wit != ctx.algo_windows->end() && wit.value()) {
            // Use QPointer so the lambda safely no-ops if the AlgoWindow's
            // display widget is destroyed between scheduling and execution
            // (e.g. user closes/undocks the dock while a frame is in flight).
            QPointer<EventDisplayWidget> disp = wit.value()->frame_display();
            if (disp) {
                QRect roi_rect(ax, ay, aw, ah);
                QImage zoom = frame.copy(roi_rect);
                QMetaObject::invokeMethod(ctx.window, [disp, zoom]() {
                    if (disp) disp->set_frame(zoom);
                }, Qt::QueuedConnection);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ReplaceStrategy
// ---------------------------------------------------------------------------

void ReplaceStrategy::apply(QImage& frame, AlgoResult& r,
                            const AlgoInfo& /*info*/, DisplayContext& /*ctx*/) {
    // Replace the main display frame with the algorithm output.
    if (r.has_frame && !r.frame.empty()) {
        frame = mat_to_qimage(r.frame);
    }
}

// ---------------------------------------------------------------------------
// StandaloneStrategy
// ---------------------------------------------------------------------------

void StandaloneStrategy::apply(QImage& /*frame*/, AlgoResult& r,
                               const AlgoInfo& info, DisplayContext& ctx) {
    // Route results to the AlgoWindow (design §5.6.6). Frame-producing algos
    // (time_surface, event_to_video, isi_analyzer, background_mask) use an
    // EventDisplayWidget; text-producing algos (freq_detector, flow_statistics,
    // auto_bias, etc.) use the default status QLabel. xyt_visualizer is handled
    // separately via SpaceTimeDisplay.
    auto wit = ctx.algo_windows->find(info.name);
    if (wit != ctx.algo_windows->end() && wit.value()) {
        QPointer<AlgoWindow> w = wit.value();
        if (r.has_frame && !r.frame.empty()) {
            // QPointer protects against the dock being closed/undocked (and its
            // display widget destroyed) between scheduling and execution of the
            // queued call.
            QPointer<EventDisplayWidget> disp = w->frame_display();
            if (disp) {
                QImage q = mat_to_qimage(r.frame);
                QMetaObject::invokeMethod(ctx.window, [disp, q]() {
                    if (disp) disp->set_frame(q);
                }, Qt::QueuedConnection);
            }
        }
        if (!r.status.empty()) {
            QString text = QString::fromStdString(r.status);
            for (const auto& t : r.texts) {
                text += QStringLiteral("\n  ");
                text += QString::fromStdString(t.text);
            }
            QMetaObject::invokeMethod(ctx.window, [w, text]() {
                if (w) w->set_status_text(text);
            }, Qt::QueuedConnection);
        }
    }
}

} // namespace gui
