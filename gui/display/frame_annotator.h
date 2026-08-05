// gui/display/frame_annotator.h — algorithm overlay renderer.
//
// Design §1.6.6 (jAER FrameAnnotater interface). Draws algorithm result
// overlays (bounding boxes, tracked-object IDs, text, lines, points,
// circles, flow arrows) onto a rendered display QImage. This is the key
// overlay renderer that paints AlgoResult outputs on top of the live frame.
//
// All draw_* methods operate on a caller-owned QImage (Format_ARGB32 or
// RGBA8888) using a QPainter; coordinates are in image (sensor) pixels.

#ifndef GUI_DISPLAY_FRAME_ANNOTATOR_H
#define GUI_DISPLAY_FRAME_ANNOTATOR_H

#include <QColor>
#include <QFont>
#include <QImage>
#include <QLineF>
#include <QPoint>
#include <QPointF>
#include <QRect>
#include <QRectF>
#include <QString>
#include <vector>

namespace gui {

/// @brief Renders algorithm overlay primitives onto a display QImage.
class FrameAnnotator {
public:
    /// @brief A tracked-object box with an optional integer id and label.
    struct Box {
        QRect rect;
        int id{-1};
        QString label;
    };

    /// @brief A flow arrow (origin + displacement in image pixels).
    struct FlowArrow {
        QPointF from;
        QPointF to;
    };

    FrameAnnotator();

    /// @brief Sets the font used by draw_text().
    void set_font(const QFont& font);
    const QFont& font() const { return font_; }

    /// @brief Draws a set of plain bounding boxes.
    void draw_bboxes(QImage& img, const std::vector<QRect>& boxes,
                     const QColor& color = QColor());

    /// @brief Draws tracked-object boxes (with id/label caption).
    void draw_boxes(QImage& img, const std::vector<Box>& boxes,
                    const QColor& color = QColor());

    /// @brief Draws a single text string at @p pos (image coords).
    void draw_text(QImage& img, const QString& text, const QPoint& pos,
                   const QColor& color = QColor(),
                   bool with_background = true);

    /// @brief Draws a set of line segments.
    void draw_lines(QImage& img, const std::vector<QLineF>& lines,
                    const QColor& color = QColor());

    /// @brief Draws a set of points (small filled squares/circles).
    void draw_points(QImage& img, const std::vector<QPointF>& points,
                     double radius = 2.0,
                     const QColor& color = QColor());

    /// @brief Draws a set of colored filled rectangles (one per point).
    /// Used for HSV optical-flow visualization (hue=direction, value=strength).
    /// @param pts  (position, color) pairs in image coords.
    /// @param side Square side length in pixels.
    void draw_colored_points(QImage& img,
                             const std::vector<std::pair<QPointF, QColor>>& pts,
                             double side = 3.0);

    /// @brief Draws a set of circles (center + radius).
    void draw_circles(QImage& img,
                      const std::vector<std::pair<QPointF, double>>& circles,
                      const QColor& color = QColor());

    /// @brief Draws optical-flow arrows.
    void draw_flow_arrows(QImage& img, const std::vector<FlowArrow>& arrows,
                          const QColor& color = QColor());

    /// @brief Draws colored events (per-event RGB pixels).
    /// Used by orientation/direction filters to color each event by its label.
    void draw_colored_events(QImage& img,
                             const std::vector<std::tuple<int, int, QColor>>& events);

    /// @brief Draws trajectories (polylines with optional id labels).
    void draw_trajectories(QImage& img,
                           const std::vector<std::pair<int, std::vector<QPointF>>>& trajs,
                           const QColor& color = QColor());

private:
    QColor resolve(const QColor& c) const;

    double pen_width_{2.0};
    QFont font_;
    QColor default_color_{QColor(255, 255, 0)};  // yellow
};

} // namespace gui

#endif // GUI_DISPLAY_FRAME_ANNOTATOR_H
