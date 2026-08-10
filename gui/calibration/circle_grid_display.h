// gui/calibration/circle_grid_display.h — static asymmetric circle-grid
// pattern for event-camera intrinsic calibration (Phase 4).
//
// Replaces the flashing ChessboardDisplay. The grid is STATIC (no flipping):
// per the Phase 4 design, events come from the user's hand micro-motion and
// the screen's refresh while the camera looks at the pattern; the wizard
// captures a 5000 µs window on demand (Space key, polarity ignored).
//
// Zhou's Circle Grid: each circle is drawn as a waffle pattern — a white
// edge ring (dot_size px thick) plus an interior grid of white dots. The
// waffle uses GLOBAL widget coordinates (a single continuous grid, not
// per-circle): an interior pixel at widget (x, y) is BLACK when
// (x / dot_size) or (y / dot_size) is even, else WHITE — so white dots appear
// only at (odd, odd) grid cells, a sparse white dot matrix on black. This
// produces far more brightness transitions per circle than a solid disc,
// giving the event camera denser, more detectable features. dot_size is
// configurable (1/2/3, default 2) via a dropdown.
//
// The physical cell spacing (mm) is supplied by the user — we deliberately
// do NOT derive it from QScreen::physicalDotsPerInch(), which is unreliable
// on X11 (Phase 4 bug-absorption). The pixel spacing is chosen only to fit
// the widget; the user measures the on-screen spacing with a ruler and enters
// the corresponding mm value in the wizard.
//
// Circle centres are placed at (2*c + (r&1))*spacing, r*spacing — identical to
// IntrinsicCalibration's AsymmetricCircles object-point formula, so the
// displayed geometry and the calibration maths agree.

#ifndef GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H
#define GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H

#include <QPixmap>
#include <QWidget>

namespace gui {

class CircleGridDisplay : public QWidget {
    Q_OBJECT
public:
    explicit CircleGridDisplay(QWidget* parent = nullptr);

    /// @brief Sets the grid dimensions (circles per row, number of rows).
    /// Recomputes the pixel layout and re-renders the cached pixmap.
    void set_pattern(int cols, int rows);

    /// @brief Sets the waffle dot size (1/2/3). The circle edge ring is
    /// dot_size px thick; interior pixels follow a waffle grid with this
    /// spacing. Recomputes the cached pixmap.
    void set_dot_size(int dot_size);

    /// @brief Sets the physical cell spacing (mm). Stored for retrieval only
    /// — it does NOT affect pixel layout (no DPI derivation), so no repaint is
    /// triggered. Changing this value has zero visual effect; it sets the
    /// real-world scale for the calibration algorithm only.
    void set_square_size_mm(float mm);

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    int dot_size() const { return dot_size_; }
    float square_size_mm() const { return square_size_mm_; }
    /// @brief Current pixel spacing between adjacent grid cells.
    int spacing_px() const { return spacing_px_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void recompute_layout();

    int cols_{6};
    int rows_{5};
    int dot_size_{2};
    int spacing_px_{0};
    int dot_radius_px_{0};
    int origin_x_{0};
    int origin_y_{0};
    float square_size_mm_{5.0f};

    /// Pre-rendered grid (like SiemensStarWidget). recompute_layout() renders
    /// the full grid into this pixmap; paintEvent() is a single drawPixmap —
    /// no per-paint circle drawing.
    QPixmap cache_;
    /// True when cache_ has been re-rendered and needs to be blitted to the
    /// backing store. Set by recompute_layout(); cleared by paintEvent(). This
    /// prevents redundant full-screen blits when the WM sends expose events
    /// for a static pixmap (e.g. Mutter in unredirected mode).
    bool cache_dirty_{true};
};

} // namespace gui

#endif // GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H
