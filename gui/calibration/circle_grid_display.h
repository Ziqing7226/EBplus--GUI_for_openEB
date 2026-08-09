// gui/calibration/circle_grid_display.h — static asymmetric circle-grid
// pattern for event-camera intrinsic calibration (Phase 4).
//
// Replaces the flashing ChessboardDisplay. The grid is STATIC (no flipping):
// per the Phase 4 design, events come from the user's hand micro-motion and
// the screen's refresh while the camera looks at the pattern; the wizard
// captures a 500 µs window on demand (Space key, polarity ignored). Black
// background with white circles matches the OpenCV CALIB_CB_ASYMMETRIC_GRID
// convention.
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

    /// @brief Sets the physical cell spacing (mm). Stored for retrieval only
    /// — it does NOT affect pixel layout (no DPI derivation), so no repaint is
    /// triggered. Changing this value has zero visual effect; it sets the
    /// real-world scale for the calibration algorithm only.
    void set_square_size_mm(float mm);

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    float square_size_mm() const { return square_size_mm_; }
    /// @brief Current pixel spacing between adjacent grid cells.
    int spacing_px() const { return spacing_px_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void recompute_layout();

    int cols_{4};
    int rows_{11};
    int spacing_px_{0};
    int dot_radius_px_{0};
    int origin_x_{0};
    int origin_y_{0};
    float square_size_mm_{5.0f};

    /// Pre-rendered grid (like SiemensStarWidget). recompute_layout() renders
    /// the full grid into this pixmap; paintEvent() is a single drawPixmap —
    /// no per-paint circle drawing.
    QPixmap cache_;
};

} // namespace gui

#endif // GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H
