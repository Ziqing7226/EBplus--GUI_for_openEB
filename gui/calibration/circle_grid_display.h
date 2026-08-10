// gui/calibration/circle_grid_display.h — static screw-head grid pattern for
// event-camera intrinsic calibration (Zhou's Method).
//
// The grid is STATIC (no flipping): events come from the user's hand micro-tremor
// while the camera looks at the pattern; the wizard captures a short event window
// on demand (Space key). Each marker is a "screw head": a dashed cross (pins the
// centre for detection) inside a solid thin ring (supplies the polarity signal).
// Both are white on a black background for camera contrast.
//
// Marker centres sit at (2*c + (r&1))*spacing, r*spacing — the asymmetric 6×5
// layout whose row offset gives 8-fold orientation disambiguation (no missing
// corners needed). This matches IntrinsicCalibration's ScrewHeadGrid object-point
// formula, so the displayed geometry and the calibration maths agree.
//
// dot_gap (1/2/3, default 2) is the dashed-cross dot period minus one: the cross
// is drawn as 1px white dots spaced (1 + dot_gap) px apart, and the solid ring's
// thickness equals dot_gap. The physical marker spacing (mm) is supplied
// by the user — screen DPI is deliberately not used (unreliable on X11).

#ifndef GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H
#define GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H

#include <QPixmap>
#include <QWidget>

namespace gui {

/// @brief Renders the static screw-head calibration grid. (Class name retained
/// from the prior circle-grid implementation; the file is modified in place
/// rather than renamed to keep the churn minimal.)
class CircleGridDisplay : public QWidget {
    Q_OBJECT
public:
    explicit CircleGridDisplay(QWidget* parent = nullptr);

    /// @brief Sets the grid dimensions (markers per row, number of rows).
    /// Recomputes the pixel layout and re-renders the cached pixmap.
    void set_pattern(int cols, int rows);

    /// @brief Sets the dashed-cross dot gap (1/2/3). The cross dots are 1px,
    /// spaced (1+dot_gap) px apart; the solid ring thickness equals dot_gap.
    /// Recomputes the cached pixmap.
    void set_dot_gap(int dot_gap);

    /// @brief Sets the physical marker spacing (mm). Stored for retrieval
    /// only — it does NOT affect pixel layout (no DPI derivation), so no repaint
    /// is triggered. Sets the real-world scale for the calibration algorithm.
    void set_square_size_mm(float mm);

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    int dot_gap() const { return dot_gap_; }
    float square_size_mm() const { return square_size_mm_; }
    /// @brief Current pixel spacing between adjacent grid cells (the half-cell).
    int spacing_px() const { return spacing_px_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void recompute_layout();

    int cols_{6};
    int rows_{5};
    int dot_gap_{2};
    int spacing_px_{0};
    int origin_x_{0};
    int origin_y_{0};
    float square_size_mm_{5.0f};

    /// Pre-rendered grid. recompute_layout() renders the full grid into this
    /// pixmap; paintEvent() is a single drawPixmap — no per-paint drawing.
    QPixmap cache_;
    /// True when cache_ has been re-rendered and needs blitting. Set by
    /// recompute_layout(); cleared by paintEvent(). Prevents redundant blits when
    /// the WM sends expose events for a static pixmap (e.g. Mutter unredirected).
    bool cache_dirty_{true};
};

} // namespace gui

#endif // GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H
