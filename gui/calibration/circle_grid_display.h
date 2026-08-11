// gui/calibration/circle_grid_display.h — blinking screw-head grid pattern for
// event-camera intrinsic calibration (Zhou's Method).
//
// The grid's dashed cross BLINKS at 30 Hz to generate events without camera
// motion: the cross is a 1px-wide line of alternating white/black pixels along
// each arm. Two pre-computed pixmaps (frame A: even offsets white; frame B: odd
// offsets white) are toggled at 30 Hz — paintEvent is a single drawPixmap blit,
// no per-frame rendering. The solid white ring (thickness 2px) is static; it
// generates events only from camera motion (hand micro-tremor).
//
// dot_gap is fixed at 1 (no longer user-adjustable): the cross alternates 1px
// white / 1px black. Ring thickness is fixed at 2px.
//
// Marker centres sit at (2*c + (r&1))*spacing, r*spacing — the asymmetric 6×5
// layout whose row offset gives 8-fold orientation disambiguation. The physical
// marker spacing (mm) is supplied by the user — screen DPI is deliberately not
// used (unreliable on X11).

#ifndef GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H
#define GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H

#include <QPixmap>
#include <QWidget>

class QTimer;

namespace gui {

/// @brief Renders the blinking screw-head calibration grid. (Class name
/// retained from the prior circle-grid implementation; the file is modified
/// in place rather than renamed to keep the churn minimal.)
class CircleGridDisplay : public QWidget {
    Q_OBJECT
public:
    explicit CircleGridDisplay(QWidget* parent = nullptr);

    /// @brief Sets the grid dimensions (markers per row, number of rows).
    /// Recomputes the pixel layout and re-renders both cached pixmaps.
    void set_pattern(int cols, int rows);

    /// @brief Sets the physical marker spacing (mm). Stored for retrieval
    /// only — it does NOT affect pixel layout (no DPI derivation), so no
    /// repaint is triggered. Sets the real-world scale for the calibration
    /// algorithm.
    void set_square_size_mm(float mm);

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    /// @brief Dashed-cross dot gap, fixed at 1 (1px white / 1px black).
    int dot_gap() const { return 1; }
    float square_size_mm() const { return square_size_mm_; }
    /// @brief Current pixel spacing between adjacent grid cells (the half-cell).
    int spacing_px() const { return spacing_px_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    /// @brief 30 Hz blink: toggles blink_frame_ and triggers a repaint.
    void on_blink_timeout();

private:
    void recompute_layout();

    int cols_{6};
    int rows_{5};
    int spacing_px_{0};
    int origin_x_{0};
    int origin_y_{0};
    float square_size_mm_{5.0f};

    /// Two pre-rendered frames for 30 Hz blinking. Frame A has even-offset
    /// cross dots white (odd black); frame B is the inverse. The ring is
    /// identical in both. recompute_layout() renders both; paintEvent() blits
    /// one — no per-frame drawing.
    QPixmap pixmap_a_;  // even offsets white
    QPixmap pixmap_b_;  // odd offsets white
    /// Current blink frame (0 = A, 1 = B). Toggled by the 30 Hz timer.
    int blink_frame_{0};
    /// Last frame blitted to the backing store. When blink_frame_ differs,
    /// paintEvent blits the new pixmap; otherwise it skips (the backing store
    /// already has the correct content). -1 forces a blit on the first paint.
    int blitted_frame_{-1};

    /// 30 Hz blink timer. Toggles blink_frame_ and calls update().
    QTimer* blink_timer_{nullptr};
};

} // namespace gui

#endif // GUI_CALIBRATION_CIRCLE_GRID_DISPLAY_H
