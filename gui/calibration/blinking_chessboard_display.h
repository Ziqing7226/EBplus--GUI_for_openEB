// gui/calibration/blinking_chessboard_display.h — on-screen blinking chessboard
// pattern for event-camera intrinsic calibration.
//
// The board BLINKS to generate events without camera motion: the chessboard
// pattern (black squares on white) alternates with a fully white blank frame at
// a 10 ms period. Pixels on black squares toggle dark↔light every cycle and
// fire both ON and OFF events; over a capture window covering ≥ one cycle they
// form a filled checkerboard in the binary "blink frame" that the detector
// feeds to cv::findChessboardCorners (see algo/calibration/blinking_detect.h).
//
// Two pre-computed pixmaps (frame A: chessboard; frame B: blank white) are
// toggled at 10 ms — paintEvent is a single drawPixmap blit, no per-frame
// rendering. Same pre-rendered-pixmap blit architecture as the previous
// pattern display (avoids the maximize-button preview stutter).

#ifndef GUI_CALIBRATION_BLINKING_CHESSBOARD_DISPLAY_H
#define GUI_CALIBRATION_BLINKING_CHESSBOARD_DISPLAY_H

#include <QPixmap>
#include <QWidget>

class QTimer;

namespace gui {

/// @brief Renders the blinking chessboard calibration pattern.
class BlinkingChessboardDisplay : public QWidget {
    Q_OBJECT
public:
    explicit BlinkingChessboardDisplay(QWidget* parent = nullptr);

    /// @brief Sets the board dimensions (INNER corner count: patternSize for
    /// cv::findChessboardCorners). The widget draws (cols+1)×(rows+1) squares.
    /// Recomputes the pixel layout and re-renders both cached pixmaps.
    void set_pattern(int cols, int rows);

    /// @brief Sets the physical square size (mm). Stored for retrieval only —
    /// it does NOT affect pixel layout (no DPI derivation), so no repaint is
    /// triggered. Sets the real-world scale for the calibration algorithm.
    void set_square_size_mm(float mm);

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    float square_size_mm() const { return square_size_mm_; }
    /// @brief Current pixel size of one square.
    int square_px() const { return square_px_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    /// @brief 10 ms blink: toggles blink_frame_ and triggers a repaint.
    void on_blink_timeout();

private:
    void recompute_layout();

    int cols_{9};
    int rows_{6};
    int square_px_{0};
    int origin_x_{0};
    int origin_y_{0};
    float square_size_mm_{20.0f};

    /// Two pre-rendered frames for the 10 ms blink. Frame A is the chessboard
    /// (black squares on white); frame B is blank white. recompute_layout()
    /// renders both; paintEvent() blits one — no per-frame drawing.
    QPixmap pixmap_a_;  // chessboard pattern
    QPixmap pixmap_b_;  // blank (white)
    /// Current blink frame (0 = A, 1 = B). Toggled by the blink timer.
    int blink_frame_{0};
    /// Last frame blitted to the backing store. When blink_frame_ differs,
    /// paintEvent blits the new pixmap; otherwise it skips (the backing store
    /// already has the correct content). -1 forces a blit on the first paint.
    int blitted_frame_{-1};

    /// 10 ms blink timer. Toggles blink_frame_ and calls update().
    QTimer* blink_timer_{nullptr};
};

} // namespace gui

#endif // GUI_CALIBRATION_BLINKING_CHESSBOARD_DISPLAY_H
