// gui/calibration/focus_dialog.h — Siemens-star focus assistant (Phase 5).
//
// Replaces the sharpness meter: a slowly rotating Siemens Star is shown next
// to the live camera output; the user turns the lens focus ring until the
// star's center is sharpest (inivation DV GUI approach). No sharpness/DFT
// computation at all — focusing is judged by eye.
//
// The star is PRE-RENDERED into a ring of phase pixmaps (user suggestion):
// at runtime the widget only does drawPixmap + phase increment — no per-frame
// QPainter wedge work, no tearing/flicker.

#ifndef GUI_CALIBRATION_FOCUS_DIALOG_H
#define GUI_CALIBRATION_FOCUS_DIALOG_H

#include <QDialog>
#include <QWidget>

#include <QImage>
#include <QPixmap>
#include <vector>

class QLabel;
class QTimer;

namespace gui {

class EventDisplayWidget;

/// @brief Slowly rotating Siemens Star. All phases are pre-rendered once at
/// construction (or on resize); the paint path is a single drawPixmap.
class SiemensStarWidget : public QWidget {
    Q_OBJECT
public:
    explicit SiemensStarWidget(QWidget* parent = nullptr);

    /// @brief Advances to the next pre-rendered phase (called by the dialog's
    /// timer).
    void advance();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    /// @brief (Re)renders the phase ring for the current widget size.
    void prerender();

    std::vector<QPixmap> phases_;
    int phase_{0};
};

/// @brief Live camera view fed by polling the main display's frame.
class FocusCameraView : public QWidget {
    Q_OBJECT
public:
    explicit FocusCameraView(QWidget* parent = nullptr);

    /// @brief Sets the latest frame (called by the dialog's poll timer).
    void set_frame(const QImage& frame);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage frame_;
};

/// @brief Focus assistant dialog: rotating Siemens Star | live camera output.
class FocusDialog : public QDialog {
    Q_OBJECT
public:
    explicit FocusDialog(QWidget* parent = nullptr);
    ~FocusDialog();

    /// @brief Sets the display to poll for camera frames. Safe with nullptr.
    void set_display(EventDisplayWidget* display);

private slots:
    void on_tick();

private:
    EventDisplayWidget* display_{nullptr};
    QTimer* timer_{nullptr};
    SiemensStarWidget* star_{nullptr};
    FocusCameraView* camera_view_{nullptr};
};

} // namespace gui

#endif // GUI_CALIBRATION_FOCUS_DIALOG_H
