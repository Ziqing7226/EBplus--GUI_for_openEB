// gui/calibration/capture_review_dialog.h — post-capture verification dialog
// (Zhou's Method Phase 4, item 4).
//
// After a Space capture the wizard detects screw-head markers and shows this
// dialog over the raw captured frame: every detected RING feature is drawn as a
// red circle (centre + radius) and every detected CROSS feature as a blue
// cross. The user can relax the ring-detection parameters (coverage threshold,
// min ring pixels, ring search margin), click Re-detect to re-run detection and
// redraw, then either Accept — which closes the dialog and commits the capture
// through the rest of the pipeline (coverage + duplicate checks, accumulation)
// — or Discard, which abandons the capture entirely (the frame is not added to
// the calibration set).
//
// The dialog is intentionally modal (exec() from the wizard): while it is open
// the wizard is blocked, so no other capture can race the review.

#ifndef GUI_CALIBRATION_CAPTURE_REVIEW_DIALOG_H
#define GUI_CALIBRATION_CAPTURE_REVIEW_DIALOG_H

#include <QDialog>
#include <QImage>
#include <QVector>

#include <QWidget>

#include "calibration_worker.h"  // CaptureReview

class QDoubleSpinBox;
class QLabel;
class QPaintEvent;
class QPushButton;
class QSpinBox;

namespace gui {

/// @brief Paints the captured frame plus the detected features scaled to fit:
/// red circles at ring centres (radius-scaled) and blue crosses at cross
/// centres.
class CaptureReviewCanvas : public QWidget {
    Q_OBJECT
public:
    explicit CaptureReviewCanvas(QWidget* parent = nullptr);

    /// @brief Sets the frame and feature markers to draw. No markers clears
    /// the overlay. Triggers a repaint.
    void set_data(const QImage& frame, const QVector<QPointF>& crosses,
                  const QVector<QPointF>& rings, const QVector<float>& radii);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage frame_;
    QVector<QPointF> crosses_;
    QVector<QPointF> rings_;
    QVector<float> radii_;
};

/// @brief Modal capture-review dialog (see file comment).
class CaptureReviewDialog : public QDialog {
    Q_OBJECT
public:
    explicit CaptureReviewDialog(QWidget* parent = nullptr);

    /// @brief Refreshes the displayed frame + feature markers, the status line,
    /// and the Accept button enablement (Accept is only meaningful when the
    /// full grid was detected). Safe to call while the dialog is already open
    /// (a re-detect result) or just before exec() (the first detection).
    void set_review(const CaptureReview& review);

signals:
    /// @brief Emitted when the user clicks Re-detect, carrying the current
    /// ring-detection parameter values.
    void re_detect_requested(double cover_frac, int min_pixels, int search_margin);

private:
    void on_re_detect();

    CaptureReviewCanvas* canvas_{nullptr};
    QDoubleSpinBox* cover_frac_{nullptr};
    QSpinBox* min_pixels_{nullptr};
    QSpinBox* search_margin_{nullptr};
    QPushButton* re_detect_btn_{nullptr};
    QPushButton* accept_btn_{nullptr};
    QPushButton* discard_btn_{nullptr};
    QLabel* status_{nullptr};
};

} // namespace gui

#endif // GUI_CALIBRATION_CAPTURE_REVIEW_DIALOG_H
