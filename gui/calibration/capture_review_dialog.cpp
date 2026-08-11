// gui/calibration/capture_review_dialog.cpp — see header (Phase 4, item 4).

#include "capture_review_dialog.h"

#include <algorithm>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace gui {

namespace {
// Overlay colours: rings = red, crosses = blue (user requirement).
const QColor kRingColor(235, 60, 60);
const QColor kCrossColor(60, 110, 235);
// Half arm length of the blue cross marker, in FRAME pixels (scaled with the
// image so it tracks the marker size).
constexpr double kCrossArmPx = 5.0;
constexpr int kOverlayPenWidthPx = 2;
} // namespace

// ---------------------------------------------------------------------------
// CaptureReviewCanvas
// ---------------------------------------------------------------------------

CaptureReviewCanvas::CaptureReviewCanvas(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 240);
    setAutoFillBackground(false);
    setAttribute(Qt::WA_OpaquePaintEvent);
}

void CaptureReviewCanvas::set_data(const QImage& frame,
                                   const QVector<QPointF>& crosses,
                                   const QVector<QPointF>& rings,
                                   const QVector<float>& radii) {
    frame_ = frame;
    crosses_ = crosses;
    rings_ = rings;
    radii_ = radii;
    update();
}

void CaptureReviewCanvas::paintEvent(QPaintEvent* /*event*/) {
    QPainter p(this);
    p.fillRect(rect(), QColor(18, 18, 18));
    if (frame_.isNull()) {
        p.setPen(QColor(150, 150, 150));
        p.drawText(rect(), Qt::AlignCenter, tr("No captured frame."));
        return;
    }

    // Fit the frame inside the widget, preserving aspect ratio.
    const double scale = std::min(
        double(width())  / double(frame_.width()),
        double(height()) / double(frame_.height()));
    const int dw = std::max(1, int(frame_.width()  * scale));
    const int dh = std::max(1, int(frame_.height() * scale));
    const int ox = (width()  - dw) / 2;
    const int oy = (height() - dh) / 2;
    p.drawImage(QRect(ox, oy, dw, dh), frame_);

    // Overlays are drawn in frame coordinates: translate + scale so the marker
    // sizes (ring radii, cross arm) stay consistent with the image content.
    p.save();
    p.translate(ox, oy);
    p.scale(scale, scale);

    // Red circles at every detected ring centre.
    const int n_rings = std::min(rings_.size(), radii_.size());
    p.setPen(QPen(kRingColor, kOverlayPenWidthPx));
    for (int i = 0; i < n_rings; ++i) {
        const qreal rad = std::max(1.0, double(radii_[i]));
        p.drawEllipse(rings_[i], rad, rad);
    }

    // Blue crosses (plus-shaped) at every detected cross centre.
    p.setPen(QPen(kCrossColor, kOverlayPenWidthPx));
    for (const QPointF& c : crosses_) {
        p.drawLine(QPointF(c.x() - kCrossArmPx, c.y()),
                   QPointF(c.x() + kCrossArmPx, c.y()));
        p.drawLine(QPointF(c.x(), c.y() - kCrossArmPx),
                   QPointF(c.x(), c.y() + kCrossArmPx));
    }

    p.restore();
}

// ---------------------------------------------------------------------------
// CaptureReviewDialog
// ---------------------------------------------------------------------------

CaptureReviewDialog::CaptureReviewDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Capture Review"));
    setModal(true);
    setMinimumSize(640, 520);

    auto* layout = new QVBoxLayout(this);

    // Status line: what was detected + the Accept/Discard hint.
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    layout->addWidget(status_);

    // Frame + overlay canvas (dominant).
    canvas_ = new CaptureReviewCanvas(this);
    layout->addWidget(canvas_, 1);

    // Legend: red = ring, blue = cross.
    auto* legend = new QLabel(
        tr("<span style=\"color:%1;\">&#9679; Ring</span>&nbsp;&nbsp;"
           "<span style=\"color:%2;\">&#10010; Cross</span>")
            .arg(kRingColor.name()).arg(kCrossColor.name()), this);
    legend->setStyleSheet("font-size:12px;");
    layout->addWidget(legend, 0, Qt::AlignHCenter);

    // Ring-detection parameters (relaxable, then Re-detect).
    auto* params_group = new QGroupBox(tr("Ring detection parameters"), this);
    auto* form = new QFormLayout(params_group);
    form->setContentsMargins(12, 12, 12, 12);
    form->setSpacing(8);

    cover_frac_ = new QDoubleSpinBox(params_group);
    cover_frac_->setRange(0.05, 1.00);
    cover_frac_->setDecimals(2);
    cover_frac_->setSingleStep(0.05);
    cover_frac_->setValue(0.60);
    cover_frac_->setToolTip(tr("Minimum fraction of the circle's angle bins "
        "that must contain ring pixels for the ring to be accepted. Lowering "
        "this accepts partial rings from one-directional motion."));
    form->addRow(tr("Ring coverage \u2265"), cover_frac_);

    min_pixels_ = new QSpinBox(params_group);
    min_pixels_->setRange(1, 200);
    min_pixels_->setValue(6);
    min_pixels_->setToolTip(tr("Minimum number of ring pixels for a valid "
        "ring detection. Lower for sparse event signal."));
    form->addRow(tr("Min ring pixels"), min_pixels_);

    search_margin_ = new QSpinBox(params_group);
    search_margin_->setRange(0, 64);
    search_margin_->setValue(12);
    search_margin_->setToolTip(tr("Extra search radius (px) around each cross "
        "candidate when looking for its ring. Increase if rings are missed "
        "because the ring extends beyond the search radius."));
    form->addRow(tr("Ring search margin (px)"), search_margin_);
    layout->addWidget(params_group);

    // Re-detect + Accept/Discard row.
    auto* buttons = new QHBoxLayout();
    re_detect_btn_ = new QPushButton(tr("Re-detect"), this);
    re_detect_btn_->setToolTip(tr("Re-run detection with the adjusted ring "
        "parameters and redraw the markers."));
    buttons->addWidget(re_detect_btn_);
    buttons->addStretch();
    accept_btn_ = new QPushButton(tr("Accept"), this);
    accept_btn_->setDefault(true);
    accept_btn_->setToolTip(tr("Keep this capture and continue the calibration "
        "(closes this dialog)."));
    discard_btn_ = new QPushButton(tr("Discard"), this);
    discard_btn_->setToolTip(tr("Abandon this capture — it is NOT used in the "
        "calibration (closes this dialog)."));
    buttons->addWidget(accept_btn_);
    buttons->addWidget(discard_btn_);
    layout->addLayout(buttons);

    connect(re_detect_btn_, &QPushButton::clicked, this, &CaptureReviewDialog::on_re_detect);
    connect(accept_btn_, &QPushButton::clicked, this, &CaptureReviewDialog::accept);
    connect(discard_btn_, &QPushButton::clicked, this, &CaptureReviewDialog::reject);

    accept_btn_->setEnabled(false);
    set_review(CaptureReview{});
}

void CaptureReviewDialog::set_review(const CaptureReview& review) {
    canvas_->set_data(review.frame, review.crosses, review.rings,
                      review.ring_radii);
    accept_btn_->setEnabled(review.found);
    if (review.found) {
        status_->setText(tr("Full grid detected (%1 markers) — "
                            "%2 ring(s), %3 cross(es). "
                            "Review the markers, then Accept or Discard.")
            .arg(review.crosses.size()).arg(review.rings.size())
            .arg(review.crosses.size()));
    } else {
        status_->setText(tr("%1  Detected %2 ring(s) and %3 cross(es) so far. "
                            "Relax the ring parameters and Re-detect, or Discard.")
            .arg(review.reason)
            .arg(review.rings.size()).arg(review.crosses.size()));
    }
}

void CaptureReviewDialog::on_re_detect() {
    emit re_detect_requested(cover_frac_->value(),
                             min_pixels_->value(),
                             search_margin_->value());
}

} // namespace gui
