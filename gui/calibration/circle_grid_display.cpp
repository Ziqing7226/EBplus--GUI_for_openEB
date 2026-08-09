// gui/calibration/circle_grid_display.cpp — see header (Phase 4).

#include "circle_grid_display.h"

#include <algorithm>

#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>

namespace gui {

CircleGridDisplay::CircleGridDisplay(QWidget* parent) : QWidget(parent) {
    setAutoFillBackground(false);
    // We paint every pixel ourselves (black fill + white circles), so Qt can
    // skip its background fill — saves a memset on each repaint.
    setAttribute(Qt::WA_OpaquePaintEvent);
    setPalette(Qt::black);
    setMinimumSize(320, 320);
    recompute_layout();
}

void CircleGridDisplay::set_pattern(int cols, int rows) {
    cols_ = std::max(1, cols);
    rows_ = std::max(1, rows);
    recompute_layout();
    update();
}

void CircleGridDisplay::set_layers(int layers) {
    // Clamp to the supported odd values. Even layer counts would make the
    // innermost band black (same as the background), making the center
    // invisible — enforce odd.
    layers_ = std::max(3, layers);
    if (layers_ % 2 == 0) layers_ += 1;
    recompute_layout();
    update();
}

void CircleGridDisplay::set_square_size_mm(float mm) {
    // Perf: no update() — the mm value does not affect pixel layout, so there
    // is nothing to repaint. The wizard reads the value via square_size_mm().
    square_size_mm_ = std::max(0.1f, mm);
}

void CircleGridDisplay::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    recompute_layout();
}

void CircleGridDisplay::recompute_layout() {
    // Asymmetric-grid footprint in cells:
    //   width  = 2*cols - 1  (odd rows are offset by one half-cell, so the
    //                          widest row spans from x=0 to x=(2*(cols-1)+1))
    //   height = rows - 1
    // Pick the largest spacing that fits ~92% of the widget, leaving a margin
    // so dots near the edge are not clipped.
    const int avail_w = std::max(0, width() - 16);
    const int avail_h = std::max(0, height() - 16);
    const int footprint_w = std::max(1, 2 * cols_ - 1);
    const int footprint_h = std::max(1, rows_ - 1);
    int sp = std::min(avail_w / footprint_w, avail_h / footprint_h);
    sp = static_cast<int>(sp * 0.92);
    spacing_px_ = std::max(8, sp);
    dot_radius_px_ = std::max(3, spacing_px_ / 4);
    const int grid_w = footprint_w * spacing_px_;
    const int grid_h = footprint_h * spacing_px_;
    origin_x_ = (width() - grid_w) / 2;
    origin_y_ = (height() - grid_h) / 2;

    // Pre-render the grid into cache_ (like SiemensStarWidget). paintEvent()
    // then does a single drawPixmap — no per-paint circle work. Re-rendered
    // only on resize or set_pattern(), both infrequent.
    if (width() <= 0 || height() <= 0) {
        cache_ = QPixmap();
        return;
    }
    cache_ = QPixmap(size());
    cache_.fill(Qt::black);
    QPainter p(&cache_);
    p.setRenderHint(QPainter::Antialiasing, true);
    // Zhou's Ring Grid: concentric rings at asymmetric-grid positions.
    //   x = (2*c + (r & 1)) * spacing, y = r * spacing
    // (matches IntrinsicCalibration's AsymmetricCircles object-point formula).
    //
    // Each circle position draws `layers_` filled circles from largest to
    // smallest, alternating white/black. The outermost (layer 0) is white
    // with radius = dot_radius_px_; each subsequent layer shrinks by the
    // ring thickness t = R / layers_. For odd layers_ the innermost band
    // (a small circle of radius t) is white.
    p.setPen(Qt::NoPen);
    const double R = static_cast<double>(dot_radius_px_);
    const double t = R / layers_;
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
            const double cy = origin_y_ + r * spacing_px_;
            for (int layer = 0; layer < layers_; ++layer) {
                const double radius = R - layer * t;
                p.setBrush((layer % 2 == 0) ? Qt::white : Qt::black);
                p.drawEllipse(QPointF(cx, cy), radius, radius);
            }
        }
    }
}

void CircleGridDisplay::paintEvent(QPaintEvent* event) {
    QPainter p(this);
    if (cache_.isNull()) {
        // Widget not yet sized — fill black and bail (recompute_layout will
        // run on the first resizeEvent).
        p.fillRect(event->rect(), Qt::black);
        return;
    }
    p.drawPixmap(0, 0, cache_);
}

} // namespace gui
