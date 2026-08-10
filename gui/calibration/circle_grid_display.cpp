// gui/calibration/circle_grid_display.cpp — see header (Zhou's Method).

#include "circle_grid_display.h"

#include <algorithm>

#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QVector>

namespace gui {

namespace {
// Marker ring mean radius as a fraction of the grid half-cell spacing. 0.20
// keeps the ring clear of the board's top/bottom boundary: the layout fits the
// centre-to-centre footprint at 92%, so the ring must leave room beyond the
// outermost centres — at 0.30 the outer ring was tangent to the edge. Tunable
// (field): too large → adjacent markers merge after dilation + boundary
// tangency; too small → too few ring-edge events for polarity verification.
constexpr double kMarkerRadiusFrac = 0.20;
} // namespace

CircleGridDisplay::CircleGridDisplay(QWidget* parent) : QWidget(parent) {
    setAutoFillBackground(false);
    // We paint every pixel ourselves (black fill + white markers), so Qt can
    // skip its background fill — saves a memset on each repaint.
    setAttribute(Qt::WA_OpaquePaintEvent);
    setPalette(Qt::black);
    setMinimumSize(320, 320);
    recompute_layout();
}

void CircleGridDisplay::set_pattern(int cols, int rows) {
    cols_ = std::max(1, cols);
    rows_ = std::max(1, rows);
    cache_ = QPixmap();  // invalidate → force re-render even if size unchanged
    recompute_layout();
    update();
}

void CircleGridDisplay::set_dot_gap(int dot_gap) {
    dot_gap_ = std::clamp(dot_gap, 1, 3);
    cache_ = QPixmap();
    recompute_layout();
    update();
}

void CircleGridDisplay::set_square_size_mm(float mm) {
    // Perf: no update() — the mm value does not affect pixel layout.
    square_size_mm_ = std::max(0.1f, mm);
}

void CircleGridDisplay::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    recompute_layout();
}

void CircleGridDisplay::recompute_layout() {
    // Skip the full pixmap re-render when the widget size has not changed and
    // the cache is still valid. Layout-param changes (cols/rows/dot_gap)
    // invalidate cache_ in their setters, so a param change still re-renders.
    if (!cache_.isNull() && cache_.size() == size()) {
        return;
    }

    // Asymmetric-grid footprint in cells:
    //   width  = 2*cols - 1  (odd rows offset by one half-cell)
    //   height = rows - 1
    // Pick the largest spacing that fits ~92% of the widget.
    const int avail_w = std::max(0, width() - 16);
    const int avail_h = std::max(0, height() - 16);
    const int footprint_w = std::max(1, 2 * cols_ - 1);
    const int footprint_h = std::max(1, rows_ - 1);
    int sp = std::min(avail_w / footprint_w, avail_h / footprint_h);
    sp = static_cast<int>(sp * 0.92);
    spacing_px_ = std::max(8, sp);
    const int grid_w = footprint_w * spacing_px_;
    const int grid_h = footprint_h * spacing_px_;
    origin_x_ = (width() - grid_w) / 2;
    origin_y_ = (height() - grid_h) / 2;

    if (width() <= 0 || height() <= 0) {
        cache_ = QPixmap();
        return;
    }

    // Screw-head marker = solid white ring + dashed white cross (1px dots).
    //   R      = kMarkerRadiusFrac * spacing   (ring mean radius)
    //   g      = dot_gap                         (ring thickness; cross period-1)
    //   R_out  = R + g/2,  R_in = R - g/2        (ring annulus)
    //   cross  = 1px dots at period (1+g) along x & y arms, within |r| < R_in-1
    //
    // Rendering: solid QPainter primitives only (drawEllipse + batched drawRects),
    // mirroring the pre-waffle path so the QPixmap stays on the native GPU-resident
    // blit path (no QImage full-widget CPU→GPU upload — that stalls the GUI thread
    // and reintroduces the maximized-window preview stutter; see project memory).
    cache_ = QPixmap(size());
    cache_.fill(Qt::black);
    QPainter p(&cache_);
    p.setRenderHint(QPainter::Antialiasing, true);  // smooth ring edges
    p.setPen(Qt::NoPen);

    const double R = kMarkerRadiusFrac * spacing_px_;
    const int g = dot_gap_;
    const double R_out = R + g * 0.5;
    const double R_in = std::max(1.0, R - g * 0.5);
    const double cross_limit = std::max(1.0, R_in - 1.0);  // dots stay inside ring
    const int period = 1 + g;

    // Phase 1 — solid white rings: white disc R_out, then black disc R_in punches
    // the interior, leaving the annulus. Two batched passes (all white, all black)
    // to keep the brush constant per pass.
    p.setBrush(Qt::white);
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
            const double cy = origin_y_ + r * spacing_px_;
            p.drawEllipse(QPointF(cx, cy), R_out, R_out);
        }
    }
    p.setBrush(Qt::black);
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
            const double cy = origin_y_ + r * spacing_px_;
            p.drawEllipse(QPointF(cx, cy), R_in, R_in);
        }
    }

    // Phase 2 — dashed cross (1px white dots) along x and y arms, batched into one
    // drawRects call. Dots at offset k*period (k integer) with |offset| < cross_limit.
    p.setBrush(Qt::white);
    QVector<QRect> dots;
    const int kmax = std::max(1, int(cross_limit / period));
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
            const double cy = origin_y_ + r * spacing_px_;
            const int icx = int(cx + 0.5);
            const int icy = int(cy + 0.5);
            for (int k = -kmax; k <= kmax; ++k) {
                const int off = k * period;
                if (std::abs(off) > int(cross_limit)) continue;
                dots.append(QRect(icx + off, icy, 1, 1));      // horizontal arm
                if (k != 0) {                                   // avoid double-drawing centre
                    dots.append(QRect(icx, icy + off, 1, 1));   // vertical arm
                }
            }
        }
    }
    if (!dots.isEmpty()) {
        p.drawRects(dots);
    }

    cache_dirty_ = true;  // new cache content — paintEvent must blit it
}

void CircleGridDisplay::paintEvent(QPaintEvent* event) {
    QPainter p(this);
    if (cache_.isNull()) {
        p.fillRect(event->rect(), Qt::black);
        return;
    }
    // Skip the blit if the cache hasn't changed since the last paint. With
    // WA_OpaquePaintEvent, Qt doesn't clear the damaged region, so the backing
    // store retains the previous (correct) content. Eliminates redundant blits
    // when the WM sends expose events for a static window.
    if (cache_dirty_) {
        p.drawPixmap(0, 0, cache_);
        cache_dirty_ = false;
    }
}

} // namespace gui
