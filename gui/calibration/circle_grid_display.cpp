// gui/calibration/circle_grid_display.cpp — see header (Phase 4).

#include "circle_grid_display.h"

#include <algorithm>

#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QVector>

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
    // Invalidate the cache so recompute_layout's size-equality short-circuit
    // does not skip the re-render: a pattern change with an unchanged widget
    // size must still redraw.
    cache_ = QPixmap();
    recompute_layout();
    update();
}

void CircleGridDisplay::set_dot_size(int dot_size) {
    // Clamp to the supported range 1..3. Each circle becomes a white edge ring
    // (dot_size px thick) plus an interior waffle grid whose cell size and
    // spacing are both dot_size. Larger values make a coarser, sparser waffle.
    dot_size_ = std::clamp(dot_size, 1, 3);
    // Invalidate the cache so recompute_layout's size-equality short-circuit
    // does not skip the re-render: a dot-size change with an unchanged widget
    // size must still redraw.
    cache_ = QPixmap();
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
    // Skip the full pixmap re-render when the widget size has not changed since
    // the last call — a guard against redundant resize events from the WM/Qt.
    // The waffle is static, so an unchanged size means the cached pixmap is
    // still valid. Layout-param changes (cols/rows/dot_size) invalidate cache_
    // in set_pattern()/set_dot_size() before calling us, so a param change with
    // an unchanged size still forces a re-render (cache_.isNull() is true then).
    if (!cache_.isNull() && cache_.size() == size()) {
        return;
    }

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
    // only on resize or set_pattern()/set_dot_size(), both infrequent.
    if (width() <= 0 || height() <= 0) {
        cache_ = QPixmap();
        return;
    }

    // Zhou's Circle Grid — waffle pattern. Each circle is:
    //   - a white edge ring (outer `ds` px of the disc);
    //   - an interior waffle grid in GLOBAL widget coordinates: a pixel at
    //     widget (x, y) is BLACK when (x/ds) or (y/ds) is even, else WHITE.
    //     So white dots appear only at (odd, odd) grid cells — a sparse white
    //     dot matrix on black. This produces far more brightness transitions
    //     per circle than a solid disc, giving the event camera denser, more
    //     detectable features when the user micro-moves the camera over it.
    //
    // Rendering: SOLID-COLOUR QPainter primitives only — drawEllipse (white
    // disc R, then black disc Rds to punch the ring) + one batched drawRects
    // for all waffle dots. This deliberately mirrors the pre-waffle
    // concentric-ring code's primitive set (solid QBrush + drawEllipse). An
    // earlier waffle attempt used a QBrush texture tile + QPainterPath/fillPath,
    // which on Mutter/xcb forced the QPixmap onto a software raster fallback,
    // re-uploading the cached pixmap to the GPU on every composited frame.
    // Solid primitives keep the QPixmap on the native (GPU-resident) path, so
    // paintEvent's drawPixmap stays a cheap blit.
    cache_ = QPixmap(size());
    cache_.fill(Qt::black);
    QPainter p(&cache_);
    p.setRenderHint(QPainter::Antialiasing, true);  // smooth ring edges
    p.setPen(Qt::NoPen);

    const double R = dot_radius_px_;
    const int ds = dot_size_;
    const double Rds = std::max(0, dot_radius_px_ - ds);  // interior radius

    // Phase 1 — white edge ring per circle: solid white disc, then solid black
    // disc of radius Rds punches out the interior, leaving the white ring.
    p.setBrush(Qt::white);
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
            const double cy = origin_y_ + r * spacing_px_;
            p.drawEllipse(QPointF(cx, cy), R, R);
        }
    }
    p.setBrush(Qt::black);
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
            const double cy = origin_y_ + r * spacing_px_;
            p.drawEllipse(QPointF(cx, cy), Rds, Rds);
        }
    }

    // Phase 2 — waffle dots. Collect every white dot (global (odd, odd) cell
    // whose centre lies within the interior disc Rds) across all circles into
    // one vector, then draw them in a single batched drawRects call (maps to
    // XRenderFillRectangles — one native op, not thousands of requests).
    // Cell (gx, gy) covers [gx*ds, gx*ds+ds) × [gy*ds, gy*ds+ds); centre at
    // (gx*ds + ds/2, gy*ds + ds/2). Dots whose centre is within Rds stay fully
    // inside R (corner ≤ Rds + 0.71·ds < Rds + ds = R), so none poke outside
    // the circle.
    if (Rds > 0) {
        QVector<QRect> dots;
        const double Rds2 = Rds * Rds;
        for (int r = 0; r < rows_; ++r) {
            for (int c = 0; c < cols_; ++c) {
                const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
                const double cy = origin_y_ + r * spacing_px_;
                const int gx_min = std::max(0, static_cast<int>((cx - Rds) / ds) - 1);
                const int gx_max = static_cast<int>((cx + Rds) / ds) + 1;
                const int gy_min = std::max(0, static_cast<int>((cy - Rds) / ds) - 1);
                const int gy_max = static_cast<int>((cy + Rds) / ds) + 1;
                for (int gx = gx_min; gx <= gx_max; ++gx) {
                    if ((gx & 1) == 0) continue;  // white only at odd gx
                    const double ccx = gx * ds + ds * 0.5;
                    const double ddx = ccx - cx;
                    for (int gy = gy_min; gy <= gy_max; ++gy) {
                        if ((gy & 1) == 0) continue;  // white only at odd gy
                        const double ccy = gy * ds + ds * 0.5;
                        const double ddy = ccy - cy;
                        if (ddx * ddx + ddy * ddy <= Rds2) {
                            dots.append(QRect(gx * ds, gy * ds, ds, ds));
                        }
                    }
                }
            }
        }
        if (!dots.isEmpty()) {
            p.setBrush(Qt::white);
            p.drawRects(dots);
        }
    }

    cache_dirty_ = true;  // New cache content — paintEvent must blit it.
}

void CircleGridDisplay::paintEvent(QPaintEvent* event) {
    QPainter p(this);
    if (cache_.isNull()) {
        // Widget not yet sized — fill black and bail (recompute_layout will
        // run on the first resizeEvent).
        p.fillRect(event->rect(), Qt::black);
        return;
    }
    // Skip the blit if the cache hasn't changed since the last paint. With
    // WA_OpaquePaintEvent, Qt doesn't clear the damaged region, so the backing
    // store retains the previous (correct) content. This eliminates redundant
    // full-screen blits when the WM sends expose events for a static window
    // (e.g. Mutter in unredirected mode sends full-window expose events).
    if (cache_dirty_) {
        p.drawPixmap(0, 0, cache_);
        cache_dirty_ = false;
    }
}

} // namespace gui
