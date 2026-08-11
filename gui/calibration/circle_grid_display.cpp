// gui/calibration/circle_grid_display.cpp — see header (Zhou's Method).

#include "circle_grid_display.h"

#include <algorithm>

#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QTimer>
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

// Ring thickness in pixels (fixed, no longer tied to dot_gap).
constexpr int kRingThickness = 2;

// Blink rate: 30 Hz. The dashed cross alternates its white/black pattern at
// this rate, generating ON/OFF events at every cross pixel without camera
// motion. 33 ms ≈ 30.3 Hz.
constexpr int kBlinkIntervalMs = 33;
} // namespace

CircleGridDisplay::CircleGridDisplay(QWidget* parent) : QWidget(parent) {
    setAutoFillBackground(false);
    // We paint every pixel ourselves (black fill + white markers), so Qt can
    // skip its background fill — saves a memset on each repaint.
    setAttribute(Qt::WA_OpaquePaintEvent);
    setPalette(Qt::black);
    setMinimumSize(320, 320);
    recompute_layout();

    // 30 Hz blink timer: toggles blink_frame_ (0↔1) and triggers update().
    // paintEvent blits the corresponding pre-computed pixmap — a single
    // drawPixmap call, no per-frame rendering. The timer is lightweight
    // (just a flag toggle + update); when the widget is hidden, update() is
    // a no-op so there's no wasted CPU.
    blink_timer_ = new QTimer(this);
    blink_timer_->setInterval(kBlinkIntervalMs);
    connect(blink_timer_, &QTimer::timeout, this, &CircleGridDisplay::on_blink_timeout);
    blink_timer_->start();
}

void CircleGridDisplay::set_pattern(int cols, int rows) {
    cols_ = std::max(1, cols);
    rows_ = std::max(1, rows);
    pixmap_a_ = QPixmap();  // invalidate → force re-render even if size unchanged
    pixmap_b_ = QPixmap();
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

void CircleGridDisplay::on_blink_timeout() {
    blink_frame_ ^= 1;  // toggle 0 ↔ 1
    update();
}

void CircleGridDisplay::recompute_layout() {
    // Skip the full pixmap re-render when the widget size has not changed and
    // both caches are still valid. Layout-param changes (cols/rows) invalidate
    // the pixmaps in their setters, so a param change still re-renders.
    if (!pixmap_a_.isNull() && !pixmap_b_.isNull() &&
        pixmap_a_.size() == size() && pixmap_b_.size() == size()) {
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
        pixmap_a_ = QPixmap();
        pixmap_b_ = QPixmap();
        return;
    }

    // Screw-head marker = solid white ring (thickness 2px) + blinking dashed
    // cross (1px white/black alternating along each arm).
    //   R           = kMarkerRadiusFrac * spacing   (ring mean radius)
    //   R_out       = R + 1,  R_in = R - 1           (ring annulus, 2px thick)
    //   cross_limit = R_in - 1                        (dots stay inside ring)
    //   Frame A: pixel at offset k is white if k is even, else black (background)
    //   Frame B: pixel at offset k is white if k is odd,  else black (background)
    //
    // Rendering: solid QPainter primitives only (drawEllipse + batched drawRects),
    // mirroring the pre-waffle path so the QPixmap stays on the native GPU-resident
    // blit path (no QImage full-widget CPU→GPU upload — that stalls the GUI thread
    // and reintroduces the maximized-window preview stutter; see project memory).
    const double R = kMarkerRadiusFrac * spacing_px_;
    const double R_out = R + kRingThickness * 0.5;
    const double R_in = std::max(1.0, R - kRingThickness * 0.5);
    const double cross_limit = std::max(1.0, R_in - 1.0);
    const int kmax = int(cross_limit);

    // Pre-compute marker centre coordinates (shared by both frames).
    struct Centre { int x, y; };
    QVector<Centre> centres;
    centres.reserve(cols_ * rows_);
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const double cx = origin_x_ + (2 * c + (r & 1)) * spacing_px_;
            const double cy = origin_y_ + r * spacing_px_;
            centres.append({int(cx + 0.5), int(cy + 0.5)});
        }
    }

    // Pre-compute the cross dot positions for each frame (even vs odd offsets).
    // Both frames share the same dot grid; only the parity filter differs.
    QVector<QRect> dots_a, dots_b;
    for (const Centre& ct : centres) {
        for (int k = -kmax; k <= kmax; ++k) {
            if (std::abs(k) > int(cross_limit)) continue;
            const QRect h(ct.x + k, ct.y, 1, 1);  // horizontal arm
            if ((k & 1) == 0) dots_a.append(h); else dots_b.append(h);
            if (k != 0) {  // avoid double-drawing centre
                const QRect v(ct.x, ct.y + k, 1, 1);  // vertical arm
                if ((k & 1) == 0) dots_a.append(v); else dots_b.append(v);
            }
        }
    }

    // Render both frames. The ring is identical in both; only the cross dots
    // differ (even vs odd offsets).
    for (int frame = 0; frame < 2; ++frame) {
        QPixmap& pm = (frame == 0) ? pixmap_a_ : pixmap_b_;
        pm = QPixmap(size());
        pm.fill(Qt::black);
        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(Qt::NoPen);

        // Phase 1 — solid white rings: white disc R_out, then black disc R_in
        // punches the interior, leaving the 2px annulus. Two batched passes
        // (all white, all black) to keep the brush constant per pass.
        p.setBrush(Qt::white);
        for (const Centre& ct : centres) {
            p.drawEllipse(QPointF(ct.x, ct.y), R_out, R_out);
        }
        p.setBrush(Qt::black);
        for (const Centre& ct : centres) {
            p.drawEllipse(QPointF(ct.x, ct.y), R_in, R_in);
        }

        // Phase 2 — blinking dashed cross. Frame A draws even-offset dots;
        // frame B draws odd-offset dots. Batched into one drawRects call.
        p.setBrush(Qt::white);
        const QVector<QRect>& dots = (frame == 0) ? dots_a : dots_b;
        if (!dots.isEmpty()) {
            p.drawRects(dots);
        }
    }

    blitted_frame_ = -1;  // force blit on next paintEvent
}

void CircleGridDisplay::paintEvent(QPaintEvent* event) {
    QPainter p(this);
    const QPixmap& pm = (blink_frame_ == 0) ? pixmap_a_ : pixmap_b_;
    if (pm.isNull()) {
        p.fillRect(event->rect(), Qt::black);
        return;
    }
    // Skip the blit if the same frame was already blitted since the last
    // toggle. With WA_OpaquePaintEvent, Qt doesn't clear the damaged region,
    // so the backing store retains the previous (correct) content. This
    // eliminates redundant blits when the WM sends expose events between
    // blinks (e.g., Mutter unredirected windows).
    if (blitted_frame_ != blink_frame_) {
        p.drawPixmap(0, 0, pm);
        blitted_frame_ = blink_frame_;
    }
}

} // namespace gui
