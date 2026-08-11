// gui/calibration/focus_dialog.cpp — see header (Phase 5).

#include "focus_dialog.h"

#include <QEvent>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QScreen>
#include <QShowEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>

#include "display/event_display_widget.h"

namespace gui {

namespace {

// Star geometry / motion. 72 sectors (36 black/white spoke pairs) is fine
// enough that defocus blurs the center into a disk well before the rim —
// that gradient is what the user focuses on. The star is rotation-periodic
// every sector step (360°/72 = 5°), so 72 phases of 1/72 sector each form a
// seamless loop; at 30 Hz one sector step takes 2.4 s (~2°/s — "slowly
// rotating", inivation style).
constexpr int kSectors = 72;
constexpr int kPhases = 72;
constexpr int kTickMs = 33;  // ~30 Hz phase advance + camera poll

// Same value and rationale as CalibrationWizard::kFullscreenGuardInset: a
// window whose geometry matches the workarea is treated by Mutter (GNOME) as
// maximized — unredirected / put on the maximized frame-sync path — which
// throttles the 30 Hz camera poll into visible stutter. Insetting by 50 px
// per side keeps the window on the smooth normal compositing path.
constexpr int kFullscreenGuardInset = 50;

// The focus assistant pairs a square-ish Siemens star with a square-ish camera
// view side by side (plus a hint line below), so the natural window shape is
// width ≈ 1.95 × height. fit_focus_geometry() scales the inset workarea to
// that ratio, keeping the window as large as possible inside it: fit by
// height when the workarea is wider than the ratio, otherwise fit by width.
constexpr double kFocusAspectRatio = 1.95;

QRect fit_focus_geometry(const QRect& base) {
    QRect g = base;
    if (g.height() * kFocusAspectRatio <= g.width()) {
        // Inset workarea wider than the target ratio → height-limited.
        g.setWidth(static_cast<int>(std::lround(g.height() * kFocusAspectRatio)));
    } else {
        // Inset workarea taller/narrower than the target ratio → width-limited.
        g.setHeight(static_cast<int>(std::lround(g.width() / kFocusAspectRatio)));
    }
    return g;
}

} // namespace

// ---------------------------------------------------------------------------
// SiemensStarWidget
// ---------------------------------------------------------------------------

SiemensStarWidget::SiemensStarWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(360, 360);
}

void SiemensStarWidget::advance() {
    if (phases_.empty()) return;
    phase_ = (phase_ + 1) % static_cast<int>(phases_.size());
    update();
}

void SiemensStarWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    prerender();
}

void SiemensStarWidget::prerender() {
    // Square canvas covering the widget; the star is centered with a small
    // margin. Black/white wedges on a mid-gray field.
    const int side = std::max(std::min(width(), height()), 64);
    const int diameter = side - 16;
    phases_.clear();
    phases_.reserve(kPhases);
    for (int p = 0; p < kPhases; ++p) {
        QPixmap pm(QSize(side, side));
        // Classic Siemens star: alternating black/white wedges on a white
        // field — draw white, then the black wedges.
        pm.fill(Qt::white);
        QPainter painter(&pm);
        painter.setRenderHint(QPainter::Antialiasing);
        const double step_deg = 360.0 / kSectors;
        const double offset_deg =
            step_deg * (static_cast<double>(p) / kPhases);
        // QPainter::drawPie uses 1/16-degree units, counter-clockwise from
        // 3 o'clock.
        const QRectF rect(8, 8, diameter, diameter);
        for (int s = 0; s < kSectors; s += 2) {
            painter.setBrush(Qt::black);
            painter.setPen(Qt::NoPen);
            painter.drawPie(rect,
                            static_cast<int>((offset_deg + s * step_deg) * 16),
                            static_cast<int>(step_deg * 16));
        }
        phases_.push_back(std::move(pm));
    }
    phase_ = 0;
}

void SiemensStarWidget::paintEvent(QPaintEvent* /*event*/) {
    if (phases_.empty()) {
        prerender();
        if (phases_.empty()) return;
    }
    QPainter p(this);
    const auto& pm = phases_[static_cast<std::size_t>(phase_)];
    p.drawPixmap((width() - pm.width()) / 2, (height() - pm.height()) / 2, pm);
}

// ---------------------------------------------------------------------------
// FocusCameraView
// ---------------------------------------------------------------------------

FocusCameraView::FocusCameraView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(360, 360);
}

void FocusCameraView::set_frame(const QImage& frame) {
    frame_ = frame;
    update();
}

void FocusCameraView::paintEvent(QPaintEvent* /*event*/) {
    QPainter p(this);
    p.fillRect(rect(), QColor(20, 20, 20));
    if (frame_.isNull()) {
        p.setPen(QColor(160, 160, 160));
        p.drawText(rect(), Qt::AlignCenter, tr("No camera connected"));
        return;
    }
    const QImage scaled = frame_.scaled(size(), Qt::KeepAspectRatio,
                                        Qt::SmoothTransformation);
    p.drawImage((width() - scaled.width()) / 2,
                (height() - scaled.height()) / 2, scaled);
}

// ---------------------------------------------------------------------------
// FocusDialog
// ---------------------------------------------------------------------------

FocusDialog::FocusDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Focus Assistant"));
    // Mirror the calibration wizard's window flags: Qt::Window (instead of the
    // default Qt::Dialog) gives a full top-level window with working
    // minimize/close buttons. The maximize button is deliberately OMITTED: a
    // maximized window covers the full workarea, which on Mutter (GNOME)
    // triggers the unredirect / maximized compositing path and makes the 30 Hz
    // camera poll stutter (see showEvent). The changeEvent override below also
    // blocks maximize from other paths (title-bar double-click, Super+Up,
    // drag-to-top edge-tiling) since those bypass the button.
    // Qt::WindowMaximizeButtonHint is intentionally absent.
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowSystemMenuHint |
                   Qt::WindowMinimizeButtonHint | Qt::WindowCloseButtonHint);
    setMinimumSize(900, 560);
    auto* layout = new QVBoxLayout(this);

    auto* row = new QHBoxLayout();
    star_ = new SiemensStarWidget(this);
    camera_view_ = new FocusCameraView(this);
    row->addWidget(star_, 1);
    row->addWidget(camera_view_, 1);
    layout->addLayout(row, 1);

    auto* hint = new QLabel(
        tr("Turn the lens focus ring until the rotating star's center is "
           "sharpest in the camera view. Defocus blurs the center into a "
           "disk first — the sharper the center point, the better the focus."),
        this);
    hint->setWordWrap(true);
    hint->setProperty("class", "hint");
    layout->addWidget(hint);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &FocusDialog::on_tick);
    timer_->start(kTickMs);
}

FocusDialog::~FocusDialog() = default;

void FocusDialog::set_display(EventDisplayWidget* display) {
    display_ = display;
}

void FocusDialog::showEvent(QShowEvent* event) {
    QDialog::showEvent(event);
    // Size the window to (nearly) the full workarea but INSET by a few px so
    // it does NOT exactly match the workarea — identical to the calibration
    // wizard's show_intrinsic(). Without the inset, a window whose geometry
    // matches the workarea is treated by Mutter (GNOME) as maximized: it is
    // unredirected (direct-scanout) and/or put on the maximized compositing
    // frame-sync path, which throttles the 30 Hz camera poll → visible
    // stutter. Applied on every show so the geometry is re-guaranteed after
    // any hide→show cycle.
    if (QScreen* screen = QGuiApplication::primaryScreen()) {
        QRect g = screen->availableGeometry();
        g.adjust(kFullscreenGuardInset, kFullscreenGuardInset,
                 -kFullscreenGuardInset, -kFullscreenGuardInset);
        setGeometry(fit_focus_geometry(g));
    }
}

void FocusDialog::changeEvent(QEvent* event) {
    QDialog::changeEvent(event);
    // Block ANY transition into Qt::WindowMaximized (title-bar double-click,
    // Super+Up, GNOME drag-to-top edge-tiling — the maximize button itself is
    // already removed via the window flags, but those other paths bypass it).
    // A maximized window covers the full workarea → Mutter unredirects it →
    // camera poll stutters (see showEvent). Immediately undo the maximize and
    // re-apply the inset workarea geometry, so the window LOOKS maximized yet
    // stays on the smooth normal compositing path.
    //
    // Deferred via QueuedConnection so the current state-change event finishes
    // first — calling setWindowState re-entrantly inside changeEvent is
    // fragile. Our own setWindowState fires a second changeEvent in which the
    // window is no longer maximized, so the guard below falls through (no
    // loop). The WM processes the un-maximize then our setGeometry in order.
    if (event->type() == QEvent::WindowStateChange &&
        (windowState() & Qt::WindowMaximized)) {
        QMetaObject::invokeMethod(this, [this] {
            setWindowState(windowState() & ~Qt::WindowMaximized);
            if (QScreen* screen = QGuiApplication::primaryScreen()) {
                QRect g = screen->availableGeometry();
                g.adjust(kFullscreenGuardInset, kFullscreenGuardInset,
                         -kFullscreenGuardInset, -kFullscreenGuardInset);
                setGeometry(fit_focus_geometry(g));
            }
        }, Qt::QueuedConnection);
    }
}

void FocusDialog::on_tick() {
    star_->advance();
    if (display_) {
        camera_view_->set_frame(display_->current_frame());
    }
}

} // namespace gui
