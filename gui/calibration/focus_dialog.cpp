// gui/calibration/focus_dialog.cpp — see header (Phase 5).

#include "focus_dialog.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
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
    resize(900, 420);
}

FocusDialog::~FocusDialog() = default;

void FocusDialog::set_display(EventDisplayWidget* display) {
    display_ = display;
}

void FocusDialog::on_tick() {
    star_->advance();
    if (display_) {
        camera_view_->set_frame(display_->current_frame());
    }
}

} // namespace gui
