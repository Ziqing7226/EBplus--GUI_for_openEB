// gui/calibration/chessboard_display.cpp

#include "chessboard_display.h"

#include <QGuiApplication>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QProgressBar>
#include <QPushButton>
#include <QResizeEvent>
#include <QScreen>
#include <QSizePolicy>
#include <QTimer>

namespace gui {

namespace {
// Flip interval = 50 ms → 20 inversions/sec. Each 50 ms cycle aligns with
// the CalibrationWizard's capture cycle so every cycle contains exactly one
// edge burst from the flip.
constexpr int kFlipIntervalMs = 50;

// Height of the HUD band at the bottom of the window. The board geometry
// excludes this band, so the 20 Hz flip repaint (scoped to the board rect)
// never intersects the HUD and the HUD doesn't flicker at 20 Hz.
constexpr int kHudHeightPx = 46;
} // namespace

ChessboardDisplay::ChessboardDisplay(QWidget* parent) : QWidget(parent, Qt::Window) {
    setWindowTitle(tr("Calibration Chessboard"));
    setAttribute(Qt::WA_DeleteOnClose);
    setAutoFillBackground(false);
    // We paint every pixel of the dirty rect ourselves (black fill + squares),
    // so Qt can skip the background fill — saves a full-screen memset on each
    // repaint of a 4K surface.
    setAttribute(Qt::WA_OpaquePaintEvent);
    // StrongFocus so the S/F/Esc shortcuts work in fullscreen; the HUD
    // button is NoFocus so clicking it doesn't steal key events.
    setFocusPolicy(Qt::StrongFocus);
    // Black background so the area outside the board doesn't generate
    // spurious edge events during flips.
    setPalette(Qt::black);

    // --- HUD band (audit §9.2-C) -------------------------------------------
    // Semi-transparent child widget pinned to the bottom edge, outside the
    // board rect. It lets the user run the whole capture flow (progress,
    // status, Start/Stop) while the fullscreen board covers the wizard
    // dialog. Kept deliberately small: it sits on the same screen the camera
    // is watching, so large or frequently-redrawing overlays would inject
    // spurious events into the capture. It updates only on state changes.
    hud_ = new QWidget(this);
    hud_->setAttribute(Qt::WA_StyledBackground, true);
    hud_->setStyleSheet(
        "QWidget { background: rgba(0, 0, 0, 170); }"
        "QLabel { color: #dddddd; }"
        "QProgressBar { color: #dddddd; }");
    auto* hud_layout = new QHBoxLayout(hud_);
    hud_layout->setContentsMargins(10, 4, 10, 4);
    hud_layout->setSpacing(10);

    hud_progress_ = new QProgressBar(hud_);
    hud_progress_->setFixedWidth(200);
    hud_progress_->setMaximumHeight(18);
    hud_progress_->setRange(0, 30);
    hud_progress_->setValue(0);
    hud_layout->addWidget(hud_progress_);

    hud_status_ = new QLabel(tr("Idle. Press S or Start to capture."), hud_);
    hud_status_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    hud_layout->addWidget(hud_status_, 1);

    hud_spec_ = new QLabel(hud_);
    hud_layout->addWidget(hud_spec_);

    hud_toggle_btn_ = new QPushButton(tr("Start"), hud_);
    hud_toggle_btn_->setFocusPolicy(Qt::NoFocus);
    hud_layout->addWidget(hud_toggle_btn_);
    connect(hud_toggle_btn_, &QPushButton::clicked, this, [this]() {
        if (capturing_) emit stopRequested();
        else            emit startRequested();
    });

    timer_ = new QTimer(this);
    timer_->setTimerType(Qt::PreciseTimer);
    timer_->setInterval(kFlipIntervalMs);
    connect(timer_, &QTimer::timeout, this, [this]() {
        inverted_ = !inverted_;
        // Only repaint the board area — the HUD band and the black margin
        // don't change between flips. This is the critical performance win:
        // a full-screen update() on a 4K surface at 20 Hz forces ~660 MB/s
        // of backing-store compositing; scoping the update to the board rect
        // cuts that by the screen-to-board area ratio.
        update(QRect(board_origin_x_, board_origin_y_, board_w_px_, board_h_px_));
    });

    attach_to_screen(QGuiApplication::primaryScreen());
    resize(800, 600);
}

ChessboardDisplay::~ChessboardDisplay() {
    if (timer_) timer_->stop();
}

void ChessboardDisplay::attach_to_screen(QScreen* screen) {
    attached_screen_ = screen ? screen : QGuiApplication::primaryScreen();
    recompute_geometry();
    if (attached_screen_) {
        // Park the window at the screen's top-left so showFullScreen() lands
        // on the right monitor; user can toggle windowed mode with F.
        move(attached_screen_->geometry().topLeft());
    }
    update();  // full repaint — geometry changed
}

void ChessboardDisplay::set_pattern(int inner_cols, int inner_rows) {
    inner_cols_ = std::max(2, inner_cols);
    inner_rows_ = std::max(2, inner_rows);
    recompute_geometry();
    update();  // full repaint — pattern changed
}

void ChessboardDisplay::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    if (hud_) hud_->setGeometry(0, height() - kHudHeightPx, width(), kHudHeightPx);
    // Geometry follows the widget's own rect (audit §9.2-B / §六-B5), so
    // fullscreen↔windowed transitions recompute automatically. The resize
    // itself already schedules a repaint — no explicit update() needed.
    recompute_geometry();
}

void ChessboardDisplay::recompute_geometry() {
    // Available drawing area: the widget's rect minus the HUD band. This is
    // the widget coordinate system that paintEvent uses, so the board is
    // always exactly where it is drawn, in both fullscreen and windowed mode.
    const int avail_w = width();
    const int avail_h = height() - (hud_ ? kHudHeightPx : 0);
    if (avail_w <= 0 || avail_h <= 0) return;

    // The board has (inner_cols+1) × (inner_rows+1) squares. Choose a square
    // size that fills ~90% of the smaller axis so corners span a wide range
    // of the field of view — improves calibration conditioning.
    const int squares_w = inner_cols_ + 1;
    const int squares_h = inner_rows_ + 1;
    const int sq = std::min(avail_w / squares_w, avail_h / squares_h);
    square_size_px_ = std::max(8, static_cast<int>(sq * 0.9));
    board_w_px_ = square_size_px_ * squares_w;
    board_h_px_ = square_size_px_ * squares_h;
    board_origin_x_ = (avail_w - board_w_px_) / 2;
    board_origin_y_ = (avail_h - board_h_px_) / 2;

    // Physical size from screen DPI. QScreen::physicalDotsPerInch() reports
    // the panel's physical DPI; square_mm = pixels / dpi * 25.4. This value
    // is only a default — the wizard shows it in an editable spinbox (audit
    // §9.2-G) because X11 DPI is often unreliable.
    qreal dpi = 0.0;
    if (attached_screen_) dpi = attached_screen_->physicalDotsPerInch();
    if (dpi <= 0.0) dpi = 96.0;  // Fallback: typical desktop DPI.
    square_size_mm_ = static_cast<float>(square_size_px_ / dpi * 25.4);

    update_spec_label();
}

void ChessboardDisplay::update_spec_label() {
    if (!hud_spec_) return;
    hud_spec_->setText(tr("%1×%2 corners  |  square ≈ %3 mm  |  S: start/stop  F: fullscreen  Esc: close")
        .arg(inner_cols_).arg(inner_rows_)
        .arg(static_cast<double>(square_size_mm_), 0, 'f', 1));
}

void ChessboardDisplay::start_flicker() {
    if (!timer_->isActive()) timer_->start();
}

void ChessboardDisplay::stop_flicker() {
    timer_->stop();
}

void ChessboardDisplay::set_capturing(bool capturing) {
    capturing_ = capturing;
    if (hud_toggle_btn_) hud_toggle_btn_->setText(capturing ? tr("Stop") : tr("Start"));
}

void ChessboardDisplay::set_progress(int accepted, int target) {
    if (!hud_progress_) return;
    hud_progress_->setRange(0, target);
    hud_progress_->setValue(accepted);
}

void ChessboardDisplay::set_status(const QString& text) {
    if (hud_status_) hud_status_->setText(text);
}

void ChessboardDisplay::paintEvent(QPaintEvent* event) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, false);

    const QRect dirty = event->rect();

    // Fill the dirty region with black. This handles both the area outside
    // the board and the board's black squares in one memset. Combined with
    // WA_OpaquePaintEvent, Qt skips its own background fill, so this is the
    // only full pass over the dirty pixels.
    p.fillRect(dirty, Qt::black);

    // Draw only the white squares that intersect the dirty region. The black
    // squares are already filled above, so we skip them. This is cheap: a
    // 9×6 inner-corner board has ≤70 squares total, and on a flip only the
    // board rect is dirty, so every square is checked but only fills are
    // issued for white ones.
    if (square_size_px_ > 0) {
        for (int r = 0; r <= inner_rows_; ++r) {
            for (int c = 0; c <= inner_cols_; ++c) {
                const bool is_white = ((r + c) % 2 == 0) ^ inverted_;
                if (!is_white) continue;
                const QRect sq(board_origin_x_ + c * square_size_px_,
                               board_origin_y_ + r * square_size_px_,
                               square_size_px_, square_size_px_);
                if (sq.intersects(dirty)) {
                    p.fillRect(sq, Qt::white);
                }
            }
        }
    }
    // No painted hint overlay: the HUD band below the board shows the spec
    // and shortcuts, and being child widgets it never intersects the board
    // rect's 20 Hz partial repaint.
}

void ChessboardDisplay::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
        case Qt::Key_S:
            if (capturing_) emit stopRequested();
            else            emit startRequested();
            break;
        case Qt::Key_F:
            fullscreen_ = !fullscreen_;
            if (fullscreen_) {
                showFullScreen();
            } else {
                showNormal();
            }
            break;
        case Qt::Key_Escape:
            close();
            break;
        default:
            QWidget::keyPressEvent(event);
    }
}

} // namespace gui
