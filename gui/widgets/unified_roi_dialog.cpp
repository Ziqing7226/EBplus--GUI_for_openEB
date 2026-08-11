// gui/widgets/unified_roi_dialog.cpp — see header (Phase 2.6 debug D-6).

#include "widgets/unified_roi_dialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace gui {

UnifiedRoiDialog::UnifiedRoiDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("ROI Settings"));
    setModal(true);

    auto* layout = new QVBoxLayout(this);
    // Single grid for the whole form so every label/field shares column
    // boundaries: row labels (Mode/Origin/Size) in col 0, sub-labels
    // (X/W) in col 1, fields in col 2, sub-labels (Y/H) in col 3, fields in
    // col 4. All labels are right-aligned + vertically centered, which puts
    // every text baseline on the row's center line, and the grid's uniform
    // horizontal spacing makes the Origin↔X and Size↔W gaps identical
    // (user feedback: baseline misalignment and uneven gaps/widths).
    auto* grid = new QGridLayout();
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setHorizontalSpacing(8);
    grid->setVerticalSpacing(8);
    layout->addLayout(grid);

    // The enable state lives on the sidebar checkboxes (user decision), so
    // the dialog only edits rect/mode.
    constexpr int kFieldW = 110;  // one width for ALL four fields (fits
                                  // "auto-center" + the spin buttons)
    const auto rvc = Qt::AlignRight | Qt::AlignVCenter;
    int row = 0;

    mode_combo_ = new QComboBox(this);
    mode_combo_->addItem(tr("ROI (keep inside)"), false);
    mode_combo_->addItem(tr("RONI (drop inside)"), true);
    mode_combo_->setFixedWidth(2 * kFieldW + 8);  // spans both field columns
    grid->addWidget(new QLabel(tr("Mode"), this), row, 0, rvc);
    grid->addWidget(mode_combo_, row++, 1, 1, 4, Qt::AlignVCenter);

    x_sp_ = new QSpinBox(this);
    x_sp_->setSpecialValueText(tr("auto-center"));
    x_sp_->setFixedWidth(kFieldW);
    y_sp_ = new QSpinBox(this);
    y_sp_->setSpecialValueText(tr("auto-center"));
    y_sp_->setFixedWidth(kFieldW);
    grid->addWidget(new QLabel(tr("Origin"), this), row, 0, rvc);
    grid->addWidget(new QLabel(tr("X"), this), row, 1, rvc);
    grid->addWidget(x_sp_, row, 2, Qt::AlignVCenter);
    grid->addWidget(new QLabel(tr("Y"), this), row, 3, rvc);
    grid->addWidget(y_sp_, row, 4, Qt::AlignVCenter);
    row++;

    w_sp_ = new QSpinBox(this);
    w_sp_->setFixedWidth(kFieldW);
    h_sp_ = new QSpinBox(this);
    h_sp_->setFixedWidth(kFieldW);
    grid->addWidget(new QLabel(tr("Size"), this), row, 0, rvc);
    grid->addWidget(new QLabel(tr("W"), this), row, 1, rvc);
    grid->addWidget(w_sp_, row, 2, Qt::AlignVCenter);
    grid->addWidget(new QLabel(tr("H"), this), row, 3, rvc);
    grid->addWidget(h_sp_, row, 4, Qt::AlignVCenter);
    row++;

    draw_btn_ = new QPushButton(tr("Draw on Display..."), this);
    draw_btn_->setToolTip(
        tr("Close this dialog, drag the ROI rectangle on the main display — "
           "the ROI takes effect immediately after the drag."));
    grid->addWidget(draw_btn_, row, 1, 1, 4);
    connect(draw_btn_, &QPushButton::clicked, this,
            [this]() { done(kDrawRequest); });

    hint_lbl_ = new QLabel(this);
    hint_lbl_->setStyleSheet(QStringLiteral("color: #c0392b;"));
    hint_lbl_->setWordWrap(true);
    hint_lbl_->setVisible(false);
    layout->addWidget(hint_lbl_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    ok_btn_ = buttons->button(QDialogButtonBox::Ok);
    layout->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    connect(x_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, &UnifiedRoiDialog::validate_inputs);
    connect(y_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, &UnifiedRoiDialog::validate_inputs);
    connect(w_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, &UnifiedRoiDialog::validate_inputs);
    connect(h_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, &UnifiedRoiDialog::validate_inputs);
}

void UnifiedRoiDialog::set_state(int x0, int y0, int x1, int y1,
                                 bool roni, int sensor_w, int sensor_h) {
    sensor_w_ = sensor_w > 0 ? sensor_w : 1280;
    sensor_h_ = sensor_h > 0 ? sensor_h : 720;

    x_sp_->setRange(-1, sensor_w_ - 1);
    y_sp_->setRange(-1, sensor_h_ - 1);
    w_sp_->setRange(1, sensor_w_);
    h_sp_->setRange(1, sensor_h_);

    int w = x1 - x0;
    int h = y1 - y0;
    int x = x0;
    int y = y0;
    if (w <= 0 || h <= 0) {
        // Never configured: default center 256×144 (Phase 2.6 debug decision).
        x = -1;
        y = -1;
        w = 256;
        h = 144;
    }

    mode_combo_->setCurrentIndex(roni ? 1 : 0);
    x_sp_->setValue(x);
    y_sp_->setValue(y);
    w_sp_->setValue(w);
    h_sp_->setValue(h);
    validate_inputs();
}

bool UnifiedRoiDialog::roni() const { return mode_combo_->currentData().toBool(); }
int UnifiedRoiDialog::x() const { return x_sp_->value(); }
int UnifiedRoiDialog::y() const { return y_sp_->value(); }
int UnifiedRoiDialog::w() const { return w_sp_->value(); }
int UnifiedRoiDialog::h() const { return h_sp_->value(); }

void UnifiedRoiDialog::validate_inputs() {
    const int w = w_sp_->value();
    const int h = h_sp_->value();
    const int x = x_sp_->value();
    const int y = y_sp_->value();
    QString error;
    // Spin ranges already guarantee 1 <= w,h <= sensor and -1 <= x,y < sensor;
    // the cross-field check is what remains (x/y = -1 = auto-center: always OK).
    if (w > sensor_w_ || h > sensor_h_) {
        error = tr("Width/Height must not exceed the sensor size (%1×%2).")
                    .arg(sensor_w_).arg(sensor_h_);
    } else if (x >= 0 && x + w > sensor_w_) {
        error = tr("X + Width (%1) exceeds the sensor width (%2). Reduce X or Width.")
                    .arg(x + w).arg(sensor_w_);
    } else if (y >= 0 && y + h > sensor_h_) {
        error = tr("Y + Height (%1) exceeds the sensor height (%2). Reduce Y or Height.")
                    .arg(y + h).arg(sensor_h_);
    }
    hint_lbl_->setText(error);
    hint_lbl_->setVisible(!error.isEmpty());
    ok_btn_->setEnabled(error.isEmpty());
}

} // namespace gui
