// gui/widgets/unified_roi_dialog.cpp — see header (Phase 2.6 debug D-6).

#include "widgets/unified_roi_dialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace gui {

UnifiedRoiDialog::UnifiedRoiDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("ROI Settings"));
    setModal(true);

    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();
    form->setContentsMargins(0, 0, 0, 0);
    layout->addLayout(form);

    enable_cb_ = new QCheckBox(tr("Enable ROI"), this);
    form->addRow(enable_cb_);

    mode_combo_ = new QComboBox(this);
    mode_combo_->addItem(tr("ROI (keep inside)"), false);
    mode_combo_->addItem(tr("RONI (drop inside)"), true);
    form->addRow(tr("Mode"), mode_combo_);

    x_sp_ = new QSpinBox(this);
    x_sp_->setSpecialValueText(tr("auto-center"));
    form->addRow(tr("X"), x_sp_);
    y_sp_ = new QSpinBox(this);
    y_sp_->setSpecialValueText(tr("auto-center"));
    form->addRow(tr("Y"), y_sp_);
    w_sp_ = new QSpinBox(this);
    form->addRow(tr("Width"), w_sp_);
    h_sp_ = new QSpinBox(this);
    form->addRow(tr("Height"), h_sp_);

    drag_cb_ = new QCheckBox(tr("ROI Drag Mode (draw the rect on the main display)"), this);
    form->addRow(drag_cb_);

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

void UnifiedRoiDialog::set_state(bool enabled, int x0, int y0, int x1, int y1,
                                 bool roni, int sensor_w, int sensor_h, bool drag_mode) {
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

    enable_cb_->setChecked(enabled);
    mode_combo_->setCurrentIndex(roni ? 1 : 0);
    x_sp_->setValue(x);
    y_sp_->setValue(y);
    w_sp_->setValue(w);
    h_sp_->setValue(h);
    drag_cb_->setChecked(drag_mode);
    validate_inputs();
}

bool UnifiedRoiDialog::roi_enabled() const { return enable_cb_->isChecked(); }
bool UnifiedRoiDialog::roni() const { return mode_combo_->currentData().toBool(); }
int UnifiedRoiDialog::x() const { return x_sp_->value(); }
int UnifiedRoiDialog::y() const { return y_sp_->value(); }
int UnifiedRoiDialog::w() const { return w_sp_->value(); }
int UnifiedRoiDialog::h() const { return h_sp_->value(); }
bool UnifiedRoiDialog::drag_mode() const { return drag_cb_->isChecked(); }

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
