// gui/panels/roi_panel.cpp

#include "roi_panel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStyle>

#include <metavision/hal/facilities/i_roi.h>

#include "app/camera_controller.h"

namespace gui {

namespace {
// Refresh a widget's styling after a property change so the matching QSS
// rule (e.g. QLabel[class="hint"]) takes effect immediately.
void restyle(QWidget* w) {
    w->style()->unpolish(w);
    w->style()->polish(w);
}
} // namespace

RoiPanel::RoiPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* form = new QFormLayout(this);
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    enable_check_ = new QCheckBox(tr("Enable ROI / RONI"), this);
    form->addRow(tr("Enabled"), enable_check_);

    mode_combo_ = new QComboBox(this);
    mode_combo_->addItem(tr("ROI (keep inside)"), static_cast<int>(Metavision::I_ROI::Mode::ROI));
    mode_combo_->addItem(tr("RONI (drop inside)"), static_cast<int>(Metavision::I_ROI::Mode::RONI));
    form->addRow(tr("Mode"), mode_combo_);

    x_spin_ = new QSpinBox(this);
    x_spin_->setRange(0, 1);
    y_spin_ = new QSpinBox(this);
    y_spin_->setRange(0, 1);
    w_spin_ = new QSpinBox(this);
    w_spin_->setRange(1, 1);
    h_spin_ = new QSpinBox(this);
    h_spin_->setRange(1, 1);
    form->addRow(tr("X"),     x_spin_);
    form->addRow(tr("Y"),     y_spin_);
    form->addRow(tr("Width"), w_spin_);
    form->addRow(tr("Height"), h_spin_);

    max_windows_label_ = new QLabel(QString(), this);
    max_windows_label_->setProperty("class", "hint");
    form->addRow(QString(), max_windows_label_);

    hint_label_ = new QLabel(tr("No live camera connected."), this);
    hint_label_->setWordWrap(true);
    hint_label_->setProperty("class", "hint");
    form->addRow(QString(), hint_label_);

    // Buttons on separate rows so the row width stays within the narrower
    // sidebar (§13.3 — splitting wide rows).
    apply_btn_ = new QPushButton(tr("Apply"), this);
    clear_btn_ = new QPushButton(tr("Clear Windows"), this);
    form->addRow(QString(), apply_btn_);
    form->addRow(QString(), clear_btn_);

    setEnabled(false);

    connect(enable_check_, &QCheckBox::toggled, this, &RoiPanel::toggle_enable);
    connect(apply_btn_, &QPushButton::clicked, this, &RoiPanel::apply);
    // Re-apply when mode changes so ROI/RONI takes effect immediately
    // without requiring the user to click "Apply" again.
    connect(mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this]() { if (populated_) apply(); });
    connect(clear_btn_, &QPushButton::clicked, this, [this]() {
        if (!camera_) return;
        auto* roi = camera_->roi_facility();
        if (!roi) return;
        try {
            roi->set_windows({});
            roi->enable(false);
        } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            return;
        }
        QSignalBlocker b(enable_check_);
        enable_check_->setChecked(false);
        emit roi_applied(0, 0, 0, 0, false);
        emit info_message(tr("ROI windows cleared."));
    });
}

void RoiPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    populate();
}

void RoiPanel::on_camera_disconnected() {
    camera_ = nullptr;
    sensor_w_ = 0;
    sensor_h_ = 0;
    populated_ = false;
    setEnabled(false);
    hint_label_->setText(tr("No live camera connected."));
    hint_label_->setProperty("class", "hint");
    restyle(hint_label_);
    max_windows_label_->clear();
    QSignalBlocker b(enable_check_);
    enable_check_->setChecked(false);
}

void RoiPanel::set_roi_from_drag(int x, int y, int w, int h) {
    if (!populated_) return;
    QSignalBlocker bx(x_spin_);
    QSignalBlocker by(y_spin_);
    QSignalBlocker bw(w_spin_);
    QSignalBlocker bh(h_spin_);
    x_spin_->setValue(x);
    y_spin_->setValue(y);
    w_spin_->setValue(w);
    h_spin_->setValue(h);
    // Auto-enable on drag.
    if (!enable_check_->isChecked()) {
        enable_check_->setChecked(true);
    } else {
        apply();
    }
}

// ---------------------------------------------------------------------------

void RoiPanel::populate() {
    if (!camera_) return;
    auto* roi = camera_->roi_facility();
    if (!roi) {
        hint_label_->setText(tr("ROI not supported by this camera."));
        setEnabled(false);
        populated_ = false;
        return;
    }

    const auto& info = camera_->sensor_info();
    sensor_w_ = info.width;
    sensor_h_ = info.height;
    if (sensor_w_ <= 0 || sensor_h_ <= 0) {
        hint_label_->setText(tr("Sensor geometry unavailable."));
        setEnabled(false);
        populated_ = false;
        return;
    }

    x_spin_->setRange(0, sensor_w_ - 1);
    y_spin_->setRange(0, sensor_h_ - 1);
    w_spin_->setRange(1, sensor_w_);
    h_spin_->setRange(1, sensor_h_);
    // Default to a centered half-size window.
    x_spin_->setValue(sensor_w_ / 4);
    y_spin_->setValue(sensor_h_ / 4);
    w_spin_->setValue(sensor_w_ / 2);
    h_spin_->setValue(sensor_h_ / 2);

    size_t max_w = 0;
    try { max_w = roi->get_max_supported_windows_count(); } catch (...) {}
    max_windows_label_->setText(tr("Sensor: %1×%2 · max windows: %3")
                                    .arg(sensor_w_).arg(sensor_h_)
                                    .arg(static_cast<qulonglong>(max_w)));

    // Reflect current facility state.
    try {
        bool en = roi->is_enabled();
        QSignalBlocker b(enable_check_);
        enable_check_->setChecked(en);
        const auto mode = roi->get_mode();
        const int idx = mode == Metavision::I_ROI::Mode::RONI ? 1 : 0;
        QSignalBlocker bm(mode_combo_);
        mode_combo_->setCurrentIndex(idx);
        const auto wins = roi->get_windows();
        if (!wins.empty()) {
            const auto& w0 = wins.front();
            QSignalBlocker bx(x_spin_); x_spin_->setValue(w0.x);
            QSignalBlocker by(y_spin_); y_spin_->setValue(w0.y);
            QSignalBlocker bw(w_spin_); w_spin_->setValue(w0.width);
            QSignalBlocker bh(h_spin_); h_spin_->setValue(w0.height);
            if (en) emit roi_applied(w0.x, w0.y, w0.width, w0.height, true);
        }
    } catch (...) {}

    hint_label_->setText(tr("Drag on the display or edit values, then Apply."));
    hint_label_->setProperty("class", "info");
    restyle(hint_label_);
    setEnabled(true);
    populated_ = true;
}

void RoiPanel::toggle_enable(bool on) {
    if (!camera_) return;
    auto* roi = camera_->roi_facility();
    if (!roi) return;
    // SDK contract (I_ROI::enable): at least one window must have been set
    // before enabling. Apply first, then enable. If apply() fails, do NOT
    // call roi->enable(true) — that would violate the contract. Revert the
    // checkbox and report the error instead.
    if (on) {
        if (!apply()) {
            QSignalBlocker b(enable_check_);
            enable_check_->setChecked(false);
            return;
        }
        try {
            roi->enable(true);
        } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            QSignalBlocker b(enable_check_);
            enable_check_->setChecked(false);
            // Clear any overlay rectangle drawn by a prior roi_applied(true):
            // enabling failed so the facility is effectively off.
            emit roi_applied(0, 0, 0, 0, false);
        }
    } else {
        try {
            roi->enable(false);
        } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            QSignalBlocker b(enable_check_);
            enable_check_->setChecked(true);
            return;
        }
        emit roi_applied(0, 0, 0, 0, false);
    }
}

bool RoiPanel::apply() {
    if (!camera_ || !populated_) return false;
    auto* roi = camera_->roi_facility();
    if (!roi) return false;

    const int x = x_spin_->value();
    const int y = y_spin_->value();
    const int w = w_spin_->value();
    const int h = h_spin_->value();
    if (w <= 0 || h <= 0) {
        emit error_message(tr("ROI width and height must be positive."));
        return false;
    }

    try {
        const auto mode = static_cast<Metavision::I_ROI::Mode>(mode_combo_->currentData().toInt());
        roi->set_mode(mode);
        Metavision::I_ROI::Window win(x, y, w, h);
        roi->set_windows({win});
    } catch (const std::exception& e) {
        emit error_message(QString::fromUtf8(e.what()));
        return false;
    }
    emit roi_applied(x, y, w, h, enable_check_->isChecked());
    return true;
}

} // namespace gui
