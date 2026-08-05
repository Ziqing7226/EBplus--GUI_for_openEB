// gui/panels/trigger_panel.cpp

#include "trigger_panel.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStyle>
#include <QVBoxLayout>

#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_trigger_out.h>

#include "app/camera_controller.h"

namespace gui {

namespace {
QString channel_label(Metavision::I_TriggerIn::Channel ch) {
    switch (ch) {
        case Metavision::I_TriggerIn::Channel::Main:     return TriggerPanel::tr("Main");
        case Metavision::I_TriggerIn::Channel::Aux:      return TriggerPanel::tr("Aux");
        case Metavision::I_TriggerIn::Channel::Loopback: return TriggerPanel::tr("Loopback");
    }
    return TriggerPanel::tr("Unknown");
}
} // namespace

TriggerPanel::TriggerPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(6);

    hint_label_ = new QLabel(tr("No live camera connected."), this);
    hint_label_->setWordWrap(true);
    hint_label_->setProperty("class", "hint");
    outer->addWidget(hint_label_);

    // --- Trigger In -------------------------------------------------------
    tin_group_ = new QGroupBox(tr("Trigger In"), this);
    tin_layout_ = new QVBoxLayout(tin_group_);
    tin_layout_->setContentsMargins(8, 8, 8, 8);
    tin_hint_ = new QLabel(QString(), tin_group_);
    tin_hint_->setWordWrap(true);
    tin_hint_->setProperty("class", "hint");
    tin_layout_->addWidget(tin_hint_);
    outer->addWidget(tin_group_);

    // --- Trigger Out ------------------------------------------------------
    tout_group_ = new QGroupBox(tr("Trigger Out"), this);
    auto* tout_form = new QFormLayout(tout_group_);
    tout_form->setContentsMargins(8, 8, 8, 8);
    tout_form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    tout_enable_ = new QCheckBox(tr("Enable"), tout_group_);
    tout_form->addRow(tr("Enabled"), tout_enable_);

    tout_period_ = new QSpinBox(tout_group_);
    tout_period_->setRange(1, 1000000000);
    tout_period_->setSuffix(" \xC2\xB5s"); // µs
    tout_period_->setValue(1000);
    tout_form->addRow(tr("Period"), tout_period_);

    tout_duty_ = new QDoubleSpinBox(tout_group_);
    tout_duty_->setRange(0.0, 1.0);
    tout_duty_->setSingleStep(0.05);
    tout_duty_->setValue(0.5);
    tout_form->addRow(tr("Duty cycle"), tout_duty_);

    tout_freq_ = new QDoubleSpinBox(tout_group_);
    // Match the period range (1 µs .. 1000 s): 1 Hz .. 1 MHz. Above 1 MHz
    // the period would round to 0 µs and set_period(0) is invalid.
    tout_freq_->setRange(1.0, 1.0e6);
    tout_freq_->setSuffix(" Hz");
    tout_freq_->setValue(1000.0);
    tout_form->addRow(tr("Frequency"), tout_freq_);
    outer->addWidget(tout_group_);

    outer->addStretch(1);

    tin_group_->setEnabled(false);
    tout_group_->setEnabled(false);

    // --- Wire -------------------------------------------------------------
    // Trigger Out enable
    connect(tout_enable_, &QCheckBox::toggled, this, [this](bool on) {
        if (!camera_) return;
        auto* to = camera_->trigger_out_facility();
        if (!to) return;
        try {
            if (on) to->enable(); else to->disable();
        } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            QSignalBlocker b(tout_enable_); tout_enable_->setChecked(!on);
        }
    });
    // Period <-> Frequency mirror.
    // Rollback helper: after a failed set_period(), re-read the hardware's
    // actual period and mirror it into both widgets so the UI doesn't show
    // a value that never took effect (audit §六-U3, same pattern as
    // BiasesPanel::refresh_row_values).
    auto refresh_period_from_hw = [this]() {
        if (!camera_) return;
        auto* to = camera_->trigger_out_facility();
        if (!to) return;
        try {
            const uint32_t p = to->get_period();
            QSignalBlocker bp(tout_period_);
            QSignalBlocker bf(tout_freq_);
            tout_period_->setValue(p > 1000000000u ? 1000000000
                                                   : static_cast<int>(p));
            if (p > 0) tout_freq_->setValue(1.0e6 / static_cast<double>(p));
        } catch (...) {}
    };
    connect(tout_period_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            [this, refresh_period_from_hw](int us) {
        if (us <= 0) return;
        QSignalBlocker b(tout_freq_);
        tout_freq_->setValue(1.0e6 / static_cast<double>(us));
        if (!camera_) return;
        auto* to = camera_->trigger_out_facility();
        if (!to) return;
        try { to->set_period(static_cast<uint32_t>(us)); }
        catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            refresh_period_from_hw();
        }
    });
    connect(tout_freq_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this, refresh_period_from_hw](double hz) {
        if (hz <= 0.0) return;
        const int us = static_cast<int>(1.0e6 / hz);
        // Defensive: never call set_period(0) — it is invalid and can leave
        // the trigger output in an undefined state.
        if (us < 1) return;
        QSignalBlocker b(tout_period_);
        tout_period_->setValue(us);
        // The period widget's signals are blocked above, so its handler will
        // NOT fire — we must apply to the hardware here.
        if (!camera_) return;
        auto* to = camera_->trigger_out_facility();
        if (!to) return;
        try { to->set_period(static_cast<uint32_t>(us)); }
        catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            refresh_period_from_hw();
        }
    });
    connect(tout_duty_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double v) {
        if (!camera_) return;
        auto* to = camera_->trigger_out_facility();
        if (!to) return;
        try { to->set_duty_cycle(v); }
        catch (const std::exception& e) { emit error_message(QString::fromUtf8(e.what())); }
    });
}

void TriggerPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    populate();
}

void TriggerPanel::on_camera_disconnected() {
    camera_ = nullptr;
    clear_trigger_in_rows();
    tin_group_->setEnabled(false);
    tout_group_->setEnabled(false);
    tin_hint_->clear();
    hint_label_->setText(tr("No live camera connected."));
    hint_label_->setProperty("class", "hint");
    restyle(hint_label_);
}

void TriggerPanel::populate() {
    if (!camera_) return;
    populate_trigger_in();
    populate_trigger_out();

    const bool any = camera_->trigger_in_facility() || camera_->trigger_out_facility();
    if (any) {
        hint_label_->setText(tr("Trigger facilities loaded."));
        hint_label_->setProperty("class", "info");
    } else {
        hint_label_->setText(tr("No trigger facilities available on this camera."));
        hint_label_->setProperty("class", "hint");
    }
    restyle(hint_label_);
}

void TriggerPanel::populate_trigger_in() {
    clear_trigger_in_rows();
    auto* tin = camera_ ? camera_->trigger_in_facility() : nullptr;
    if (!tin) {
        tin_group_->setEnabled(false);
        tin_hint_->setText(tr("Trigger In not supported."));
        return;
    }
    tin_group_->setEnabled(true);

    std::map<Metavision::I_TriggerIn::Channel, short> avail;
    try { avail = tin->get_available_channels(); } catch (...) {}
    if (avail.empty()) {
        tin_hint_->setText(tr("No trigger-in channels exposed by this device."));
        return;
    }
    tin_hint_->setText(tr("%1 channel(s) available.").arg(avail.size()));

    for (const auto& [ch, id] : avail) {
        const int key = static_cast<int>(ch);
        auto* cb = new QCheckBox(channel_label(ch), tin_group_);
        try { cb->setChecked(tin->is_enabled(ch)); } catch (...) {}
        // Insert before the hint label (which is the first child); append after it.
        tin_layout_->addWidget(cb);
        connect(cb, &QCheckBox::toggled, this, [this, ch, cb](bool on) {
            if (!camera_) return;
            auto* tin = camera_->trigger_in_facility();
            if (!tin) return;
            try {
                if (on) tin->enable(ch); else tin->disable(ch);
            } catch (const std::exception& e) {
                emit error_message(QString::fromUtf8(e.what()));
                QSignalBlocker b(cb); cb->setChecked(!on);
            }
        });
        tin_checks_.insert(key, cb);
    }
}

void TriggerPanel::populate_trigger_out() {
    auto* to = camera_ ? camera_->trigger_out_facility() : nullptr;
    if (!to) {
        tout_group_->setEnabled(false);
        return;
    }
    tout_group_->setEnabled(true);
    try {
        QSignalBlocker b0(tout_enable_); tout_enable_->setChecked(to->is_enabled());
        const uint32_t period = to->get_period();
        // Clamp to the spinbox range (1 µs .. 1e9 µs) before casting to int:
        // static_cast<int>(period) wraps for period > INT_MAX (2.15e9) and
        // would display a negative value clamped to 1 µs.
        QSignalBlocker b1(tout_period_);
        tout_period_->setValue(period > 1000000000u ? 1000000000 : static_cast<int>(period));
        const double duty = to->get_duty_cycle();
        QSignalBlocker b2(tout_duty_); tout_duty_->setValue(duty);
        if (period > 0) {
            QSignalBlocker b3(tout_freq_);
            tout_freq_->setValue(1.0e6 / static_cast<double>(period));
        }
    } catch (const std::exception& e) {
        emit error_message(tr("Trigger Out init: %1").arg(QString::fromUtf8(e.what())));
    }
}

void TriggerPanel::clear_trigger_in_rows() {
    for (auto it = tin_checks_.begin(); it != tin_checks_.end(); ++it) {
        if (it.value()) it.value()->deleteLater();
    }
    tin_checks_.clear();
}

} // namespace gui
