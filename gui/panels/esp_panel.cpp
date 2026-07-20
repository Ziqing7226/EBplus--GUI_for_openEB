// gui/panels/esp_panel.cpp

#include "esp_panel.h"

#include <algorithm>
#include <climits>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStyle>
#include <QVBoxLayout>

#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>

#include "app/camera_controller.h"

namespace gui {

namespace {
// Convenience: wraps a QFormLayout in a QGroupBox with a checkable title.
QGroupBox* make_group(QWidget* parent, const QString& title) {
    auto* gb = new QGroupBox(title, parent);
    auto* l = new QFormLayout(gb);
    l->setContentsMargins(8, 8, 8, 8);
    l->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    return gb;
}
} // namespace

EspPanel::EspPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(6);

    hint_label_ = new QLabel(tr("Connect a live camera to enable ESP settings."), this);
    hint_label_->setWordWrap(true);
    hint_label_->setProperty("class", "hint");
    outer->addWidget(hint_label_);

    // --- Anti-Flicker -----------------------------------------------------
    af_group_ = make_group(this, tr("Anti-Flicker"));
    auto* af_form = qobject_cast<QFormLayout*>(af_group_->layout());
    af_enable_ = new QCheckBox(tr("Enable"), af_group_);
    af_form->addRow(tr("Enabled"), af_enable_);
    af_mode_ = new QComboBox(af_group_);
    af_mode_->addItem(tr("Band Stop (drop in-band)"),
                      static_cast<int>(Metavision::I_AntiFlickerModule::BAND_STOP));
    af_mode_->addItem(tr("Band Pass (keep in-band)"),
                      static_cast<int>(Metavision::I_AntiFlickerModule::BAND_PASS));
    af_form->addRow(tr("Mode"), af_mode_);
    af_preset_ = new QComboBox(af_group_);
    af_preset_->addItem(tr("Custom"));
    af_preset_->addItem(tr("50 Hz mains (band 90–110 Hz)"));
    af_preset_->addItem(tr("60 Hz mains (band 110–130 Hz)"));
    af_form->addRow(tr("Preset"), af_preset_);
    af_low_ = new QSpinBox(af_group_);
    af_low_->setRange(1, 100000);
    af_low_->setSuffix(" Hz");
    af_low_->setValue(90);
    af_high_ = new QSpinBox(af_group_);
    af_high_->setRange(1, 100000);
    af_high_->setSuffix(" Hz");
    af_high_->setValue(110);
    // Low/high on separate rows so the row fits the narrower sidebar
    // (§13.3 — splitting wide rows).
    af_form->addRow(tr("Freq. low"), af_low_);
    af_form->addRow(tr("Freq. high"), af_high_);
    af_duty_ = new QDoubleSpinBox(af_group_);
    af_duty_->setRange(0.0, 1.0);
    af_duty_->setSingleStep(0.05);
    af_duty_->setValue(0.5);
    af_form->addRow(tr("Duty cycle"), af_duty_);
    af_start_thr_ = new QSpinBox(af_group_);
    af_start_thr_->setRange(0, 1000000);
    af_start_thr_->setValue(1);
    af_form->addRow(tr("Start threshold"), af_start_thr_);
    af_stop_thr_ = new QSpinBox(af_group_);
    af_stop_thr_->setRange(0, 1000000);
    af_stop_thr_->setValue(1);
    af_form->addRow(tr("Stop threshold"), af_stop_thr_);
    outer->addWidget(af_group_);

    // --- Trail Filter -----------------------------------------------------
    tf_group_ = make_group(this, tr("Trail Filter"));
    auto* tf_form = qobject_cast<QFormLayout*>(tf_group_->layout());
    tf_enable_ = new QCheckBox(tr("Enable"), tf_group_);
    tf_form->addRow(tr("Enabled"), tf_enable_);
    tf_type_ = new QComboBox(tf_group_);
    tf_type_->addItem(tr("TRAIL"),
                      static_cast<int>(Metavision::I_EventTrailFilterModule::Type::TRAIL));
    tf_type_->addItem(tr("STC Cut Trail"),
                      static_cast<int>(Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL));
    tf_type_->addItem(tr("STC Keep Trail"),
                      static_cast<int>(Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL));
    tf_form->addRow(tr("Type"), tf_type_);
    tf_threshold_ = new QSpinBox(tf_group_);
    tf_threshold_->setRange(0, 10000000);
    tf_threshold_->setSuffix(" \xC2\xB5s"); // µs
    tf_threshold_->setValue(1000);
    tf_form->addRow(tr("Threshold"), tf_threshold_);
    outer->addWidget(tf_group_);

    // --- ERC --------------------------------------------------------------
    erc_group_ = make_group(this, tr("Event Rate Controller (ERC)"));
    auto* erc_form = qobject_cast<QFormLayout*>(erc_group_->layout());
    erc_enable_ = new QCheckBox(tr("Enable"), erc_group_);
    erc_form->addRow(tr("Enabled"), erc_enable_);
    erc_rate_ = new QSpinBox(erc_group_);
    erc_rate_->setRange(1, 1000000000);
    erc_rate_->setSuffix(" ev/s");
    erc_rate_->setValue(1000000);
    erc_form->addRow(tr("Target rate"), erc_rate_);
    outer->addWidget(erc_group_);

    outer->addStretch(1);
    set_all_enabled(false);

    // --- Wire -------------------------------------------------------------
    // Anti-Flicker
    connect(af_enable_, &QCheckBox::toggled, this, [this](bool on) {
        if (!camera_) return;
        auto* af = camera_->anti_flicker_facility();
        if (!af) return;
        try { af->enable(on); } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            QSignalBlocker b(af_enable_); af_enable_->setChecked(!on);
        }
    });
    connect(af_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
        if (!camera_) return;
        auto* af = camera_->anti_flicker_facility();
        if (!af) return;
        const auto m = static_cast<Metavision::I_AntiFlickerModule::AntiFlickerMode>(
            af_mode_->currentData().toInt());
        try { af->set_filtering_mode(m); } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
        }
    });
    connect(af_preset_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int idx) {
        if (idx == 0) return; // Custom
        QSignalBlocker bl(af_low_);
        QSignalBlocker bh(af_high_);
        if (idx == 1) { af_low_->setValue(90);  af_high_->setValue(110); }
        else          { af_low_->setValue(110); af_high_->setValue(130); }
        // Apply.
        if (!camera_) return;
        auto* af = camera_->anti_flicker_facility();
        if (!af) return;
        try { af->set_frequency_band(af_low_->value(), af_high_->value()); }
        catch (const std::exception& e) { emit error_message(QString::fromUtf8(e.what())); }
    });
    auto apply_band = [this]() {
        if (!camera_) return;
        auto* af = camera_->anti_flicker_facility();
        if (!af) return;
        const int lo = af_low_->value();
        const int hi = af_high_->value();
        if (hi < lo) { emit error_message(tr("High frequency must be >= low.")); return; }
        try { af->set_frequency_band(lo, hi); } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
        }
    };
    // Validate/apply on editingFinished (Enter, focus-out, or arrow step)
    // instead of valueChanged — per-keystroke validation popped a modal
    // error box on every intermediate value while typing (audit §六-U2).
    connect(af_low_,  &QSpinBox::editingFinished, this, apply_band);
    connect(af_high_, &QSpinBox::editingFinished, this, apply_band);
    connect(af_duty_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v) {
        if (!camera_) return;
        auto* af = camera_->anti_flicker_facility();
        if (!af) return;
        try { af->set_duty_cycle(static_cast<float>(v)); }
        catch (const std::exception& e) { emit error_message(QString::fromUtf8(e.what())); }
    });
    connect(af_start_thr_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v) {
        if (!camera_) return;
        auto* af = camera_->anti_flicker_facility();
        if (!af) return;
        try { af->set_start_threshold(static_cast<uint32_t>(v)); }
        catch (const std::exception& e) { emit error_message(QString::fromUtf8(e.what())); }
    });
    connect(af_stop_thr_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v) {
        if (!camera_) return;
        auto* af = camera_->anti_flicker_facility();
        if (!af) return;
        try { af->set_stop_threshold(static_cast<uint32_t>(v)); }
        catch (const std::exception& e) { emit error_message(QString::fromUtf8(e.what())); }
    });

    // Trail Filter
    connect(tf_enable_, &QCheckBox::toggled, this, [this](bool on) {
        if (!camera_) return;
        auto* tf = camera_->trail_filter_facility();
        if (!tf) return;
        try { tf->enable(on); } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            QSignalBlocker b(tf_enable_); tf_enable_->setChecked(!on);
        }
    });
    connect(tf_type_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
        if (!camera_) return;
        auto* tf = camera_->trail_filter_facility();
        if (!tf) return;
        const auto t = static_cast<Metavision::I_EventTrailFilterModule::Type>(
            tf_type_->currentData().toInt());
        try { tf->set_type(t); } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
        }
    });
    connect(tf_threshold_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v) {
        if (!camera_) return;
        auto* tf = camera_->trail_filter_facility();
        if (!tf) return;
        try { tf->set_threshold(static_cast<uint32_t>(v)); }
        catch (const std::exception& e) { emit error_message(QString::fromUtf8(e.what())); }
    });

    // ERC
    connect(erc_enable_, &QCheckBox::toggled, this, [this](bool on) {
        if (!camera_) return;
        auto* erc = camera_->erc_facility();
        if (!erc) return;
        try { erc->enable(on); } catch (const std::exception& e) {
            emit error_message(QString::fromUtf8(e.what()));
            QSignalBlocker b(erc_enable_); erc_enable_->setChecked(!on);
        }
    });
    connect(erc_rate_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v) {
        if (!camera_) return;
        auto* erc = camera_->erc_facility();
        if (!erc) return;
        try { erc->set_cd_event_rate(static_cast<uint32_t>(v)); }
        catch (const std::exception& e) { emit error_message(QString::fromUtf8(e.what())); }
    });
}

void EspPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    populate();
}

void EspPanel::on_camera_disconnected() {
    camera_ = nullptr;
    set_all_enabled(false);
    hint_label_->setText(tr("Connect a live camera to enable ESP settings."));
    hint_label_->setProperty("class", "hint");
    restyle(hint_label_);
}

void EspPanel::populate() {
    if (!camera_) return;
    populate_antiflicker();
    populate_trail();
    populate_erc();

    const bool any = camera_->anti_flicker_facility() ||
                     camera_->trail_filter_facility() ||
                     camera_->erc_facility();
    if (any) {
        hint_label_->setText(tr("ESP facilities loaded. Edits apply immediately."));
        hint_label_->setProperty("class", "info");
    } else {
        hint_label_->setText(tr("No ESP facilities available on this camera."));
        hint_label_->setProperty("class", "hint");
    }
    restyle(hint_label_);
    // Note: do NOT call set_all_enabled(true) here — each populate_* function
    // already enables its group if the facility exists and disables it if
    // not. Calling set_all_enabled(true) afterwards would override the
    // disable, re-enabling groups whose facility is missing (BUG-2: N8).
}

void EspPanel::populate_antiflicker() {
    auto* af = camera_ ? camera_->anti_flicker_facility() : nullptr;
    if (!af) { af_group_->setEnabled(false); return; }
    af_group_->setEnabled(true);
    QString err;
    auto first_err = [&](const std::exception& e) {
        if (err.isEmpty()) err = QString::fromUtf8(e.what());
    };
    bool enabled = false;
    try { enabled = af->is_enabled(); }
    catch (const std::exception& e) { first_err(e); }
    auto mode = Metavision::I_AntiFlickerModule::BAND_STOP;
    try { mode = af->get_filtering_mode(); }
    catch (const std::exception& e) { first_err(e); }
    uint32_t low_f = 90, high_f = 110;
    try { low_f = af->get_band_low_frequency(); }
    catch (const std::exception& e) { first_err(e); }
    try { high_f = af->get_band_high_frequency(); }
    catch (const std::exception& e) { first_err(e); }
    uint32_t min_f = 1, max_f = 100000;
    try { min_f = af->get_min_supported_frequency(); max_f = af->get_max_supported_frequency(); }
    catch (const std::exception& e) { first_err(e); }
    float duty_min = 0.0f, duty_max = 1.0f, duty = 0.5f;
    try { duty_min = af->get_min_supported_duty_cycle(); duty_max = af->get_max_supported_duty_cycle(); }
    catch (const std::exception& e) { first_err(e); }
    try { duty = af->get_duty_cycle(); }
    catch (const std::exception& e) { first_err(e); }
    uint32_t st_min = 0, st_max = 1000000, st = 1;
    try { st_min = af->get_min_supported_start_threshold(); st_max = af->get_max_supported_start_threshold(); }
    catch (const std::exception& e) { first_err(e); }
    try { st = af->get_start_threshold(); }
    catch (const std::exception& e) { first_err(e); }
    uint32_t sp_min = 0, sp_max = 1000000, sp = 1;
    try { sp_min = af->get_min_supported_stop_threshold(); sp_max = af->get_max_supported_stop_threshold(); }
    catch (const std::exception& e) { first_err(e); }
    try { sp = af->get_stop_threshold(); }
    catch (const std::exception& e) { first_err(e); }
    if (!err.isEmpty())
        emit info_message(tr("Anti-Flicker init: %1").arg(err));
    QSignalBlocker b0(af_enable_); af_enable_->setChecked(enabled);
    QSignalBlocker b1(af_mode_);
    af_mode_->setCurrentIndex(mode == Metavision::I_AntiFlickerModule::BAND_PASS ? 1 : 0);
    const int minfi = static_cast<int>(std::min<uint32_t>(min_f, INT_MAX));
    const int maxfi = static_cast<int>(std::min<uint32_t>(max_f, INT_MAX));
    QSignalBlocker b2(af_low_);
    QSignalBlocker b3(af_high_);
    // setRange BEFORE setValue: with the old (1..100000) range still active,
    // a hardware value outside it would be silently clamped and the widget
    // would show a wrong value (audit §六-U4).
    af_low_->setRange(minfi, maxfi);
    af_high_->setRange(minfi, maxfi);
    af_low_->setValue(static_cast<int>(std::min<uint32_t>(low_f, INT_MAX)));
    af_high_->setValue(static_cast<int>(std::min<uint32_t>(high_f, INT_MAX)));
    QSignalBlocker b4(af_duty_);
    af_duty_->setRange(duty_min, duty_max);
    af_duty_->setValue(duty);
    QSignalBlocker b5(af_start_thr_);
    af_start_thr_->setRange(static_cast<int>(std::min<uint32_t>(st_min, INT_MAX)),
                            static_cast<int>(std::min<uint32_t>(st_max, INT_MAX)));
    af_start_thr_->setValue(static_cast<int>(std::min<uint32_t>(st, INT_MAX)));
    QSignalBlocker b6(af_stop_thr_);
    af_stop_thr_->setRange(static_cast<int>(std::min<uint32_t>(sp_min, INT_MAX)),
                           static_cast<int>(std::min<uint32_t>(sp_max, INT_MAX)));
    af_stop_thr_->setValue(static_cast<int>(std::min<uint32_t>(sp, INT_MAX)));
    // BUG-G7: restore the af_preset_ selection based on the camera's current
    // band so the user can quickly identify the active mains frequency.
    QSignalBlocker b7(af_preset_);
    if (low_f == 90 && high_f == 110) af_preset_->setCurrentIndex(1);
    else if (low_f == 110 && high_f == 130) af_preset_->setCurrentIndex(2);
    else af_preset_->setCurrentIndex(0);
}

void EspPanel::populate_trail() {
    auto* tf = camera_ ? camera_->trail_filter_facility() : nullptr;
    if (!tf) { tf_group_->setEnabled(false); return; }
    tf_group_->setEnabled(true);
    QString err;
    auto first_err = [&](const std::exception& e) {
        if (err.isEmpty()) err = QString::fromUtf8(e.what());
    };
    std::set<Metavision::I_EventTrailFilterModule::Type> avail;
    try { avail = tf->get_available_types(); } catch (...) {}
    bool enabled = false;
    try { enabled = tf->is_enabled(); }
    catch (const std::exception& e) { first_err(e); }
    auto cur = Metavision::I_EventTrailFilterModule::Type::TRAIL;
    try { cur = tf->get_type(); }
    catch (const std::exception& e) { first_err(e); }
    uint32_t min_thr = 0, max_thr = 10000000, thr = 1000;
    try {
        min_thr = tf->get_min_supported_threshold();
        max_thr = tf->get_max_supported_threshold();
    } catch (const std::exception& e) { first_err(e); }
    try { thr = tf->get_threshold(); }
    catch (const std::exception& e) { first_err(e); }
    if (!err.isEmpty())
        emit info_message(tr("Trail Filter init: %1").arg(err));
    // Rebuild the combo from scratch every time we connect — removing
    // unsupported entries in place would permanently shrink the combo
    // across reconnects to cameras with different filter support.
    QSignalBlocker b1(tf_type_);
    tf_type_->clear();
    auto add_if = [&](const QString& label, Metavision::I_EventTrailFilterModule::Type t) {
        if (avail.empty() || avail.find(t) != avail.end()) {
            tf_type_->addItem(label, static_cast<int>(t));
        }
    };
    add_if(tr("Trail"),          Metavision::I_EventTrailFilterModule::Type::TRAIL);
    add_if(tr("STC Cut Trail"),  Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL);
    add_if(tr("STC Keep Trail"), Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL);
    QSignalBlocker b0(tf_enable_); tf_enable_->setChecked(enabled);
    const int idx = tf_type_->findData(static_cast<int>(cur));
    if (idx >= 0) tf_type_->setCurrentIndex(idx);
    QSignalBlocker b2(tf_threshold_);
    tf_threshold_->setRange(static_cast<int>(std::min<uint32_t>(min_thr, INT_MAX)),
                            static_cast<int>(std::min<uint32_t>(max_thr, INT_MAX)));
    tf_threshold_->setValue(static_cast<int>(std::min<uint32_t>(thr, INT_MAX)));
}

void EspPanel::populate_erc() {
    auto* erc = camera_ ? camera_->erc_facility() : nullptr;
    if (!erc) { erc_group_->setEnabled(false); return; }
    erc_group_->setEnabled(true);
    QString err;
    auto first_err = [&](const std::exception& e) {
        if (err.isEmpty()) err = QString::fromUtf8(e.what());
    };
    bool enabled = false;
    try { enabled = erc->is_enabled(); }
    catch (const std::exception& e) { first_err(e); }
    uint32_t min_rate = 1, max_rate = 1000000000, rate = 1000000;
    try {
        min_rate = erc->get_min_supported_cd_event_rate();
        max_rate = erc->get_max_supported_cd_event_rate();
    } catch (const std::exception& e) { first_err(e); }
    try { rate = erc->get_cd_event_rate(); }
    catch (const std::exception& e) { first_err(e); }
    if (!err.isEmpty())
        emit info_message(tr("ERC init: %1").arg(err));
    QSignalBlocker b0(erc_enable_); erc_enable_->setChecked(enabled);
    QSignalBlocker b1(erc_rate_);
    erc_rate_->setRange(static_cast<int>(std::min<uint32_t>(min_rate, INT_MAX)),
                        static_cast<int>(std::min<uint32_t>(max_rate, INT_MAX)));
    erc_rate_->setValue(static_cast<int>(std::min<uint32_t>(rate, INT_MAX)));
}

void EspPanel::set_all_enabled(bool on) {
    // Used to disable all groups (on=false) when no camera is connected.
    // The on=true case is intentionally NOT called from populate() — each
    // populate_* function manages its own group's enable state based on
    // facility availability (BUG-2: N8 fix caused set_all_enabled(true) to
    // override populate_*'s facility-missing disables).
    af_group_->setEnabled(on);
    tf_group_->setEnabled(on);
    erc_group_->setEnabled(on);
}

} // namespace gui
