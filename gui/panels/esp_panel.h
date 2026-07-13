// gui/panels/esp_panel.h — Event Signal Processing (design §3.4).
//
// Three sub-sections, each backed by a HAL facility:
//   • Anti-Flicker  (I_AntiFlickerModule)  — BAND_PASS/BAND_STOP + freq band
//   • Trail Filter  (I_EventTrailFilterModule) — TRAIL / STC_CUT/TRAIL / STC_KEEP
//   • ERC           (I_ErcModule)            — target event rate limiter
//
// All controls are disabled until a live camera providing the corresponding
// facility is connected. Edits apply immediately.

#ifndef GUI_PANELS_ESP_PANEL_H
#define GUI_PANELS_ESP_PANEL_H

#include <QWidget>

#include "abstract_panel.h"

class QCheckBox;
class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QLabel;
class QGroupBox;

namespace gui {

class CameraController;

class EspPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit EspPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("esp"); }
    QString panel_title() const override { return tr("ESP"); }
    QString panel_group() const override { return QStringLiteral("Hardware"); }

public slots:
    void on_camera_connected(CameraController* controller) override;
    void on_camera_disconnected() override;

private:
    void populate();
    void populate_antiflicker();
    void populate_trail();
    void populate_erc();
    void set_all_enabled(bool on);

    // Anti-Flicker
    QGroupBox*    af_group_{nullptr};
    QCheckBox*    af_enable_{nullptr};
    QComboBox*    af_mode_{nullptr};
    QComboBox*    af_preset_{nullptr};
    QSpinBox*     af_low_{nullptr};
    QSpinBox*     af_high_{nullptr};
    QDoubleSpinBox* af_duty_{nullptr};
    QSpinBox*     af_start_thr_{nullptr};
    QSpinBox*     af_stop_thr_{nullptr};

    // Trail Filter
    QGroupBox*    tf_group_{nullptr};
    QCheckBox*    tf_enable_{nullptr};
    QComboBox*    tf_type_{nullptr};
    QSpinBox*     tf_threshold_{nullptr};

    // ERC
    QGroupBox*    erc_group_{nullptr};
    QCheckBox*    erc_enable_{nullptr};
    QSpinBox*     erc_rate_{nullptr};

    QLabel*       hint_label_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_ESP_PANEL_H
