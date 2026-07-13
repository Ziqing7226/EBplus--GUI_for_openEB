// gui/panels/trigger_panel.h — Trigger In / Trigger Out (design §3.5).
//
// Trigger In:  one checkbox per available I_TriggerIn channel (Main / Aux /
//               Loopback). The facility exposes only channel enable/disable;
//               there is no mode or polarity knob in OpenEB 5.2.0.
// Trigger Out: enable + period (µs) + duty cycle (0–1). A frequency helper
//               spinbox mirrors period as Hz for convenience.

#ifndef GUI_PANELS_TRIGGER_PANEL_H
#define GUI_PANELS_TRIGGER_PANEL_H

#include <QHash>
#include <QWidget>

#include "abstract_panel.h"

class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QGroupBox;
class QVBoxLayout;

namespace gui {

class CameraController;

class TriggerPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit TriggerPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("trigger"); }
    QString panel_title() const override { return tr("Trigger"); }
    QString panel_group() const override { return QStringLiteral("Hardware"); }

public slots:
    void on_camera_connected(CameraController* controller) override;
    void on_camera_disconnected() override;

private:
    void populate();
    void populate_trigger_in();
    void populate_trigger_out();
    void clear_trigger_in_rows();

    QGroupBox*   tin_group_{nullptr};
    QVBoxLayout* tin_layout_{nullptr};
    QLabel*      tin_hint_{nullptr};
    // One checkbox per channel; key is the channel enum as int.
    QHash<int, QCheckBox*> tin_checks_;

    QGroupBox*       tout_group_{nullptr};
    QCheckBox*       tout_enable_{nullptr};
    QSpinBox*        tout_period_{nullptr};   // µs
    QDoubleSpinBox*  tout_duty_{nullptr};     // 0.0 – 1.0
    QDoubleSpinBox*  tout_freq_{nullptr};     // Hz (mirror of period)

    QLabel* hint_label_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_TRIGGER_PANEL_H
