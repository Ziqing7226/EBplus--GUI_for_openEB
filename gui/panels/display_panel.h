// gui/panels/display_panel.h — frame display parameters (accumulation, fps, palette).
//
// Phase 1 scope (design §3.2.1): accumulation time + color theme.
// Frame-rate and fps-limit controls are unified with the playback bar — both
// UIs read from / write to the same FramePipeline parameters.

#ifndef GUI_PANELS_DISPLAY_PANEL_H
#define GUI_PANELS_DISPLAY_PANEL_H

#include <QWidget>

#include "abstract_panel.h"

class QSlider;
class QSpinBox;
class QComboBox;

namespace gui {

class DisplayPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit DisplayPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("display"); }
    QString panel_title() const override { return tr("Display"); }
    QString panel_group() const override { return QStringLiteral("Display & Stats"); }

    int accumulation_time_us() const;
    int fps() const;
    int fps_limit() const;

    /// @brief Sets the accumulation time (us) and updates the slider/spin
    /// without emitting signals. Used to sync the UI when the FramePipeline
    /// reports a value change.
    void set_accumulation_time_us(int us);

    /// @brief Sets the fps spinbox value without emitting signals.
    void set_fps(int fps);

    /// @brief Sets the fps-limit spinbox value and updates the fps spinbox
    /// range without emitting signals.
    void set_fps_limit(int limit);

signals:
    void accumulation_time_changed_us(int us);
    void fps_changed(int fps);
    void fps_limit_changed(int limit);
    void color_palette_changed(int index); // 0=Dark,1=Light,2=CoolWarm,3=Gray

private:
    QSlider* accum_slider_{nullptr};
    QSpinBox* accum_spin_{nullptr};
    QSpinBox* fps_spin_{nullptr};
    QSpinBox* fps_limit_spin_{nullptr};
    QComboBox* palette_combo_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_DISPLAY_PANEL_H
