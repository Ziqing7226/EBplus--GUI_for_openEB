// gui/panels/display_panel.h — frame display parameters (accumulation, fps, palette).
//
// Phase 1 scope (design §3.2.1): accumulation time + color theme.
// Frame-rate and fps-limit controls are unified with the playback bar — both
// UIs read from / write to the same FramePipeline parameters.
// Frame-mode selection (7 modes) is wired in Phase 5; the combo is present
// but disabled here.

#ifndef GUI_PANELS_DISPLAY_PANEL_H
#define GUI_PANELS_DISPLAY_PANEL_H

#include <QWidget>

class QSlider;
class QDoubleSpinBox;
class QSpinBox;
class QComboBox;

namespace gui {

class DisplayPanel : public QWidget {
    Q_OBJECT
public:
    explicit DisplayPanel(QWidget* parent = nullptr);

    int accumulation_time_us() const;
    int color_palette_index() const;
    int frame_mode_index() const;
    int fps() const;
    int fps_limit() const;

    /// @brief Sets the active frame mode (0..6). See design §3.2.2.
    void set_frame_mode(int index);

    /// @brief Sets the accumulation time (ms) and updates the slider/spin
    /// without emitting signals. Used to sync the UI when a frame-mode preset
    /// is selected or when the FramePipeline reports a value change.
    void set_accumulation_time_ms(double ms);

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
    void frame_mode_changed(int index);    // 0=Diff,1=Integration,2=Histogram,
                                           // 3=TimeDecay,4=ContrastMap,5=Periodic,
                                           // 6=OnDemand

private:
    QSlider* accum_slider_{nullptr};
    QDoubleSpinBox* accum_spin_{nullptr};
    QSpinBox* fps_spin_{nullptr};
    QSpinBox* fps_limit_spin_{nullptr};
    QComboBox* palette_combo_{nullptr};
    QComboBox* mode_combo_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_DISPLAY_PANEL_H
