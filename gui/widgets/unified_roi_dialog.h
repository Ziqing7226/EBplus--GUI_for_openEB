// gui/widgets/unified_roi_dialog.h — unified ROI settings dialog (Phase 2.6
// debug D-6).
//
// Modal dialog editing the single unified ROI state (live hardware ROI /
// file software crop). Opened from the "ROI Settings..." buttons on the
// Hardware and Algorithms pages (and automatically when either "Enable ROI"
// checkbox is turned on). All values are validated before OK is allowed.

#ifndef GUI_WIDGETS_UNIFIED_ROI_DIALOG_H
#define GUI_WIDGETS_UNIFIED_ROI_DIALOG_H

#include <QDialog>

class QCheckBox;
class QComboBox;
class QLabel;
class QPushButton;
class QSpinBox;

namespace gui {

class UnifiedRoiDialog : public QDialog {
    Q_OBJECT
public:
    explicit UnifiedRoiDialog(QWidget* parent = nullptr);

    /// @brief Fills the controls from the current unified state and sets the
    /// sensor-dependent spin ranges. Call before exec(). Rect [x0,x1)×[y0,y1);
    /// an empty rect (w/h <= 0) initializes to the default center 256×144.
    /// @p drag_mode = current ROI drag-mode state of the main display.
    void set_state(bool enabled, int x0, int y0, int x1, int y1, bool roni,
                   int sensor_w, int sensor_h, bool drag_mode);

    bool roi_enabled() const;
    bool roni() const;
    /// Rectangle in the set_unified_roi convention: x/y = -1 = auto-center.
    int x() const;
    int y() const;
    int w() const;
    int h() const;
    bool drag_mode() const;

private slots:
    /// @brief Cross-field validation (x+w <= sensor_w etc.). Disables OK and
    /// shows a hint while invalid (Phase 2.6 debug D-6, user requirement).
    void validate_inputs();

private:
    QCheckBox* enable_cb_{nullptr};
    QComboBox* mode_combo_{nullptr};
    QSpinBox* x_sp_{nullptr};
    QSpinBox* y_sp_{nullptr};
    QSpinBox* w_sp_{nullptr};
    QSpinBox* h_sp_{nullptr};
    QCheckBox* drag_cb_{nullptr};
    QLabel* hint_lbl_{nullptr};
    QPushButton* ok_btn_{nullptr};
    int sensor_w_{1280};
    int sensor_h_{720};
};

} // namespace gui

#endif // GUI_WIDGETS_UNIFIED_ROI_DIALOG_H
