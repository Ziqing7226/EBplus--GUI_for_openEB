// gui/panels/roi_panel.h — unified ROI entry on the Hardware page (Phase 2.6
// debug D-6).
//
// Reduced to a single "Enable ROI" checkbox + "ROI Settings..." button (the
// rect/mode/drag settings live in the modal UnifiedRoiDialog). The ROI
// works on both live cameras (hardware I_ROI) and file playback (software
// crop); all writes go through CameraController::set_unified_roi.

#ifndef GUI_PANELS_ROI_PANEL_H
#define GUI_PANELS_ROI_PANEL_H

#include <QWidget>

#include "abstract_panel.h"

class QCheckBox;
class QPushButton;

namespace gui {

class CameraController;

class RoiPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit RoiPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("roi"); }
    QString panel_title() const override { return tr("ROI"); }
    QString panel_group() const override { return QStringLiteral("Hardware"); }

public slots:
    void on_camera_connected(CameraController* controller) override;
    void on_camera_disconnected() override;

    /// @brief Syncs the "Enable ROI" checkbox from the unified ROI state
    /// (driven by CameraController::roi_state_changed via MainWindow;
    /// QSignalBlocker — does not re-emit roi_enable_toggled).
    void set_roi_enabled(bool enabled);

signals:
    /// @brief Emitted when the USER toggles "Enable ROI". MainWindow applies
    /// it via CameraController::set_unified_roi (and opens the settings
    /// dialog when turned on).
    void roi_enable_toggled(bool on);

    /// @brief Emitted when the "ROI Settings..." button is clicked.
    void roi_settings_requested();

private:
    QCheckBox*    enable_cb_{nullptr};
    QPushButton*  settings_btn_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_ROI_PANEL_H
