// gui/panels/abstract_panel.h — common base class for all sidebar panels.
//
// Design §3.3.1 (Phase 2 architecture slimming). Provides:
//   • panel_id() / panel_title() / panel_group() — declarative identity used
//     by SettingsPanel's registry and Phase 3's CollapsibleSection grouping.
//   • on_camera_connected() / on_camera_disconnected() slots — default no-op;
//     subclasses override to populate their UI from the connected camera.
//   • bind_camera() — single entry point that disconnects the previously-bound
//     camera (if any) and forwards its connected/disconnected signals to the
//     slots above. MainWindow calls this once per panel at setup time instead
//     of reaching into each panel's on_camera_connected slot from wire_signals.
//   • info_message / error_message signals — forwarded to the status bar /
//     message box by MainWindow's forward_panel_message helper.
//
// CameraController is forward-declared to avoid pulling the HAL / SDK headers
// into every panel translation unit; only panels that actually use the camera
// include "app/camera_controller.h" in their .cpp.

#ifndef GUI_PANELS_ABSTRACT_PANEL_H
#define GUI_PANELS_ABSTRACT_PANEL_H

#include <QObject>
#include <QString>
#include <QWidget>

namespace gui {

class CameraController;

class AbstractPanel : public QWidget {
    Q_OBJECT
public:
    explicit AbstractPanel(QWidget* parent = nullptr) : QWidget(parent) {}

    /// Unique identifier (used for layout persistence, config serialization,
    /// and MainWindow lookups via SettingsPanel::find_panel).
    virtual QString panel_id() const = 0;
    /// Human-readable title shown in the collapsible-section header (§3.7).
    virtual QString panel_title() const = 0;
    /// Group key used by Phase 3's CollapsibleSection to aggregate panels.
    /// One of: "Camera" / "Display & Stats" / "Hardware" / "Algorithms" / "Tools".
    virtual QString panel_group() const = 0;

    /// @brief Binds @p cam as the camera source for this panel.
    /// Disconnects any previously-bound camera and connects the
    /// connected/disconnected signals to the on_camera_connected /
    /// on_camera_disconnected slots. Safe to call with nullptr (unbinds).
    void bind_camera(CameraController* cam);

public slots:
    /// @brief Called when a camera connects. Default no-op; subclasses
    /// override to populate their UI from the connected camera's facilities.
    virtual void on_camera_connected(CameraController* /*cam*/) {}
    /// @brief Called when the camera disconnects. Default no-op.
    virtual void on_camera_disconnected() {}

signals:
    void info_message(const QString& msg);
    void error_message(const QString& msg);

protected:
    /// The currently-bound camera (nullptr until bind_camera is called).
    /// Stored as a raw pointer; MainWindow owns the CameraController.
    CameraController* camera_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_ABSTRACT_PANEL_H
