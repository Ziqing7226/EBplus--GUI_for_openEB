// gui/panels/settings_panel.h — sidebar container.
//
// VSCode-style sidebar (gui_optimization.md §10.3 + §11): an ActivityBar
// (48px icon column) on the left selects one of 5 groups; a QStackedWidget
// on the right shows the selected group's panels directly (no
// CollapsibleSection — the ActivityBar already handles group switching, so
// per-group collapse is redundant per §11.2 point 1).
//
// A toggle button at the bottom of the ActivityBar controls sidebar content
// visibility (hide/show the QStackedWidget). When hidden, the dock shrinks
// to just the ActivityBar width (48px); MainWindow updates the toggle icon
// (chevron direction) based on dock area and visibility state (§11.2 point 5).
// The dock has no title bar (§11.2 point 5).
//
// Phase 2 (§3.3.3): panels are stored in a registry (panels_) keyed by
// panel_id(), so MainWindow looks up panels via find_panel("biases") instead
// of a hardcoded accessor per panel. The registry is the single source of
// truth; the visible layout is independent of it, so programmatic
// find_panel() access is unaffected by the layout change.

#ifndef GUI_PANELS_SETTINGS_PANEL_H
#define GUI_PANELS_SETTINGS_PANEL_H

#include <QString>
#include <QWidget>
#include <memory>
#include <unordered_map>
#include <vector>

#include "abstract_panel.h"

class QGroupBox;
class QStackedWidget;

namespace gui {

class ActivityBar;
class InformationPanel;
class StatisticsPanel;
class DisplayPanel;
class DevicesPanel;
class BiasesPanel;
class RoiPanel;
class EspPanel;
class TriggerPanel;
class PreprocessingPanel;
class AlgorithmsPanel;
class FileToolsPanel;
class AlgoBridge;
class FileConverter;

class SettingsPanel : public QWidget {
    Q_OBJECT
public:
    explicit SettingsPanel(AlgoBridge* bridge = nullptr,
                           FileConverter* converter = nullptr,
                           QWidget* parent = nullptr);

    /// @brief Registers a panel into the internal registry. The panel is
    /// keyed by its panel_id(); lookups use find_panel(). Ownership of the
    /// unique_ptr transfers to SettingsPanel.
    /// @return Raw pointer to the registered panel (still valid until this
    /// SettingsPanel is destroyed).
    AbstractPanel* register_panel(std::unique_ptr<AbstractPanel> panel);

    /// @brief Looks up a panel by id. Returns nullptr if not found.
    AbstractPanel* find_panel(const QString& id) const;

    /// @brief Returns all panels whose panel_group() equals @p group.
    std::vector<AbstractPanel*> panels_in_group(const QString& group) const;

    /// @brief Returns all registered panels.
    const std::vector<std::unique_ptr<AbstractPanel>>& panels() const { return panels_; }

    // Type-safe accessors (kept for MainWindow compatibility; each delegates
    // to find_panel() + static_cast so the registry is the single source of
    // truth). Phase 3 will migrate MainWindow to find_panel() directly.
    InformationPanel*   information_panel()    const;
    StatisticsPanel*    statistics_panel()     const;
    DisplayPanel*       display_panel()        const;
    DevicesPanel*       devices_panel()        const;
    BiasesPanel*        biases_panel()         const;
    RoiPanel*           roi_panel()            const;
    EspPanel*           esp_panel()            const;
    TriggerPanel*       trigger_panel()        const;
    PreprocessingPanel* preprocessing_panel() const;
    AlgorithmsPanel*    algorithms_panel()     const;
    FileToolsPanel*     file_tools_panel()     const;

    /// @brief Installs an externally-built calibration panel (Phase 9).
    void set_calibration_panel(QWidget* panel);

    /// @brief Returns the title of the currently-active group, used by
    /// MainWindow to update the dock window title.
    QString current_title() const;

    /// @brief Re-renders the ActivityBar icons with the current theme's
    /// foreground color. Called by MainWindow on theme_changed so icons
    /// stay in sync after a light/dark switch (BUG-4 fix pattern).
    void refresh_icons();

    /// @brief Toggles the visibility of the sidebar content (QStackedWidget).
    /// When hidden, only the ActivityBar (48px) remains visible so the dock
    /// shrinks to that width. Emits content_toggled() so MainWindow can
    /// resize the dock and update the toggle button icon (§11.2 point 5).
    void toggle_content();

    /// @brief Returns whether the sidebar content is currently visible.
    bool is_content_visible() const;

signals:
    /// @brief Emitted when the active group changes (user clicks an Activity
    /// Bar entry, or the group is restored from QSettings at startup).
    void current_title_changed(const QString& title);

    /// @brief Emitted when the sidebar content visibility changes (via
    /// toggle_content()). MainWindow connects this to resize the dock and
    /// update the toggle button's chevron icon (§11.2 point 5).
    void content_toggled(bool visible);

private:
    // Registry of all panels (owns via unique_ptr; Qt parent-child also
    // applies once the panel is added to a layout, but the unique_ptr is the
    // authoritative owner — see register_panel implementation note).
    std::vector<std::unique_ptr<AbstractPanel>> panels_;
    std::unordered_map<QString, AbstractPanel*> panel_index_;

    // VSCode-style sidebar (§10.3 + §11): ActivityBar selects a group;
    // QStackedWidget shows one group's panels directly (no CollapsibleSection).
    ActivityBar* activity_bar_{nullptr};
    QStackedWidget* stacked_{nullptr};

    // Phase 9 — calibration placeholder group box and any panel installed
    // into it. Tracked by pointer so set_calibration_panel can replace the
    // placeholder cleanly without recursive findChildren() searches that
    // would delete QLabels inside an already-installed panel.
    QGroupBox* calibration_group_{nullptr};
    QWidget* calibration_installed_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_SETTINGS_PANEL_H
