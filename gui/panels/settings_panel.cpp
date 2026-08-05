// gui/panels/settings_panel.cpp

#include "settings_panel.h"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSettings>
#include <QScrollArea>
#include <QStackedWidget>
#include <QStyle>
#include <QVBoxLayout>
#include <utility>

#include "algorithms_panel.h"
#include "biases_panel.h"
#include "devices_panel.h"
#include "display_panel.h"
#include "esp_panel.h"
#include "file_tools_panel.h"
#include "information_panel.h"
#include "preprocessing_panel.h"
#include "roi_panel.h"
#include "statistics_panel.h"
#include "trigger_panel.h"
#include "algo_bridge/algo_bridge.h"
#include "app/file_converter.h"
#include "widgets/activity_bar.h"

namespace gui {

AbstractPanel* SettingsPanel::register_panel(std::unique_ptr<AbstractPanel> panel) {
    AbstractPanel* raw = panel.get();
    panel_index_[raw->panel_id()] = raw;
    panels_.push_back(std::move(panel));
    return raw;
}

AbstractPanel* SettingsPanel::find_panel(const QString& id) const {
    auto it = panel_index_.find(id);
    return it == panel_index_.end() ? nullptr : it->second;
}

std::vector<AbstractPanel*> SettingsPanel::panels_in_group(const QString& group) const {
    std::vector<AbstractPanel*> out;
    for (const auto& p : panels_) {
        if (p->panel_group() == group) out.push_back(p.get());
    }
    return out;
}

SettingsPanel::SettingsPanel(AlgoBridge* bridge, FileConverter* converter,
                             QWidget* parent)
    : QWidget(parent) {
    // --- VSCode-style sidebar (gui_optimization.md §10.3 + §11) ---
    // ActivityBar (48px icon column) on the left + QStackedWidget on the
    // right. Each group becomes one page containing a QScrollArea with the
    // group's panels stacked directly (no CollapsibleSection — the
    // ActivityBar already handles group switching, so per-group collapse is
    // redundant per §11.2 point 1). Selecting an ActivityBar entry switches
    // the page. The active group is persisted under "sidebar/active_group".
    auto* outer = new QHBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);

    activity_bar_ = new ActivityBar(this);
    outer->addWidget(activity_bar_);

    stacked_ = new QStackedWidget(this);
    outer->addWidget(stacked_, 1);

    // Register every panel into the registry first (ownership via panels_).
    // The unique_ptr is the authoritative owner; addWidget reparents each
    // widget into the group page (Qt removes it from the old parent first),
    // which is safe — deleting a QWidget removes it from its parent's child
    // list, so the unique_ptr can destroy it later.
    register_panel(std::make_unique<DevicesPanel>(nullptr));
    register_panel(std::make_unique<InformationPanel>(nullptr));
    register_panel(std::make_unique<StatisticsPanel>(nullptr));
    register_panel(std::make_unique<DisplayPanel>(nullptr));
    register_panel(std::make_unique<BiasesPanel>(nullptr));
    register_panel(std::make_unique<RoiPanel>(nullptr));
    register_panel(std::make_unique<EspPanel>(nullptr));
    register_panel(std::make_unique<TriggerPanel>(nullptr));
    register_panel(std::make_unique<PreprocessingPanel>(nullptr));
    register_panel(std::make_unique<FileToolsPanel>(converter, nullptr));
    register_panel(std::make_unique<AlgorithmsPanel>(bridge, nullptr));

    // Group order, icon, and tooltip (design §3.7.3 table + §10.3.4 icon
    // mapping). No default_collapsed — CollapsibleSection removed (§11.2).
    struct GroupDef {
        QString name;
        QString icon;
        QString tooltip;
    };
    const GroupDef groups[] = {
        {QStringLiteral("Camera"),          QStringLiteral("camera"),
         tr("Camera devices and connection info")},
        {QStringLiteral("Display & Stats"), QStringLiteral("chart"),
         tr("Display settings and statistics")},
        {QStringLiteral("Hardware"),        QStringLiteral("cpu"),
         tr("Biases, ROI, ESP and trigger configuration")},
        {QStringLiteral("Algorithms"),      QStringLiteral("blocks"),
         tr("Algorithm selection and preprocessing")},
        {QStringLiteral("Tools"),           QStringLiteral("tools"),
         tr("File conversion and tools")},
    };

    for (const auto& g : groups) {
        const auto panels = panels_in_group(g.name);
        if (panels.empty()) continue;

        // Each group lives in its own scrollable page so only one group is
        // visible at a time (VSCode Activity Bar behavior). Panels are added
        // directly to the page layout — no CollapsibleSection wrapper.
        auto* scroll = new QScrollArea(stacked_);
        scroll->setWidgetResizable(true);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto* host = new QWidget(scroll);
        auto* host_layout = new QVBoxLayout(host);
        host_layout->setContentsMargins(6, 6, 6, 6);
        host_layout->setSpacing(8);

        for (auto* p : panels) {
            host_layout->addWidget(p);
        }

        host_layout->addStretch(1);
        scroll->setWidget(host);

        stacked_->addWidget(scroll);
        activity_bar_->add_button(g.icon, g.name, g.tooltip);
    }

    // ActivityBar → QStackedWidget page switch.
    connect(activity_bar_, &ActivityBar::group_selected, this,
            [this](int index, const QString&) {
                if (index >= 0 && index < stacked_->count()) {
                    stacked_->setCurrentIndex(index);
                    QSettings().setValue(QStringLiteral("sidebar/active_group"),
                                         index);
                }
            });

    // ActivityBar toggle button → hide/show sidebar content (§11.2 point 5).
    connect(activity_bar_, &ActivityBar::toggle_clicked, this,
            [this]() { toggle_content(); });

    // Restore the last active group (default 0 = Camera).
    const int saved = QSettings()
                          .value(QStringLiteral("sidebar/active_group"), 0)
                          .toInt();
    activity_bar_->select(saved);
}

void SettingsPanel::refresh_icons() {
    if (activity_bar_) activity_bar_->refresh_icons();
    // Force QSS re-polish on the entire panel so all child widgets pick up the
    // new theme colors (§12.2.2 — theme color lag fix).
    style()->unpolish(this);
    style()->polish(this);
    update();
}

void SettingsPanel::toggle_content() {
    if (!stacked_) return;
    const bool new_visible = !stacked_->isVisible();
    stacked_->setVisible(new_visible);
    emit content_toggled(new_visible);
}

bool SettingsPanel::is_content_visible() const {
    return stacked_ && stacked_->isVisible();
}

// Type-safe accessors — each delegates to find_panel() + static_cast so the
// registry (panels_ / panel_index_) is the single source of truth.
InformationPanel*   SettingsPanel::information_panel()    const { return static_cast<InformationPanel*>(find_panel(QStringLiteral("information"))); }
StatisticsPanel*    SettingsPanel::statistics_panel()     const { return static_cast<StatisticsPanel*>(find_panel(QStringLiteral("statistics"))); }
DisplayPanel*       SettingsPanel::display_panel()        const { return static_cast<DisplayPanel*>(find_panel(QStringLiteral("display"))); }
DevicesPanel*       SettingsPanel::devices_panel()        const { return static_cast<DevicesPanel*>(find_panel(QStringLiteral("devices"))); }
BiasesPanel*        SettingsPanel::biases_panel()         const { return static_cast<BiasesPanel*>(find_panel(QStringLiteral("biases"))); }
RoiPanel*           SettingsPanel::roi_panel()            const { return static_cast<RoiPanel*>(find_panel(QStringLiteral("roi"))); }
EspPanel*           SettingsPanel::esp_panel()            const { return static_cast<EspPanel*>(find_panel(QStringLiteral("esp"))); }
TriggerPanel*       SettingsPanel::trigger_panel()        const { return static_cast<TriggerPanel*>(find_panel(QStringLiteral("trigger"))); }
PreprocessingPanel* SettingsPanel::preprocessing_panel() const { return static_cast<PreprocessingPanel*>(find_panel(QStringLiteral("preprocessing"))); }
AlgorithmsPanel*    SettingsPanel::algorithms_panel()     const { return static_cast<AlgorithmsPanel*>(find_panel(QStringLiteral("algorithms"))); }
FileToolsPanel*     SettingsPanel::file_tools_panel()     const { return static_cast<FileToolsPanel*>(find_panel(QStringLiteral("file_tools"))); }

} // namespace gui
