// gui/tests/test_panel_visibility.cpp — Phase 1 capability framework: the
// facility-backed hardware panels (ESP, Trigger) show/hide per the connected
// source's capabilities via SettingsPanel::apply_source_capabilities().
// Offscreen QApplication; no camera needed (the visibility switch only
// touches QWidget state).

#include <gtest/gtest.h>

#include <QApplication>

#include "panels/settings_panel.h"

#include <memory>

namespace {

gui::SettingsPanel* make_panel() {
    // Null bridge/converter: the AlgorithmsPanel and FileToolsPanel tolerate
    // both (same as the defaulted SettingsPanel constructor arguments).
    return new gui::SettingsPanel(nullptr, nullptr);
}

} // namespace

TEST(PanelVisibility, HardwarePanelsExistByDefaultVisible) {
    std::unique_ptr<gui::SettingsPanel> sp(make_panel());
    ASSERT_NE(sp->find_panel(QStringLiteral("esp")), nullptr);
    ASSERT_NE(sp->find_panel(QStringLiteral("trigger")), nullptr);
    // A fresh widget is not explicitly hidden — isHidden() is false even
    // though the panel was never shown (no parent visible).
    EXPECT_FALSE(sp->find_panel(QStringLiteral("esp"))->isHidden());
    EXPECT_FALSE(sp->find_panel(QStringLiteral("trigger"))->isHidden());
}

TEST(PanelVisibility, AbsentCapabilitiesHideBothPanels) {
    std::unique_ptr<gui::SettingsPanel> sp(make_panel());
    sp->apply_source_capabilities(false, false);
    EXPECT_TRUE(sp->find_panel(QStringLiteral("esp"))->isHidden());
    EXPECT_TRUE(sp->find_panel(QStringLiteral("trigger"))->isHidden());
}

TEST(PanelVisibility, CapabilitiesToggleIndependently) {
    std::unique_ptr<gui::SettingsPanel> sp(make_panel());

    sp->apply_source_capabilities(true, false);
    EXPECT_FALSE(sp->find_panel(QStringLiteral("trigger"))->isHidden());
    EXPECT_TRUE(sp->find_panel(QStringLiteral("esp"))->isHidden());

    sp->apply_source_capabilities(false, true);
    EXPECT_TRUE(sp->find_panel(QStringLiteral("trigger"))->isHidden());
    EXPECT_FALSE(sp->find_panel(QStringLiteral("esp"))->isHidden());
}

TEST(PanelVisibility, ReconnectLikeRestoreShowsBothAgain) {
    std::unique_ptr<gui::SettingsPanel> sp(make_panel());
    sp->apply_source_capabilities(false, false);
    // The disconnect handler restores the pre-Phase-1 all-visible layout.
    sp->apply_source_capabilities(true, true);
    EXPECT_FALSE(sp->find_panel(QStringLiteral("esp"))->isHidden());
    EXPECT_FALSE(sp->find_panel(QStringLiteral("trigger"))->isHidden());
}

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
