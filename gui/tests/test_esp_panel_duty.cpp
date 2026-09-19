// gui/tests/test_esp_panel_duty.cpp — the Anti-Flicker duty-cycle widget must
// stay in the facility's percentage domain. The HAL's
// get_min_supported_duty_cycle() reports the 1/16 register granularity as a
// fraction (0.0625, live-verified on IMX636) while get_duty_cycle()/
// set_duty_cycle() are percentages (0–100, live readback 50) — feeding the raw
// getter into the widget range used to mix both scales and let the user type
// values the hardware silently clamps to 6.25%.
// Offscreen QApplication; no camera needed (constructor state only).

#include <gtest/gtest.h>

#include <QApplication>
#include <QDoubleSpinBox>

#include "panels/esp_panel.h"

#include <memory>

TEST(EspPanelDuty, WidgetIsPercentageDomain) {
    gui::EspPanel panel;
    auto* duty = panel.findChild<QDoubleSpinBox*>(QStringLiteral("esp_af_duty_cycle"));
    ASSERT_NE(duty, nullptr);
    EXPECT_DOUBLE_EQ(duty->minimum(), 6.25);
    EXPECT_DOUBLE_EQ(duty->maximum(), 100.0);
    EXPECT_DOUBLE_EQ(duty->value(), 50.0);
    EXPECT_EQ(duty->suffix(), QStringLiteral(" %"));
}

TEST(EspPanelDuty, SubFloorFractionValuesAreClamped) {
    gui::EspPanel panel;
    auto* duty = panel.findChild<QDoubleSpinBox*>(QStringLiteral("esp_af_duty_cycle"));
    ASSERT_NE(duty, nullptr);
    duty->setValue(0.5);  // the old fraction-domain default
    EXPECT_DOUBLE_EQ(duty->value(), 6.25);
}

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
