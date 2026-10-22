#include "test_control_mode.h"

#include <QTest>

#include <array>
#include <cstddef>
#include <string_view>

#include "control_mode.h"

namespace pendulum::gui {

namespace {

constexpr std::array<ControlMode, 3> kAllModes{ControlMode::Idle, ControlMode::Lqr,
                                               ControlMode::Rl};

} // namespace

void ControlModeTest::display_name_is_not_empty() {
    for (const ControlMode mode : kAllModes) {
        QVERIFY(!to_display_name(mode).empty());
    }
}

void ControlModeTest::display_names_are_distinct() {
    for (std::size_t first = 0; first < kAllModes.size(); ++first) {
        for (std::size_t second = first + 1; second < kAllModes.size(); ++second) {
            QVERIFY(to_display_name(kAllModes[first]) != to_display_name(kAllModes[second]));
        }
    }
}

void ControlModeTest::display_name_matches_known_modes() {
    QCOMPARE(to_display_name(ControlMode::Idle), std::string_view{"Idle"});
    QCOMPARE(to_display_name(ControlMode::Lqr), std::string_view{"LQR"});
    QCOMPARE(to_display_name(ControlMode::Rl), std::string_view{"RL"});
}

} // namespace pendulum::gui

QTEST_APPLESS_MAIN(pendulum::gui::ControlModeTest)
