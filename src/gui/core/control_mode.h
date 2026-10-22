#pragma once

#include <cstdint>
#include <string_view>

namespace pendulum::gui {

// Control modes selectable from the GUI control panel. These are part of the
// link protocol with the firmware: change them in one place, never on one side.
enum class ControlMode : std::uint8_t {
    Idle,
    Lqr,
    Rl,
};

// Human readable label for the GUI, e.g. "LQR".
[[nodiscard]] std::string_view to_display_name(ControlMode mode);

} // namespace pendulum::gui
