#pragma once

#include <cstdint>
#include <string_view>

namespace pendulum::gui {

// Modes selectable from the control panel; part of the firmware link protocol.
enum class ControlMode : std::uint8_t {
    Idle,
    Lqr,
    Rl,
};

// Human readable label for the GUI, e.g. "LQR".
[[nodiscard]] std::string_view to_display_name(ControlMode mode);

} // namespace pendulum::gui
