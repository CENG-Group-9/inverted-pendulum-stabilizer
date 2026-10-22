#include "control_mode.h"

namespace pendulum::gui {

std::string_view to_display_name(ControlMode mode) {
    switch (mode) {
    case ControlMode::Idle:
        return "Idle";
    case ControlMode::Lqr:
        return "LQR";
    case ControlMode::Rl:
        return "RL";
    }
    // Unreachable for a valid enumerator; keeps -Wswitch-value expansion honest.
    return "Unknown";
}

} // namespace pendulum::gui
