"""Independent hybrid energy-based swing-up and LQR controller."""

import numpy as np

from lqr.fatima_controller import FatimaLQRController
from lqr.fatima_swingup import FatimaSwingUpController
from lqr.fatima_dynamics import F_max, x_limit


class FatimaHybridController:
    """Combine energy-based swing-up with LQR balancing."""

    def __init__(self):

        self.lqr = FatimaLQRController()
        self.swingup = FatimaSwingUpController()

        # Switching thresholds
        self.enter_angle = np.radians(20.0)
        self.exit_angle = np.radians(25.0)

        self.enter_velocity = 3.0
        self.exit_velocity = 6.0

        self.capture_position = 0.25

        # Track protection parameters
        self.soft_limit = 0.30
        self.hard_limit = 0.36

        self.k_boundary = 30.0
        self.k_braking = 5.0

        self.mode = "swingup"

    def reset(self):
        """Reset controller mode before each episode."""

        self.mode = "swingup"

    def update_mode(self, state):
        """Select swing-up or LQR using hysteresis."""

        x, x_dot, theta, theta_dot = state

        theta = (
            (theta + np.pi) % (2 * np.pi)
        ) - np.pi

        if self.mode == "swingup":

            if (
                abs(theta) <= self.enter_angle
                and abs(theta_dot) <= self.enter_velocity
                and abs(x) <= self.capture_position
            ):
                self.mode = "lqr"

        elif self.mode == "lqr":

            if (
                abs(theta) >= self.exit_angle
                or abs(theta_dot) >= self.exit_velocity
            ):
                self.mode = "swingup"

    def protect_track(self, state, force):
        """Reduce the risk of exceeding track limits."""

        x, x_dot, _, _ = state

        direction = np.sign(x)

        outward_velocity = max(
            0.0, direction * x_dot
        )

        if abs(x) > self.soft_limit:

            correction = (
                self.k_boundary
                * (abs(x) - self.soft_limit)
                + self.k_braking * outward_velocity
            )

            force -= direction * correction

        # Strong inward correction near the boundary
        if abs(x) >= self.hard_limit:

            force = -direction * F_max

        return float(np.clip(
            force, -F_max, F_max
        ))

    def compute_force(self, state):
        """Calculate force using the selected controller."""

        self.update_mode(state)

        if self.mode == "lqr":

            force = self.lqr.compute_force(state)

        else:

            force = self.swingup.compute_force(state)

        force = self.protect_track(state, force)

        return force

    def compute_action(self, state):
        """Convert motor force to normalized action."""

        force = self.compute_force(state)

        return float(np.clip(
            force / F_max, -1.0, 1.0
        ))


if __name__ == "__main__":

    controller = FatimaHybridController()

    print("Independent Hybrid Controller")
    print("=============================")

    test_states = [
        ("Hanging position",
         [0.0, 0.0, np.pi, 0.0]),

        ("Near upright - 8 degrees",
         [0.0, 0.0, np.radians(8), 0.0]),

        ("Hysteresis - 18 degrees",
         [0.0, 0.0, np.radians(18), 0.0]),

        ("Loss of balance - 40 degrees",
         [0.0, 0.0, np.radians(40), 0.0]),

        ("Near upright but rotating quickly",
         [0.0, 0.0, np.radians(5), 8.0]),

        ("Near right track boundary",
         [0.37, 0.2, np.pi, 0.0]),

        ("Near left track boundary",
         [-0.37, -0.2, np.pi, 0.0])
    ]

    for name, state in test_states:

        force = controller.compute_force(state)
        action = force / F_max

        print(f"\nTest: {name}")
        print("Controller mode:", controller.mode)
        print(f"Motor force: {force:.6f} N")
        print(f"Normalized action: {action:.6f}")
        print("Within force limit:", abs(force) <= F_max)