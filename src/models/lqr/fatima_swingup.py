"""Independent energy-based pendulum swing-up controller."""

import numpy as np

from lqr.fatima_dynamics import (
    M, m, l, J, g, b, d,
    F_max, x_limit
)


class FatimaSwingUpController:
    """Energy-based swing-up with cart position feedback."""

    def __init__(self):

        # Optimized controller gains
        self.k_energy = 14.0
        self.k_position = 8.0
        self.k_velocity = 2.0

        # Initial excitation acceleration
        self.kick_acceleration = 7.0

        # Desired energy with additional energy margin
        self.target_energy = 1.06 * 2.0 * m * g * l

    def compute_energy(self, state):
        """Calculate the pendulum energy relative to its pivot."""

        _, _, theta, theta_dot = state

        kinetic_energy = 0.5 * J * theta_dot**2

        potential_energy = (
            m * g * l * (1.0 + np.cos(theta))
        )

        return kinetic_energy + potential_energy

    def compute_force(self, state):
        """Calculate the swing-up motor force."""

        x, x_dot, theta, theta_dot = state

        energy = self.compute_energy(state)

        energy_error = energy - self.target_energy

        # Energy-based desired cart acceleration
        acceleration = (
            self.k_energy
            * energy_error
            * theta_dot
            * np.cos(theta)
        )

        # Cart centering and velocity damping
        acceleration -= self.k_position * x
        acceleration -= self.k_velocity * x_dot

        # Initial excitation near the hanging equilibrium
        if (
            abs(theta_dot) < 0.1
            and np.cos(theta) < -0.99
            and abs(x_dot) < 0.05
            and abs(x) < 0.05
        ):
            acceleration += self.kick_acceleration

        # Pendulum angular acceleration
        theta_ddot = (
            m * g * l * np.sin(theta)
            - d * theta_dot
            - m * l * np.cos(theta) * acceleration
        ) / J

        # Convert desired acceleration into motor force
        force = (
            (M + m) * acceleration
            + m * l * np.cos(theta) * theta_ddot
            - m * l * np.sin(theta) * theta_dot**2
            + b * x_dot
        )

        return float(np.clip(force, -F_max, F_max))

    def compute_action(self, state):
        """Return a normalized simulation action."""

        force = self.compute_force(state)

        return float(np.clip(
            force / F_max, -1.0, 1.0
        ))


if __name__ == "__main__":

    controller = FatimaSwingUpController()

    print("Independent Energy-Based Swing-Up")
    print("==================================")

    print("Target energy:",
          controller.target_energy, "J")

    print("Initial excitation acceleration:",
          controller.kick_acceleration, "m/s^2")

    test_states = [
        ("Hanging position",
         [0.0, 0.0, np.pi, 0.0]),

        ("Horizontal position",
         [0.0, 0.0, np.pi / 2, 0.0]),

        ("Upright position",
         [0.0, 0.0, 0.0, 0.0]),

        ("Moving pendulum",
         [0.0, 0.0, np.pi / 2, 3.0]),

        ("Swinging near downward position",
         [0.0, 0.0, 2.8, 2.0])
    ]

    for name, state in test_states:

        energy = controller.compute_energy(state)
        force = controller.compute_force(state)
        action = controller.compute_action(state)

        print(f"\nTest: {name}")
        print(f"Pendulum energy: {energy:.6f} J")
        print(f"Motor force: {force:.6f} N")
        print(f"Normalized action: {action:.6f}")
        print("Within force limit:", abs(force) <= F_max)