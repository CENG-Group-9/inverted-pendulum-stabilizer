"""Diagnose swing-up failures under randomized friction."""

import numpy as np

from simulation import params
from simulation.env import CartPoleEnv
from lqr.fatima_hybrid import FatimaHybridController


def diagnose(seed, gains=(15, 8, 2)):

    env = CartPoleEnv(randomize_friction=True)

    controller = FatimaHybridController()

    controller.swingup.k_energy = gains[0]
    controller.swingup.k_position = gains[1]
    controller.swingup.k_velocity = gains[2]

    controller.reset()
    env.reset(seed=seed)

    dt = 1.0 / params.CONTROL_FREQUENCY_HERTZ

    max_steps = int(
        params.EPISODE_DURATION_SECONDS / dt
    )

    cart = env.model.joint("cart_slide")
    pole = env.model.joint("pole_hinge")

    cart_qpos = cart.qposadr[0]
    pole_qpos = pole.qposadr[0]

    cart_dof = cart.dofadr[0]
    pole_dof = pole.dofadr[0]

    def read_state():

        return np.array([
            env.data.qpos[cart_qpos],
            env.data.qvel[cart_dof],
            env.data.qpos[pole_qpos],
            env.data.qvel[pole_dof]
        ])

    energies = []
    angles = []
    positions = []
    velocities = []
    angular_velocities = []
    forces = []

    mode_switches = 0
    previous_mode = controller.mode

    print(f"\nDiagnostic seed: {seed}")
    print("==========================")

    print("Controller gains:", gains)

    print("Cart viscous friction:",
          env.model.dof_damping[cart_dof])

    print("Cart dry friction:",
          env.model.dof_frictionloss[cart_dof])

    print("Pole viscous friction:",
          env.model.dof_damping[pole_dof])

    print("Pole dry friction:",
          env.model.dof_frictionloss[pole_dof])

    try:

        for step in range(max_steps):

            state = read_state()

            action = controller.compute_action(state)

            if controller.mode != previous_mode:

                mode_switches += 1

                print(
                    f"Mode switch at {step * dt:.3f} s: "
                    f"{previous_mode} -> {controller.mode}"
                )

                previous_mode = controller.mode

            forces.append(
                abs(action * params.MAX_FORCE_NEWTONS)
            )

            _, _, terminated, truncated, _ = env.step(
                np.array([action], dtype=np.float32)
            )

            state = read_state()

            x, x_dot, theta, theta_dot = state

            theta = (
                (theta + np.pi) % (2 * np.pi)
            ) - np.pi

            energy = controller.swingup.compute_energy(
                state
            )

            energies.append(energy)
            angles.append(abs(np.degrees(theta)))
            positions.append(abs(x))
            velocities.append(abs(x_dot))
            angular_velocities.append(abs(theta_dot))

            if terminated or truncated:
                break

        target = controller.swingup.target_energy

        print("\nDiagnostic Results")
        print("--------------------------")

        print(f"Target energy: {target:.6f} J")

        print(f"Maximum energy: {max(energies):.6f} J")

        print(
            f"Energy achieved: "
            f"{100 * max(energies) / target:.2f}%"
        )

        print(
            f"Closest upright angle: "
            f"{min(angles):.3f} degrees"
        )

        print(
            f"Maximum cart displacement: "
            f"{max(positions):.4f} m"
        )

        print(
            f"Maximum cart velocity: "
            f"{max(velocities):.4f} m/s"
        )

        print(
            f"Maximum angular velocity: "
            f"{max(angular_velocities):.4f} rad/s"
        )

        print(
            f"Maximum motor force: "
            f"{max(forces):.4f} N"
        )

        print(
            f"Mean absolute motor force: "
            f"{np.mean(forces):.4f} N"
        )

        print("Mode switches:", mode_switches)
        print("Final controller mode:", controller.mode)

    finally:

        env.close()


if __name__ == "__main__":

    diagnose(seed=206)
    diagnose(seed=207)
    diagnose(seed=215)
    diagnose(seed=218)