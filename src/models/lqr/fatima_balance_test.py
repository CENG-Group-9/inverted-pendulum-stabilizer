"""Evaluate the independent LQR balancing controller."""

import numpy as np
import mujoco

from simulation import params
from simulation.env import CartPoleEnv
from lqr.fatima_controller import FatimaLQRController


def run_balance_test(initial_angle_deg):
    """Test balancing from an initial angular disturbance."""

    env = CartPoleEnv(randomize_friction=False)
    controller = FatimaLQRController()

    dt = 1.0 / params.CONTROL_FREQUENCY_HERTZ
    max_steps = int(params.EPISODE_DURATION_SECONDS / dt)

    env.reset(seed=42)

    # Locate the cart and pole joints.
    cart_joint = env.model.joint("cart_slide")
    pole_joint = env.model.joint("pole_hinge")

    cart_qpos = cart_joint.qposadr[0]
    pole_qpos = pole_joint.qposadr[0]

    cart_dof = cart_joint.dofadr[0]
    pole_dof = pole_joint.dofadr[0]

    # Set the initial state.
    env.data.qpos[cart_qpos] = 0.0
    env.data.qpos[pole_qpos] = np.radians(initial_angle_deg)

    env.data.qvel[:] = 0.0
    env.data.ctrl[:] = 0.0

    mujoco.mj_forward(env.model, env.data)

    def read_state():
        return np.array([
            env.data.qpos[cart_qpos],
            env.data.qvel[cart_dof],
            env.data.qpos[pole_qpos],
            env.data.qvel[pole_dof]
        ])

    state = read_state()

    angles = [initial_angle_deg]
    positions = [state[0]]
    angular_velocities = [state[3]]
    cart_velocities = [state[1]]
    actions = []

    terminated = False

    for step in range(max_steps):

        action = controller.compute_action(state)

        _, reward, terminated, truncated, info = env.step(
            np.array([action], dtype=np.float32)
        )

        state = read_state()

        angle = (
            (state[2] + np.pi) % (2 * np.pi)
        ) - np.pi

        angles.append(np.degrees(angle))
        positions.append(state[0])
        angular_velocities.append(state[3])
        cart_velocities.append(state[1])
        actions.append(action)

        if terminated or truncated:
            break

    angles = np.asarray(angles)
    positions = np.asarray(positions)
    actions = np.asarray(actions)

    # Performance metrics.
    upright_percentage = 100 * np.mean(
        np.abs(angles[1:]) <= 10.0
    )

    max_displacement = np.max(np.abs(positions))

    saturation_percentage = 100 * np.mean(
        np.abs(actions) >= 0.999
    )

    # Settling criteria.
    settled = (
        (np.abs(angles) <= 2.0)
        & (np.abs(positions) <= 0.02)
        & (np.abs(angular_velocities) <= 0.1)
        & (np.abs(cart_velocities) <= 0.05)
    )

    settling_time = None

    # Find when the system enters and stays within tolerance.
    for i in range(len(settled)):
        if np.all(settled[i:]) and (len(settled) - 1 - i) * dt >= 1.0:
            settling_time = i * dt
            break

    success = (
        not terminated
        and len(actions) == max_steps
        and settling_time is not None
        and max_displacement < params.CART_CENTER_TRAVEL_LIMIT_METERS
    )

    print(f"\nInitial angle: {initial_angle_deg} degrees")
    print("----------------------------------------")
    print(f"Simulation duration: {len(actions) * dt:.3f} s")
    print(f"Upright percentage: {upright_percentage:.2f}%")
    print(f"Maximum cart displacement: {max_displacement:.4f} m")
    print(f"Final angle: {angles[-1]:.4f} degrees")
    print(f"Final cart position: {positions[-1]:.4f} m")
    print(f"Motor saturation: {saturation_percentage:.2f}%")
    print("Settling time:", settling_time)
    print("Balancing successful:", success)

    env.close()

    return success


if __name__ == "__main__":

    print("Independent LQR Balancing Evaluation")
    print("====================================")

    results = []

    for angle in [5, 10, 15, 20]:
        results.append(run_balance_test(angle))

    print("\nOverall Results")
    print("===============")
    print("Successful tests:", sum(results), "/", len(results))