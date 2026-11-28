"""Visual demonstration of the independent hybrid LQR controller.

Run from src/models:
    python -m lqr.fatima_demo
    python -m lqr.fatima_demo --randomize-friction
"""

import argparse
import time

import mujoco.viewer
import numpy as np

from simulation import params
from simulation.env import CartPoleEnv
from lqr.fatima_hybrid import FatimaHybridController


def run_demo(seed=0, randomize_friction=False):

    env = CartPoleEnv(
        randomize_friction=randomize_friction
    )

    controller = FatimaHybridController()

    dt = 1.0 / params.CONTROL_FREQUENCY_HERTZ

    episode = 0
    steps = 0
    total_reward = 0.0
    upright_steps = 0
    max_displacement = 0.0

    env.reset(seed=seed)
    controller.reset()

    previous_mode = controller.mode
    done = False

    print("Independent Hybrid LQR Demonstration")
    print("====================================")
    print("Randomized friction:", randomize_friction)
    print("Starting seed:", seed)
    print("Close the MuJoCo window to exit.")

    def read_state():

        state = env.state()

        return np.array([
            state.cart_position_meters,
            state.cart_velocity_meters_per_second,
            state.pole_angle_radians,
            state.pole_angular_velocity_radians_per_second
        ])

    try:

        with mujoco.viewer.launch_passive(
            env.model, env.data
        ) as viewer:

            # Side view of the cart and pendulum
            with viewer.lock():

                viewer.cam.lookat[:] = [
                    0.0, 0.0, 0.12
                ]

                viewer.cam.distance = 1.5
                viewer.cam.azimuth = 90.0
                viewer.cam.elevation = -10.0

            next_step_time = time.perf_counter()

            print(f"\nEpisode {episode + 1}")
            print("Mode: swingup")

            while viewer.is_running():

                if done:

                    print("\nEpisode Results")
                    print("-------------------------")

                    print(
                        f"Return: {total_reward:.2f}"
                    )

                    print(
                        f"Upright: "
                        f"{100 * upright_steps / steps:.2f}%"
                    )

                    print(
                        f"Maximum displacement: "
                        f"{max_displacement:.4f} m"
                    )

                    print(
                        f"Duration: {steps * dt:.2f} s"
                    )

                    print(
                        "Termination reason:",
                        info["termination_reason"]
                    )

                    episode += 1

                    with viewer.lock():

                        env.reset(
                            seed=seed + episode
                        )

                        controller.reset()

                    steps = 0
                    total_reward = 0.0
                    upright_steps = 0
                    max_displacement = 0.0

                    previous_mode = controller.mode
                    done = False

                    print(f"\nEpisode {episode + 1}")
                    print("Mode: swingup")

                    next_step_time = time.perf_counter()

                with viewer.lock():

                    state = read_state()

                    action = controller.compute_action(
                        state
                    )

                    if controller.mode != previous_mode:

                        print(
                            f"Time {steps * dt:.3f} s: "
                            f"{previous_mode} -> "
                            f"{controller.mode}"
                        )

                        previous_mode = controller.mode

                    _, reward, terminated, truncated, info = (
                        env.step(
                            np.array(
                                [action],
                                dtype=np.float32
                            )
                        )
                    )

                    state = read_state()

                    max_displacement = max(
                        max_displacement,
                        abs(state[0])
                    )

                    steps += 1

                    total_reward += float(reward)

                    upright_steps += int(
                        info["is_upright"]
                    )

                    done = terminated or truncated

                viewer.sync()

                next_step_time += dt

                time.sleep(
                    max(
                        0.0,
                        next_step_time
                        - time.perf_counter()
                    )
                )

    finally:

        env.close()

        print("\nDemonstration closed.")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Visualize the hybrid LQR controller."
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=0
    )

    parser.add_argument(
        "--randomize-friction",
        action="store_true"
    )

    args = parser.parse_args()

    run_demo(
        seed=args.seed,
        randomize_friction=args.randomize_friction
    )