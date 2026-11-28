"""Evaluate hybrid LQR robustness under sensor noise."""

import argparse
import csv

import numpy as np

from simulation import params
from simulation.env import CartPoleEnv
from lqr.fatima_hybrid import FatimaHybridController


# Noise standard deviations:
# cart position, cart velocity,
# pole angle, pole angular velocity

NOISE_LEVELS = {
    "none": np.array([
        0.0, 0.0, 0.0, 0.0
    ]),

    "low": np.array([
        0.0005, 0.005, np.radians(0.1), 0.01
    ]),

    "medium": np.array([
        0.001, 0.01, np.radians(0.5), 0.05
    ]),

    "high": np.array([
        0.002, 0.02, np.radians(1.0), 0.1
    ])
}


def run_episode(seed, noise_level,
                randomize_friction=False):

    env = CartPoleEnv(
        randomize_friction=randomize_friction
    )

    controller = FatimaHybridController()

    controller.reset()
    env.reset(seed=seed)

    # Independent random generator for sensor noise
    rng = np.random.default_rng(seed + 100000)

    noise_std = NOISE_LEVELS[noise_level]

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

    total_reward = 0.0
    upright_steps = 0
    saturation_steps = 0
    consecutive_upright = 0

    mode_switches = 0
    previous_mode = controller.mode

    required_upright_steps = int(1.0 / dt)

    swingup_time = None

    max_displacement = abs(read_state()[0])

    terminated = False
    truncated = False

    balance_angles = []
    balance_positions = []

    try:

        for step in range(max_steps):

            # True physical state
            true_state = read_state()

            # Simulated measurement error
            noise = rng.normal(
                loc=0.0,
                scale=noise_std,
                size=4
            )

            measured_state = true_state + noise

            # Controller receives noisy measurements
            action = controller.compute_action(
                measured_state
            )

            if controller.mode != previous_mode:

                mode_switches += 1
                previous_mode = controller.mode

            if abs(action) >= 0.999:

                saturation_steps += 1

            _, reward, terminated, truncated, _ = (
                env.step(
                    np.array(
                        [action],
                        dtype=np.float32
                    )
                )
            )

            total_reward += float(reward)

            # Evaluate using true physical state
            true_state = read_state()

            x = true_state[0]

            theta = (
                (true_state[2] + np.pi)
                % (2 * np.pi)
            ) - np.pi

            max_displacement = max(
                max_displacement, abs(x)
            )

            if (
                abs(theta)
                <= params.UPRIGHT_ANGLE_TOLERANCE_RADIANS
            ):

                upright_steps += 1
                consecutive_upright += 1

                if (
                    swingup_time is None
                    and consecutive_upright
                    >= required_upright_steps
                ):

                    swingup_time = (
                        step + 2 - required_upright_steps
                    ) * dt

            else:

                consecutive_upright = 0

            # Record balancing accuracy during
            # the final five seconds
            if (step + 1) * dt > 10.0:

                balance_angles.append(
                    np.degrees(theta)
                )

                balance_positions.append(x)

            if terminated or truncated:

                break

        episode_length = step + 1

        track_failure = (
            max_displacement
            >= params.CART_CENTER_TRAVEL_LIMIT_METERS
        )

        success = (
            not terminated
            and not track_failure
            and episode_length == max_steps
            and swingup_time is not None
            and consecutive_upright
            >= required_upright_steps
        )

        if success and balance_angles:

            angle_rms = float(
                np.sqrt(
                    np.mean(
                        np.square(balance_angles)
                    )
                )
            )

            position_rms = float(
                np.sqrt(
                    np.mean(
                        np.square(balance_positions)
                    )
                )
            )

        else:

            angle_rms = None
            position_rms = None

        return {
            "seed": seed,
            "noise_level": noise_level,
            "randomized_friction": randomize_friction,
            "return": total_reward,
            "length": episode_length,
            "upright": (
                100.0 * upright_steps / max_steps
            ),
            "swingup_time": swingup_time,
            "max_displacement": max_displacement,
            "saturation": (
                100.0 * saturation_steps
                / episode_length
            ),
            "mode_switches": mode_switches,
            "track_failure": track_failure,
            "success": success,
            "angle_rms_deg": angle_rms,
            "position_rms_m": position_rms
        }

    finally:

        env.close()


def evaluate(episodes=20, seed=700, quick=False):

    if quick:

        episodes = 3

    all_results = []

    print("Sensor Noise Robustness Evaluation")
    print("==================================")

    print("Episodes per condition:", episodes)

    print(
        "Total simulation episodes:",
        episodes * 8
    )

    for randomize_friction in [False, True]:

        for noise_level in NOISE_LEVELS:

            results = []

            print("\n----------------------------------")

            print(
                "Randomized friction:",
                randomize_friction
            )

            print("Noise level:", noise_level)

            for index in range(episodes):

                result = run_episode(
                    seed=seed + index,
                    noise_level=noise_level,
                    randomize_friction=randomize_friction
                )

                results.append(result)
                all_results.append(result)

            successful = [
                r for r in results
                if r["success"]
            ]

            success_rate = (
                100.0 * len(successful) / episodes
            )

            track_failures = sum(
                r["track_failure"]
                for r in results
            )

            mean_upright = np.mean([
                r["upright"] for r in results
            ])

            mean_return = np.mean([
                r["return"] for r in results
            ])

            max_displacement = max(
                r["max_displacement"]
                for r in results
            )

            mean_saturation = np.mean([
                r["saturation"] for r in results
            ])

            mean_switches = np.mean([
                r["mode_switches"] for r in results
            ])

            effective_time = np.mean([
                r["swingup_time"]
                if r["success"]
                else params.EPISODE_DURATION_SECONDS
                for r in results
            ])

            print(
                f"Successful episodes: "
                f"{len(successful)}/{episodes}"
            )

            print(
                f"Success rate: "
                f"{success_rate:.2f}%"
            )

            print(
                f"Mean upright: "
                f"{mean_upright:.2f}%"
            )

            print(
                f"Mean return: "
                f"{mean_return:.2f}"
            )

            print(
                f"Effective swing-up time: "
                f"{effective_time:.3f} s"
            )

            print(
                f"Maximum displacement: "
                f"{max_displacement:.4f} m"
            )

            print(
                f"Track failures: {track_failures}"
            )

            print(
                f"Mean motor saturation: "
                f"{mean_saturation:.2f}%"
            )

            print(
                f"Mean mode switches: "
                f"{mean_switches:.2f}"
            )

            if successful:

                mean_angle_rms = np.mean([
                    r["angle_rms_deg"]
                    for r in successful
                ])

                mean_position_rms = np.mean([
                    r["position_rms_m"]
                    for r in successful
                ])

                print(
                    f"Mean balancing angle RMS: "
                    f"{mean_angle_rms:.4f} degrees"
                )

                print(
                    f"Mean balancing position RMS: "
                    f"{1000 * mean_position_rms:.4f} mm"
                )

    filename = (
        "fatima_noise_quick.csv"
        if quick
        else "fatima_noise_results.csv"
    )

    with open(
        filename, "w", newline=""
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=list(all_results[0].keys())
        )

        writer.writeheader()
        writer.writerows(all_results)

    print("\n==================================")

    print(
        "Total completed episodes:",
        len(all_results)
    )

    print("Results saved to:", filename)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--episodes", type=int, default=20
    )

    parser.add_argument(
        "--seed", type=int, default=700
    )

    parser.add_argument(
        "--quick", action="store_true"
    )

    args = parser.parse_args()

    if args.episodes < 1:

        parser.error(
            "--episodes must be positive"
        )

    evaluate(
        episodes=args.episodes,
        seed=args.seed,
        quick=args.quick
    )