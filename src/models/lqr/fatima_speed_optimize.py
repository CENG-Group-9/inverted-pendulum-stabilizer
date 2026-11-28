"""Focused optimization of hybrid controller swing-up speed."""

import argparse
import csv
import itertools

import numpy as np

from simulation import params
from simulation.env import CartPoleEnv

from lqr.fatima_hybrid import FatimaHybridController
from lqr.fatima_dynamics import m, g, l


def run_trial(candidate, seed, randomize_friction):

    kick, energy_gain, energy_multiplier = candidate

    env = CartPoleEnv(
        randomize_friction=randomize_friction
    )

    controller = FatimaHybridController()

    # Apply experimental parameters
    controller.swingup.kick_acceleration = kick
    controller.swingup.k_energy = energy_gain
    controller.swingup.k_position = 8.0
    controller.swingup.k_velocity = 2.0

    controller.swingup.target_energy = (
        energy_multiplier * 2.0 * m * g * l
    )

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

    upright_steps = 0
    consecutive_upright = 0
    saturation_steps = 0
    mode_switches = 0

    swingup_time = None
    previous_mode = controller.mode

    max_displacement = abs(read_state()[0])

    required_upright_steps = int(1.0 / dt)

    terminated = False

    try:

        for step in range(max_steps):

            state = read_state()

            action = controller.compute_action(state)

            if controller.mode != previous_mode:

                mode_switches += 1
                previous_mode = controller.mode

            if abs(action) >= 0.999:

                saturation_steps += 1

            _, _, terminated, truncated, _ = env.step(
                np.array([action], dtype=np.float32)
            )

            state = read_state()

            x = state[0]

            theta = (
                (state[2] + np.pi) % (2 * np.pi)
            ) - np.pi

            max_displacement = max(
                max_displacement, abs(x)
            )

            if abs(theta) <= params.UPRIGHT_ANGLE_TOLERANCE_RADIANS:

                upright_steps += 1
                consecutive_upright += 1

                if (
                    swingup_time is None
                    and consecutive_upright >= required_upright_steps
                ):

                    swingup_time = (
                        step + 2 - required_upright_steps
                    ) * dt

            else:

                consecutive_upright = 0

            if terminated or truncated:
                break

        length = step + 1

        track_failure = (
            max_displacement
            >= params.CART_CENTER_TRAVEL_LIMIT_METERS
        )

        success = (
            not terminated
            and not track_failure
            and length == max_steps
            and swingup_time is not None
            and consecutive_upright >= required_upright_steps
        )

        return {
            "success": success,
            "upright": 100.0 * upright_steps / max_steps,
            "swingup_time": swingup_time,
            "max_displacement": max_displacement,
            "track_failure": track_failure,
            "saturation": (
                100.0 * saturation_steps / length
            ),
            "mode_switches": mode_switches
        }

    finally:

        env.close()


def evaluate_candidate(candidate, num_seeds):

    nominal_results = []
    randomized_results = []

    for index in range(num_seeds):

        nominal_results.append(
            run_trial(candidate, index, False)
        )

        randomized_results.append(
            run_trial(candidate, 400 + index, True)
        )

    all_results = nominal_results + randomized_results

    nominal_success = 100.0 * np.mean([
        r["success"] for r in nominal_results
    ])

    randomized_success = 100.0 * np.mean([
        r["success"] for r in randomized_results
    ])

    nominal_upright = np.mean([
        r["upright"] for r in nominal_results
    ])

    randomized_upright = np.mean([
        r["upright"] for r in randomized_results
    ])

    # Penalize unsuccessful episodes using
    # the full episode duration.
    def effective_time(result):

        if not result["success"]:
            return params.EPISODE_DURATION_SECONDS

        return result["swingup_time"]

    nominal_time = np.mean([
        effective_time(r) for r in nominal_results
    ])

    randomized_time = np.mean([
        effective_time(r) for r in randomized_results
    ])

    track_failures = sum(
        r["track_failure"] for r in all_results
    )

    max_displacement = max(
        r["max_displacement"] for r in all_results
    )

    mean_saturation = np.mean([
        r["saturation"] for r in all_results
    ])

    mean_switches = np.mean([
        r["mode_switches"] for r in all_results
    ])

    return {
        "kick": candidate[0],
        "energy_gain": candidate[1],
        "energy_multiplier": candidate[2],
        "nominal_success": nominal_success,
        "randomized_success": randomized_success,
        "nominal_upright": nominal_upright,
        "randomized_upright": randomized_upright,
        "nominal_time": nominal_time,
        "randomized_time": randomized_time,
        "max_displacement": max_displacement,
        "track_failures": track_failures,
        "mean_saturation": mean_saturation,
        "mean_switches": mean_switches
    }


def ranking_key(result):

    # Lower ranking keys are better.
    # Reliability and track protection come first.

    return (
        result["track_failures"],
        -min(
            result["nominal_success"],
            result["randomized_success"]
        ),
        -(
            result["nominal_success"]
            + result["randomized_success"]
        ),
        max(
            0.0,
            result["max_displacement"] - 0.35
        ),
        (
            result["nominal_time"]
            + result["randomized_time"]
        ) / 2.0,
        result["mean_saturation"]
    )


def optimize(num_seeds=3, quick=False):

    if quick:

        candidates = [
            (3.0, 15.0, 1.06),
            (5.0, 15.0, 1.06),
            (7.0, 15.0, 1.06)
        ]

    else:

        candidates = list(itertools.product(
            [3.0, 5.0, 7.0],
            [14.0, 15.0, 16.0, 17.0],
            [1.04, 1.06, 1.08]
        ))

    filename = (
        "fatima_speed_quick.csv"
        if quick
        else "fatima_speed_optimization.csv"
    )

    results = []

    print("Focused Swing-Up Speed Optimization")
    print("===================================")

    print("Candidates:", len(candidates))
    print("Seeds per friction condition:", num_seeds)

    print(
        "Total simulation episodes:",
        len(candidates) * num_seeds * 2
    )

    for index, candidate in enumerate(
        candidates, start=1
    ):

        print(
            f"\nCandidate {index}/{len(candidates)}",
            flush=True
        )

        print(
            "Kick, energy gain, multiplier:",
            candidate,
            flush=True
        )

        result = evaluate_candidate(
            candidate, num_seeds
        )

        results.append(result)

        print(
            f"Nominal success: "
            f"{result['nominal_success']:.2f}%"
        )

        print(
            f"Randomized success: "
            f"{result['randomized_success']:.2f}%"
        )

        print(
            f"Nominal effective time: "
            f"{result['nominal_time']:.3f} s"
        )

        print(
            f"Randomized effective time: "
            f"{result['randomized_time']:.3f} s"
        )

        print(
            f"Maximum displacement: "
            f"{result['max_displacement']:.4f} m"
        )

        print(
            "Track failures:",
            result["track_failures"],
            flush=True
        )

        ranked = sorted(
            results, key=ranking_key
        )

        # Save results after every candidate
        with open(filename, "w", newline="") as file:

            writer = csv.DictWriter(
                file,
                fieldnames=list(result.keys())
            )

            writer.writeheader()
            writer.writerows(ranked)

    results.sort(key=ranking_key)

    print("\nTop Parameter Combinations")
    print("==========================")

    for rank, result in enumerate(
        results[:5], start=1
    ):

        print(f"\nRank {rank}")

        for key, value in result.items():

            print(f"{key}: {value}")

    print("\nResults saved to:", filename)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--seeds", type=int, default=3
    )

    parser.add_argument(
        "--quick", action="store_true"
    )

    args = parser.parse_args()

    if args.seeds < 1:
        parser.error("--seeds must be positive")

    optimize(
        num_seeds=args.seeds,
        quick=args.quick
    )