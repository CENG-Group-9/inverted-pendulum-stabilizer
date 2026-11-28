"""Systematic optimization of swing-up controller parameters."""

import csv
import itertools
import argparse

import numpy as np

from simulation import params
from simulation.env import CartPoleEnv
from lqr.fatima_hybrid import FatimaHybridController


def run_trial(gains, seed, randomize_friction):

    env = CartPoleEnv(
        randomize_friction=randomize_friction
    )

    controller = FatimaHybridController()

    # Apply candidate controller gains
    controller.swingup.k_energy = gains[0]
    controller.swingup.k_position = gains[1]
    controller.swingup.k_velocity = gains[2]

    controller.reset()

    env.reset(seed=seed)

    dt = 1.0 / params.CONTROL_FREQUENCY_HERTZ

    max_steps = int(
        params.EPISODE_DURATION_SECONDS / dt
    )

    cart_joint = env.model.joint("cart_slide")
    pole_joint = env.model.joint("pole_hinge")

    cart_qpos = cart_joint.qposadr[0]
    pole_qpos = pole_joint.qposadr[0]

    cart_dof = cart_joint.dofadr[0]
    pole_dof = pole_joint.dofadr[0]

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

    total_reward = 0.0
    swingup_time = None

    max_displacement = abs(read_state()[0])

    terminated = False

    required_upright_steps = int(1.0 / dt)

    try:

        for step in range(max_steps):

            state = read_state()

            action = controller.compute_action(state)

            if abs(action) >= 0.999:
                saturation_steps += 1

            _, reward, terminated, truncated, info = env.step(
                np.array([action], dtype=np.float32)
            )

            total_reward += float(reward)

            state = read_state()

            theta = (
                (state[2] + np.pi) % (2 * np.pi)
            ) - np.pi

            max_displacement = max(
                max_displacement,
                abs(state[0])
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
            "return": total_reward,
            "track_failure": track_failure,
            "max_displacement": max_displacement,
            "swingup_time": swingup_time,
            "saturation": 100.0 * saturation_steps / length
        }

    finally:

        env.close()


def evaluate_candidate(gains, seeds):

    nominal_results = []
    randomized_results = []

    for seed in seeds:

        nominal_results.append(
            run_trial(gains, seed, False)
        )

        randomized_results.append(
            run_trial(gains, seed, True)
        )

    all_results = nominal_results + randomized_results

    nominal_upright = np.mean([
        r["upright"] for r in nominal_results
    ])

    randomized_upright = np.mean([
        r["upright"] for r in randomized_results
    ])

    nominal_success = 100.0 * np.mean([
        r["success"] for r in nominal_results
    ])

    randomized_success = 100.0 * np.mean([
        r["success"] for r in randomized_results
    ])

    track_failures = sum(
        r["track_failure"] for r in all_results
    )

    mean_saturation = np.mean([
        r["saturation"] for r in all_results
    ])

    max_displacement = max(
        r["max_displacement"] for r in all_results
    )

    # Preliminary optimization objective
    score = (
        0.2 * nominal_upright
        + 0.8 * randomized_upright
        + 0.2 * nominal_success
        + 0.8 * randomized_success
        - 25.0 * track_failures
        - 0.1 * mean_saturation
    )

    return {
        "k_energy": gains[0],
        "k_position": gains[1],
        "k_velocity": gains[2],
        "nominal_upright": nominal_upright,
        "randomized_upright": randomized_upright,
        "nominal_success": nominal_success,
        "randomized_success": randomized_success,
        "track_failures": track_failures,
        "mean_saturation": mean_saturation,
        "max_displacement": max_displacement,
        "score": score
    }


def optimize(num_seeds=3, quick=False):

    if quick:

        candidates = [
            (15.0, 8.0, 4.0),
            (30.0, 8.0, 2.0),
            (60.0, 8.0, 2.0)
        ]

    else:

        energy_values = [15, 30, 60, 100, 150]
        position_values = [4, 8, 12]
        velocity_values = [1, 2, 4]

        candidates = list(itertools.product(
            energy_values,
            position_values,
            velocity_values
        ))

    seeds = list(range(num_seeds))

    results = []

    filename = (
        "fatima_optimization_quick.csv"
        if quick
        else "fatima_optimization_results.csv"
    )

    print("Swing-Up Parameter Optimization")
    print("================================")

    print("Candidate combinations:", len(candidates))
    print("Seeds:", seeds)

    print(
        "Total simulation episodes:",
        len(candidates) * len(seeds) * 2
    )

    for index, gains in enumerate(candidates, start=1):

        print(
            f"\nCandidate {index}/{len(candidates)}",
            flush=True
        )

        print("Gains:", gains, flush=True)

        result = evaluate_candidate(gains, seeds)

        results.append(result)

        print(
            f"Nominal upright: "
            f"{result['nominal_upright']:.2f}%"
        )

        print(
            f"Randomized upright: "
            f"{result['randomized_upright']:.2f}%"
        )

        print(
            f"Randomized success: "
            f"{result['randomized_success']:.2f}%"
        )

        print("Track failures:", result["track_failures"])

        print(f"Score: {result['score']:.4f}", flush=True)

        # Save progress after each candidate
        ranked = sorted(
            results,
            key=lambda r: r["score"],
            reverse=True
        )

        with open(filename, "w", newline="") as file:

            writer = csv.DictWriter(
                file,
                fieldnames=list(result.keys())
            )

            writer.writeheader()
            writer.writerows(ranked)

    results.sort(
        key=lambda r: r["score"],
        reverse=True
    )

    print("\nTop Parameter Combinations")
    print("==========================")

    for rank, result in enumerate(results[:5], start=1):

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