"""Evaluate the independent hybrid LQR controller."""

import argparse
import numpy as np

from simulation import params
from simulation.env import CartPoleEnv
from lqr.fatima_hybrid import FatimaHybridController


def run_episode(
    seed=0,
    randomize_friction=False,
    gains=None
):

    env = CartPoleEnv(
        randomize_friction=randomize_friction
    )

    controller = FatimaHybridController()

    if gains is not None:
        controller.swingup.k_energy = gains[0]
        controller.swingup.k_position = gains[1]
        controller.swingup.k_velocity = gains[2]

    controller.reset()

    env.reset(seed=seed)

    dt = 1.0 / params.CONTROL_FREQUENCY_HERTZ

    max_steps = int(
        params.EPISODE_DURATION_SECONDS / dt
    )

    # Locate cart and pole joints
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

    total_reward = 0.0
    upright_steps = 0
    saturation_steps = 0
    mode_switches = 0

    swingup_time = None
    max_displacement = abs(read_state()[0])

    previous_mode = controller.mode

    terminated = False
    truncated = False

    consecutive_upright = 0
    required_upright_steps = int(1.0 / dt)

    for step in range(max_steps):

        state = read_state()

        action = controller.compute_action(state)

        if controller.mode != previous_mode:
            mode_switches += 1
            previous_mode = controller.mode

        if abs(action) >= 0.999:
            saturation_steps += 1

        _, reward, terminated, truncated, info = env.step(
            np.array([action], dtype=np.float32)
        )

        total_reward += float(reward)

        state = read_state()

        x = state[0]

        theta = (
            (state[2] + np.pi) % (2 * np.pi)
        ) - np.pi

        max_displacement = max(
            max_displacement, abs(x)
        )

        # Measure upright performance
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

    episode_length = step + 1

    upright_percentage = (
        100.0 * upright_steps / episode_length
    )

    saturation_percentage = (
        100.0 * saturation_steps / episode_length
    )

    track_failure = (
        max_displacement
        >= params.CART_CENTER_TRAVEL_LIMIT_METERS
    )

    success = (
        not terminated
        and not track_failure
        and episode_length == max_steps
        and swingup_time is not None
        and consecutive_upright >= required_upright_steps
    )

    results = {
        "return": total_reward,
        "length": episode_length,
        "upright": upright_percentage,
        "swingup_time": swingup_time,
        "max_displacement": max_displacement,
        "saturation": saturation_percentage,
        "mode_switches": mode_switches,
        "track_failure": track_failure,
        "success": success
    }

    env.close()

    return results


def evaluate(
    episodes=20,
    randomize_friction=False,
    seed=0,
    gains=None
):

    all_results = []

    print("Independent Hybrid Controller Evaluation")
    print("========================================")

    print("Episodes:", episodes)
    print("Randomized friction:", randomize_friction)
    print("Starting seed:", seed)

    for episode in range(episodes):

        result = run_episode(
            seed=seed + episode,
            randomize_friction=randomize_friction,
            gains=gains
        )

        all_results.append(result)

        print(f"\nEpisode {episode}")

        print(f"Return: {result['return']:.2f}")
        print(f"Length: {result['length']}")
        print(f"Upright: {result['upright']:.2f}%")
        print(f"Swing-up time: {result['swingup_time']}")
        print(f"Maximum displacement: {result['max_displacement']:.4f} m")
        print(f"Motor saturation: {result['saturation']:.2f}%")
        print(f"Mode switches: {result['mode_switches']}")
        print(f"Track failure: {result['track_failure']}")
        print(f"Success: {result['success']}")

    mean_return = np.mean([
        r["return"] for r in all_results
    ])

    mean_length = np.mean([
        r["length"] for r in all_results
    ])

    mean_upright = np.mean([
        r["upright"] for r in all_results
    ])

    swingup_times = [
        r["swingup_time"]
        for r in all_results
        if r["swingup_time"] is not None
    ]

    successes = sum(
        r["success"] for r in all_results
    )

    failures = sum(
        r["track_failure"] for r in all_results
    )

    print("\nOverall Evaluation Results")
    print("==========================")

    print(f"Mean return: {mean_return:.2f}")
    print(f"Mean episode length: {mean_length:.1f}")
    print(f"Mean upright percentage: {mean_upright:.2f}%")

    if swingup_times:
        print(
            f"Mean swing-up time: "
            f"{np.mean(swingup_times):.3f} s"
        )
    else:
        print("Mean swing-up time: Not achieved")

    print(f"Successful episodes: {successes}/{episodes}")
    print(f"Success rate: {100 * successes / episodes:.2f}%")
    print(f"Track limit failures: {failures}")

    print(
        f"Mean motor saturation: "
        f"{np.mean([r['saturation'] for r in all_results]):.2f}%"
    )

    print(
        f"Mean mode switches: "
        f"{np.mean([r['mode_switches'] for r in all_results]):.2f}"
    )


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--episodes", type=int, default=20
    )

    parser.add_argument(
        "--randomize-friction",
        action="store_true"
    )

    parser.add_argument(
        "--seed", type=int, default=0
    )
    parser.add_argument(
        "--gains",
        type=float,
        nargs=3,
        default=None,
        metavar=("ENERGY", "POSITION", "VELOCITY")
    )
    args = parser.parse_args()

    if args.episodes < 1:
        parser.error("--episodes must be positive")

    evaluate(
        episodes=args.episodes,
        randomize_friction=args.randomize_friction,
        seed=args.seed,
        gains=args.gains
    )