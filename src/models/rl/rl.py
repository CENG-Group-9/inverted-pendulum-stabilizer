"""Train a reinforcement learning policy for the cart-pole swing-up.

Run from ``src/models``::

    python -m rl.rl
    python -m rl.rl --algorithm sac --timesteps 500000

Training randomizes friction on every episode; evaluation uses the nominal
friction. Checkpoints, the best policy and the final policy are written to
``<repository>/models/weights``, which git already ignores.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

from simulation.env import CartPoleEnv

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_DIRECTORY = REPOSITORY_ROOT / "models" / "weights"
DEFAULT_CHECKPOINT_INTERVAL = 50_000
DEFAULT_EVALUATION_INTERVAL = 25_000


def parse_arguments() -> argparse.Namespace:
    """Read the command line options."""
    parser = argparse.ArgumentParser(description="Train a cart-pole swing-up policy.")
    parser.add_argument(
        "--algorithm",
        choices=("ppo", "sac"),
        default="ppo",
        help="learning algorithm (default: ppo)",
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=200_000,
        help="total environment steps to learn (default: 200000)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="random seed, also covers the environment (default: 0)",
    )
    parser.add_argument(
        "--gamma",
        type=float,
        default=0.999,
        help="discount factor; 0.999 is about a five second horizon (default)",
    )
    parser.add_argument(
        "--device",
        default="cpu",
        help="torch device, cpu or cuda (default: cpu)",
    )
    parser.add_argument(
        "--no-randomize-friction",
        dest="randomize_friction",
        action="store_false",
        help="train with the nominal friction instead of randomizing it",
    )
    parser.add_argument(
        "--eval-episodes",
        type=int,
        default=5,
        help="episodes per evaluation (default: 5)",
    )
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=DEFAULT_OUTPUT_DIRECTORY,
        help="where checkpoints and policies are written",
    )
    return parser.parse_args()


def make_environment(randomize_friction: bool) -> VecMonitor:
    """Build a monitored vectorized environment with one worker."""

    def create_environment() -> CartPoleEnv:
        return CartPoleEnv(randomize_friction=randomize_friction)

    return VecMonitor(DummyVecEnv([create_environment]))


def build_model(
    algorithm: str,
    environment: VecMonitor,
    seed: int,
    gamma: float,
    device: str,
) -> BaseAlgorithm:
    """Create the algorithm with sensible defaults for this task."""
    if algorithm == "ppo":
        return PPO(
            "MlpPolicy",
            environment,
            seed=seed,
            gamma=gamma,
            device=device,
            n_steps=2048,
            batch_size=256,
            learning_rate=3e-4,
            verbose=1,
        )
    return SAC(
        "MlpPolicy",
        environment,
        seed=seed,
        gamma=gamma,
        device=device,
        buffer_size=300_000,
        batch_size=256,
        learning_rate=3e-4,
        learning_starts=1_000,
        verbose=1,
    )


def build_callbacks(
    algorithm: str,
    output_directory: Path,
    evaluation_environment: VecMonitor,
    evaluation_episodes: int,
) -> list[CheckpointCallback | EvalCallback]:
    """Save checkpoints and evaluate the policy while it trains."""
    checkpoint_directory = output_directory / "checkpoints"
    evaluation_directory = output_directory / "evaluations"
    checkpoint_directory.mkdir(parents=True, exist_ok=True)
    evaluation_directory.mkdir(parents=True, exist_ok=True)

    checkpoint_callback = CheckpointCallback(
        save_freq=DEFAULT_CHECKPOINT_INTERVAL,
        save_path=str(checkpoint_directory),
        name_prefix=algorithm,
    )
    evaluation_callback = EvalCallback(
        evaluation_environment,
        best_model_save_path=str(evaluation_directory),
        log_path=str(evaluation_directory),
        eval_freq=DEFAULT_EVALUATION_INTERVAL,
        n_eval_episodes=evaluation_episodes,
        deterministic=True,
        verbose=1,
    )
    return [checkpoint_callback, evaluation_callback]


def main() -> None:
    """Train, evaluate and save one policy."""
    arguments = parse_arguments()
    arguments.output_directory.mkdir(parents=True, exist_ok=True)

    # A tiny network trains faster with few torch threads.
    torch.set_num_threads(4)

    training_environment = make_environment(arguments.randomize_friction)
    evaluation_environment = make_environment(randomize_friction=False)

    print(
        f"algorithm={arguments.algorithm}  timesteps={arguments.timesteps}  "
        f"gamma={arguments.gamma}  device={arguments.device}  "
        f"randomize_friction={arguments.randomize_friction}"
    )

    model = build_model(
        algorithm=arguments.algorithm,
        environment=training_environment,
        seed=arguments.seed,
        gamma=arguments.gamma,
        device=arguments.device,
    )
    model.learn(
        total_timesteps=arguments.timesteps,
        callback=build_callbacks(
            algorithm=arguments.algorithm,
            output_directory=arguments.output_directory,
            evaluation_environment=evaluation_environment,
            evaluation_episodes=arguments.eval_episodes,
        ),
    )

    policy_path = arguments.output_directory / f"{arguments.algorithm}_final"
    model.save(policy_path)
    mean_reward, standard_deviation = evaluate_policy(
        model,
        evaluation_environment,
        n_eval_episodes=arguments.eval_episodes,
        deterministic=True,
    )
    print(
        f"final evaluation: {mean_reward:.2f} +/- {standard_deviation:.2f} "
        f"over {arguments.eval_episodes} episodes"
    )
    print(f"saved policy: {policy_path}.zip")

    training_environment.close()
    evaluation_environment.close()


if __name__ == "__main__":
    main()
