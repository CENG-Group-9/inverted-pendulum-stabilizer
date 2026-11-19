"""Run a trained policy on the cart-pole environment.

Run from ``src/models``::

    python -m rl.evaluate                      # ten episodes, printed results
    python -m rl.evaluate --render             # watch it in the MuJoCo viewer
    python -m rl.evaluate --episodes 20 --randomize-friction

Evaluation uses the nominal friction unless ``--randomize-friction`` is given,
and the policy always acts deterministically.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco.viewer
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.base_class import BaseAlgorithm

from simulation import params
from simulation.env import CartPoleEnv

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_WEIGHTS_DIRECTORY = REPOSITORY_ROOT / "models" / "weights"


@dataclass(frozen=True)
class EpisodeResult:
    """What one evaluation episode produced."""

    episode_return: float
    length: int
    upright_fraction: float
    termination_reason: str | None


class PolicyRun:
    """Steps a trained policy through one episode and collects statistics."""

    def __init__(self, environment: CartPoleEnv, model: BaseAlgorithm) -> None:
        self._environment = environment
        self._model = model
        self._observation = environment.observation_space.sample()
        self._episode_return = 0.0
        self._steps = 0
        self._upright_steps = 0
        self._termination_reason: str | None = None
        self.done = True

    def reset(self, seed: int | None = None) -> None:
        """Start a new episode."""
        self._observation, _ = self._environment.reset(seed=seed)
        self._episode_return = 0.0
        self._steps = 0
        self._upright_steps = 0
        self._termination_reason = None
        self.done = False

    def step(self) -> None:
        """Advance the policy and the environment by one control step."""
        action, _ = self._model.predict(self._observation, deterministic=True)
        self._observation, reward, terminated, truncated, info = (
            self._environment.step(action)
        )
        self._episode_return += reward
        self._steps += 1
        self._upright_steps += int(info["is_upright"])
        self._termination_reason = info["termination_reason"]
        self.done = terminated or truncated

    def result(self) -> EpisodeResult:
        """Statistics of the finished episode."""
        upright_fraction = self._upright_steps / max(self._steps, 1)
        return EpisodeResult(
            episode_return=self._episode_return,
            length=self._steps,
            upright_fraction=upright_fraction,
            termination_reason=self._termination_reason,
        )


def print_result(episode: int, result: EpisodeResult) -> None:
    """Print one episode with the metrics that matter for the report."""
    print(
        f"episode {episode:3d}:  return={result.episode_return:8.2f}  "
        f"length={result.length:5d}  upright={result.upright_fraction:5.1%}  "
        f"end={result.termination_reason}"
    )


def run_headless(environment: CartPoleEnv, model: BaseAlgorithm, arguments) -> None:
    """Run a fixed number of episodes and print a summary."""
    policy_run = PolicyRun(environment, model)
    results: list[EpisodeResult] = []

    for episode in range(arguments.episodes):
        policy_run.reset(seed=arguments.seed + episode)
        while not policy_run.done:
            policy_run.step()
        result = policy_run.result()
        results.append(result)
        print_result(episode, result)

    mean_return = sum(result.episode_return for result in results) / len(results)
    mean_length = sum(result.length for result in results) / len(results)
    mean_upright = sum(result.upright_fraction for result in results) / len(results)
    print(
        f"summary:  mean return={mean_return:.2f}  mean length={mean_length:.0f}  "
        f"mean upright={mean_upright:.1%}"
    )


def run_viewer(environment: CartPoleEnv, model: BaseAlgorithm, seed: int) -> None:
    """Watch the policy in the MuJoCo viewer until the window closes."""
    policy_run = PolicyRun(environment, model)
    policy_run.reset(seed=seed)
    episode = 0
    control_period_seconds = 1.0 / params.CONTROL_FREQUENCY_HERTZ

    with mujoco.viewer.launch_passive(environment.model, environment.data) as viewer:
        next_step_time = time.perf_counter()
        while viewer.is_running():
            if policy_run.done:
                print_result(episode, policy_run.result())
                episode += 1
                with viewer.lock():
                    policy_run.reset()

            with viewer.lock():
                policy_run.step()
            viewer.sync()

            next_step_time += control_period_seconds
            time.sleep(max(0.0, next_step_time - time.perf_counter()))


def main() -> None:
    """Load the policy and evaluate it."""
    parser = argparse.ArgumentParser(description="Run a trained cart-pole policy.")
    parser.add_argument(
        "--model-path",
        type=Path,
        default=None,
        help="saved policy file (default: models/weights/<algorithm>_final.zip)",
    )
    parser.add_argument(
        "--algorithm",
        choices=("ppo", "sac"),
        default="ppo",
        help="algorithm the policy was trained with (default: ppo)",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=10,
        help="episodes to run without the viewer (default: 10)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="first episode seed (default: 0)",
    )
    parser.add_argument(
        "--randomize-friction",
        action="store_true",
        help="draw a new friction configuration on every episode",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="watch the episodes in the MuJoCo viewer",
    )
    arguments = parser.parse_args()

    if arguments.model_path is None:
        arguments.model_path = (
            DEFAULT_WEIGHTS_DIRECTORY / f"{arguments.algorithm}_final.zip"
        )
    if not arguments.model_path.exists():
        raise SystemExit(f"policy not found: {arguments.model_path}")

    algorithm = PPO if arguments.algorithm == "ppo" else SAC
    model = algorithm.load(arguments.model_path, device="cpu")
    environment = CartPoleEnv(randomize_friction=arguments.randomize_friction)

    if arguments.render:
        run_viewer(environment, model, seed=arguments.seed)
    else:
        run_headless(environment, model, arguments)


if __name__ == "__main__":
    main()
