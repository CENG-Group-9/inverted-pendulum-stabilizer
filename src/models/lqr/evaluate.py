"""Evaluate the LQR + swing-up controller on the MuJoCo cart-pole environment.

Run from ``src/models``::

    python -m lqr.evaluate                      # ten swing-up episodes
    python -m lqr.evaluate --render             # watch it in the MuJoCo viewer
    python -m lqr.evaluate --episodes 20 --randomize-friction
    python -m lqr.evaluate --print-gains        # print LQR gains for firmware
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import mujoco.viewer

from lqr.lqr import LqrSwingUpController
from simulation import params
from simulation.env import CartPoleEnv


@dataclass(frozen=True)
class EpisodeResult:
    """Metrics collected from one evaluation episode."""

    episode_return: float
    length: int
    upright_fraction: float
    termination_reason: str | None


class LqrEpisodeRunner:
    """Steps the LQR + swing-up controller through one episode."""

    def __init__(
        self,
        environment: CartPoleEnv,
        controller: LqrSwingUpController,
        start_upright: bool = False,
    ) -> None:
        self._environment = environment
        self._controller = controller
        self._start_upright = start_upright
        self._episode_return = 0.0
        self._steps = 0
        self._upright_steps = 0
        self._termination_reason: str | None = None
        self.done = True

    def reset(self, seed: int | None = None) -> None:
        """Start a new episode and reset controller hysteresis."""
        options = (
            {"pole_angle_radians": params.POLE_ANGLE_UPRIGHT_RADIANS}
            if self._start_upright
            else None
        )
        self._environment.reset(seed=seed, options=options)
        self._controller.reset()
        self._episode_return = 0.0
        self._steps = 0
        self._upright_steps = 0
        self._termination_reason = None
        self.done = False

    def step(self) -> None:
        """Advance the controller and simulation by one control step."""
        force_command = self._controller.force_command(self._environment.state())
        _, reward, terminated, truncated, info = self._environment.step(
            [force_command]
        )
        self._episode_return += reward
        self._steps += 1
        self._upright_steps += int(info["is_upright"])
        self._termination_reason = info["termination_reason"]
        self.done = terminated or truncated

    def result(self) -> EpisodeResult:
        """Return summary statistics for the finished episode."""
        upright_fraction = self._upright_steps / max(self._steps, 1)
        return EpisodeResult(
            episode_return=self._episode_return,
            length=self._steps,
            upright_fraction=upright_fraction,
            termination_reason=self._termination_reason,
        )


def print_result(episode: int, result: EpisodeResult) -> None:
    """Print one episode's evaluation metrics."""
    print(
        f"episode {episode:3d}:  return={result.episode_return:8.2f}  "
        f"length={result.length:5d}  upright={result.upright_fraction:5.1%}  "
        f"end={result.termination_reason}"
    )


def run_headless(
    environment: CartPoleEnv,
    controller: LqrSwingUpController,
    episodes: int,
    seed: int,
    start_upright: bool,
) -> None:
    """Run episodes without rendering and print a summary."""
    runner = LqrEpisodeRunner(environment, controller, start_upright=start_upright)
    results: list[EpisodeResult] = []

    for episode in range(episodes):
        runner.reset(seed=seed + episode)
        while not runner.done:
            runner.step()
        result = runner.result()
        results.append(result)
        print_result(episode, result)

    mean_return = sum(item.episode_return for item in results) / len(results)
    mean_length = sum(item.length for item in results) / len(results)
    mean_upright = sum(item.upright_fraction for item in results) / len(results)
    print(
        f"summary:  mean return={mean_return:.2f}  mean length={mean_length:.0f}  "
        f"mean upright={mean_upright:.1%}"
    )


def run_viewer(
    environment: CartPoleEnv,
    controller: LqrSwingUpController,
    seed: int,
    start_upright: bool,
) -> None:
    """Watch the LQR + swing-up controller in the MuJoCo viewer."""
    runner = LqrEpisodeRunner(environment, controller, start_upright=start_upright)
    runner.reset(seed=seed)
    episode = 0
    control_period_seconds = 1.0 / params.CONTROL_FREQUENCY_HERTZ

    with mujoco.viewer.launch_passive(environment.model, environment.data) as viewer:
        next_step_time = time.perf_counter()
        while viewer.is_running():
            if runner.done:
                print_result(episode, runner.result())
                episode += 1
                with viewer.lock():
                    runner.reset(seed=seed + episode)

            with viewer.lock():
                runner.step()
            viewer.sync()

            next_step_time += control_period_seconds
            time.sleep(max(0.0, next_step_time - time.perf_counter()))


def main() -> None:
    """Parse CLI flags and evaluate the LQR controller."""
    parser = argparse.ArgumentParser(
        description="Evaluate the LQR + swing-up controller."
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
        "--start-upright",
        action="store_true",
        help="start episodes upright instead of hanging down",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="watch the episodes in the MuJoCo viewer",
    )
    parser.add_argument(
        "--print-gains",
        action="store_true",
        help="print the computed LQR gain vector and exit",
    )
    arguments = parser.parse_args()

    controller = LqrSwingUpController()
    if arguments.print_gains:
        gains = controller.gains
        print(
            "LQR gains [cart_pos, cart_vel, pole_angle, pole_ang_vel]: "
            f"[{', '.join(f'{value:.6f}' for value in gains)}]"
        )
        return

    environment = CartPoleEnv(randomize_friction=arguments.randomize_friction)
    if arguments.render:
        run_viewer(
            environment,
            controller,
            seed=arguments.seed,
            start_upright=arguments.start_upright,
        )
    else:
        run_headless(
            environment,
            controller,
            episodes=arguments.episodes,
            seed=arguments.seed,
            start_upright=arguments.start_upright,
        )


if __name__ == "__main__":
    main()
