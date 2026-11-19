"""Watch the cart-pole environment in the MuJoCo viewer.

Run from ``src/models``::

    python -m simulation.view
    python -m simulation.view --random-friction

Keys:
    Left / Right arrow: push the cart at full force for 50 ms
    Space: pause or resume
    R: reset the episode
    U: toggle an upright start and reset
"""

from __future__ import annotations

import argparse
import math
import time

import mujoco
import mujoco.viewer

from simulation import params
from simulation.env import CartPoleEnv, wrap_angle_radians

KEY_LEFT_ARROW = 263
KEY_RIGHT_ARROW = 262
KEY_SPACE = 32
KEY_R = 82
KEY_U = 85

PUSH_DURATION_SECONDS = 0.05
STATUS_INTERVAL_SECONDS = 0.5


class ManualControl:
    """Turns key presses into force commands and reset requests."""

    def __init__(self, steps_per_push: int) -> None:
        self._steps_per_push = steps_per_push
        self._direction = 0
        self._steps_remaining = 0
        self.paused = False
        self.reset_requested = False
        self.start_upright = False

    def on_key(self, keycode: int) -> None:
        """Handle one key press from the viewer."""
        if keycode == KEY_RIGHT_ARROW:
            self._push(direction=1)
        elif keycode == KEY_LEFT_ARROW:
            self._push(direction=-1)
        elif keycode == KEY_SPACE:
            self.paused = not self.paused
        elif keycode == KEY_R:
            self.reset_requested = True
        elif keycode == KEY_U:
            self.start_upright = not self.start_upright
            self.reset_requested = True

    def force_command(self) -> float:
        """Force command of the current push, or zero when it is over."""
        if self._steps_remaining <= 0:
            return 0.0
        self._steps_remaining -= 1
        return float(self._direction)

    def _push(self, direction: int) -> None:
        self._direction = direction
        self._steps_remaining = self._steps_per_push


def start_episode(env: CartPoleEnv, start_upright: bool) -> None:
    """Reset the environment, upright or hanging."""
    options = (
        {"pole_angle_radians": params.POLE_ANGLE_UPRIGHT_RADIANS}
        if start_upright
        else None
    )
    env.reset(options=options)


def print_status(env: CartPoleEnv) -> None:
    """Print one line with the current state."""
    state = env.state()
    angle_degrees = math.degrees(wrap_angle_radians(state.pole_angle_radians))
    print(
        f"x={state.cart_position_meters:+.3f} m  "
        f"angle={angle_degrees:+6.1f} deg  "
        f"x_dot={state.cart_velocity_meters_per_second:+.2f} m/s  "
        f"angle_dot={state.pole_angular_velocity_radians_per_second:+.2f} rad/s"
    )


def main() -> None:
    """Open the viewer and run the simulation until the window closes."""
    parser = argparse.ArgumentParser(description="Watch the cart-pole simulation.")
    parser.add_argument(
        "--random-friction",
        action="store_true",
        help="draw a new friction configuration on every reset",
    )
    arguments = parser.parse_args()

    env = CartPoleEnv(randomize_friction=arguments.random_friction)
    env.reset(seed=0)

    steps_per_push = round(PUSH_DURATION_SECONDS * params.CONTROL_FREQUENCY_HERTZ)
    control = ManualControl(steps_per_push=steps_per_push)
    control_period_seconds = 1.0 / params.CONTROL_FREQUENCY_HERTZ

    with mujoco.viewer.launch_passive(
        env.model, env.data, key_callback=control.on_key
    ) as viewer:
        next_step_time = time.perf_counter()
        last_status_time = 0.0

        while viewer.is_running():
            if control.reset_requested:
                with viewer.lock():
                    start_episode(env, control.start_upright)
                control.reset_requested = False
                last_status_time = 0.0

            if not control.paused:
                with viewer.lock():
                    _, _, terminated, truncated, info = env.step(
                        [control.force_command()]
                    )
                viewer.sync()

                if terminated or truncated:
                    print(f"episode ended: {info['termination_reason']}")
                    with viewer.lock():
                        start_episode(env, control.start_upright)

                now = time.perf_counter()
                if now - last_status_time > STATUS_INTERVAL_SECONDS:
                    print_status(env)
                    last_status_time = now

            next_step_time += control_period_seconds
            time.sleep(max(0.0, next_step_time - time.perf_counter()))


if __name__ == "__main__":
    main()
