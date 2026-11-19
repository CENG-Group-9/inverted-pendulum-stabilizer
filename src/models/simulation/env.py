"""Cart-pole environment backed by MuJoCo.

Conventions
-----------
* Lengths are meters, masses kilograms, times seconds and angles radians.
* The cart slides along a one meter track. A cart position of zero is the
  middle of the track.
* The pole is a hinge on the cart. An angle of 0 rad points the pole straight
  up, an angle of pi rad lets it hang straight down.
* The action is a normalized force command in ``[-1, 1]``; 1.0 pushes the cart
  along +x with ``MAX_FORCE_NEWTONS``.
* The observation is
  ``[cart_position / travel_limit, cos(angle), sin(angle), cart_velocity,
  pole_angular_velocity]``.

This module owns the physics only. A controller reads :meth:`CartPoleEnv.state`
and feeds a force command back into :meth:`CartPoleEnv.step`, so the same
environment runs an LQR controller and an RL policy:

    force_command = lqr_controller.force_command(env.state())
    env.step([force_command])  # LQR

    action, _ = policy.predict(observation)
    env.step(action)  # RL

Run from ``src/models`` so that ``simulation`` can be imported:

    from simulation.env import CartPoleEnv
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from simulation import params


@dataclass(frozen=True)
class CartPoleState:
    """Full physical state of the rig, in SI units."""

    cart_position_meters: float
    pole_angle_radians: float
    cart_velocity_meters_per_second: float
    pole_angular_velocity_radians_per_second: float


def wrap_angle_radians(angle_radians: float) -> float:
    """Return the equivalent angle in ``[-pi, pi)``."""
    return (angle_radians + math.pi) % (2 * math.pi) - math.pi


def build_model_xml() -> str:
    """Return the MuJoCo XML description of the rig."""
    cart_half_width = params.CART_WIDTH_METERS / 2
    cart_half_depth = params.CART_DEPTH_METERS / 2
    cart_half_height = params.CART_HEIGHT_METERS / 2
    travel_limit = params.CART_CENTER_TRAVEL_LIMIT_METERS
    return f"""<mujoco model="cart_pole">
  <option timestep="{params.PHYSICS_TIMESTEP_SECONDS}"
          gravity="0 0 -{params.GRAVITY_METERS_PER_SECOND_SQUARED}"
          integrator="RK4"/>
  <!-- Inertia follows from the geometry and the mass of every geom. -->
  <compiler inertiafromgeom="true"/>
  <!-- No contacts: the pole never touches the cart or a table in this model. -->
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <!-- The track is drawn only: the worldbody is static and has no contacts. -->
    <geom name="rail" type="box" size="{params.TRACK_LENGTH_METERS / 2} 0.012 0.004"
          pos="0 0 -0.024" rgba="0.45 0.47 0.52 1"/>
    <geom name="rail_start_margin" type="box"
          size="{params.TRACK_EDGE_MARGIN_METERS / 2} 0.014 0.004"
          pos="{-(params.TRACK_LENGTH_METERS - params.TRACK_EDGE_MARGIN_METERS) / 2} 0 -0.024"
          rgba="0.85 0.45 0.45 1"/>
    <geom name="rail_end_margin" type="box"
          size="{params.TRACK_EDGE_MARGIN_METERS / 2} 0.014 0.004"
          pos="{(params.TRACK_LENGTH_METERS - params.TRACK_EDGE_MARGIN_METERS) / 2} 0 -0.024"
          rgba="0.85 0.45 0.45 1"/>
    <body name="cart">
      <!-- The joint limit is a backstop; the episode ends at the same place. -->
      <joint name="cart_slide" type="slide" axis="1 0 0" limited="true"
             range="{-travel_limit} {travel_limit}" solreflimit="0.005 1"/>
      <geom name="cart_body" type="box"
            size="{cart_half_width} {cart_half_depth} {cart_half_height}"
            mass="{params.CART_MASS_KILOGRAMS}" rgba="0.24 0.55 0.85 1"/>
      <body name="pole">
        <!-- Positive angles rotate the pole towards +x. No angle limit. -->
        <joint name="pole_hinge" type="hinge" axis="0 1 0"/>
        <geom name="pole_rod" type="cylinder"
              fromto="0 0 0 0 0 {params.POLE_LENGTH_METERS}"
              size="{params.POLE_RADIUS_METERS}"
              mass="{params.POLE_MASS_KILOGRAMS}" rgba="0.95 0.76 0.30 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <!-- Control 1.0 pushes along +x with MAX_FORCE_NEWTONS. -->
    <motor joint="cart_slide" gear="{params.MAX_FORCE_NEWTONS}"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""


class CartPoleEnv(gym.Env):
    """Swing-up and balancing task for the cart-pole rig.

    Args:
        randomize_friction: Draw a new friction configuration on every reset.
        friction: Friction used when ``randomize_friction`` is false.
        terminate_on_pole_angle_radians: End the episode when the pole leaves
            this angle around upright. ``None`` never ends on the angle, which
            is what swing-up training needs.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        randomize_friction: bool = False,
        friction: params.FrictionParameters = params.NOMINAL_FRICTION,
        terminate_on_pole_angle_radians: float | None = None,
    ) -> None:
        self._physics_steps_per_action = self._checked_physics_steps_per_action()
        self._max_control_steps = round(
            params.EPISODE_DURATION_SECONDS * params.CONTROL_FREQUENCY_HERTZ
        )

        self._model = mujoco.MjModel.from_xml_string(build_model_xml())
        self._data = mujoco.MjData(self._model)

        self._cart_position_address = self._model.joint("cart_slide").qposadr[0]
        self._pole_angle_address = self._model.joint("pole_hinge").qposadr[0]
        self._cart_velocity_address = self._model.joint("cart_slide").dofadr[0]
        self._pole_velocity_address = self._model.joint("pole_hinge").dofadr[0]

        self._randomize_friction = randomize_friction
        self._friction = friction
        self._active_friction = friction
        self._terminate_on_pole_angle_radians = terminate_on_pole_angle_radians
        self._elapsed_control_steps = 0

        self._apply_friction(friction)

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(5,), dtype=np.float32
        )

    @staticmethod
    def _checked_physics_steps_per_action() -> int:
        """Return how many physics steps one control step takes."""
        control_period_seconds = 1.0 / params.CONTROL_FREQUENCY_HERTZ
        physics_steps = control_period_seconds / params.PHYSICS_TIMESTEP_SECONDS
        if not math.isclose(physics_steps, round(physics_steps)):
            raise ValueError(
                "the control period must be a whole number of physics steps, "
                f"got {physics_steps}"
            )
        return round(physics_steps)

    def state(self) -> CartPoleState:
        """Return the current physical state."""
        return CartPoleState(
            cart_position_meters=float(
                self._data.qpos[self._cart_position_address]
            ),
            pole_angle_radians=float(self._data.qpos[self._pole_angle_address]),
            cart_velocity_meters_per_second=float(
                self._data.qvel[self._cart_velocity_address]
            ),
            pole_angular_velocity_radians_per_second=float(
                self._data.qvel[self._pole_velocity_address]
            ),
        )

    @property
    def active_friction(self) -> params.FrictionParameters:
        """Friction configuration used by the current episode."""
        return self._active_friction

    @property
    def model(self) -> mujoco.MjModel:
        """MuJoCo model, shared with viewers and diagnostics."""
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """MuJoCo data, shared with viewers and diagnostics."""
        return self._data

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Start a new episode.

        With no options the pole starts hanging down and the cart near the
        middle, both with a small random offset. Options override either start
        value exactly:

        * ``pole_angle_radians``
        * ``cart_position_meters``
        """
        super().reset(seed=seed)
        mujoco.mj_resetData(self._model, self._data)
        self._apply_friction(
            self._sample_friction() if self._randomize_friction else self._friction
        )

        options = {} if options is None else options
        unknown_options = set(options) - {"pole_angle_radians", "cart_position_meters"}
        if unknown_options:
            raise ValueError(f"unknown reset options: {sorted(unknown_options)}")

        pole_angle_radians = options.get("pole_angle_radians")
        if pole_angle_radians is None:
            noise = params.INITIAL_POLE_ANGLE_NOISE_RADIANS
            pole_angle_radians = params.POLE_ANGLE_HANGING_RADIANS + float(
                self.np_random.uniform(-noise, noise)
            )

        cart_position_meters = options.get("cart_position_meters")
        if cart_position_meters is None:
            noise = params.INITIAL_CART_POSITION_NOISE_METERS
            cart_position_meters = float(self.np_random.uniform(-noise, noise))

        self._data.qpos[self._cart_position_address] = float(cart_position_meters)
        self._data.qpos[self._pole_angle_address] = float(pole_angle_radians)
        mujoco.mj_forward(self._model, self._data)

        self._elapsed_control_steps = 0
        return self._observation(), {}

    def step(
        self, action: np.ndarray | float
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Advance the simulation by one control period.

        Args:
            action: Force command in ``[-1, 1]``, either a scalar or a shape
                ``(1,)`` array.

        Returns:
            Observation, reward, terminated, truncated and info. Info carries
            the full state, whether the pole is upright and why the episode
            ended.
        """
        force_command = self._validated_force_command(action)
        self._data.ctrl[0] = force_command
        for _ in range(self._physics_steps_per_action):
            mujoco.mj_step(self._model, self._data)
        self._elapsed_control_steps += 1

        state = self.state()
        is_upright = (
            abs(wrap_angle_radians(state.pole_angle_radians))
            <= params.UPRIGHT_ANGLE_TOLERANCE_RADIANS
        )
        terminated, termination_reason = self._termination(state)
        truncated = (
            not terminated and self._elapsed_control_steps >= self._max_control_steps
        )
        if truncated:
            termination_reason = "time_limit"

        reward = self._reward(state, force_command)
        info = {
            "state": state,
            "is_upright": is_upright,
            "termination_reason": termination_reason,
        }
        return self._observation(), reward, terminated, truncated, info

    def _observation(self) -> np.ndarray:
        """Normalized observation of the current state."""
        state = self.state()
        return np.array(
            [
                state.cart_position_meters / params.CART_CENTER_TRAVEL_LIMIT_METERS,
                math.cos(state.pole_angle_radians),
                math.sin(state.pole_angle_radians),
                state.cart_velocity_meters_per_second,
                state.pole_angular_velocity_radians_per_second,
            ],
            dtype=np.float32,
        )

    def _validated_force_command(self, action: np.ndarray | float) -> float:
        """Return the action as one force command in ``[-1, 1]``."""
        values = np.asarray(action, dtype=np.float64).reshape(-1)
        if values.size != 1:
            raise ValueError(
                f"action must hold exactly one value, got {values.size}"
            )
        force_command = float(values[0])
        if not math.isfinite(force_command):
            raise ValueError(f"action must be finite, got {force_command}")
        if not -1.0 <= force_command <= 1.0:
            raise ValueError(
                f"action must lie in [-1, 1], got {force_command}"
            )
        return force_command

    def _reward(self, state: CartPoleState, force_command: float) -> float:
        """Reward of one step: upright is good, distance and effort are not."""
        upright = (1.0 + math.cos(state.pole_angle_radians)) / 2.0
        cart_position_fraction = (
            state.cart_position_meters / params.CART_CENTER_TRAVEL_LIMIT_METERS
        )
        return float(
            params.UPRIGHT_REWARD_WEIGHT * upright
            - params.CART_POSITION_PENALTY_WEIGHT * cart_position_fraction**2
            - params.FORCE_PENALTY_WEIGHT * force_command**2
        )

    def _termination(self, state: CartPoleState) -> tuple[bool, str | None]:
        """Decide whether the episode failed and why."""
        state_values = (
            state.cart_position_meters,
            state.pole_angle_radians,
            state.cart_velocity_meters_per_second,
            state.pole_angular_velocity_radians_per_second,
        )
        if not all(math.isfinite(value) for value in state_values):
            return True, "numerical_divergence"

        if abs(state.cart_position_meters) > params.CART_CENTER_TRAVEL_LIMIT_METERS:
            return True, "track_limit"

        if self._terminate_on_pole_angle_radians is not None:
            angle_error = abs(wrap_angle_radians(state.pole_angle_radians))
            if angle_error > self._terminate_on_pole_angle_radians:
                return True, "pole_fell"

        return False, None

    def _sample_friction(self) -> params.FrictionParameters:
        """Draw one friction configuration for the next episode."""
        return params.FrictionParameters(
            cart_viscous_newton_seconds_per_meter=self._sample_range(
                params.CART_VISCOUS_FRICTION_RANGE
            ),
            cart_dry_newtons=self._sample_range(params.CART_DRY_FRICTION_RANGE),
            pole_viscous_newton_meter_seconds_per_radian=self._sample_range(
                params.POLE_VISCOUS_FRICTION_RANGE
            ),
            pole_dry_newton_meters=self._sample_range(params.POLE_DRY_FRICTION_RANGE),
        )

    def _sample_range(self, friction_range: params.FrictionRange) -> float:
        """Draw one uniform value from a friction range."""
        return float(
            self.np_random.uniform(friction_range.lowest, friction_range.highest)
        )

    def _apply_friction(self, friction: params.FrictionParameters) -> None:
        """Write a friction configuration into the MuJoCo model."""
        self._model.dof_damping[self._cart_velocity_address] = (
            friction.cart_viscous_newton_seconds_per_meter
        )
        self._model.dof_frictionloss[self._cart_velocity_address] = (
            friction.cart_dry_newtons
        )
        self._model.dof_damping[self._pole_velocity_address] = (
            friction.pole_viscous_newton_meter_seconds_per_radian
        )
        self._model.dof_frictionloss[self._pole_velocity_address] = (
            friction.pole_dry_newton_meters
        )
        self._active_friction = friction
