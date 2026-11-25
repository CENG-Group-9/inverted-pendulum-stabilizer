"""LQR stabilization and energy-based swing-up controller for the cart-pole rig.

The controller combines two phases:
1. Energy-based swing-up (Astrom-Furuta) with cart centering when the pole is
   far from upright.
2. Discrete-time LQR stabilization around the upright equilibrium
   ``[cart_position=0, cart_velocity=0, pole_angle=0, pole_angular_velocity=0]``.

Run from ``src/models`` so that ``simulation`` can be imported::

    from lqr.lqr import LqrSwingUpController
    from simulation.env import CartPoleEnv
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import linalg, signal

from simulation import params
from simulation.env import CartPoleState, wrap_angle_radians


@dataclass(frozen=True)
class LqrConfig:
    """Tuning parameters for LQR stabilization and energy-based swing-up."""

    cart_position_cost: float = 15.0
    cart_velocity_cost: float = 1.0
    pole_angle_cost: float = 60.0
    pole_angular_velocity_cost: float = 2.0
    force_cost: float = 0.5
    switch_to_lqr_angle_radians: float = math.radians(30.0)
    switch_to_swing_up_angle_radians: float = math.radians(40.0)
    swing_up_energy_gain: float = 12.0
    swing_up_max_energy_acceleration: float = 9.5
    swing_up_centering_position_gain: float = 8.5
    swing_up_centering_velocity_gain: float = 3.0
    swing_up_kick_acceleration: float = 2.0
    control_frequency_hertz: float = float(params.CONTROL_FREQUENCY_HERTZ)


def pole_moment_of_inertia_about_center() -> float:
    """Return the cylinder pole's moment of inertia about its center of mass."""
    length = params.POLE_LENGTH_METERS
    radius = params.POLE_RADIUS_METERS
    mass = params.POLE_MASS_KILOGRAMS
    return mass * (length**2 / 12.0 + radius**2 / 4.0)


def compute_linearized_system(
    friction: params.FrictionParameters = params.NOMINAL_FRICTION,
) -> tuple[np.ndarray, np.ndarray]:
    """Return continuous-time ``(state_matrix, input_matrix)`` around upright.

    State vector is ``[cart_position_meters, cart_velocity_meters_per_second,
    pole_angle_radians, pole_angular_velocity_radians_per_second]``, and input
    is horizontal cart force in newtons.
    """
    cart_mass = params.CART_MASS_KILOGRAMS
    pole_mass = params.POLE_MASS_KILOGRAMS
    half_pole_length = params.POLE_LENGTH_METERS / 2.0
    gravity = params.GRAVITY_METERS_PER_SECOND_SQUARED

    total_mass = cart_mass + pole_mass
    pole_inertia_about_hinge = (
        pole_moment_of_inertia_about_center() + pole_mass * half_pole_length**2
    )
    mass_product = pole_mass * half_pole_length
    determinant = total_mass * pole_inertia_about_hinge - mass_product**2

    cart_viscous = friction.cart_viscous_newton_seconds_per_meter
    pole_viscous = friction.pole_viscous_newton_meter_seconds_per_radian

    state_matrix = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [
                0.0,
                -pole_inertia_about_hinge * cart_viscous / determinant,
                -(mass_product**2) * gravity / determinant,
                mass_product * pole_viscous / determinant,
            ],
            [0.0, 0.0, 0.0, 1.0],
            [
                0.0,
                mass_product * cart_viscous / determinant,
                total_mass * mass_product * gravity / determinant,
                -total_mass * pole_viscous / determinant,
            ],
        ],
        dtype=np.float64,
    )

    input_matrix = np.array(
        [
            [0.0],
            [pole_inertia_about_hinge / determinant],
            [0.0],
            [-mass_product / determinant],
        ],
        dtype=np.float64,
    )
    return state_matrix, input_matrix


def compute_lqr_gains(
    config: LqrConfig = LqrConfig(),
    friction: params.FrictionParameters = params.NOMINAL_FRICTION,
) -> np.ndarray:
    """Compute discrete-time LQR feedback gains ``K`` for ``u = -K @ state``."""
    continuous_state_matrix, continuous_input_matrix = compute_linearized_system(
        friction=friction
    )
    output_matrix = np.eye(4, dtype=np.float64)
    feedthrough_matrix = np.zeros((4, 1), dtype=np.float64)
    sample_period_seconds = 1.0 / config.control_frequency_hertz

    discrete_state_matrix, discrete_input_matrix, _, _, _ = signal.cont2discrete(
        (
            continuous_state_matrix,
            continuous_input_matrix,
            output_matrix,
            feedthrough_matrix,
        ),
        dt=sample_period_seconds,
        method="zoh",
    )

    state_cost_matrix = np.diag(
        [
            config.cart_position_cost,
            config.cart_velocity_cost,
            config.pole_angle_cost,
            config.pole_angular_velocity_cost,
        ]
    )
    input_cost_matrix = np.array([[config.force_cost]], dtype=np.float64)

    riccati_solution = linalg.solve_discrete_are(
        discrete_state_matrix,
        discrete_input_matrix,
        state_cost_matrix,
        input_cost_matrix,
    )
    gain_matrix = np.linalg.solve(
        input_cost_matrix
        + discrete_input_matrix.T @ riccati_solution @ discrete_input_matrix,
        discrete_input_matrix.T @ riccati_solution @ discrete_state_matrix,
    )
    return np.asarray(gain_matrix.reshape(4), dtype=np.float64)


class LqrSwingUpController:
    """Combined energy-based swing-up and LQR stabilizing controller."""

    def __init__(
        self,
        config: LqrConfig | None = None,
        friction: params.FrictionParameters = params.NOMINAL_FRICTION,
    ) -> None:
        self._config = LqrConfig() if config is None else config
        self._gains = compute_lqr_gains(config=self._config, friction=friction)
        self._lqr_active = False

        half_pole_length = params.POLE_LENGTH_METERS / 2.0
        self._pole_inertia_about_hinge = (
            pole_moment_of_inertia_about_center()
            + params.POLE_MASS_KILOGRAMS * half_pole_length**2
        )
        self._potential_energy_scale = (
            params.POLE_MASS_KILOGRAMS
            * params.GRAVITY_METERS_PER_SECOND_SQUARED
            * half_pole_length
        )
        self._total_mass = params.CART_MASS_KILOGRAMS + params.POLE_MASS_KILOGRAMS

    @property
    def gains(self) -> np.ndarray:
        """LQR state-feedback gains ``[k_pos, k_vel, k_angle, k_ang_vel]``."""
        return self._gains.copy()

    @property
    def is_lqr_active(self) -> bool:
        """Whether the controller is currently in the LQR stabilization phase."""
        return self._lqr_active

    def reset(self) -> None:
        """Reset phase hysteresis before starting a new episode."""
        self._lqr_active = False

    def force_newtons(
        self,
        state: CartPoleState,
        target_cart_position_meters: float = 0.0,
    ) -> float:
        """Compute the desired horizontal cart force in newtons."""
        clamped_target_meters = float(
            np.clip(
                target_cart_position_meters,
                -params.CART_CENTER_TRAVEL_LIMIT_METERS,
                params.CART_CENTER_TRAVEL_LIMIT_METERS,
            )
        )
        wrapped_angle = wrap_angle_radians(state.pole_angle_radians)
        angle_magnitude = abs(wrapped_angle)

        if self._lqr_active:
            if angle_magnitude > self._config.switch_to_swing_up_angle_radians:
                self._lqr_active = False
        elif angle_magnitude <= self._config.switch_to_lqr_angle_radians:
            self._lqr_active = True

        if self._lqr_active:
            raw_force = self._lqr_force_newtons(
                state, wrapped_angle, clamped_target_meters
            )
        else:
            raw_force = self._swing_up_force_newtons(
                state, wrapped_angle, clamped_target_meters
            )

        return float(
            np.clip(raw_force, -params.MAX_FORCE_NEWTONS, params.MAX_FORCE_NEWTONS)
        )

    def force_command(
        self,
        state: CartPoleState,
        target_cart_position_meters: float = 0.0,
    ) -> float:
        """Compute normalized force command in ``[-1.0, 1.0]`` for ``CartPoleEnv``."""
        return float(
            np.clip(
                self.force_newtons(state, target_cart_position_meters)
                / params.MAX_FORCE_NEWTONS,
                -1.0,
                1.0,
            )
        )

    def _lqr_force_newtons(
        self,
        state: CartPoleState,
        wrapped_angle: float,
        target_cart_position_meters: float,
    ) -> float:
        state_error = np.array(
            [
                state.cart_position_meters - target_cart_position_meters,
                state.cart_velocity_meters_per_second,
                wrapped_angle,
                state.pole_angular_velocity_radians_per_second,
            ],
            dtype=np.float64,
        )
        return float(-self._gains @ state_error)

    def _swing_up_force_newtons(
        self,
        state: CartPoleState,
        wrapped_angle: float,
        target_cart_position_meters: float,
    ) -> float:
        angular_velocity = state.pole_angular_velocity_radians_per_second
        energy_error = 0.5 * self._pole_inertia_about_hinge * angular_velocity**2 + (
            self._potential_energy_scale * (math.cos(wrapped_angle) - 1.0)
        )
        normalized_energy_error = energy_error / self._potential_energy_scale

        # Pump energy when swinging; break symmetry if hanging nearly motionless.
        swing_indicator = angular_velocity * math.cos(wrapped_angle)
        if abs(angular_velocity) < 0.05 and abs(wrapped_angle) > math.pi / 2.0:
            kick_direction = 1.0 if wrapped_angle >= 0.0 else -1.0
            energy_acceleration = kick_direction * self._config.swing_up_kick_acceleration
        else:
            energy_acceleration = float(
                np.clip(
                    self._config.swing_up_energy_gain
                    * normalized_energy_error
                    * math.tanh(10.0 * swing_indicator),
                    -self._config.swing_up_max_energy_acceleration,
                    self._config.swing_up_max_energy_acceleration,
                )
            )

        position_error = state.cart_position_meters - target_cart_position_meters
        rail_fraction = abs(state.cart_position_meters) / params.CART_CENTER_TRAVEL_LIMIT_METERS
        centering_acceleration = (
            -(self._config.swing_up_centering_position_gain + 25.0 * rail_fraction**2)
            * position_error
            - self._config.swing_up_centering_velocity_gain
            * state.cart_velocity_meters_per_second
        )
        desired_acceleration = energy_acceleration + centering_acceleration
        return self._total_mass * desired_acceleration
