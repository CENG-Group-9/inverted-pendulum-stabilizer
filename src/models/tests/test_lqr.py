"""Tests for the discrete-time LQR and energy-based swing-up controller."""

from __future__ import annotations

import math

import numpy as np
from scipy import signal

from lqr.lqr import (
    LqrConfig,
    LqrSwingUpController,
    compute_linearized_system,
    compute_lqr_gains,
)
from simulation import params
from simulation.env import CartPoleEnv, wrap_angle_radians


def test_lqr_closed_loop_is_stable() -> None:
    """Closed-loop discrete-time poles must lie strictly inside the unit circle."""
    config = LqrConfig()
    gains = compute_lqr_gains(config=config)
    assert gains.shape == (4,)
    assert np.all(np.isfinite(gains))

    continuous_state_matrix, continuous_input_matrix = compute_linearized_system()
    discrete_state_matrix, discrete_input_matrix, _, _, _ = signal.cont2discrete(
        (
            continuous_state_matrix,
            continuous_input_matrix,
            np.eye(4),
            np.zeros((4, 1)),
        ),
        dt=1.0 / config.control_frequency_hertz,
        method="zoh",
    )
    closed_loop_matrix = discrete_state_matrix - discrete_input_matrix @ gains.reshape(
        1, 4
    )
    eigenvalue_magnitudes = np.abs(np.linalg.eigvals(closed_loop_matrix))
    assert np.all(eigenvalue_magnitudes < 1.0)


def test_lqr_stabilizes_from_upright_perturbation() -> None:
    """Controller must stabilize the pole from a 10-degree initial offset."""
    environment = CartPoleEnv(terminate_on_pole_angle_radians=math.radians(35.0))
    controller = LqrSwingUpController()
    environment.reset(
        seed=0,
        options={
            "pole_angle_radians": math.radians(10.0),
            "cart_position_meters": 0.1,
        },
    )
    controller.reset()

    for _ in range(400):
        force_command = controller.force_command(environment.state())
        _, _, terminated, truncated, _ = environment.step([force_command])
        assert not terminated
        if truncated:
            break

    final_state = environment.state()
    assert controller.is_lqr_active
    assert abs(wrap_angle_radians(final_state.pole_angle_radians)) < math.radians(1.0)
    assert abs(final_state.cart_position_meters) < 0.02


def test_swing_up_and_stabilize_nominal_and_randomized_friction() -> None:
    """Controller must swing up from hanging down and balance for the episode."""
    for randomize_friction in (False, True):
        environment = CartPoleEnv(randomize_friction=randomize_friction)
        controller = LqrSwingUpController()
        for seed in range(3):
            environment.reset(seed=seed)
            controller.reset()
            upright_steps = 0
            total_steps = 0

            done = False
            while not done:
                force_command = controller.force_command(environment.state())
                _, _, terminated, truncated, info = environment.step([force_command])
                assert not terminated, (
                    f"terminated early ({info['termination_reason']}) on "
                    f"seed={seed}, randomize_friction={randomize_friction}"
                )
                upright_steps += int(info["is_upright"])
                total_steps += 1
                done = terminated or truncated

            assert upright_steps / total_steps > 0.65
            final_state = environment.state()
            assert abs(
                wrap_angle_radians(final_state.pole_angle_radians)
            ) < math.radians(2.0)


def test_lqr_tracks_target_cart_position() -> None:
    """Controller must move the cart to a non-zero target while staying upright."""
    environment = CartPoleEnv()
    controller = LqrSwingUpController()
    environment.reset(
        seed=0,
        options={
            "pole_angle_radians": 0.0,
            "cart_position_meters": 0.0,
        },
    )
    controller.reset()

    target_position_meters = 0.15
    assert abs(target_position_meters) < params.CART_CENTER_TRAVEL_LIMIT_METERS

    for _ in range(600):
        force_command = controller.force_command(
            environment.state(),
            target_cart_position_meters=target_position_meters,
        )
        _, _, terminated, _, _ = environment.step([force_command])
        assert not terminated

    final_state = environment.state()
    assert abs(final_state.cart_position_meters - target_position_meters) < 0.02
    assert abs(wrap_angle_radians(final_state.pole_angle_radians)) < math.radians(1.0)
