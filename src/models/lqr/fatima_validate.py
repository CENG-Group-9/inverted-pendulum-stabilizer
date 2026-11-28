"""Validate the analytical cart-pole model against MuJoCo."""

from dataclasses import replace

import mujoco
import numpy as np

from simulation import params
from simulation.env import CartPoleEnv
from lqr.fatima_dynamics import nonlinear_dynamics


def validate_dynamics():
    """Compare analytical and MuJoCo state derivatives."""

    # Disable dry friction for an equivalent model comparison.
    friction = replace(
        params.NOMINAL_FRICTION,
        cart_dry_newtons=0.0,
        pole_dry_newton_meters=0.0
    )

    env = CartPoleEnv(
        randomize_friction=False,
        friction=friction
    )

    # Locate the MuJoCo joints.
    cart_joint = env.model.joint("cart_slide")
    pole_joint = env.model.joint("pole_hinge")

    cart_qpos = cart_joint.qposadr[0]
    pole_qpos = pole_joint.qposadr[0]

    cart_dof = cart_joint.dofadr[0]
    pole_dof = pole_joint.dofadr[0]

    test_cases = [
        ("Upright equilibrium",
         [0.0, 0.0, 0.0, 0.0], 0.0),

        ("Tilted by 10 degrees",
         [0.0, 0.0, np.radians(10), 0.0], 0.0),

        ("Applied force",
         [0.0, 0.0, 0.0, 0.0], 1.0),

        ("Moving cart and pole",
         [0.05, 0.2, 0.15, 0.5], 2.0),

        ("Near downward position",
         [0.0, 0.0, np.pi - 0.1, 0.2], -1.0),
    ]

    all_passed = True

    for name, state, force in test_cases:

        env.reset(options={
            "cart_position_meters": state[0],
            "pole_angle_radians": state[2]
        })

        env.data.qpos[cart_qpos] = state[0]
        env.data.qpos[pole_qpos] = state[2]

        env.data.qvel[cart_dof] = state[1]
        env.data.qvel[pole_dof] = state[3]

        # Convert physical force into normalized action.
        env.data.ctrl[0] = (
            force / params.MAX_FORCE_NEWTONS
        )

        mujoco.mj_forward(env.model, env.data)

        # State derivatives from MuJoCo.
        mujoco_result = np.array([
            env.data.qvel[cart_dof],
            env.data.qacc[cart_dof],
            env.data.qvel[pole_dof],
            env.data.qacc[pole_dof]
        ])

        # State derivatives from our mathematical model.
        analytical_result = nonlinear_dynamics(
            state, force
        )

        error = np.max(np.abs(
            analytical_result - mujoco_result
        ))

        passed = np.allclose(
            analytical_result,
            mujoco_result,
            atol=1e-6,
            rtol=0
        )

        all_passed = all_passed and passed

        print(f"\nTest: {name}")
        print("Analytical:", analytical_result)
        print("MuJoCo:    ", mujoco_result)
        print(f"Maximum error: {error:.10e}")
        print("Passed:", passed)

    env.close()

    print("\nOverall validation:", all_passed)


if __name__ == "__main__":
    validate_dynamics()
