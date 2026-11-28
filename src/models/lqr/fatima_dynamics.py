"""Physical model for the independent LQR controller.

All physical quantities use SI units.
"""

import numpy as np

from simulation import params


# Physical properties
M = params.CART_MASS_KILOGRAMS
m = params.POLE_MASS_KILOGRAMS
L = params.POLE_LENGTH_METERS
r = params.POLE_RADIUS_METERS
g = params.GRAVITY_METERS_PER_SECOND_SQUARED

# Distance from pivot to center of mass
l = L / 2.0

# Moment of inertia about the center of mass
I_c = m * (3 * r**2 + L**2) / 12.0

# Moment of inertia about the pivot
J = I_c + m * l**2

# Nominal viscous friction
b = params.NOMINAL_FRICTION.cart_viscous_newton_seconds_per_meter

d = params.NOMINAL_FRICTION.pole_viscous_newton_meter_seconds_per_radian

# Control parameters
dt = 1.0 / params.CONTROL_FREQUENCY_HERTZ
F_max = params.MAX_FORCE_NEWTONS
x_limit = params.CART_CENTER_TRAVEL_LIMIT_METERS

def nonlinear_dynamics(state, force):
    """Calculate the derivatives of the cart-pole state.

    State order:
        cart position (m)
        cart velocity (m/s)
        pole angle (rad)
        pole angular velocity (rad/s)

    Force is expressed in newtons.
    """

    x, x_dot, theta, theta_dot = state

    # Trigonometric terms
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    # Coupled mass matrix
    mass_matrix = np.array([
        [M + m, m * l * cos_theta],
        [m * l * cos_theta, J]
    ])

    # Applied forces and torques
    rhs = np.array([
        force - b * x_dot
        + m * l * sin_theta * theta_dot**2,

        m * g * l * sin_theta - d * theta_dot
    ])

    # Solve for cart and angular accelerations
    x_ddot, theta_ddot = np.linalg.solve(
        mass_matrix, rhs
    )

    # Return derivatives of all four states
    return np.array([
        x_dot,
        x_ddot,
        theta_dot,
        theta_ddot
    ])
def linearized_matrices():
    """Calculate continuous-time A and B matrices
    around the upright equilibrium.
    """

    # Determinant of the upright mass matrix
    D = (M + m) * J - (m * l)**2

    # State matrix
    A = np.array([
        [0, 1, 0, 0],

        [0, -J * b / D,
         -(m * l)**2 * g / D,
         m * l * d / D],

        [0, 0, 0, 1],

        [0, m * l * b / D,
         (M + m) * m * g * l / D,
         -(M + m) * d / D]
    ])

    # Input matrix (force in newtons)
    B = np.array([
        [0],
        [J / D],
        [0],
        [-m * l / D]
    ])

    return A, B
def numerical_linearization(epsilon=1e-6):
    """Numerically calculate A and B using
    central finite differences.
    """

    equilibrium = np.zeros(4)

    A_num = np.zeros((4, 4))
    B_num = np.zeros((4, 1))

    # Calculate derivatives with respect to states
    for i in range(4):
        perturbation = np.zeros(4)
        perturbation[i] = epsilon

        f_plus = nonlinear_dynamics(
            equilibrium + perturbation, 0.0
        )

        f_minus = nonlinear_dynamics(
            equilibrium - perturbation, 0.0
        )

        A_num[:, i] = (
            f_plus - f_minus
        ) / (2 * epsilon)

    # Calculate derivatives with respect to force
    f_plus = nonlinear_dynamics(
        equilibrium, epsilon
    )

    f_minus = nonlinear_dynamics(
        equilibrium, -epsilon
    )

    B_num[:, 0] = (
        f_plus - f_minus
    ) / (2 * epsilon)

    return A_num, B_num

if __name__ == "__main__":
    print("Physical Model Parameters")
    print("-------------------------")
    print(f"Cart mass: {M:.6f} kg")
    print(f"Rod mass: {m:.6f} kg")
    print(f"Rod length: {L:.6f} m")
    print(f"Center of mass: {l:.6f} m")
    print(f"Center inertia: {I_c:.8f} kg*m^2")
    print(f"Pivot inertia: {J:.8f} kg*m^2")
    print(f"Gravity: {g:.6f} m/s^2")
    print(f"Cart viscous friction: {b}")
    print(f"Pole viscous friction: {d}")
    print(f"Control timestep: {dt} s")
    print(f"Maximum force: {F_max} N")
    print(f"Cart travel limit: {x_limit} m")
    print("\nNonlinear Dynamics Tests") 
    print("------------------------")

    upright_state = np.array([0.0, 0.0, 0.0, 0.0])

    print("Upright equilibrium:")
    print(nonlinear_dynamics(upright_state, 0.0))

    tilted_state = np.array([
        0.0, 0.0, np.radians(10.0), 0.0
    ])

    print("Pendulum tilted by 10 degrees:")
    print(nonlinear_dynamics(tilted_state, 0.0))
    print("\nLinearized State-Space Model")
    print("----------------------------")

    A, B = linearized_matrices()

    print("Matrix A:")
    print(A)

    print("\nMatrix B:")
    print(B)

    print("\nMatrix A shape:", A.shape)
    print("Matrix B shape:", B.shape)

    print("\nOpen-loop eigenvalues:")
    print(np.linalg.eigvals(A))
    print("\nNumerical Linearization Verification")
    print("------------------------------------")

    A_num, B_num = numerical_linearization()

    print("Numerical Matrix A:")
    print(A_num)

    print("\nNumerical Matrix B:")
    print(B_num)

    A_error = np.max(np.abs(A - A_num))
    B_error = np.max(np.abs(B - B_num))

    print("\nMaximum A matrix error:", A_error)
    print("Maximum B matrix error:", B_error)

    print("\nA matrix verification:",
          np.allclose(A, A_num, atol=1e-6, rtol=0))

    print("B matrix verification:",
          np.allclose(B, B_num, atol=1e-6, rtol=0))