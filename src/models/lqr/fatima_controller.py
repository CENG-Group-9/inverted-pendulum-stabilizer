"""Independent discrete-time LQR controller."""

import numpy as np
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are

from lqr.fatima_dynamics import (
    linearized_matrices,
    dt,
    F_max,
    x_limit
)


def discretize_system():
    """Convert continuous A and B to discrete matrices."""

    A, B = linearized_matrices()

    C = np.eye(4)
    D = np.zeros((4, 1))

    Ad, Bd, _, _, _ = cont2discrete(
        (A, B, C, D),
        dt,
        method="zoh"
    )

    return Ad, Bd


def controllability_matrix(Ad, Bd):
    """Construct the discrete controllability matrix."""

    return np.hstack([
        Bd,
        Ad @ Bd,
        Ad @ Ad @ Bd,
        Ad @ Ad @ Ad @ Bd
    ])
def calculate_lqr_gain(Ad, Bd):
    """Calculate the optimal discrete-time LQR gain."""

    # State error weights
    Q = np.diag([
        100.0,  # Cart position
        10.0,   # Cart velocity
        200.0,  # Pole angle
        10.0    # Pole angular velocity
    ])

    # Motor force penalty
    R = np.array([[1.0]])

    # Solve the discrete Riccati equation
    P = solve_discrete_are(Ad, Bd, Q, R)

    # Calculate the LQR feedback gain
    K = np.linalg.solve(
        R + Bd.T @ P @ Bd,
        Bd.T @ P @ Ad
    )

    return K, Q, R
class FatimaLQRController:
    """Independent LQR balancing controller."""

    def __init__(self):

        self.Ad, self.Bd = discretize_system()

        self.K, self.Q, self.R = calculate_lqr_gain(
            self.Ad, self.Bd
        )

    def compute_force(self, state, target_position=0.0):
        """Calculate the required balancing force."""

        state = np.asarray(state, dtype=float).copy()

        # Cart position error
        state[0] -= target_position

        # Wrap pole angle between -pi and pi
        state[2] = (
            (state[2] + np.pi) % (2 * np.pi)
        ) - np.pi

        # LQR feedback
        force = -(self.K @ state).item()

        # Apply motor force limits
        force = np.clip(force, -F_max, F_max)

        return float(force)

    def compute_action(self, state, target_position=0.0):
        """Convert motor force into simulation action."""

        force = self.compute_force(
            state, target_position
        )

        action = force / F_max

        return float(np.clip(action, -1.0, 1.0))

if __name__ == "__main__":

    Ad, Bd = discretize_system()

    print("Discrete-Time State-Space Model")
    print("--------------------------------")

    print("Control timestep:", dt)

    print("\nDiscrete Matrix Ad:")
    print(Ad)

    print("\nDiscrete Matrix Bd:")
    print(Bd)

    print("\nControllability Analysis")
    print("------------------------")

    Wc = controllability_matrix(Ad, Bd)
    rank = np.linalg.matrix_rank(Wc)

    print("Controllability matrix:")
    print(Wc)

    print("\nControllability rank:", rank)
    print("Number of states:", Ad.shape[0])
    print("System is controllable:", rank == Ad.shape[0])
    print("\nLQR Controller Design")
    print("---------------------")

    K, Q, R = calculate_lqr_gain(Ad, Bd)

    print("State weight matrix Q:")
    print(Q)

    print("\nForce weight matrix R:")
    print(R)

    print("\nLQR gain matrix K:")
    print(K)

    # Closed-loop system
    A_closed = Ad - Bd @ K

    eigenvalues = np.linalg.eigvals(A_closed)

    print("\nClosed-loop eigenvalues:")
    print(eigenvalues)

    print("\nEigenvalue magnitudes:")
    print(np.abs(eigenvalues))

    stable = np.all(np.abs(eigenvalues) < 1.0)

    print("\nClosed-loop system is stable:", stable)
    print("\nLQR Balancing Controller Tests")
    print("------------------------------")

    controller = FatimaLQRController()

    test_states = [
        ("Perfect upright",
         [0.0, 0.0, 0.0, 0.0]),

        ("Tilted right by 5 degrees",
         [0.0, 0.0, np.radians(5), 0.0]),

        ("Tilted left by 5 degrees",
         [0.0, 0.0, np.radians(-5), 0.0]),

        ("Cart displaced by 10 cm",
         [0.1, 0.0, 0.0, 0.0]),

        ("Large angle - saturation test",
         [0.0, 0.0, np.radians(30), 0.0])
    ]

    for name, state in test_states:

        force = controller.compute_force(state)
        action = controller.compute_action(state)

        print(f"\nTest: {name}")
        print(f"Motor force: {force:.6f} N")
        print(f"Normalized action: {action:.6f}")
        print("Within force limit:", abs(force) <= F_max)