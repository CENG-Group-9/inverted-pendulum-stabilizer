"""Physical parameters of the cart-pole rig, in SI units.

Lengths are meters, masses kilograms, times seconds, angles radians,
forces newtons and torques newton-meters.
"""

import math
from dataclasses import dataclass

# --- Track ---------------------------------------------------------------
# The track is one meter long. A five centimeter strip at each end is never
# used, so the cart stops before it. The strip is reported but not simulated.
TRACK_LENGTH_METERS = 1.0
TRACK_EDGE_MARGIN_METERS = 0.05
USABLE_TRACK_LENGTH_METERS = TRACK_LENGTH_METERS - 2 * TRACK_EDGE_MARGIN_METERS

CART_WIDTH_METERS = 0.10  # along the track
CART_DEPTH_METERS = 0.08  # across the track
CART_HEIGHT_METERS = 0.04

# Distance from the middle of the track to the middle of the cart at the stop.
CART_CENTER_TRAVEL_LIMIT_METERS = (
    USABLE_TRACK_LENGTH_METERS / 2 - CART_WIDTH_METERS / 2
)

# --- Cart ----------------------------------------------------------------
CART_MASS_KILOGRAMS = 0.40

# --- Pole ----------------------------------------------------------------
POLE_LENGTH_METERS = 0.30
POLE_RADIUS_METERS = 0.003
POLE_DENSITY_KILOGRAMS_PER_CUBIC_METER = 7850.0  # steel
POLE_MASS_KILOGRAMS = (
    math.pi
    * POLE_RADIUS_METERS**2
    * POLE_LENGTH_METERS
    * POLE_DENSITY_KILOGRAMS_PER_CUBIC_METER
)

# --- Motor ---------------------------------------------------------------
MAX_FORCE_NEWTONS = 6.0  # an action of 1.0 pushes the cart with this force

# --- Environment ---------------------------------------------------------
GRAVITY_METERS_PER_SECOND_SQUARED = 9.81
CONTROL_FREQUENCY_HERTZ = 200
PHYSICS_TIMESTEP_SECONDS = 0.001
EPISODE_DURATION_SECONDS = 15.0

# --- Angles --------------------------------------------------------------
POLE_ANGLE_UPRIGHT_RADIANS = 0.0
POLE_ANGLE_HANGING_RADIANS = math.pi
UPRIGHT_ANGLE_TOLERANCE_RADIANS = math.radians(10.0)

# --- Initial state -------------------------------------------------------
INITIAL_POLE_ANGLE_NOISE_RADIANS = 0.05
INITIAL_CART_POSITION_NOISE_METERS = 0.02

# --- Reward --------------------------------------------------------------
UPRIGHT_REWARD_WEIGHT = 1.0
CART_POSITION_PENALTY_WEIGHT = 0.05
FORCE_PENALTY_WEIGHT = 0.01


@dataclass(frozen=True)
class FrictionParameters:
    """Friction of one rig configuration."""

    cart_viscous_newton_seconds_per_meter: float
    cart_dry_newtons: float
    pole_viscous_newton_meter_seconds_per_radian: float
    pole_dry_newton_meters: float


@dataclass(frozen=True)
class FrictionRange:
    """Closed interval used for domain randomization."""

    lowest: float
    highest: float


# Nominal values of the real rig, used by LQR and by evaluation runs.
NOMINAL_FRICTION = FrictionParameters(
    cart_viscous_newton_seconds_per_meter=0.05,
    cart_dry_newtons=0.02,
    pole_viscous_newton_meter_seconds_per_radian=1e-4,
    pole_dry_newton_meters=1e-5,
)

# Ranges the RL policy is trained on, drawn uniformly once per episode.
CART_VISCOUS_FRICTION_RANGE = FrictionRange(0.01, 0.30)
CART_DRY_FRICTION_RANGE = FrictionRange(0.005, 0.10)
POLE_VISCOUS_FRICTION_RANGE = FrictionRange(1e-5, 1e-3)
POLE_DRY_FRICTION_RANGE = FrictionRange(1e-6, 1e-4)
