"""Legged NMPC parameters (docs/17, corrected in docs/18 §1). ASSUMPTION marks sandbox choices, not measurements.

MASS, Z_NOM, FOOT_Y, FOOT_HX, FOOT_HY, FOOT_OFFSET_X and IZ are measured from vendor/t800 and checked by
tests/test_legged_nmpc.py. The whole-body layer recomputes the foot-origin offsets from the model.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class LeggedNMPCConfig:
    dt: float = 0.05          # s, step of the exact discretisation (ASSUMPTION)
    N: int = 16               # stages, 0.8 s horizon (ASSUMPTION; was 24 / 1.2 s, too slow for real time, see docs/18 §10.3)
    G: float = 9.81           # m/s^2
    MASS: float = 84.917      # kg, sum of body masses in vendor/t800 (measured)
    IZ: float = 1.7           # kg m^2, yaw inertia about the whole-body CoM (measured, docs/18 §4)
    Z_NOM: float = 0.9536     # m, CoM height with the soles on the ground (measured)
    FOOT_Y: float = 0.121     # m, lateral sole-centre offset (measured)
    FOOT_HX: float = 0.120    # m, support half-length (measured, convex hull of the flat sole blocks)
    FOOT_HY: float = 0.055    # m, support half-width (measured)
    FOOT_OFFSET_X: float = 0.037  # m, sole centre ahead of the foot origin in the foot frame (measured)
    MU: float = 0.6           # friction coefficient (ASSUMPTION)
    FZ_MAX: float = 1700.0    # N, per-foot normal force bound (ASSUMPTION)
    FZ_MIN: float = 400.0     # N, total normal force lower bound (ASSUMPTION)
    PERIOD: float = 0.9       # s, gait period (ASSUMPTION)
    STANCE_T: float = 0.6     # s, stance duration per foot (ASSUMPTION)
    STEP_MAX: float = 0.5     # m, same-foot step length limit (ASSUMPTION)
    SWING_H: float = 0.04     # m, swing clearance (ASSUMPTION)
    Q_P: float = 5.0          # horizontal position weight (ASSUMPTION)
    Q_Z: float = 20.0         # vertical position weight (ASSUMPTION)
    Q_V: float = 10.0         # velocity weight (ASSUMPTION)
    Q_PSI: float = 20.0       # yaw weight (ASSUMPTION)
    Q_W: float = 5.0          # yaw-rate weight (ASSUMPTION)
    Q_TERM: float = 10.0      # terminal weight multiplier (ASSUMPTION)
    R_F: float = 1e-5         # force weight, per N^2 (ASSUMPTION)
    max_iter: int = 300       # IPOPT iteration limit per solve (ASSUMPTION)
    ipopt_tol: float = 1e-6   # IPOPT tolerance (ASSUMPTION; default 1e-8 was not needed for the reference tracking)

    @property
    def FX_MAX(self) -> float:
        return self.MU * self.FZ_MAX

    @property
    def SWING_T(self) -> float:
        return self.PERIOD - self.STANCE_T
