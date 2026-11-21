"""Legged NMPC parameters (docs/17). Values marked ASSUMPTION are sandbox choices, not measurements.

MASS, Z_NOM and FOOT_Y are read from vendor/t800 (see tests/test_legged_nmpc.py, which checks them).
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class LeggedNMPCConfig:
    dt: float = 0.05          # s, step of the exact discretisation (ASSUMPTION)
    N: int = 24               # stages, 1.2 s horizon (ASSUMPTION)
    G: float = 9.81           # m/s^2
    MASS: float = 84.917      # kg, sum of body masses in vendor/t800 (measured)
    Z_NOM: float = 0.967      # m, CoM height at the default pose (measured)
    FOOT_Y: float = 0.12      # m, lateral foot-centre offset at default pose (measured)
    FOOT_HX: float = 0.12     # m, support-box half length (ASSUMPTION; foot geometry not measured)
    FOOT_HY: float = 0.05     # m, support-box half width (ASSUMPTION)
    MU: float = 0.6           # friction coefficient (ASSUMPTION)
    FZ_MAX: float = 1700.0    # N, per-foot normal force bound (ASSUMPTION)
    FZ_MIN: float = 400.0     # N, total normal force lower bound, keeps the ZMP well defined (ASSUMPTION)
    PERIOD: float = 0.9       # s, gait period (ASSUMPTION)
    STANCE_T: float = 0.6     # s, stance duration per foot within the period (ASSUMPTION)
    Q_P: float = 5.0          # horizontal position weight (ASSUMPTION)
    Q_Z: float = 20.0         # vertical position weight (ASSUMPTION)
    Q_V: float = 10.0         # velocity weight (ASSUMPTION)
    Q_TERM: float = 10.0      # terminal weight multiplier (ASSUMPTION)
    R_F: float = 1e-5         # force weight, per N^2 (ASSUMPTION)
    max_iter: int = 300       # IPOPT iteration limit per solve (ASSUMPTION)

    @property
    def FX_MAX(self) -> float:
        return self.MU * self.FZ_MAX
