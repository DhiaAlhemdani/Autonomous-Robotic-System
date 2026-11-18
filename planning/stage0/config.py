"""Stage 0 parameters (theory 09 §5, scenario S2 in docs/00 §2).

Every value marked ASSUMPTION is a placeholder chosen for this stage, not a measured
or specified value. They must be replaced by values from the platform (docs/11 §7)
before any result is reported as a property of the robot.
"""
from dataclasses import dataclass
from statistics import NormalDist


@dataclass(frozen=True)
class Stage0Config:
    # Discretisation and horizon
    dt: float = 0.1                # s. ASSUMPTION: planning step; not the control rate
    horizon: int = 20              # steps (2.0 s at dt = 0.1). ASSUMPTION

    # Robot model: planar double integrator (position, velocity, acceleration command).
    # This is a point-mass abstraction of the base, NOT the humanoid's dynamics.
    v_max: float = 1.0             # m/s. ASSUMPTION
    a_max: float = 1.0             # m/s^2. ASSUMPTION

    # Separation margins per class (theory 09 §2: d_min depends on class)
    d_min_person: float = 0.8      # m. ASSUMPTION
    d_min_object: float = 0.5      # m. ASSUMPTION

    # Chance constraint: P(distance < d_min) <= risk, Gaussian approximation
    risk: float = 0.05             # delta. ASSUMPTION

    # Uncertainty
    sigma_robot: float = 0.05      # m, 1-sigma localisation error of the robot. ASSUMPTION
    agent_sigma0: float = 0.10     # m, 1-sigma of an agent's observed position. ASSUMPTION
    agent_sigma_rate: float = 0.20 # m/s, growth of the 1-sigma prediction error. ASSUMPTION

    # Cost weights (tracking, effort, comfort)
    w_goal: float = 10.0
    w_effort: float = 0.1
    w_jerk: float = 0.5
    # Lateral acceleration (theory 09 §2 comfort term), relative to the current heading.
    # Default 0 keeps earlier results reproducible; set by the smoothness sweep.
    w_lat: float = 0.0

    @property
    def z_delta(self) -> float:
        """One-sided Gaussian quantile for the allowed risk."""
        return NormalDist().inv_cdf(1.0 - self.risk)
