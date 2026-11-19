"""Humanoid NMPC parameters (docs/15).

Every value is an ASSUMPTION unless it says otherwise. Values marked "upstream" come
from the T800 walking-policy scales (docs/14 §2.2), used here only as limits. Nothing
here is measured on the robot. Replace values only with measured or specified ones,
and record the source in the commit.
"""
from dataclasses import dataclass
from math import exp
from statistics import NormalDist


@dataclass(frozen=True)
class HumanoidNMPCConfig:
    # Discretisation (docs/15 §2, §6). ASSUMPTION
    dt: float = 0.1                  # s, planning step (10 Hz output)
    horizon: int = 20                # steps (2.0 s)

    # Lag model: v_{k+1} = v_k + (1 - exp(-dt/tau)) (u_k - v_k), per axis (docs/15 §3).
    tau_vx: float = 0.3              # s. ASSUMPTION; to be identified by N1
    tau_vy: float = 0.3              # s. ASSUMPTION; to be identified by N1
    tau_wz: float = 0.3              # s. ASSUMPTION; to be identified by N1

    # Command and state limits (docs/15 §5). Upstream scales used as limits (docs/14 §3.1).
    vx_min: float = -0.6             # m/s, upstream backward scale 0.6
    vx_max: float = 1.0              # m/s, upstream forward scale 1.0
    vy_max: float = 0.4              # m/s, upstream lateral scale 0.4
    wz_max: float = 1.0              # rad/s, upstream yaw scale 1.0
    du_max: float = 0.2              # per-step command change limit (per axis). ASSUMPTION

    # Path reference (docs/15 §4). ASSUMPTION
    v_ref: float = 0.6               # m/s, speed used to place reference points on the path

    # Cost weights (docs/15 §4). ASSUMPTION; starting values, not validated
    w_path: float = 10.0
    w_goal: float = 20.0
    w_u: float = 0.1
    w_du: float = 2.0
    w_head: float = 1.0               # heading tracking of the path tangent. ASSUMPTION

    # Separation and chance constraints (docs/15 §5). ASSUMPTION
    d_min_person: float = 0.8        # m, Stage 0 placeholder, not a humanoid value
    d_min_object: float = 0.5        # m, Stage 0 placeholder
    risk: float = 0.05               # chance-constraint risk delta (Stage 0 value)
    sigma_robot: float = 0.05        # m, 1-sigma localisation error
    agent_sigma0: float = 0.10       # m, 1-sigma observation error of an agent
    agent_sigma_rate: float = 0.20   # m/s, growth of prediction error for persons

    # Solver (docs/15 §6). ASSUMPTION
    solver: str = "quadprog"        # "quadprog" (active set) or "osqp". Chosen by N4 measurement
    sqp_iters: int = 3               # re-linearisations per solve
    solve_budget_s: float = 0.05     # s, half the 10 Hz period; sandbox measurement only
    max_iter: int = 4000             # OSQP iteration cap
    eps: float = 1e-6                # OSQP tolerance

    # Back-off added to every separation margin in the QP (tightening, never relaxing).
    # Absorbs solver residual and linearisation drift. ASSUMPTION
    margin_backoff: float = 0.02     # m

    # Verification tolerance on nonlinear rollouts (m for margins, m/s for limits)
    verify_tol: float = 1e-3

    # Fallback (docs/15 §7). ASSUMPTION
    a_stop: float = 0.5              # m/s^2, ramp-to-zero deceleration

    @property
    def alpha(self) -> tuple:
        """Discrete lag gains per axis (vx, vy, wz)."""
        return (1.0 - exp(-self.dt / self.tau_vx),
                1.0 - exp(-self.dt / self.tau_vy),
                1.0 - exp(-self.dt / self.tau_wz))

    @property
    def z_delta(self) -> float:
        """One-sided Gaussian quantile for the allowed risk (docs/13 §2)."""
        return NormalDist().inv_cdf(1.0 - self.risk)
