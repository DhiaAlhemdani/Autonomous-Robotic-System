"""Stage 0 NMPC: planar double integrator, separation constraints with Gaussian margins.

Problem solved at each step (theory 09 §3.1, with the chance constraint replaced by
its Gaussian tightening):

    min_U  w_goal * |p_N - goal|^2 + w_effort * dt * sum |a_k|^2
           + w_jerk * sum |a_k - a_{k-1}|^2
    s.t.   |a_k| <= a_max                                     (box)
           |v_k|^2 <= v_max^2                                 (speed)
           |p_k - o^i_k|^2 >= (d_min^i + z * sigma^i_k)^2     (separation, all k, i)

sigma^i_k combines the agent prediction error and the robot localisation error in
quadrature. This is an approximation: it assumes Gaussian errors (theory 09 §3.3).

The solver is SciPy SLSQP with finite-difference gradients. It is a correctness
baseline, not the production solver. Timing numbers from it say nothing about a
production implementation on the target hardware.
"""
import time
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from .config import Stage0Config


@dataclass
class Agent:
    """Predicted agent: positions at steps 1..N, 1-sigma, and class margin."""
    pred: np.ndarray       # (N, 2)
    sigma: np.ndarray      # (N,)
    d_min: float


@dataclass
class SolveResult:
    accel: np.ndarray      # (2,) command applied this step
    status: str            # "ok" or "infeasible" (brake fallback applied)
    solve_time_s: float
    iterations: int
    min_constraint: float  # min over constraints; >= 0 means satisfied
    reason: str = "ok"     # "ok", "solver_failed", "constraint_violation", "nonfinite"
    message: str = ""      # SciPy status message from the solver


class Stage0NMPC:
    def __init__(self, cfg: Stage0Config):
        self.cfg = cfg
        self._warm = None
        self._prev_accel = np.zeros(2)

    def _rollout(self, x0, U):
        dt = self.cfg.dt
        p = x0[:2].copy()
        v = x0[2:].copy()
        P = np.empty((len(U), 2))
        V = np.empty((len(U), 2))
        for k, a in enumerate(U):
            p = p + v * dt + 0.5 * a * dt * dt
            v = v + a * dt
            P[k] = p
            V[k] = v
        return P, V

    def _constraints(self, U, x0, agents, radii):
        P, V = self._rollout(x0, U.reshape(-1, 2))
        parts = [self.cfg.v_max ** 2 - np.sum(V * V, axis=1)]
        for ag, R in zip(agents, radii):
            d2 = np.sum((P - ag.pred) ** 2, axis=1)
            parts.append(d2 - R * R)
        return np.concatenate(parts)

    def _cost(self, U, x0, goal):
        cfg = self.cfg
        A = U.reshape(-1, 2)
        P, _ = self._rollout(x0, A)
        e = P[-1] - goal
        seq = np.vstack([self._prev_accel[None, :], A])
        jerk = np.sum(np.diff(seq, axis=0) ** 2)
        effort = cfg.dt * np.sum(A * A)
        return cfg.w_goal * float(e @ e) + cfg.w_effort * effort + cfg.w_jerk * jerk

    def _brake(self, x0):
        v = x0[2:]
        a = -v / self.cfg.dt
        n = np.linalg.norm(a)
        if n > self.cfg.a_max:
            a = a * (self.cfg.a_max / n)
        return a

    def solve(self, x0, goal, agents) -> SolveResult:
        cfg = self.cfg
        N = cfg.horizon
        z = cfg.z_delta
        radii = [ag.d_min + z * np.sqrt(ag.sigma ** 2 + cfg.sigma_robot ** 2) for ag in agents]

        if self._warm is None:
            U0 = np.zeros((N, 2))
        else:
            U0 = np.vstack([self._warm[1:], self._warm[-1:]])

        bounds = [(-cfg.a_max, cfg.a_max)] * (2 * N)
        cons = [{"type": "ineq", "fun": lambda U: self._constraints(U, x0, agents, radii)}]

        t0 = time.perf_counter()
        res = minimize(
            self._cost, U0.ravel(), args=(x0, goal), method="SLSQP",
            bounds=bounds, constraints=cons,
            options={"maxiter": 100, "ftol": 1e-6},
        )
        elapsed = time.perf_counter() - t0

        g = self._constraints(res.x, x0, agents, radii) if np.all(np.isfinite(res.x)) else np.array([-np.inf])
        min_c = float(np.min(g))
        ok = bool(res.success) and min_c >= -1e-4

        if not np.all(np.isfinite(res.x)):
            reason = "nonfinite"
        elif not res.success:
            reason = "solver_failed"
        elif min_c < -1e-4:
            reason = "constraint_violation"
        else:
            reason = "ok"

        if ok:
            U = res.x.reshape(N, 2)
            accel = U[0].copy()
            self._warm = U
            status = "ok"
        else:
            accel = self._brake(x0)
            self._warm = None
            status = "infeasible"

        self._prev_accel = accel
        return SolveResult(accel=accel, status=status, solve_time_s=elapsed,
                           iterations=int(res.nit), min_constraint=min_c,
                           reason=reason, message=str(res.message))
