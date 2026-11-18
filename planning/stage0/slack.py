"""Step 2b: slack-relaxed feasibility check (diagnosis only; not used by the controller).

Same model, speed limit and margins as Stage0NMPC, but each separation constraint gets a
slack s_j >= 0 on the distance scale (metres):

    dist_j(U) + s_j >= R_j        minimise  SLACK_WEIGHT * sum(s_j) + small effort

The reported shortfall is max_j (R_j - dist_j) at the returned point, in metres. A value at
or below FEAS_TOL means the returned point satisfies every margin: the problem is feasible.
A value above FEAS_TOL at a converged solution means no point found satisfies all margins.
This does not prove infeasibility over all possible starts.
"""
import numpy as np
from scipy.optimize import minimize

from .nmpc import Stage0NMPC

SLACK_WEIGHT = 1e3
FEAS_TOL = 1e-3  # m


def slack_check(cfg, x0, goal, agents, init=None):
    N = cfg.horizon
    z = cfg.z_delta
    radii = np.concatenate([
        ag.d_min + z * np.sqrt(ag.sigma ** 2 + cfg.sigma_robot ** 2) for ag in agents
    ]) if agents else np.zeros(0)
    preds = [ag.pred for ag in agents]
    M = len(agents) * N
    helper = Stage0NMPC(cfg)

    def dists(U):
        P, V = helper._rollout(x0, U.reshape(-1, 2))
        d = [np.sqrt(np.sum((P - pr) ** 2, axis=1) + 1e-12) for pr in preds]
        return P, V, (np.concatenate(d) if d else np.zeros(0))

    def cons_sep(w):
        U, s = w[:2 * N], w[2 * N:]
        _, _, d = dists(U)
        return d + s - radii

    def cons_speed(w):
        _, V, _ = dists(w[:2 * N])
        return cfg.v_max ** 2 - np.sum(V * V, axis=1)

    def obj(w):
        U, s = w[:2 * N], w[2 * N:]
        return SLACK_WEIGHT * np.sum(s) + 1e-3 * np.sum(U * U)

    U0 = np.zeros(2 * N) if init is None else np.asarray(init, dtype=float).ravel()
    w0 = np.concatenate([U0, np.zeros(M)])
    bounds = [(-cfg.a_max, cfg.a_max)] * (2 * N) + [(0.0, None)] * M
    cons = [{"type": "ineq", "fun": cons_sep}, {"type": "ineq", "fun": cons_speed}]
    res = minimize(obj, w0, method="SLSQP", bounds=bounds, constraints=cons,
                   options={"maxiter": 200, "ftol": 1e-9})

    U = res.x[:2 * N]
    _, V, d = dists(U)
    shortfall = float(np.max(radii - d)) if d.size else -np.inf
    speed_ok = bool(np.max(np.sum(V * V, axis=1)) <= cfg.v_max ** 2 + 1e-6)
    return {"success": bool(res.success), "message": str(res.message),
            "shortfall_m": shortfall, "speed_ok": speed_ok,
            "feasible_within_tol": bool(shortfall <= FEAS_TOL and speed_ok)}
