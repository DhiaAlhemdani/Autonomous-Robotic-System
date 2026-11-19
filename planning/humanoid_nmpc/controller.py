"""Humanoid NMPC controller, layer L3 (docs/15).

Each call solves a receding-horizon problem over the base model in model.py:

    min  sum_k w_path |p_k - r_k|^2 + w_goal |p_N - goal|^2 + sum_k w_head (psi_k - psi_ref_k)^2
       + sum_k w_u |u_k|^2 + w_du |u_k - u_{k-1}|^2
    s.t. x_0 = measured state                                   (equality)
         x_{k+1} = f(x_k, u_k)  linearised at the nominal         (dynamics)
         lo <= u_k <= hi, lo <= v_k <= hi                         (hard limits)
         |u_k - u_{k-1}| <= du_max                                (hard rate limit)
         n_k . (p_k - o_k) >= d_min + z sigma_k                   (hard separation)

Separation is a half-plane tangent to the exclusion circle at the nominal position.
A point in that half-plane is at least the margin away from o_k, so the linearised
constraint is an inner approximation: it cannot admit a point the circle forbids.
After each solve the nonlinear rollout is checked against the same margins. If the
check fails, the solve is rejected.

Solver: a condensed QP in the 3N commands, one per SQP iteration (docs/15 §6). The default
backend is quadprog (dense active set); OSQP is available for comparison. There are no soft
constraints in this formulation, so the fallback chain of docs/15 §7 is:

  1. nominal  - QP from the warm start, verified.
  2. retry    - QP from a cold start, verified. (docs/15 §7 step 1 becomes this retry,
                because there are no soft constraints to relax; separation is hard.)
  3. escape   - best held command from a small candidate set, under the constant-velocity
                prediction over the horizon. Added because a ramp to zero cannot leave a
                state where an agent is closing (see docs/15 §7, as amended).
  4. reactive_stop - ramp command to zero; valid only if the ramp keeps the margins.
  5. protective_stop - the same ramp, flagged valid=False. L2 must take over.

Outputs are a body-frame velocity command for L1 locomotion. The controller never
writes joint targets and never removes torque.
"""
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import osqp
import scipy.sparse as sp

from .config import HumanoidNMPCConfig
from .model import NU, NX, jacobians, rollout, step

INF = np.inf


@dataclass(frozen=True)
class Obstacle:
    """A tracked agent with constant-velocity prediction (vel=0 for static objects)."""
    pos0: tuple            # (x, y) world frame, m
    vel: tuple = (0.0, 0.0)  # m/s
    kind: str = "person"   # "person" or "object"


@dataclass
class Output:
    cmd: np.ndarray        # (3,) body-frame [vx, vy, wz] for L1
    level: str             # nominal | retry | reactive_stop | protective_stop
    valid: bool            # False only for protective_stop
    solve_time_s: float
    iterations: int
    reason: str            # ok | solver_failed | time_limit | margin_violation | limit_violation
    min_margin: float      # min over steps and agents of (distance - R); >= 0 is satisfied
    attempts: List[str] = field(default_factory=list)


class HumanoidNMPC:
    def __init__(self, cfg: Optional[HumanoidNMPCConfig] = None):
        self.cfg = cfg or HumanoidNMPCConfig()
        self._u_seq: Optional[np.ndarray] = None   # warm start, shape (N, 3)
        self._u_prev = np.zeros(NU)                # last applied command
        self._lo = np.array([self.cfg.vx_min, -self.cfg.vy_max, -self.cfg.wz_max])
        self._hi = np.array([self.cfg.vx_max, self.cfg.vy_max, self.cfg.wz_max])

    # ------------------------------------------------------------------ public
    def reset(self):
        self._u_seq = None
        self._u_prev = np.zeros(NU)

    def step(self, x: np.ndarray, path: np.ndarray, obstacles: List[Obstacle]) -> Output:
        t0 = time.perf_counter()
        cfg = self.cfg
        N = cfg.horizon
        x = np.asarray(x, dtype=float)
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] != 2:
            raise ValueError("path must be (M, 2) with M >= 2")

        refs, psi_ref = self._reference(path, x[0:2], x[2])
        goal = path[-1]
        preds = [self._predict(o) for o in obstacles]
        R = [self._margins(o) for o in obstacles]

        attempts: List[str] = []
        total_iters = 0
        last_reason = "solver_failed"
        result = None

        inits = [("nominal", self._warm_guess(x, refs))]
        inits.append(("retry", self._cold_guess(x, refs)))
        for name, u_init in inits:
            attempts.append(name)
            res = self._solve(x, u_init, refs, psi_ref, goal, preds, R)
            total_iters += res["iters"]
            if res["status"] != "solved":
                last_reason = res["status"]
                continue
            ok, margin, why = self._verify(x, res["U"], preds, R)
            if ok:
                result = (name, res["U"], margin)
                break
            last_reason = why

        if result is not None:
            name, U, margin = result
            cmd = U[0].copy()
            self._u_seq = U.copy()
            level, valid, reason = name, True, "ok"
        else:
            self._u_seq = None
            cmd_e, margin_e, ok_e = self._escape(x, preds, R)
            attempts.append("escape")
            if ok_e:
                cmd, margin = cmd_e, margin_e
                level, valid, reason = "escape", True, last_reason
            else:
                cmd, margin_stop, ok_stop = self._stop(x, preds, R)
                attempts.append("stop")
                margin = margin_stop
                if ok_stop:
                    level, valid, reason = "reactive_stop", True, last_reason
                else:
                    level, valid, reason = "protective_stop", False, last_reason

        self._u_prev = cmd.copy()
        return Output(cmd=cmd, level=level, valid=valid,
                      solve_time_s=time.perf_counter() - t0,
                      iterations=total_iters, reason=reason,
                      min_margin=float(margin), attempts=attempts)

    # --------------------------------------------------------------- reference
    def _reference(self, path, p, psi0=0.0):
        """Reference points r_1..r_N and tangent headings on the polyline.

        Points are v_ref * k * dt ahead of the projection of p onto the path.
        Headings are the path tangent at each point, unwrapped to be near psi0.
        """
        seg = np.diff(path, axis=0)
        L = np.hypot(seg[:, 0], seg[:, 1])
        cum = np.concatenate([[0.0], np.cumsum(L)])
        best_d, best_s = np.inf, 0.0
        for i in range(len(seg)):
            if L[i] <= 0.0:
                continue
            t = np.clip(np.dot(p - path[i], seg[i]) / L[i] ** 2, 0.0, 1.0)
            q = path[i] + t * seg[i]
            d = np.hypot(*(p - q))
            if d < best_d:
                best_d, best_s = d, cum[i] + t * L[i]
        k = np.arange(1, self.cfg.horizon + 1)
        s = np.minimum(best_s + self.cfg.v_ref * self.cfg.dt * k, cum[-1])
        pts = np.stack([np.interp(s, cum, path[:, 0]), np.interp(s, cum, path[:, 1])], axis=1)
        idx = np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(seg) - 1)
        seg_unit = seg[idx] / np.maximum(L[idx], 1e-12)[:, None]
        psi = np.arctan2(seg_unit[:, 1], seg_unit[:, 0])
        psi = psi + 2 * np.pi * np.round((psi0 - psi) / (2 * np.pi))
        return pts, psi

    # -------------------------------------------------------------- prediction
    def _predict(self, o: Obstacle):
        k = np.arange(1, self.cfg.horizon + 1) * self.cfg.dt
        pos = np.asarray(o.pos0, float)[None, :] + np.asarray(o.vel, float)[None, :] * k[:, None]
        return pos

    def _margins(self, o: Obstacle):
        """R_k = d_min + z * sigma_k, sigma combined in quadrature with robot localisation."""
        cfg = self.cfg
        k = np.arange(1, cfg.horizon + 1) * cfg.dt
        d_min = cfg.d_min_person if o.kind == "person" else cfg.d_min_object
        if o.kind == "person":
            sig_agent = cfg.agent_sigma0 + cfg.agent_sigma_rate * k
        else:
            sig_agent = np.full_like(k, cfg.agent_sigma0)
        sigma = np.sqrt(sig_agent ** 2 + cfg.sigma_robot ** 2)
        return d_min + cfg.z_delta * sigma

    # ----------------------------------------------------------------- guesses
    def _warm_guess(self, x, refs):
        if self._u_seq is None:
            return self._cold_guess(x, refs)
        U = np.vstack([self._u_seq[1:], self._u_seq[-1:]])
        return np.clip(U, self._lo, self._hi)

    def _cold_guess(self, x, refs):
        cfg = self.cfg
        d = refs[0] - x[0:2]
        dist = np.hypot(*d)
        v_world = np.zeros(2) if dist < 1e-9 else d / dist * min(cfg.v_ref, dist / 1.0)
        c, s = np.cos(-x[2]), np.sin(-x[2])
        v_body = np.array([c * v_world[0] - s * v_world[1], s * v_world[0] + c * v_world[1]])
        u = np.array([v_body[0], v_body[1], 0.0])
        u = np.clip(u, self._lo, self._hi)
        return np.tile(u, (cfg.horizon, 1))

    # ------------------------------------------------------------------- QP
    def _solve(self, x0, U_init, refs, psi_ref, goal, preds, R):
        """SQP-RTI: re-linearise at the nominal, solve the condensed QP, repeat."""
        cfg = self.cfg
        U_nom = np.asarray(U_init, float).copy()
        res = {"status": "solver_failed", "iters": 0, "U": None}
        for _ in range(cfg.sqp_iters):
            Xb = rollout(x0, U_nom, cfg)
            P, q, A, l, u = self._build(x0, Xb, U_nom, refs, psi_ref, goal, preds, R)
            status, x, iters = self._qp(P, q, A, l, u, U_nom)
            res["iters"] += iters
            if status not in ("solved", "solved inaccurate") or x is None:
                res["status"] = "time_limit" if "time" in status else "solver_failed"
                return res
            U_nom = x.reshape(cfg.horizon, NU)
        res["status"] = "solved"
        res["U"] = U_nom
        return res

    def _qp(self, P, q, A, l, u, U_init):
        """Solve min 0.5 x'Px + q'x, l <= Ax <= u. Returns (status, x, iterations)."""
        cfg = self.cfg
        if cfg.solver == "quadprog":
            import quadprog
            n = P.shape[0]
            G = 0.5 * (P + P.T) + 1e-9 * np.eye(n)
            rows, b = [], []
            for i in range(A.shape[0]):
                if np.isfinite(l[i]):
                    rows.append(A[i]); b.append(l[i])
                if np.isfinite(u[i]):
                    rows.append(-A[i]); b.append(-u[i])
            if not rows:
                x = np.linalg.solve(G, -q)
                return "solved", x, 0
            C = np.array(rows).T
            try:
                x, _f, _xu, it, _lag, _iact = quadprog.solve_qp(G, -q, C, np.array(b), 0)
            except ValueError:
                return "infeasible", None, 0
            return "solved", x, int(np.asarray(it).ravel()[0])
        if cfg.solver == "osqp":
            solver = osqp.OSQP()
            solver.setup(P=sp.csc_matrix(np.triu(P)), q=q, A=sp.csc_matrix(A), l=l, u=u,
                         verbose=False, eps_abs=cfg.eps, eps_rel=cfg.eps,
                         max_iter=cfg.max_iter, polish=False,
                         time_limit=cfg.solve_budget_s, warm_start=True)
            solver.warm_start(x=np.asarray(U_init, float).ravel())
            sol = solver.solve()
            return sol.info.status, sol.x, int(sol.info.iter or 0)
        raise ValueError(f"unknown solver {cfg.solver!r}")

    def _build(self, x0, Xb, Ub, refs, psi_ref, goal, preds, R):
        """Condensed QP in the commands U = (u_0, ..., u_{N-1}).

        x_k = S_k x0 + M_k U + d_k, from the linearised dynamics at the nominal (Xb, Ub).
        """
        cfg = self.cfg
        N = cfg.horizon
        nU = NU * N
        S = np.eye(NX)
        M = np.zeros((NX, nU))
        d = np.zeros(NX)
        Ss, Ms, ds = [S], [M], [d]
        for k in range(N):
            A, B = jacobians(Xb[k], cfg)
            c = step(Xb[k], Ub[k], cfg) - A @ Xb[k] - B @ Ub[k]
            Bfull = np.zeros((NX, nU))
            Bfull[:, NU * k:NU * (k + 1)] = B
            M = A @ M + Bfull
            d = A @ d + c
            S = A @ S
            Ss.append(S); Ms.append(M); ds.append(d)

        def pos_map(k):
            return Ms[k][0:2], Ss[k][0:2] @ x0 + ds[k][0:2]

        # --- cost (in U)
        H = np.zeros((nU, nU))
        g = np.zeros(nU)
        for k in range(1, N + 1):
            G, off = pos_map(k)
            r = refs[k - 1]
            H += 2 * cfg.w_path * G.T @ G
            g += -2 * cfg.w_path * G.T @ (r - off)
        for k in range(1, N + 1):
            Gp = Ms[k][2:3]
            offp = Ss[k][2:3] @ x0 + ds[k][2:3]
            H += 2 * cfg.w_head * Gp.T @ Gp
            g += -2 * cfg.w_head * Gp.T @ (psi_ref[k - 1] - offp)
        G, off = pos_map(N)
        H += 2 * cfg.w_goal * G.T @ G
        g += -2 * cfg.w_goal * G.T @ (goal - off)
        for k in range(N):
            sl = slice(NU * k, NU * (k + 1))
            H[sl, sl] += 2 * cfg.w_u * np.eye(NU)
            H[sl, sl] += 2 * cfg.w_du * np.eye(NU)
            if k == 0:
                g[sl] += -2 * cfg.w_du * self._u_prev
            else:
                sp_ = slice(NU * (k - 1), NU * k)
                H[sp_, sp_] += 2 * cfg.w_du * np.eye(NU)
                H[sl, sp_] += -2 * cfg.w_du * np.eye(NU)
                H[sp_, sl] += -2 * cfg.w_du * np.eye(NU)

        # --- constraints (rows in U)
        rows, lo_b, hi_b = [], [], []

        def add(row, lb, ub):
            rows.append(row)
            lo_b.append(lb)
            hi_b.append(ub)

        for k in range(N):
            for a in range(NU):
                r = np.zeros(nU)
                r[NU * k + a] = 1.0
                add(r, self._lo[a], self._hi[a])

        for k in range(1, N + 1):
            Mv, offv = Ms[k][3:6], Ss[k][3:6] @ x0 + ds[k][3:6]
            for a in range(NU):
                add(Mv[a], self._lo[a] - offv[a], self._hi[a] - offv[a])

        du = cfg.du_max
        u_prev = self._u_prev
        for k in range(N):
            for a in range(NU):
                r = np.zeros(nU)
                r[NU * k + a] = 1.0
                if k == 0:
                    add(r, u_prev[a] - du, u_prev[a] + du)
                else:
                    r[NU * (k - 1) + a] = -1.0
                    add(r, -du, du)

        for pred, Rk in zip(preds, R):
            for k in range(1, N + 1):
                o = pred[k - 1]
                dvec = Xb[k, 0:2] - o
                nrm = np.hypot(*dvec)
                n = dvec / nrm if nrm > 1e-9 else np.array([1.0, 0.0])
                G, off = pos_map(k)
                row = n @ G
                lb = float(n @ o + Rk[k - 1] + cfg.margin_backoff - n @ off)
                add(row, lb, INF)

        A = np.vstack(rows)
        l = np.array(lo_b, dtype=float)
        u = np.array(hi_b, dtype=float)
        return H, g, A, l, u

    # ------------------------------------------------------------ verification
    def _verify(self, x0, U, preds, R):
        cfg = self.cfg
        tol = cfg.verify_tol
        X = rollout(x0, U, cfg)
        # limits
        if np.any(U < self._lo - tol) or np.any(U > self._hi + tol):
            return False, np.inf, "limit_violation"
        if np.any(X[1:, 3:6] < self._lo - tol) or np.any(X[1:, 3:6] > self._hi + tol):
            return False, np.inf, "limit_violation"
        prev = np.vstack([self._u_prev[None, :], U[:-1]])
        if np.any(np.abs(U - prev) > cfg.du_max + tol):
            return False, np.inf, "limit_violation"
        margin = np.inf
        for pred, Rk in zip(preds, R):
            dist = np.hypot(X[1:, 0] - pred[:, 0], X[1:, 1] - pred[:, 1])
            margin = min(margin, float(np.min(dist - Rk)))
        if margin < -tol:
            return False, margin, "margin_violation"
        return True, margin, "ok"

    # ------------------------------------------------------------- escape
    def _escape(self, x, preds, R):
        """Hold each candidate command over the horizon; pick the one with the largest clearance.

        Candidates are reachable from the previous command under the rate limit and lie in
        the command box. Each candidate is checked against the constant-velocity prediction
        of every agent. Returns (command, min_margin, ok). ok means the held command keeps
        every margin, under the prediction. It is not a guarantee beyond the horizon.
        """
        cfg = self.cfg
        N = cfg.horizon
        du = cfg.du_max
        up = self._u_prev
        axes = []
        for a in range(NU):
            lo = max(self._lo[a], up[a] - du)
            hi = min(self._hi[a], up[a] + du)
            axes.append(np.linspace(lo, hi, 5 if a else 9))
        C = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, NU)  # (K, 3)
        K = C.shape[0]
        st = np.tile(x, (K, 1)).astype(float)
        ax, ay, aw = cfg.alpha
        dt = cfg.dt
        best_score = np.full(K, np.inf)
        for k in range(N):
            v = st[:, 3:6]
            c, s_ = np.cos(st[:, 2]), np.sin(st[:, 2])
            px = st[:, 0] + dt * (c * v[:, 0] - s_ * v[:, 1])
            py = st[:, 1] + dt * (s_ * v[:, 0] + c * v[:, 1])
            psi = st[:, 2] + dt * v[:, 2]
            vx = v[:, 0] + ax * (C[:, 0] - v[:, 0])
            vy = v[:, 1] + ay * (C[:, 1] - v[:, 1])
            vw = v[:, 2] + aw * (C[:, 2] - v[:, 2])
            st = np.stack([px, py, psi, vx, vy, vw], axis=1)
            for pred, Rk in zip(preds, R):
                d = np.hypot(st[:, 0] - pred[k, 0], st[:, 1] - pred[k, 1])
                best_score = np.minimum(best_score, d - Rk[k])
        i = int(np.argmax(best_score))
        margin = float(best_score[i])
        return C[i].copy(), margin, bool(margin >= -cfg.verify_tol)

    # ------------------------------------------------------------- fallback
    def _ramp(self, u_prev):
        """Move the command toward zero at a_stop per step, bounded by the rate limit."""
        cfg = self.cfg
        s = min(cfg.a_stop * cfg.dt, cfg.du_max)
        return u_prev - np.clip(u_prev, -s, s)

    def _stop(self, x, preds, R):
        cfg = self.cfg
        U = np.zeros((cfg.horizon, NU))
        u = self._u_prev.copy()
        for k in range(cfg.horizon):
            u = self._ramp(u)
            U[k] = u
        X = rollout(x, U, cfg)
        margin = np.inf
        for pred, Rk in zip(preds, R):
            dist = np.hypot(X[1:, 0] - pred[:, 0], X[1:, 1] - pred[:, 1])
            margin = min(margin, float(np.min(dist - Rk)))
        ok = margin >= -cfg.verify_tol if np.isfinite(margin) else True
        cmd = self._ramp(self._u_prev)
        return cmd, margin, bool(ok)
