"""NMPC with direct multiple shooting, solved by IPOPT through CasADi (docs/16).

Each call builds nothing: the NLP is built once; the initial state, goal, previous input and
obstacles enter as parameters. The previous solution is shifted to warm-start the next solve.
"""
import time

import casadi as ca
import numpy as np

from .config import NMPCConfig
from .model import make_dynamics

INACTIVE_OBS = (1e3, 1e3, 0.0)   # padding for unused obstacle slots: far away, zero radius


class NMPC:
    def __init__(self, cfg: NMPCConfig = NMPCConfig()):
        self.cfg = cfg
        c = cfg
        N, M = c.N, c.MAX_OBS
        F = make_dynamics(c.dt)

        X = ca.MX.sym("X", 3 * (N + 1))
        U = ca.MX.sym("U", 2 * N)
        S = ca.MX.sym("S", N * M)
        # parameters: x0(3), goal(2), u_prev(2), obstacles (cx, cy, r) * M
        P = ca.MX.sym("P", 7 + 3 * M)

        def x(k):
            return X[3 * k:3 * k + 3]

        def u(k):
            return U[2 * k:2 * k + 2]

        g, lbg, ubg = [], [], []
        J = 0
        g.append(x(0) - P[0:3])
        lbg += [0.0] * 3
        ubg += [0.0] * 3
        for k in range(N):
            g.append(x(k + 1) - F(x(k), u(k)))
            lbg += [0.0] * 3
            ubg += [0.0] * 3
            du = u(k) - (P[5:7] if k == 0 else u(k - 1))
            J += c.Q_P * ca.sumsqr(x(k)[0:2] - P[3:5]) + c.R_U * ca.sumsqr(u(k)) + c.R_DU * ca.sumsqr(du)
            for j in range(M):
                cx, cy, r = P[7 + 3 * j], P[8 + 3 * j], P[9 + 3 * j]
                s = S[k * M + j]
                g.append(ca.sumsqr(x(k)[0:2] - ca.vertcat(cx, cy)) - (r + c.MARGIN) ** 2 + s)
                lbg.append(0.0)
                ubg.append(np.inf)
                J += c.W_SLACK * s
        J += c.Q_TERM * ca.sumsqr(x(N)[0:2] - P[3:5])

        nlp = {"x": ca.vertcat(X, U, S), "p": P, "f": J, "g": ca.vertcat(*g)}
        opts = {"print_time": 0, "ipopt.print_level": 0, "ipopt.max_iter": c.max_iter,
                "ipopt.tol": 1e-4, "ipopt.sb": "yes",
                "ipopt.warm_start_init_point": "yes"}
        self.solver = ca.nlpsol("nmpc", "ipopt", nlp, opts)
        self.n_x, self.n_u, self.n_s = 3 * (N + 1), 2 * N, N * M
        self.lbx = np.concatenate([np.full(self.n_x, -np.inf), np.tile([-c.V_MAX, -c.W_MAX], N),
                                   np.zeros(self.n_s)])
        self.ubx = np.concatenate([np.full(self.n_x, np.inf), np.tile([c.V_MAX, c.W_MAX], N),
                                   np.full(self.n_s, np.inf)])
        self.lbg, self.ubg = np.array(lbg), np.array(ubg)
        self.w0 = None
        self.fails = 0

    def reset(self):
        self.w0 = None

    def _guess(self, x0, goal):
        c, N = self.cfg, self.cfg.N
        d = goal - x0[:2]
        dist = np.hypot(*d)
        th = np.arctan2(d[1], d[0]) if dist > 1e-9 else x0[2]
        v = min(c.V_MAX, dist / (N * c.dt))
        Xg = np.zeros((N + 1, 3))
        for k in range(N + 1):
            f = k / N
            Xg[k, :2] = x0[:2] + f * d
            Xg[k, 2] = th
        Xg[0] = x0
        Ug = np.tile([v, 0.0], (N, 1))
        return np.concatenate([Xg.ravel(), Ug.ravel(), np.zeros(self.n_s)])

    def _shift(self, w):
        N, M = self.cfg.N, self.cfg.MAX_OBS
        X = w[:self.n_x].reshape(N + 1, 3)
        U = w[self.n_x:self.n_x + self.n_u].reshape(N, 2)
        S = w[self.n_x + self.n_u:].reshape(N, M)
        X = np.vstack([X[1:], X[-1:]])
        U = np.vstack([U[1:], U[-1:]])
        S = np.vstack([S[1:], S[-1:]])
        return np.concatenate([X.ravel(), U.ravel(), S.ravel()])

    def solve(self, x0, goal, obstacles=(), u_prev=(0.0, 0.0)):
        """Returns (u0, info). u0 is the applied input. On a failed solve, u0 is a brake (zero)."""
        c = self.cfg
        if len(obstacles) > c.MAX_OBS:
            raise ValueError(f"at most {c.MAX_OBS} obstacles")
        x0 = np.asarray(x0, dtype=float)
        goal = np.asarray(goal, dtype=float)
        obs = list(obstacles) + [INACTIVE_OBS] * (c.MAX_OBS - len(obstacles))
        p = np.concatenate([x0, goal, np.asarray(u_prev, dtype=float), np.array(obs, dtype=float).ravel()])
        w0 = self.w0 if self.w0 is not None else self._guess(x0, goal)
        t0 = time.perf_counter()
        sol = self.solver(x0=w0, p=p, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        stats = self.solver.stats()
        success = bool(stats["success"])
        w = np.array(sol["x"]).ravel()
        Xp = w[:self.n_x].reshape(c.N + 1, 3)
        u0 = w[self.n_x:self.n_x + 2].copy()
        slack = w[self.n_x + self.n_u:]
        info = dict(success=success, status=stats["return_status"], iters=int(stats["iter_count"]),
                    solve_ms=dt_ms, slack_max=float(slack.max()) if slack.size else 0.0,
                    x_pred=Xp)
        if success:
            self.w0 = self._shift(w)
        else:
            self.fails += 1
            self.w0 = None
            u0 = np.zeros(2)
        info["u"] = u0
        return u0, info
