"""Legged NMPC (docs/17): direct multiple shooting, IPOPT via CasADi, receding horizon, warm start.

Decision variables: states X (6 x N+1) and forces U (6 x N). Hard constraints only (no slack).
Per-solve parameters: initial state, reference, force bounds (swing feet are zero), ZMP half-planes.
"""
import time

import casadi
import numpy as np

from .config import LeggedNMPCConfig
from .model import ZMP_ROWS, discrete_matrices, reference, stage_data


class LeggedNMPC:
    def __init__(self, cfg: LeggedNMPCConfig | None = None):
        self.cfg = cfg or LeggedNMPCConfig()
        self.Ad, self.Bd, self.cd = discrete_matrices(self.cfg)
        self._build()
        self._prev = None            # (X, U) of the previous solve, for the warm start
        self.n_solves = 0
        self.n_failures = 0
        self.solve_times = []

    def _build(self):
        c, N = self.cfg, self.cfg.N
        o = casadi.Opti()
        X = o.variable(6, N + 1)
        U = o.variable(6, N)
        x0 = o.parameter(6)
        Xref = o.parameter(6, N + 1)
        lbu = o.parameter(6, N)
        ubu = o.parameter(6, N)
        Pn = o.parameter(2 * ZMP_ROWS, N)   # row 2*j + comp holds the normal component
        Poff = o.parameter(ZMP_ROWS, N)

        o.subject_to(X[:, 0] == x0)
        for k in range(N):
            o.subject_to(X[:, k + 1] == casadi.mtimes(self.Ad, X[:, k])
                         + casadi.mtimes(self.Bd, U[:, k]) + self.cd)

        o.subject_to(casadi.vec(U) >= casadi.vec(lbu))
        o.subject_to(casadi.vec(U) <= casadi.vec(ubu))

        # Friction pyramid per foot (fz = 0 for a swing foot forces fx = fy = 0 through the bounds too).
        fx, fy, fz = U[0::3, :], U[1::3, :], U[2::3, :]
        o.subject_to(casadi.vec(c.MU * fz - fx) >= 0)
        o.subject_to(casadi.vec(c.MU * fz + fx) >= 0)
        o.subject_to(casadi.vec(c.MU * fz - fy) >= 0)
        o.subject_to(casadi.vec(c.MU * fz + fy) >= 0)
        o.subject_to(casadi.sum1(fz) >= c.FZ_MIN)

        # ZMP inside the support hull. zmp = (px - pz*Fx/Fz, py - pz*Fy/Fz); multiply by Fz > 0.
        for k in range(N):
            Fz = U[2, k] + U[5, k]
            Fx = U[0, k] + U[3, k]
            Fy = U[1, k] + U[4, k]
            px, py, pz = X[0, k], X[1, k], X[2, k]
            for j in range(ZMP_ROWS):
                n1, n2 = Pn[2 * j, k], Pn[2 * j + 1, k]
                o.subject_to(n1 * (px * Fz - pz * Fx) + n2 * (py * Fz - pz * Fy) + Poff[j, k] * Fz <= 0)

        J = 0
        W = [c.Q_P, c.Q_P, c.Q_Z, c.Q_V, c.Q_V, c.Q_V]
        for k in range(N):
            e = X[:, k] - Xref[:, k]
            J += casadi.sum1(casadi.vertcat(*[W[i] * e[i] ** 2 for i in range(6)]))
            J += c.R_F * casadi.sumsqr(U[:, k])
        e = X[:, N] - Xref[:, N]
        J += c.Q_TERM * casadi.sum1(casadi.vertcat(*[W[i] * e[i] ** 2 for i in range(6)]))
        o.minimize(J)
        o.solver("ipopt", {"print_time": False, "ipopt.print_level": 0,
                           "ipopt.max_iter": c.max_iter, "ipopt.warm_start_init_point": "yes", "ipopt.sb": "yes"})
        self.o, self.X, self.U = o, X, U
        self.x0, self.Xref, self.lbu, self.ubu, self.Pn, self.Poff = x0, Xref, lbu, ubu, Pn, Poff

    def _initial_guess(self, x_now, gait):
        N = self.cfg.N
        Xg = np.tile(x_now.reshape(6, 1), (1, N + 1))
        Ug = np.zeros((6, N))
        n_stance = 2 if gait == "stand" else 1
        for k in range(N):
            # Gravity-compensating vertical force, split over the feet in contact (approximate).
            Ug[2, k] = Ug[5, k] = self.cfg.MASS * self.cfg.G / 2.0 if gait == "stand" else self.cfg.MASS * self.cfg.G / n_stance
        return Xg, Ug

    def _shifted_guess(self):
        Xp, Up = self._prev
        return np.hstack([Xp[:, 1:], Xp[:, -1:]]), np.hstack([Up[:, 1:], Up[:, -1:]])

    def solve(self, t0, x_now, v_cmd, gait="walk"):
        """Solve from time t0 with the measured state. Returns (u0, ok, elapsed_s)."""
        c, N = self.cfg, self.cfg.N
        n, off, lbu, ubu = stage_data(c, t0, v_cmd, gait)
        p, v = reference(c, t0, v_cmd, gait)
        o = self.o
        o.set_value(self.x0, x_now)
        o.set_value(self.Xref, np.hstack([p, v]).T)
        o.set_value(self.lbu, lbu.T)
        o.set_value(self.ubu, ubu.T)
        o.set_value(self.Pn, n.reshape(N, 2 * ZMP_ROWS).T)
        o.set_value(self.Poff, off.T)

        Xg, Ug = self._shifted_guess() if self._prev is not None else self._initial_guess(x_now, gait)
        o.set_initial(self.X, Xg)
        o.set_initial(self.U, Ug)

        t_start = time.perf_counter()
        try:
            sol = o.solve()
            ok = True
            Xs, Us = sol.value(self.X), sol.value(self.U)
        except RuntimeError:
            ok = False
            Xs, Us = Xg, Ug            # fall back to the warm-start guess; reported as a failure
        elapsed = time.perf_counter() - t_start

        self.n_solves += 1
        self.n_failures += 0 if ok else 1
        self.solve_times.append(elapsed)
        self._prev = (Xs, Us)
        u0 = np.asarray(Us)[:, 0].copy()
        return u0, ok, elapsed
