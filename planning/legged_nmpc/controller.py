"""Legged NMPC (docs/17, extended in docs/18): direct multiple shooting, IPOPT via CasADi, receding horizon.

Decision variables: states X (8 x N+1) and forces U (6 x N). Hard constraints only (no slack).
Per-solve parameters: initial state, reference, force bounds, ZMP half-planes, stance foot centres (yaw torque).
"""
import time

import casadi
import numpy as np

from .config import LeggedNMPCConfig
from .model import NX, ZMP_ROWS, discrete_matrices, reference, stage_data


class LeggedNMPC:
    def __init__(self, cfg: LeggedNMPCConfig | None = None):
        self.cfg = cfg or LeggedNMPCConfig()
        self.Ad, self.Bd, self.cd = discrete_matrices(self.cfg)
        self._build()
        self._prev = None
        self.n_solves = 0
        self.n_failures = 0
        self.solve_times = []

    def _build(self):
        c, N = self.cfg, self.cfg.N
        o = casadi.Opti()
        X = o.variable(NX, N + 1)
        U = o.variable(6, N)
        x0 = o.parameter(NX)
        Xref = o.parameter(NX, N + 1)
        lbu = o.parameter(6, N)
        ubu = o.parameter(6, N)
        Pn = o.parameter(2 * ZMP_ROWS, N)
        Poff = o.parameter(ZMP_ROWS, N)
        Rf = o.parameter(4, N)

        o.subject_to(X[:, 0] == x0)
        for k in range(N):
            xk, uk = X[:, k], U[:, k]
            tau = 0
            for f in range(2):
                tau += (Rf[2 * f, k] - xk[0]) * uk[3 * f + 1] - (Rf[2 * f + 1, k] - xk[1]) * uk[3 * f]
            o.subject_to(X[0:6, k + 1] == casadi.mtimes(self.Ad, X[0:6, k])
                         + casadi.mtimes(self.Bd, uk) + self.cd)
            o.subject_to(X[6, k + 1] == xk[6] + c.dt * xk[7] + 0.5 * c.dt ** 2 * tau / c.IZ)
            o.subject_to(X[7, k + 1] == xk[7] + c.dt * tau / c.IZ)

        o.subject_to(casadi.vec(U) >= casadi.vec(lbu))
        o.subject_to(casadi.vec(U) <= casadi.vec(ubu))

        fx, fy, fz = U[0::3, :], U[1::3, :], U[2::3, :]
        o.subject_to(casadi.vec(c.MU * fz - fx) >= 0)
        o.subject_to(casadi.vec(c.MU * fz + fx) >= 0)
        o.subject_to(casadi.vec(c.MU * fz - fy) >= 0)
        o.subject_to(casadi.vec(c.MU * fz + fy) >= 0)
        o.subject_to(casadi.sum1(fz) >= c.FZ_MIN)

        for k in range(N):
            Fz = U[2, k] + U[5, k]
            Fx = U[0, k] + U[3, k]
            Fy = U[1, k] + U[4, k]
            px, py, pz = X[0, k], X[1, k], X[2, k]
            for j in range(ZMP_ROWS):
                n1, n2 = Pn[2 * j, k], Pn[2 * j + 1, k]
                o.subject_to(n1 * (px * Fz - pz * Fx) + n2 * (py * Fz - pz * Fy) + Poff[j, k] * Fz <= 0)

        W = [c.Q_P, c.Q_P, c.Q_Z, c.Q_V, c.Q_V, c.Q_V, c.Q_PSI, c.Q_W]

        def stage_cost(e):
            return casadi.sum1(casadi.vertcat(*[W[i] * e[i] ** 2 for i in range(NX)]))

        J = 0
        for k in range(N):
            J += stage_cost(X[:, k] - Xref[:, k]) + c.R_F * casadi.sumsqr(U[:, k])
        J += c.Q_TERM * stage_cost(X[:, N] - Xref[:, N])
        o.minimize(J)
        o.solver("ipopt", {"print_time": False, "ipopt.print_level": 0, "ipopt.sb": "yes",
                           "ipopt.max_iter": c.max_iter, "ipopt.tol": c.ipopt_tol,
                           "ipopt.acceptable_tol": 10 * c.ipopt_tol, "ipopt.warm_start_init_point": "yes"})
        self.o, self.X, self.U = o, X, U
        self.x0, self.Xref, self.lbu, self.ubu = x0, Xref, lbu, ubu
        self.Pn, self.Poff, self.Rf = Pn, Poff, Rf

    def _initial_guess(self, x_now):
        """Start guess: CoM at its measured state, half the weight on each foot's vertical force."""
        N = self.cfg.N
        Xg = np.tile(x_now.reshape(NX, 1), (1, N + 1))
        Ug = np.zeros((6, N))
        Ug[2, :] = Ug[5, :] = self.cfg.MASS * self.cfg.G / 2.0
        return Xg, Ug

    def solve(self, t0, x_now, v, w, gait="walk"):
        """Solve from time t0 with the measured state. Returns (u0, ok, elapsed_s)."""
        c, N = self.cfg, self.cfg.N
        n, off, lbu, ubu, rf = stage_data(c, t0, v, w, gait)
        ref = reference(c, t0, v, w, gait)
        o = self.o
        o.set_value(self.x0, x_now)
        o.set_value(self.Xref, ref.T)
        o.set_value(self.lbu, lbu.T)
        o.set_value(self.ubu, ubu.T)
        o.set_value(self.Pn, n.reshape(N, 2 * ZMP_ROWS).T)
        o.set_value(self.Poff, off.T)
        o.set_value(self.Rf, rf.T)

        Xg, Ug = self._shifted_guess() if self._prev is not None else self._initial_guess(x_now)
        o.set_initial(self.X, Xg)
        o.set_initial(self.U, Ug)

        t_start = time.perf_counter()
        try:
            sol = o.solve()
            ok = True
            Xs, Us = sol.value(self.X), sol.value(self.U)
        except RuntimeError:
            ok = False
            Xs, Us = Xg, Ug
        elapsed = time.perf_counter() - t_start

        self.n_solves += 1
        self.n_failures += 0 if ok else 1
        self.solve_times.append(elapsed)
        self._prev = (np.asarray(Xs), np.asarray(Us))
        return np.asarray(Us)[:, 0].copy(), ok, elapsed

    def _shifted_guess(self):
        Xp, Up = self._prev
        return np.hstack([Xp[:, 1:], Xp[:, -1:]]), np.hstack([Up[:, 1:], Up[:, -1:]])
