"""Unicycle model and RK4 discretisation (docs/16 §2). Numeric and symbolic versions share one formula."""
import numpy as np
import casadi as ca


def _f(x, u):
    return ca.vertcat(u[0] * ca.cos(x[2]), u[0] * ca.sin(x[2]), u[1])


def make_dynamics(dt):
    """Symbolic one-step map F(x, u) -> x_next (RK4)."""
    x = ca.MX.sym("x", 3)
    u = ca.MX.sym("u", 2)
    k1 = _f(x, u)
    k2 = _f(x + dt / 2 * k1, u)
    k3 = _f(x + dt / 2 * k2, u)
    k4 = _f(x + dt * k3, u)
    return ca.Function("F", [x, u], [x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])


def rk4_step(x, u, dt):
    """Numeric one-step map, same formula as make_dynamics."""
    def f(s):
        return np.array([u[0] * np.cos(s[2]), u[0] * np.sin(s[2]), u[1]])
    k1 = f(x)
    k2 = f(x + dt / 2 * k1)
    k3 = f(x + dt / 2 * k2)
    k4 = f(x + dt * k3)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def exact_constant_input(x, u, t):
    """Analytic unicycle motion for constant (v, w) over time t (used to check the RK4 step)."""
    v, w = u
    th0 = x[2]
    th1 = th0 + w * t
    if abs(w) < 1e-12:
        return np.array([x[0] + v * t * np.cos(th0), x[1] + v * t * np.sin(th0), th1])
    return np.array([x[0] + v / w * (np.sin(th1) - np.sin(th0)),
                     x[1] - v / w * (np.cos(th1) - np.cos(th0)), th1])
