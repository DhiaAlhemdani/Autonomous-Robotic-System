"""Base-level prediction model for the humanoid NMPC (docs/15 §3).

State x = [px, py, psi, vx, vy, wz]: world-frame position and heading, body-frame
velocity and yaw rate. Input u = [vx*, vy*, wz*]: body-frame velocity command.

    v_{k+1}   = v_k + alpha * (u_k - v_k)            (per axis, first-order lag)
    p_{k+1}   = p_k + dt * R(psi_k) [vx, vy]_k
    psi_{k+1} = psi_k + dt * wz_k

This is the only model the controller uses. It does not represent gait, joints or
foot contact. The simulator in sim.py uses the same model, so tests check the
controller, not model fidelity. Model fidelity to the walking policy is N1 in docs/15.
"""
import numpy as np

NX = 6
NU = 3


def rot(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s], [s, c]])


def step(x: np.ndarray, u: np.ndarray, cfg) -> np.ndarray:
    ax, ay, aw = cfg.alpha
    dt = cfg.dt
    xn = np.empty(NX)
    v = x[3:6]
    xn[3] = v[0] + ax * (u[0] - v[0])
    xn[4] = v[1] + ay * (u[1] - v[1])
    xn[5] = v[2] + aw * (u[2] - v[2])
    xn[0:2] = x[0:2] + dt * rot(x[2]) @ v[0:2]
    xn[2] = x[2] + dt * v[2]
    return xn


def jacobians(x: np.ndarray, cfg):
    """A = df/dx and B = df/du at (x, u) for the step() above. B does not depend on x."""
    ax, ay, aw = cfg.alpha
    dt = cfg.dt
    psi = x[2]
    vxy = x[3:5]
    c, s = np.cos(psi), np.sin(psi)
    R = np.array([[c, -s], [s, c]])
    dR = np.array([[-s, -c], [c, -s]])
    A = np.zeros((NX, NX))
    A[0:2, 0:2] = np.eye(2)
    A[0:2, 2] = dt * (dR @ vxy)
    A[0:2, 3:5] = dt * R
    A[2, 2] = 1.0
    A[2, 5] = dt
    A[3, 3] = 1.0 - ax
    A[4, 4] = 1.0 - ay
    A[5, 5] = 1.0 - aw
    B = np.zeros((NX, NU))
    B[3, 0] = ax
    B[4, 1] = ay
    B[5, 2] = aw
    return A, B


def rollout(x0: np.ndarray, U: np.ndarray, cfg) -> np.ndarray:
    """X[0] = x0, X[k+1] = step(X[k], U[k]). Returns shape (len(U)+1, 6)."""
    X = np.empty((len(U) + 1, NX))
    X[0] = x0
    for k in range(len(U)):
        X[k + 1] = step(X[k], U[k], cfg)
    return X
