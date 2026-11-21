"""Reduced model: exact discrete CoM dynamics, gait, support geometry and ZMP half-planes (docs/17 §2-3)."""
import math

import numpy as np
from scipy.spatial import ConvexHull

from .config import LeggedNMPCConfig

LEFT, RIGHT = 0, 1
ZMP_ROWS = 8          # fixed number of half-planes per stage; unused rows are non-binding
NONBINDING_OFF = -1e3

# State x = [px, py, pz, vx, vy, vz]. Input u = [fx1, fy1, fz1, fx2, fy2, fz2].


def discrete_matrices(cfg: LeggedNMPCConfig):
    """x_{k+1} = Ad x_k + Bd u_k + cd, exact for constant forces over one step."""
    dt, M = cfg.dt, cfg.MASS
    I3 = np.eye(3)
    Z3 = np.zeros((3, 3))
    Ad = np.block([[I3, dt * I3], [Z3, I3]])
    half = dt ** 2 / (2.0 * M) * np.hstack([I3, I3])
    full = dt / M * np.hstack([I3, I3])
    Bd = np.vstack([half, full])
    g_vec = np.array([0.0, 0.0, -cfg.G])
    cd = np.concatenate([0.5 * dt ** 2 * g_vec, dt * g_vec])
    return Ad, Bd, cd


def foot_y(cfg, foot):
    return cfg.FOOT_Y if foot == LEFT else -cfg.FOOT_Y


def foot_start(cfg, foot):
    return 0.0 if foot == LEFT else 0.5 * cfg.PERIOD


def contact_state(cfg, t, v_cmd, gait):
    """Per foot: (in_contact, support centre xy or None) at time t.

    'walk': each foot is in stance for STANCE_T of every PERIOD, offset by half a period between feet.
    The support centre is the reference CoM position at the middle of that stance window.
    'stand': both feet in contact at the origin.
    """
    out = []
    for foot in (LEFT, RIGHT):
        if gait == "stand":
            out.append((True, np.array([0.0, foot_y(cfg, foot)])))
            continue
        rel = t - foot_start(cfg, foot)
        k = math.floor(rel / cfg.PERIOD)
        into = rel - k * cfg.PERIOD
        if into < cfg.STANCE_T:
            start_k = foot_start(cfg, foot) + k * cfg.PERIOD
            cx = v_cmd * (start_k + 0.5 * cfg.STANCE_T)
            out.append((True, np.array([cx, foot_y(cfg, foot)])))
        else:
            out.append((False, None))
    return out


def _box_corners(cfg, centre):
    cx, cy = centre
    return np.array([[cx + sx * cfg.FOOT_HX, cy + sy * cfg.FOOT_HY]
                     for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)])


def zmp_rows(cfg, contacts):
    """Half-planes n.z + off <= 0 whose intersection is the convex hull of the active support boxes.

    Returns n (ZMP_ROWS, 2) and off (ZMP_ROWS,). Unused rows have n = 0 and off = NONBINDING_OFF.
    """
    pts = np.vstack([_box_corners(cfg, c) for ok, c in contacts if ok])
    eq = ConvexHull(pts).equations            # rows: n.x + off <= 0 inside the hull
    m = eq.shape[0]
    if m > ZMP_ROWS:
        raise ValueError(f"hull has {m} facets, more than ZMP_ROWS={ZMP_ROWS}")
    n = np.zeros((ZMP_ROWS, 2))
    off = np.full(ZMP_ROWS, NONBINDING_OFF)
    n[:m] = eq[:, :2]
    off[:m] = eq[:, 2]
    return n, off


def reference(cfg, t0, v_cmd, gait):
    """Reference states for stages 0..N: positions (N+1, 3) and velocities (N+1, 3)."""
    t = t0 + np.arange(cfg.N + 1) * cfg.dt
    if gait == "stand":
        v_cmd = 0.0
    p = np.stack([v_cmd * t, np.zeros_like(t), np.full_like(t, cfg.Z_NOM)], axis=1)
    v = np.tile([v_cmd, 0.0, 0.0], (cfg.N + 1, 1))
    return p, v


def stage_data(cfg, t0, v_cmd, gait):
    """Per stage k in 0..N-1: ZMP half-planes, and force bounds (zero for a swing foot)."""
    N = cfg.N
    n = np.zeros((N, ZMP_ROWS, 2))
    off = np.zeros((N, ZMP_ROWS))
    lbu = np.zeros((N, 6))
    ubu = np.zeros((N, 6))
    for k in range(N):
        contacts = contact_state(cfg, t0 + k * cfg.dt, v_cmd, gait)
        n[k], off[k] = zmp_rows(cfg, contacts)
        for f, (ok, _) in enumerate(contacts):
            s = slice(3 * f, 3 * f + 3)
            if ok:
                lbu[k, s] = [-cfg.FX_MAX, -cfg.FX_MAX, 0.0]
                ubu[k, s] = [cfg.FX_MAX, cfg.FX_MAX, cfg.FZ_MAX]
    return n, off, lbu, ubu


def zmp(state, u):
    """Centre of pressure of the applied forces, z = p - pz * F_h / F_z (ground plane z = 0)."""
    Fz = u[2] + u[5]
    Fx = u[0] + u[3]
    Fy = u[1] + u[4]
    return np.array([state[0] - state[2] * Fx / Fz, state[1] - state[2] * Fy / Fz])
