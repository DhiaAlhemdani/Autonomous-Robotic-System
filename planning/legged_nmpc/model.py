"""Reduced model (docs/17 §2-3, docs/18 §2-4): exact discrete CoM dynamics, yaw, gait, footsteps, swing, ZMP rows.

State x = [px, py, pz, vx, vy, vz, psi, w]. Input u = [fx1, fy1, fz1, fx2, fy2, fz2].
"""
import math

import numpy as np
from scipy.spatial import ConvexHull

from .config import LeggedNMPCConfig

LEFT, RIGHT = 0, 1
ZMP_ROWS = 8          # fixed number of half-planes per stage; unused rows are non-binding
NONBINDING_OFF = -1e3
NX = 8


def discrete_matrices(cfg: LeggedNMPCConfig):
    """Linear part: x_{k+1} = Ad x_k + Bd u_k + cd on the translational states (rows 0..5).

    Yaw rows (6, 7) are nonlinear in the stage torque tau_z, handled in the controller.
    Returns Ad (6x6), Bd (6x6), cd (6,).
    """
    dt, M = cfg.dt, cfg.MASS
    I3 = np.eye(3)
    Z3 = np.zeros((3, 3))
    Ad = np.block([[I3, dt * I3], [Z3, I3]])
    Bd = np.vstack([dt ** 2 / (2.0 * M) * np.hstack([I3, I3]), dt / M * np.hstack([I3, I3])])
    g_vec = np.array([0.0, 0.0, -cfg.G])
    cd = np.concatenate([0.5 * dt ** 2 * g_vec, dt * g_vec])
    return Ad, Bd, cd


def rot(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def ref_com_xy(cfg, t, v, w):
    """Closed-form reference CoM (xy) and heading at time t for forward speed v and yaw rate w."""
    if abs(w) < 1e-9:
        return np.array([v * t, 0.0]), 0.0
    psi = w * t
    return np.array([v / w * math.sin(psi), v / w * (1.0 - math.cos(psi))]), psi


def ref_velocity_xy(v, psi):
    return np.array([v * math.cos(psi), v * math.sin(psi)])


def _stance_index(cfg, t, foot):
    """Return (k, into) for foot: stance intervals start at s_foot + k*PERIOD."""
    s = 0.0 if foot == LEFT else 0.5 * cfg.PERIOD
    rel = t - s
    k = math.floor(rel / cfg.PERIOD)
    return s + k * cfg.PERIOD, rel - k * cfg.PERIOD


def placement(cfg, t_start, v, w, foot):
    """Sole centre (xy) and yaw of the stance that starts at t_start (placed at mid-stance)."""
    t_mid = t_start + 0.5 * cfg.STANCE_T
    ref, psi = ref_com_xy(cfg, t_mid, v, w)
    lateral = np.array([0.0, cfg.FOOT_Y if foot == LEFT else -cfg.FOOT_Y])
    return ref + rot(psi) @ lateral, psi


def next_placement(cfg, t, v, w, foot):
    """Placement of the stance containing t, or of the next stance if the foot is in swing."""
    start, into = _stance_index(cfg, t, foot)
    if into >= cfg.STANCE_T:
        start += cfg.PERIOD
    return placement(cfg, start, v, w, foot)


def contact_state(cfg, t, v, w, gait):
    """Per foot: (in_contact, sole centre xy, yaw) at time t. Swing feet report their next placement."""
    out = []
    for foot in (LEFT, RIGHT):
        if gait == "stand":
            c = np.array([0.0, cfg.FOOT_Y if foot == LEFT else -cfg.FOOT_Y])
            out.append((True, c, 0.0))
            continue
        start, into = _stance_index(cfg, t, foot)
        if into < cfg.STANCE_T:
            c, psi = placement(cfg, start, v, w, foot)
            out.append((True, c, psi))
        else:
            c, psi = next_placement(cfg, t, v, w, foot)
            out.append((False, c, psi))
    return out


def swing_pose(cfg, t, v, w, foot):
    """Kinematic sole reference of a foot at time t: (xy, z, yaw). Stance: z = 0 at the placement."""
    start, into = _stance_index(cfg, t, foot)
    if into < cfg.STANCE_T:
        c, psi = placement(cfg, start, v, w, foot)
        return c, 0.0, psi
    s = (into - cfg.STANCE_T) / cfg.SWING_T
    c0, psi0 = placement(cfg, start, v, w, foot)            # previous stance
    c1, psi1 = placement(cfg, start + cfg.PERIOD, v, w, foot)  # next stance
    e = 0.5 * (1.0 - math.cos(math.pi * s))
    xy = c0 + (c1 - c0) * e
    z = cfg.SWING_H * math.sin(math.pi * s)
    return xy, z, psi0 + (psi1 - psi0) * s


def _box_corners(cfg, centre, psi):
    R = rot(psi)
    local = np.array([[sx * cfg.FOOT_HX, sy * cfg.FOOT_HY] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)])
    return centre + local @ R.T


def zmp_rows(cfg, contacts):
    """Half-planes n.z + off <= 0 for the convex hull of the active (rotated) support boxes."""
    pts = np.vstack([_box_corners(cfg, c, psi) for ok, c, psi in contacts if ok])
    eq = ConvexHull(pts).equations
    m = eq.shape[0]
    if m > ZMP_ROWS:
        raise ValueError(f"hull has {m} facets, more than ZMP_ROWS={ZMP_ROWS}")
    n = np.zeros((ZMP_ROWS, 2))
    off = np.full(ZMP_ROWS, NONBINDING_OFF)
    n[:m] = eq[:, :2]
    off[:m] = eq[:, 2]
    return n, off


def reference(cfg, t0, v, w, gait):
    """Reference states for stages 0..N: x_ref (N+1, NX)."""
    if gait == "stand":
        v, w = 0.0, 0.0
    ref = np.zeros((cfg.N + 1, NX))
    for k in range(cfg.N + 1):
        t = t0 + k * cfg.dt
        p, psi = ref_com_xy(cfg, t, v, w)
        vel = ref_velocity_xy(v, psi)
        ref[k, :3] = [p[0], p[1], cfg.Z_NOM]
        ref[k, 3:6] = [vel[0], vel[1], 0.0]
        ref[k, 6] = psi
        ref[k, 7] = w
    return ref


def stage_data(cfg, t0, v, w, gait):
    """Per stage k in 0..N-1: ZMP rows, force bounds (zero for swing), foot centres for the yaw torque."""
    N = cfg.N
    n = np.zeros((N, ZMP_ROWS, 2))
    off = np.zeros((N, ZMP_ROWS))
    lbu = np.zeros((N, 6))
    ubu = np.zeros((N, 6))
    rf = np.zeros((N, 4))
    for k in range(N):
        t = t0 + k * cfg.dt
        contacts = contact_state(cfg, t, v, w, gait)
        n[k], off[k] = zmp_rows(cfg, contacts)
        for f, (ok, c, _) in enumerate(contacts):
            s = slice(3 * f, 3 * f + 3)
            if ok:
                lbu[k, s] = [-cfg.FX_MAX, -cfg.FX_MAX, 0.0]
                ubu[k, s] = [cfg.FX_MAX, cfg.FX_MAX, cfg.FZ_MAX]
            rf[k, 2 * f:2 * f + 2] = c
    return n, off, lbu, ubu, rf


def yaw_torque(state, u, rf_k):
    """tau_z = sum_i (r_ix - px) fy_i - (r_iy - py) fx_i, for numpy arrays."""
    tau = 0.0
    for f in range(2):
        fx, fy = u[3 * f], u[3 * f + 1]
        rx, ry = rf_k[2 * f], rf_k[2 * f + 1]
        tau += (rx - state[0]) * fy - (ry - state[1]) * fx
    return tau


def zmp(state, u):
    """Centre of pressure of the applied forces: (px - pz Fx/Fz, py - pz Fy/Fz)."""
    Fz = u[2] + u[5]
    Fx = u[0] + u[3]
    Fy = u[1] + u[4]
    return np.array([state[0] - state[2] * Fx / Fz, state[1] - state[2] * Fy / Fz])
