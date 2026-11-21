"""Tests declared in docs/17 §6 and docs/18 §6, written before the implementation. Simulation only.

Thresholds are the declared ones. A failing test is reported as a failure, not relaxed.
"""
from pathlib import Path

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from planning.legged_nmpc import model as M
from planning.legged_nmpc.config import LeggedNMPCConfig
from planning.legged_nmpc.sim import run_episode, scenario
from planning.legged_nmpc.wbik import WholeBody

CFG = LeggedNMPCConfig()
VENDOR = Path(__file__).resolve().parents[2] / "vendor" / "t800"


def _nominal(scn):
    base = dict(gait="walk", v=0.0, w=0.0, offset=[0.0, 0.0], psi0=0.0, push_N=0.0, push_dir=0.0,
                push_t=0.0, push_dur=0.0, T=10.0)
    base.update(scn)
    return base


def test_1_model_matches_ode_for_constant_force():
    Ad, Bd, cd = M.discrete_matrices(CFG)
    rng = np.random.default_rng(0)
    x0 = rng.normal(size=6)
    u = rng.normal(size=6) * 100.0
    x_disc = Ad @ x0 + Bd @ u + cd
    a = (u[:3] + u[3:]) / CFG.MASS + np.array([0.0, 0.0, -CFG.G])

    sol = solve_ivp(lambda _, y: np.concatenate([y[3:], a]), (0.0, CFG.dt), x0, rtol=1e-12, atol=1e-12)
    assert np.max(np.abs(sol.y[:, -1] - x_disc)) < 1e-9


@pytest.mark.skipif(not VENDOR.exists(), reason="vendor/t800 not present")
def test_0_constants_match_vendor_model():
    import mujoco
    from planning.legged_nmpc.wbik import load_model
    m = load_model()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    com = d.subtree_com[0]
    assert abs(sum(m.body_mass) - CFG.MASS) < 1e-3
    wb = WholeBody(m)
    sole_bottom_z = d.xpos[wb.feet[0]][2] - wb.origin_to_sole_bottom[0]   # sole plane, default pose
    assert abs((com[2] - sole_bottom_z) - CFG.Z_NOM) < 1e-3
    assert abs(wb.origin_to_sole[0][0] - CFG.FOOT_OFFSET_X) < 2e-3
    # Yaw inertia about the whole-body CoM, from body inertias and parallel-axis terms.
    Iz = 0.0
    for i in range(1, m.nbody):
        R = d.ximat[i].reshape(3, 3)
        r = d.xipos[i] - com
        Iz += m.body_mass[i] * (r[0] ** 2 + r[1] ** 2) + (R @ np.diag(m.body_inertia[i]) @ R.T)[2, 2]
    assert abs(Iz - CFG.IZ) < 0.05


def test_2_standing_converges_to_nominal():
    r = run_episode(_nominal(dict(gait="stand", offset=[0.02, 0.0], T=3.0)))
    final_err = float(np.linalg.norm(np.array(r["final_state"][:2])))
    assert r["solver_failures"] == 0
    assert r["zmp_violation_max_m"] < 1e-6
    assert final_err < 1e-3, f"final CoM error {final_err:.4f} m, declared limit 0.001 m"


def test_3_walking_command_tracked():
    r = run_episode(_nominal(dict(v=0.3, T=10.0)))
    assert r["solver_failures"] == 0
    assert r["zmp_violation_max_m"] < 1e-6
    assert r["friction_violation_max_N"] < 1e-6
    assert r["force_bound_violation_max_N"] < 1e-6
    assert r["vel_mean_error_last_5s"] < 0.02, f"windowed mean velocity error {r['vel_mean_error_last_5s']:.4f}"


def test_4_yaw_tracking():
    r = run_episode(_nominal(dict(v=0.2, w=0.2, T=10.0)))
    assert r["solver_failures"] == 0
    assert r["heading_error_mean_last_5s"] < 0.05, f"heading error {r['heading_error_mean_last_5s']:.4f} rad"


def test_5_footstep_properties():
    v, w = 0.25, 0.2
    cfg = CFG
    for k in range(60):
        t = 0.1 * k
        for foot in (M.LEFT, M.RIGHT):
            ok, c, psi = M.contact_state(cfg, t, v, w, "walk")[foot]
    # Alternation and step length over 10 s, per foot.
    placements = {M.LEFT: [], M.RIGHT: []}
    for foot in (M.LEFT, M.RIGHT):
        s = 0.0 if foot == M.LEFT else 0.5 * cfg.PERIOD
        for k in range(int(10 / cfg.PERIOD) + 1):
            c, psi = M.placement(cfg, s + k * cfg.PERIOD, v, w, foot)
            placements[foot].append((c, psi))
    for foot in (M.LEFT, M.RIGHT):
        for (c0, _), (c1, _) in zip(placements[foot][:-1], placements[foot][1:]):
            assert np.linalg.norm(c1 - c0) <= cfg.STEP_MAX
    # Lateral offset of each sole centre from the CoM reference at its mid-stance time: +/- FOOT_Y within 1 mm.
    for foot, sign in ((M.LEFT, 1.0), (M.RIGHT, -1.0)):
        s = 0.0 if foot == M.LEFT else 0.5 * cfg.PERIOD
        for k in range(int(10 / cfg.PERIOD)):
            t_mid = s + k * cfg.PERIOD + 0.5 * cfg.STANCE_T
            ref, psi = M.ref_com_xy(cfg, t_mid, v, w)
            c, _ = M.placement(cfg, s + k * cfg.PERIOD, v, w, foot)
            lat = M.rot(psi).T @ (c - ref)
            assert abs(lat[1] - sign * cfg.FOOT_Y) < 1e-3
    # Foot yaw equals heading at mid-stance.
    for foot in (M.LEFT, M.RIGHT):
        s = 0.0 if foot == M.LEFT else 0.5 * cfg.PERIOD
        _, psi = M.placement(cfg, s, v, w, foot)
        assert abs(psi - w * (s + 0.5 * cfg.STANCE_T)) < 1e-12


def test_6_swing_properties():
    cfg = CFG
    v, w = 0.25, 0.2
    for foot in (M.LEFT, M.RIGHT):
        s0 = 0.0 if foot == M.LEFT else 0.5 * cfg.PERIOD
        zs = []
        for k in range(20):
            t_lift = s0 + cfg.STANCE_T + k * cfg.PERIOD
            t_land = t_lift + cfg.SWING_T
            xy0, z0, _ = M.swing_pose(cfg, t_lift + 1e-9, v, w, foot)
            xy1, z1, _ = M.swing_pose(cfg, t_land - 1e-9, v, w, foot)
            c_prev, _ = M.placement(cfg, s0 + k * cfg.PERIOD, v, w, foot)
            c_next, _ = M.placement(cfg, s0 + (k + 1) * cfg.PERIOD, v, w, foot)
            assert np.linalg.norm(xy0 - c_prev) < 1e-6 and abs(z0) < 1e-6
            assert np.linalg.norm(xy1 - c_next) < 1e-3 and abs(z1) < 0.01
            for s in np.linspace(0, 1, 41)[1:-1]:
                _, z, _ = M.swing_pose(cfg, t_lift + s * cfg.SWING_T, v, w, foot)
                zs.append(z)
                assert z >= 0.0
        assert abs(max(zs) - cfg.SWING_H) < 1e-3


def test_7_whole_body_reference_tracking():
    r = run_episode(_nominal(dict(v=0.25, w=0.2, T=10.0)))
    assert r["com_res_max_m"] < 0.005, f"CoM residual {r['com_res_max_m']:.4f} m, declared 0.005 m"
    assert r["feet_res_stance_max_m"] < 0.005, f"stance residual {r['feet_res_stance_max_m']:.4f} m, declared 0.005 m"


def test_8_determinism():
    a = run_episode(scenario(3))
    b = run_episode(scenario(3))
    assert a["final_state"] == b["final_state"]
    assert a["solver_failures"] == b["solver_failures"]


def test_9_applied_forces_respect_bounds():
    r = run_episode(scenario(11))
    assert r["friction_violation_max_N"] < 1e-6
    assert r["force_bound_violation_max_N"] < 1e-6
