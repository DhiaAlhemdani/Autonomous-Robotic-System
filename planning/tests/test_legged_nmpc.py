"""Tests declared in docs/17 §6, written before the implementation. Simulation only.

Thresholds are the declared ones. A failing test is reported as a failure, not relaxed.
"""
from pathlib import Path

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from planning.legged_nmpc.config import LeggedNMPCConfig
from planning.legged_nmpc.model import discrete_matrices
from planning.legged_nmpc.sim import run_episode, scenario

CFG = LeggedNMPCConfig()
VENDOR = Path(__file__).resolve().parents[2] / "vendor" / "t800"


def test_1_model_matches_ode_for_constant_force():
    Ad, Bd, cd = discrete_matrices(CFG)
    rng = np.random.default_rng(0)
    x0 = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
    u = rng.normal(size=6) * 100.0
    x_disc = Ad @ x0 + Bd @ u + cd
    F = u[:3] + u[3:]
    a = F / CFG.MASS + np.array([0.0, 0.0, -CFG.G])

    def rhs(_, y):
        return np.concatenate([y[3:], a])

    sol = solve_ivp(rhs, (0.0, CFG.dt), x0, rtol=1e-12, atol=1e-12)
    assert np.max(np.abs(sol.y[:, -1] - x_disc)) < 1e-9


@pytest.mark.skipif(not VENDOR.exists(), reason="vendor/t800 not present")
def test_0_constants_match_vendor_model():
    import glob
    import mujoco
    xml = glob.glob(str(VENDOR / "**" / "serial_t800.xml"), recursive=True)[0]
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert abs(sum(m.body_mass) - CFG.MASS) < 1e-3
    assert abs(d.subtree_com[0][2] - CFG.Z_NOM) < 1e-3
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "LINK_FOOT_L")
    assert abs(d.xpos[i][1] - CFG.FOOT_Y) < 1e-3


def test_2_standing_converges_to_nominal():
    r = run_episode(dict(gait="stand", v_cmd=0.0, offset=[0.02, 0.0], push_N=0.0, push_dir=0.0,
                         push_t=0.0, push_dur=0.0, T=3.0))
    final_err = float(np.linalg.norm(np.array(r["final_state"][:2])))
    assert r["solver_failures"] == 0
    assert r["zmp_violation_max_m"] < 1e-6
    assert final_err < 1e-3, f"final CoM error {final_err:.4f} m, declared limit 0.001 m"


def test_3_walking_command_tracked():
    scn = dict(gait="walk", v_cmd=0.3, offset=[0.0, 0.0], push_N=0.0, push_dir=0.0,
               push_t=0.0, push_dur=0.0, T=10.0)
    r = run_episode(scn)
    assert r["solver_failures"] == 0
    assert r["zmp_violation_max_m"] < 1e-6
    assert r["friction_violation_max_N"] < 1e-6
    assert r["force_bound_violation_max_N"] < 1e-6
    assert r["vel_mean_error_last_5s"] < 0.02, f"windowed mean velocity error {r['vel_mean_error_last_5s']:.4f}"


def test_4_determinism():
    a = run_episode(scenario(3))
    b = run_episode(scenario(3))
    assert a["final_state"] == b["final_state"]
    assert a["solver_failures"] == b["solver_failures"]


def test_5_applied_forces_respect_bounds():
    r = run_episode(scenario(11))
    assert r["friction_violation_max_N"] < 1e-6
    assert r["force_bound_violation_max_N"] < 1e-6
