"""Tests for the humanoid NMPC (docs/15). Run from planning/: pytest tests -q

These test the controller on the base model in model.py. They do not test prediction
accuracy against people, the walking policy, or any robot. See docs/15 §8.
"""
import numpy as np
import pytest

from humanoid_nmpc.config import HumanoidNMPCConfig
from humanoid_nmpc.controller import HumanoidNMPC, Obstacle
from humanoid_nmpc.model import jacobians, rollout, step
from humanoid_nmpc.sim import run

PATH = np.array([[0.0, 0.0], [6.0, 0.0]])
X0 = np.zeros(6)


def _lims(cfg):
    lo = np.array([cfg.vx_min, -cfg.vy_max, -cfg.wz_max])
    hi = np.array([cfg.vx_max, cfg.vy_max, cfg.wz_max])
    return lo, hi


# ---------------------------------------------------------------- model
def test_lag_converges_to_command_and_zero_input_holds_position():
    cfg = HumanoidNMPCConfig()
    x = np.zeros(6)
    for _ in range(int(10 * cfg.tau_vx / cfg.dt)):
        x = step(x, np.array([0.8, 0.0, 0.0]), cfg)
    assert x[3] == pytest.approx(0.8, rel=1e-2)
    x = np.zeros(6)
    x[3] = 0.5
    for _ in range(20):
        x = step(x, np.zeros(3), cfg)
    assert x[0] != 0.0  # it moved while decelerating
    still = step(np.zeros(6), np.zeros(3), cfg)
    assert np.allclose(still, 0.0)


def test_jacobians_match_finite_differences():
    cfg = HumanoidNMPCConfig()
    rng = np.random.default_rng(0)
    for _ in range(5):
        x = np.concatenate([rng.normal(size=3), rng.uniform(-0.5, 0.5, 3)])
        u = rng.uniform(-0.3, 0.3, 3)
        A, B = jacobians(x, cfg)
        eps = 1e-6
        Anum = np.zeros((6, 6))
        for j in range(6):
            dx = np.zeros(6)
            dx[j] = eps
            Anum[:, j] = (step(x + dx, u, cfg) - step(x - dx, u, cfg)) / (2 * eps)
        Bnum = np.zeros((6, 3))
        for j in range(3):
            du = np.zeros(3)
            du[j] = eps
            Bnum[:, j] = (step(x, u + du, cfg) - step(x, u - du, cfg)) / (2 * eps)
        assert np.allclose(A, Anum, atol=1e-6)
        assert np.allclose(B, Bnum, atol=1e-6)


def test_rollout_matches_repeated_step():
    cfg = HumanoidNMPCConfig()
    U = np.full((5, 3), 0.2)
    X = rollout(X0, U, cfg)
    x = X0.copy()
    for k in range(5):
        x = step(x, U[k], cfg)
        assert np.allclose(X[k + 1], x)


# ---------------------------------------------------------------- controller
def test_free_space_reaches_goal_without_fallback():
    ep = run(HumanoidNMPC(), X0, PATH, [], T=150)
    assert ep.reached_goal
    assert set(ep.levels) == {"nominal"}


def test_crossing_person_keeps_true_distance_above_dmin():
    cfg = HumanoidNMPCConfig()
    obs = [Obstacle(pos0=(3.0, -2.0), vel=(0.0, 0.5), kind="person")]
    ep = run(HumanoidNMPC(cfg), X0, PATH, obs, T=150)
    assert ep.min_true_dist >= cfg.d_min_person
    assert ep.reached_goal or ep.goal_dist < 1.0  # it must not freeze far from the goal


def test_applied_commands_respect_limits_and_rate_limit_in_closed_loop():
    cfg = HumanoidNMPCConfig()
    lo, hi = _lims(cfg)
    obs = [Obstacle(pos0=(3.0, -2.0), vel=(0.0, 0.5), kind="person")]
    ep = run(HumanoidNMPC(cfg), X0, PATH, obs, T=150)
    assert np.all(ep.cmds >= lo - 1e-6) and np.all(ep.cmds <= hi + 1e-6)
    prev = np.zeros(3)
    for c in ep.cmds:
        assert np.all(np.abs(c - prev) <= cfg.du_max + 1e-6)
        prev = c


def test_infeasible_scene_falls_back_without_error_and_stays_bounded():
    cfg = HumanoidNMPCConfig()
    lo, hi = _lims(cfg)
    # A person standing 0.6 m ahead of the robot: inside d_min, so no safe motion exists.
    obs = [Obstacle(pos0=(0.6, 0.0), vel=(0.0, 0.0), kind="person")]
    ctrl = HumanoidNMPC(cfg)
    ep = run(ctrl, X0, PATH, obs, T=30)
    assert len(ep.levels) == 30
    assert any(lv != "nominal" for lv in ep.levels)
    assert np.all(np.isfinite(ep.cmds))
    assert np.all(ep.cmds >= lo - 1e-6) and np.all(ep.cmds <= hi + 1e-6)
    # protective_stop is never reported as valid
    for lv, v in zip(ep.levels, ep.valid):
        if lv == "protective_stop":
            assert v is False


def test_stop_ramp_reaches_zero_when_nothing_is_near():
    cfg = HumanoidNMPCConfig()
    ctrl = HumanoidNMPC(cfg)
    ctrl._u_prev = np.array([0.8, 0.2, -0.5])
    cmds = []
    for _ in range(60):
        cmd, _margin, _ok = ctrl._stop(np.zeros(6), [], [])
        ctrl._u_prev = cmd
        cmds.append(cmd.copy())
    assert np.allclose(cmds[-1], 0.0)
    prev = np.array([0.8, 0.2, -0.5])
    for c in cmds:
        assert np.all(np.abs(c - prev) <= cfg.du_max + 1e-9)
        prev = c


def test_linearised_separation_is_an_inner_approximation():
    """If n.(p-o) >= R then |p-o| >= R for any p: the half-plane cannot admit a forbidden point."""
    rng = np.random.default_rng(1)
    for _ in range(2000):
        n = rng.normal(size=2)
        n /= np.linalg.norm(n)
        o = rng.normal(size=2)
        R = rng.uniform(0.1, 2.0)
        p = o + rng.normal(size=2) * 3
        if n @ (p - o) >= R:
            assert np.linalg.norm(p - o) >= R - 1e-12


def test_deterministic_across_runs():
    obs = [Obstacle(pos0=(3.0, -2.0), vel=(0.0, 0.5), kind="person")]
    a = run(HumanoidNMPC(), X0, PATH, obs, T=60)
    b = run(HumanoidNMPC(), X0, PATH, obs, T=60)
    assert np.array_equal(a.cmds, b.cmds)
    assert a.levels == b.levels


def test_unknown_solver_name_is_rejected():
    ctrl = HumanoidNMPC(HumanoidNMPCConfig(solver="nope"))
    with pytest.raises(ValueError):
        ctrl.step(X0, PATH, [])


def test_path_must_have_two_points():
    with pytest.raises(ValueError):
        HumanoidNMPC().step(X0, np.array([[0.0, 0.0]]), [])
