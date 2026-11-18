import numpy as np
import pytest

from stage0.config import Stage0Config
from stage0.nmpc import Agent, Stage0NMPC
from stage0.predictor import predict_constant_velocity
from stage0.scenario import run_episode


def test_constant_velocity_prediction_is_linear_and_sigma_grows():
    pred, sig = predict_constant_velocity([1.0, 2.0], [0.5, -1.0], horizon=4, dt=0.1,
                                          sigma0=0.1, sigma_rate=0.2)
    assert np.allclose(pred[0], [1.05, 1.9])
    assert np.allclose(pred[3], [1.0 + 0.5 * 0.4, 2.0 - 1.0 * 0.4])
    assert np.all(np.diff(sig) > 0)


def test_margin_z_is_gaussian_quantile():
    cfg = Stage0Config(risk=0.05)
    assert cfg.z_delta == pytest.approx(1.6449, abs=1e-3)


def test_free_space_reaches_goal_without_infeasible_steps():
    cfg = Stage0Config()
    ctrl = Stage0NMPC(cfg)
    x = np.zeros(4)
    goal = np.array([4.0, 0.0])
    statuses = []
    for _ in range(60):
        r = ctrl.solve(x, goal, [])
        statuses.append(r.status)
        x[:2] += x[2:] * cfg.dt + 0.5 * r.accel * cfg.dt ** 2
        x[2:] += r.accel * cfg.dt
    assert set(statuses) == {"ok"}
    assert np.linalg.norm(x[:2] - goal) < 0.5


def test_solution_respects_separation_margin_for_agent_ahead():
    cfg = Stage0Config()
    ctrl = Stage0NMPC(cfg)
    x = np.array([0.0, 0.0, 0.8, 0.0])
    goal = np.array([6.0, 0.0])
    pred, sig = predict_constant_velocity([2.0, 0.0], [0.0, 0.0], cfg.horizon, cfg.dt,
                                          cfg.agent_sigma0, 0.0)
    r = ctrl.solve(x, goal, [Agent(pred=pred, sigma=sig, d_min=cfg.d_min_person)])
    assert r.status == "ok"
    assert r.min_constraint >= -1e-4


def test_infeasible_problem_falls_back_to_brake():
    cfg = Stage0Config()
    ctrl = Stage0NMPC(cfg)
    x = np.array([0.0, 0.0, 0.5, 0.0])
    # Agent sitting on the robot: no feasible trajectory exists
    pred = np.tile([0.0, 0.0], (cfg.horizon, 1))
    sig = np.full(cfg.horizon, 0.1)
    r = ctrl.solve(x, np.array([5.0, 0.0]), [Agent(pred=pred, sigma=sig, d_min=5.0)])
    assert r.status == "infeasible"
    assert np.linalg.norm(r.accel) <= cfg.a_max + 1e-9
    assert r.accel[0] < 0  # brakes against the velocity


def test_episode_is_reproducible_from_seed():
    cfg = Stage0Config()
    a = run_episode(3, cfg, max_time=5.0)
    b = run_episode(3, cfg, max_time=5.0)
    assert a.min_dist_person == b.min_dist_person
    assert a.statuses == b.statuses


def test_slack_check_feasible_case_has_no_shortfall():
    from stage0.slack import slack_check
    cfg = Stage0Config()
    pred, sig = predict_constant_velocity([3.0, 3.0], [0.0, 0.0], cfg.horizon, cfg.dt,
                                          cfg.agent_sigma0, 0.0)
    r = slack_check(cfg, np.zeros(4), np.array([6.0, 0.0]), [Agent(pred, sig, cfg.d_min_person)])
    assert r["feasible_within_tol"]


def test_slack_check_agent_on_robot_is_short():
    from stage0.slack import slack_check
    cfg = Stage0Config()
    pred = np.tile([0.0, 0.0], (cfg.horizon, 1))
    sig = np.full(cfg.horizon, 0.1)
    r = slack_check(cfg, np.zeros(4), np.array([5.0, 0.0]), [Agent(pred, sig, 5.0)])
    assert not r["feasible_within_tol"]
    assert r["shortfall_m"] > 1.0


def _fd(f, U, h=1e-6):
    g = np.zeros_like(U)
    for i in range(U.size):
        e = np.zeros_like(U); e[i] = h
        g.flat[i] = (f(U + e) - f(U - e)) / (2 * h)
    return g


def test_analytic_gradients_match_finite_differences():
    cfg = Stage0Config()
    ctrl = Stage0NMPC(cfg, analytic=True)
    ctrl._prev_accel = np.array([0.2, -0.1])
    x0 = np.array([0.0, 0.0, 0.5, 0.2])
    goal = np.array([4.0, 1.0])
    pred, sig = predict_constant_velocity([2.0, 0.5], [0.0, -0.3], cfg.horizon, cfg.dt,
                                          cfg.agent_sigma0, 0.1)
    agents = [Agent(pred, sig, cfg.d_min_person)]
    radii = [a.d_min + cfg.z_delta * np.sqrt(a.sigma ** 2 + cfg.sigma_robot ** 2) for a in agents]
    rng = np.random.default_rng(0)
    U = rng.uniform(-0.5, 0.5, size=2 * cfg.horizon)

    g_fd = _fd(lambda u: ctrl._cost(u, x0, goal), U)
    assert np.allclose(ctrl._cost_grad(U, x0, goal), g_fd, atol=1e-5)

    J_fd = np.zeros((len(radii) * 0 + cfg.horizon + cfg.horizon, U.size))
    cons = lambda u: ctrl._constraints(u, x0, agents, radii)
    J_an = ctrl._cons_jac(U, x0, agents, radii)
    for i in range(U.size):
        e = np.zeros_like(U); e[i] = 1e-6
        J_fd[:, i] = (cons(U + e) - cons(U - e)) / 2e-6
    assert np.allclose(J_an, J_fd, atol=1e-4)


def test_analytic_gradients_match_with_lateral_term():
    from dataclasses import replace
    cfg = replace(Stage0Config(), w_lat=2.0)
    ctrl = Stage0NMPC(cfg, analytic=True)
    ctrl._prev_accel = np.array([0.1, 0.3])
    x0 = np.array([0.0, 0.0, 0.6, -0.2])
    goal = np.array([5.0, 0.5])
    ctrl._Pperp = np.eye(2) - np.outer(x0[2:] / np.linalg.norm(x0[2:]), x0[2:] / np.linalg.norm(x0[2:]))
    rng = np.random.default_rng(1)
    U = rng.uniform(-0.5, 0.5, size=2 * cfg.horizon)
    g_fd = _fd(lambda u: ctrl._cost(u, x0, goal), U)
    assert np.allclose(ctrl._cost_grad(U, x0, goal), g_fd, atol=1e-5)
