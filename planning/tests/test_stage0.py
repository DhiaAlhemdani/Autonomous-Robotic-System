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
