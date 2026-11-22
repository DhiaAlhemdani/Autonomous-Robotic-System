"""Tests for the from-scratch NMPC (docs/16 §5). Run from planning/: pytest tests -q"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from nmpc.config import NMPCConfig  # noqa: E402
from nmpc.model import exact_constant_input, rk4_step  # noqa: E402
from nmpc.sim import run  # noqa: E402

CFG = NMPCConfig()


def test_rk4_matches_analytic_unicycle():
    x = np.array([0.3, -0.2, 0.4])
    for u in [np.array([1.0, 0.5]), np.array([0.6, -1.2]), np.array([0.8, 0.0])]:
        assert np.abs(rk4_step(x, u, CFG.dt) - exact_constant_input(x, u, CFG.dt)).max() < 1e-6


def test_free_space_reaches_goal():
    res = run(dict(start=np.zeros(3), goal=np.array([3.0, 0.0]), obstacles=[]), CFG, T=15.0)
    assert res["reached"] and res["final_err"] < 0.05
    assert res["fails"] == 0


def test_obstacle_on_path_keeps_clearance_without_slack():
    scene = dict(start=np.zeros(3), goal=np.array([4.0, 0.0]), obstacles=[(2.0, 0.0, 0.4)])
    res = run(scene, CFG, T=20.0)
    assert res["min_center_dist"] >= 0.4 + CFG.MARGIN - 0.02
    assert res["max_slack"] < 1e-3
    assert res["fails"] == 0


def test_applied_inputs_respect_bounds():
    scene = dict(start=np.zeros(3), goal=np.array([4.0, 1.0]), obstacles=[(2.0, 0.4, 0.3)])
    res = run(scene, CFG, T=10.0)
    assert np.all(np.abs(res["u"][:, 0]) <= CFG.V_MAX + 1e-9)
    assert np.all(np.abs(res["u"][:, 1]) <= CFG.W_MAX + 1e-9)


def test_same_scene_gives_same_trajectory():
    scene = dict(start=np.zeros(3), goal=np.array([3.0, -0.5]), obstacles=[(1.5, -0.2, 0.3)])
    a = run(scene, CFG, T=8.0)
    b = run(scene, CFG, T=8.0)
    assert np.array_equal(a["traj"], b["traj"])
