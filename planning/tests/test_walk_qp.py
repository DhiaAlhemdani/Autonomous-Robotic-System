"""Full-dynamics QP tests (docs/18 §10.2). MuJoCo T800 with floor; simulation only.

test_standing_hold and test_standing_push are declared for the standing QP. test_walk_3s_upright is declared
for walking and is expected to fail (reported, not relaxed).
"""
import numpy as np
import pytest

from planning.legged_nmpc.walk_qp import run


def test_standing_hold():
    r = run(v=0.0, w=0.0, T=3.0)
    assert not r["fell"]
    assert r["nmpc_fail"] == 0
    assert r["qp"].get("failed", 0) == 0
    com = np.array(r["com_final"])
    assert np.linalg.norm(com[:2]) < 0.001, f"CoM xy error {np.linalg.norm(com[:2]):.4f} m, declared 0.001 m"


def test_standing_push():
    r = run(v=0.0, w=0.0, T=4.0, push=np.array([30.0, 0.0, 0.0]), push_t=1.0, push_dur=0.2)
    assert not r["fell"]
    com = np.array(r["com_final"])
    assert np.linalg.norm(com[:2]) < 0.002, f"CoM xy after push {np.linalg.norm(com[:2]):.4f} m, declared 0.002 m"


def test_walk_3s_upright():
    r = run(v=0.2, w=0.0, T=3.0)
    assert not r["fell"], f"fell at t={r['t_end']} s (declared: upright for 3 s at 0.2 m/s)"
