"""Full-dynamics QP tests (docs/18 §10.2, docs/19 §7). MuJoCo T800 with floor; simulation only.

Standing and push tests use the declared gate timing (G0, G1 in docs/19 §7).
The walking test is a known failure (docs/18 §10.20): the walk is stopped, the gap is recorded, not relaxed.
"""
import numpy as np
import pytest

from planning.legged_nmpc.walk_qp import run


def test_standing_hold_G0():
    r = run(v=0.0, w=0.0, T=3.0)
    assert not r["fell"]
    assert r["nmpc_fail"] == 0
    assert r["qp"].get("failed", 0) == 0
    com = np.array(r["com_final"])
    assert np.linalg.norm(com[:2]) < 0.001, f"CoM xy error {np.linalg.norm(com[:2]):.4f} m, declared 0.001 m"


@pytest.mark.parametrize("force", [(30.0, 0.0), (0.0, 60.0)], ids=["push_x_30N", "push_y_60N"])
def test_push_recovery_G1(force):
    # docs/19 G1: push 0.2 s at t = 3 s, no fall, CoM xy error < 2 mm after 6 s.
    r = run(v=0.0, w=0.0, T=6.0, push=np.array([force[0], force[1], 0.0]), push_t=3.0, push_dur=0.2)
    assert not r["fell"], f"fell at t={r['t_end']} s"
    com = np.array(r["com_final"])
    assert np.linalg.norm(com[:2]) < 0.002, f"CoM xy error {np.linalg.norm(com[:2]):.4f} m, declared 0.002 m"


@pytest.mark.xfail(strict=False, reason="walking not achieved (docs/18 §10.20); declared gate G2 fails")
def test_walk_3s_upright():
    r = run(v=0.2, w=0.0, T=3.0)
    assert not r["fell"], f"fell at t={r['t_end']} s (declared: upright for 3 s at 0.2 m/s)"
