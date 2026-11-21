import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from safety.supervisor import Inputs, State, Supervisor, SupervisorParams  # noqa: E402

P = SupervisorParams()
GOOD_CMD = (0.8, 0.0, 0.1)


def inp(t=0.0, cmd=GOOD_CMD, cmd_time=None, dist=5.0, sensors=True, sigma=0.02,
        estop=False, hb=True):
    return Inputs(t=t, cmd=cmd, cmd_time=t if cmd_time is None else cmd_time, nearest_dist=dist,
                  sensors_ok=sensors, loc_sigma=sigma, estop=estop, heartbeat_ok=hb)


def test_nominal_passes_command_through():
    out = Supervisor().step(inp())
    assert out.state == State.NOMINAL and out.cmd == GOOD_CMD and out.speed_limit == P.v_max


def test_no_command_without_fresh_setpoint():
    s = Supervisor()
    out = s.step(inp(t=1.0, cmd_time=1.0 - (P.cmd_timeout + 0.01)))
    assert out.state == State.PROTECTIVE_STOP and out.cmd == (0.0, 0.0, 0.0)
    assert "no_fresh_command" in out.reasons
    out = s.step(inp(t=2.0, cmd=None))
    assert out.cmd == (0.0, 0.0, 0.0)


def test_safety_field_zeroes_command():
    out = Supervisor().step(inp(dist=P.d_stop))
    assert out.state == State.PROTECTIVE_STOP and out.cmd == (0.0, 0.0, 0.0)
    assert "safety_field" in out.reasons


def test_speed_limit_monotone_in_distance():
    s = Supervisor()
    lims = [s.speed_limit(d) for d in [P.d_stop, 0.8, 1.0, 1.2, P.d_slow, 3.0]]
    assert lims == sorted(lims)
    assert lims[0] == 0.0 and lims[-1] == P.v_max


def test_command_never_exceeds_limit_or_grows():
    out = Supervisor().step(inp(dist=1.0, cmd=(1.0, 0.0, 0.2)))
    vx, vy, wz = out.cmd
    assert (vx * vx + vy * vy) ** 0.5 <= out.speed_limit + 1e-12
    assert abs(vx) <= 1.0 and wz == 0.2   # yaw rate is not scaled by the distance limit here


def test_protective_stop_needs_clear_time_before_motion():
    s = Supervisor()
    s.step(inp(t=0.0, dist=P.d_stop))
    out = s.step(inp(t=0.1))
    assert out.state == State.PROTECTIVE_STOP and out.cmd == (0.0, 0.0, 0.0)
    out = s.step(inp(t=0.1 + P.clear_time + 0.01))
    assert out.state == State.NOMINAL and out.cmd != (0.0, 0.0, 0.0)


def test_sensor_loss_is_degraded_with_reduced_speed():
    out = Supervisor().step(inp(sensors=False))
    assert out.state == State.DEGRADED_SENSING
    assert out.speed_limit == pytest.approx(P.v_max * P.degraded_scale)
    assert "sensor_lost" in out.reasons


def test_localisation_uncertainty_is_reduced_autonomy():
    out = Supervisor().step(inp(sigma=P.loc_sigma_max + 0.01))
    assert out.state == State.REDUCED_AUTONOMY and "localisation_uncertain" in out.reasons


def test_estop_latches_until_clearance():
    s = Supervisor()
    out = s.step(inp(t=0.0, estop=True))
    assert out.state == State.SAFE_STOP and out.cmd == (0.0, 0.0, 0.0)
    # E-stop released, but the latch holds.
    assert s.step(inp(t=0.1)).state == State.SAFE_STOP
    s.clear(0.2)
    assert s.step(inp(t=0.25)).state == State.PROTECTIVE_STOP   # clear time applies
    out = s.step(inp(t=0.2 + P.clear_time + 0.05))
    assert out.state == State.NOMINAL


def test_heartbeat_loss_is_safe_stop():
    out = Supervisor().step(inp(hb=False))
    assert out.state == State.SAFE_STOP and out.cmd == (0.0, 0.0, 0.0)
    assert "heartbeat_lost" in out.reasons
