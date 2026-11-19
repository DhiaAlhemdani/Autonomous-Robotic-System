"""L2 safety supervisor: degradation ladder, watchdog, distance-based speed limit.

Platform-agnostic logic. It sits between the L3 command and the actuators and can only
reduce or zero a command. It never increases it, and it is not learning-enabled.

Not specified here, and not claimed:
  - what the humanoid does on a safe stop. docs/02 §6 leaves it open, so SAFE_STOP only
    zeroes the velocity command and latches. Torque removal is not decided here.
  - any measured reaction time. The step counts reported are simulation bookkeeping.

Every numeric value in SupervisorParams is an ASSUMPTION placeholder (see docs/02 §6).
"""
from dataclasses import dataclass, field
from enum import Enum


class State(str, Enum):
    NOMINAL = "nominal"
    DEGRADED_SENSING = "degraded_sensing"
    REDUCED_AUTONOMY = "reduced_autonomy"
    PROTECTIVE_STOP = "protective_stop"
    SAFE_STOP = "safe_stop"


@dataclass(frozen=True)
class SupervisorParams:
    v_max: float = 1.0          # m/s. ASSUMPTION
    d_stop: float = 0.5         # m: inside this distance the command is zeroed. ASSUMPTION
    d_slow: float = 1.5         # m: the speed limit reaches v_max at this distance. ASSUMPTION
    degraded_scale: float = 0.5 # speed scale in DEGRADED_SENSING. ASSUMPTION
    loc_sigma_max: float = 0.10 # m: localisation 1-sigma above this -> REDUCED_AUTONOMY. ASSUMPTION
    cmd_timeout: float = 0.2    # s: no fresh command for longer -> protective stop. ASSUMPTION
    infeasible_max: int = 3     # consecutive infeasible solves -> protective stop. ASSUMPTION
    clear_time: float = 1.0     # s: conditions must hold this long before leaving a stop. ASSUMPTION


@dataclass
class Inputs:
    t: float                    # s, monotonic clock
    cmd: tuple                  # (vx, vy, wz) body-frame command from L3, or None if none
    cmd_time: float             # s: time the command was produced
    nearest_dist: float         # m: distance to nearest person or object
    sensors_ok: bool            # all perception modalities fresh
    loc_sigma: float            # m: localisation 1-sigma
    solve_ok: bool              # L3 solve status for this step
    estop: bool = False         # operator or hardware E-stop asserted
    heartbeat_ok: bool = True   # edge-computer heartbeat present (for the MCU side)


@dataclass
class Output:
    cmd: tuple                  # (vx, vy, wz) actually sent
    state: State
    speed_limit: float          # m/s
    reasons: list = field(default_factory=list)


class Supervisor:
    def __init__(self, params: SupervisorParams = SupervisorParams()):
        self.p = params
        self.state = State.NOMINAL
        self._infeasible_run = 0
        self._clear_since = None   # time since which the stop conditions have been clear
        self._latched_safe = False

    def speed_limit(self, dist):
        p = self.p
        if dist <= p.d_stop:
            return 0.0
        frac = min(1.0, (dist - p.d_stop) / (p.d_slow - p.d_stop))
        return p.v_max * frac

    def step(self, x: Inputs) -> Output:
        p = self.p
        reasons = []

        # Latched safe stop: only operator clearance (clear()) leaves it.
        if x.estop or not x.heartbeat_ok:
            self._latched_safe = True
            reasons.append("estop" if x.estop else "heartbeat_lost")
        if self._latched_safe:
            self.state = State.SAFE_STOP
            return Output((0.0, 0.0, 0.0), self.state, 0.0, reasons)

        self._infeasible_run = 0 if x.solve_ok else self._infeasible_run + 1
        stale = (x.cmd is None) or (x.t - x.cmd_time > p.cmd_timeout)
        in_field = x.nearest_dist <= p.d_stop
        protective = stale or in_field or self._infeasible_run >= p.infeasible_max
        if stale:
            reasons.append("no_fresh_command")
        if in_field:
            reasons.append("safety_field")
        if self._infeasible_run >= p.infeasible_max:
            reasons.append("nmpc_infeasible")

        if protective:
            self._clear_since = None
            self.state = State.PROTECTIVE_STOP
            return Output((0.0, 0.0, 0.0), self.state, 0.0, reasons)

        # Not protective. Leaving a protective stop needs the clear time to elapse.
        if self._was_protective():
            if self._clear_since is None:
                self._clear_since = x.t
            if x.t - self._clear_since < p.clear_time:
                self.state = State.PROTECTIVE_STOP
                return Output((0.0, 0.0, 0.0), self.state, 0.0, ["clearing"])

        scale = 1.0
        if not x.sensors_ok:
            reasons.append("sensor_lost")
            self.state = State.DEGRADED_SENSING
            scale = p.degraded_scale
        if x.loc_sigma > p.loc_sigma_max:
            reasons.append("localisation_uncertain")
            self.state = State.REDUCED_AUTONOMY
            scale = min(scale, p.degraded_scale)
        if self.state not in (State.DEGRADED_SENSING, State.REDUCED_AUTONOMY):
            self.state = State.NOMINAL
        self._clear_since = None
        limit = min(self.speed_limit(x.nearest_dist), p.v_max * scale)
        cmd = _clamp_speed(x.cmd, limit)
        return Output(cmd, self.state, limit, reasons)

    def clear(self, t):
        """Operator clearance: leaves a latched safe stop. Stop conditions are still checked."""
        self._latched_safe = False
        self.state = State.PROTECTIVE_STOP   # the clear time applies before motion resumes
        self._clear_since = t

    def _was_protective(self):
        return self.state == State.PROTECTIVE_STOP


def _clamp_speed(cmd, limit):
    vx, vy, wz = cmd
    n = (vx * vx + vy * vy) ** 0.5
    if n > limit:
        s = limit / n if n > 0 else 0.0
        vx, vy = vx * s, vy * s
    return (vx, vy, wz)
