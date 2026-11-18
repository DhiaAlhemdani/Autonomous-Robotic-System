"""Scripted scenario S2 for Stage 0 (docs/00 §2): people and objects in the robot's path.

Ground truth and observations are separate:
- Persons move with a random-walk velocity that the predictor does not know about.
  This creates prediction error, which is the point of the test.
- Persons are NOT influenced by the robot. This is the assumption that produces the
  freezing failure (theory 09 §3.3). It is not how real people behave.
- Observations are the true state plus Gaussian noise (config.agent_sigma0).
- Objects are static.

The robot follows the planar double-integrator model with perfect tracking, which is
the same model the NMPC uses. Model mismatch is therefore absent by construction, and
the results do not show any effect of real tracking errors.
"""
from dataclasses import dataclass, field

import numpy as np

from .config import Stage0Config
from .nmpc import Agent, Stage0NMPC
from .predictor import predict_constant_velocity

# ASSUMPTION: person velocity random-walk intensity (m/s per sqrt(s)); the predictor does not model it
PERSON_ACCEL_NOISE = 0.3
# ASSUMPTION: observed velocity noise (m/s)
VEL_OBS_NOISE = 0.1
GOAL_TOL = 0.2
FREEZE_SPEED = 0.05
FREEZE_NEAR = 2.0


@dataclass
class EpisodeResult:
    seed: int
    goal_reached: bool
    steps: int
    min_dist_person: float
    min_dist_object: float
    violation_steps: int
    freezing_steps: int
    solve_times: list = field(default_factory=list)
    statuses: list = field(default_factory=list)
    reasons: list = field(default_factory=list)
    messages: list = field(default_factory=list)
    min_constraints: list = field(default_factory=list)


def _place_scene(rng):
    goal = np.array([8.0, rng.uniform(-1.0, 1.0)])
    persons = []
    for _ in range(3):
        p = np.array([rng.uniform(2.0, 6.0), rng.uniform(-3.0, 3.0)])
        v = np.array([rng.uniform(-0.3, 0.3), rng.choice([-1.0, 1.0]) * rng.uniform(0.6, 1.2)])
        persons.append((p, v))
    objects = []
    for _ in range(2):
        objects.append(np.array([rng.uniform(2.0, 6.0), rng.uniform(-2.0, 2.0)]))
    return goal, persons, objects


def run_episode(seed: int, cfg: Stage0Config, max_time: float = 30.0, hook=None) -> EpisodeResult:
    rng = np.random.default_rng(seed)
    goal, persons, objects = _place_scene(rng)
    dt = cfg.dt

    x = np.zeros(4)  # robot: px, py, vx, vy
    p_pos = [p.copy() for p, _ in persons]
    p_vel = [v.copy() for _, v in persons]

    ctrl = Stage0NMPC(cfg)
    res = EpisodeResult(seed=seed, goal_reached=False, steps=0,
                        min_dist_person=np.inf, min_dist_object=np.inf,
                        violation_steps=0, freezing_steps=0)

    max_steps = int(round(max_time / dt))
    for step in range(max_steps):
        # Observe agents (noisy)
        agents = []
        for i in range(len(persons)):
            obs_p = p_pos[i] + rng.normal(0.0, cfg.agent_sigma0, 2)
            obs_v = p_vel[i] + rng.normal(0.0, VEL_OBS_NOISE, 2)
            pred, sig = predict_constant_velocity(obs_p, obs_v, cfg.horizon, dt,
                                                  cfg.agent_sigma0, cfg.agent_sigma_rate)
            agents.append(Agent(pred=pred, sigma=sig, d_min=cfg.d_min_person))
        for o in objects:
            obs_p = o + rng.normal(0.0, cfg.agent_sigma0, 2)
            pred, sig = predict_constant_velocity(obs_p, np.zeros(2), cfg.horizon, dt,
                                                  cfg.agent_sigma0, 0.0)
            agents.append(Agent(pred=pred, sigma=sig, d_min=cfg.d_min_object))

        r = ctrl.solve(x, goal, agents)
        if hook is not None and r.status == "infeasible":
            hook(x.copy(), goal, agents, r)
        res.solve_times.append(r.solve_time_s)
        res.statuses.append(r.status)
        res.reasons.append(r.reason)
        res.messages.append(r.message)
        res.min_constraints.append(r.min_constraint)

        # Robot step (ideal tracking of the planned acceleration)
        a = r.accel
        x[:2] = x[:2] + x[2:] * dt + 0.5 * a * dt * dt
        x[2:] = x[2:] + a * dt

        # Agents step (ground truth)
        for i in range(len(persons)):
            p_vel[i] = p_vel[i] + rng.normal(0.0, PERSON_ACCEL_NOISE * dt, 2)
            p_pos[i] = p_pos[i] + p_vel[i] * dt

        res.steps = step + 1
        d_person = min(np.linalg.norm(x[:2] - p) for p in p_pos)
        d_object = min(np.linalg.norm(x[:2] - o) for o in objects)
        res.min_dist_person = min(res.min_dist_person, d_person)
        res.min_dist_object = min(res.min_dist_object, d_object)
        if any(np.linalg.norm(x[:2] - p) < cfg.d_min_person for p in p_pos) or \
           any(np.linalg.norm(x[:2] - o) < cfg.d_min_object for o in objects):
            res.violation_steps += 1

        dist_goal = np.linalg.norm(x[:2] - goal)
        speed = np.linalg.norm(x[2:])
        if speed < FREEZE_SPEED and dist_goal > 0.5 and d_person < FREEZE_NEAR:
            res.freezing_steps += 1

        if dist_goal < GOAL_TOL:
            res.goal_reached = True
            break

    return res


def summarize(results):
    n = len(results)
    times = np.concatenate([np.array(r.solve_times) for r in results])
    statuses = [s for r in results for s in r.statuses]
    min_person = np.array([r.min_dist_person for r in results])
    reasons = [s for r in results for s in r.reasons]
    messages = [m for r in results for s, m in zip(r.reasons, r.messages) if s != "ok"]
    infeasible_min_c = [c for r in results for s, c in zip(r.reasons, r.min_constraints) if s == "constraint_violation"]
    from collections import Counter
    return {
        "infeasible_reasons": dict(Counter(r for r in reasons if r != "ok")),
        "infeasible_messages": dict(Counter(messages).most_common(10)),
        "constraint_violation_min_c_p50": float(np.percentile(infeasible_min_c, 50)) if infeasible_min_c else None,
        "episodes": n,
        "goal_reached_rate": sum(r.goal_reached for r in results) / n,
        "episodes_with_violation": sum(r.violation_steps > 0 for r in results) / n,
        "episodes_with_freezing": sum(r.freezing_steps > 0 for r in results) / n,
        "infeasible_step_fraction": statuses.count("infeasible") / max(len(statuses), 1),
        "min_dist_person_m": {
            "p5": float(np.percentile(min_person, 5)),
            "p50": float(np.percentile(min_person, 50)),
        },
        "solve_time_ms": {
            "p50": float(np.percentile(times, 50) * 1e3),
            "p99": float(np.percentile(times, 99) * 1e3),
            "max": float(times.max() * 1e3),
            "n_solves": int(times.size),
        },
    }
