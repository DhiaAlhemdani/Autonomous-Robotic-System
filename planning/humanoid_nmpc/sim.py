"""Closed-loop simulation of the humanoid NMPC on the base model (docs/15 §8).

The plant is the model in model.py with no mismatch. Agents move at constant velocity,
which is exactly the controller's prediction. Therefore these runs test the controller
logic, not prediction accuracy, and not the walking policy. Those are N1, N3 and N5 in
docs/15 §8, and they need the MuJoCo simulation.

All distances in the metrics are ground truth from the plant, never from the controller.
"""
from dataclasses import dataclass, field
from typing import List

import numpy as np

from .controller import HumanoidNMPC, Obstacle
from .model import NU, step


@dataclass
class Episode:
    states: np.ndarray            # (T+1, 6)
    cmds: np.ndarray              # (T, 3) applied commands
    levels: List[str]
    valid: List[bool]
    solve_times: List[float]
    iterations: List[int]
    reasons: List[str]
    min_true_dist: float          # min over steps and agents of plant distance, m
    min_true_margin: float        # min over steps and agents of (plant distance - d_min)
    reached_goal: bool
    goal_dist: float


def agent_pos(o: Obstacle, t: float) -> np.ndarray:
    return np.asarray(o.pos0, float) + np.asarray(o.vel, float) * t


def run(ctrl: HumanoidNMPC, x0, path, obstacles: List[Obstacle], T: int,
        goal_tol: float = 0.2) -> Episode:
    cfg = ctrl.cfg
    x = np.asarray(x0, float).copy()
    path = np.asarray(path, float)
    goal = path[-1]
    states = [x.copy()]
    cmds, levels, valid, times, iters, reasons = [], [], [], [], [], []
    min_dist = np.inf
    min_margin = np.inf
    reached = False
    for t in range(T):
        # The controller sees agents at their current positions, as a tracker would.
        t_now = t * cfg.dt
        seen = [Obstacle(pos0=tuple(agent_pos(o, t_now)), vel=o.vel, kind=o.kind)
                for o in obstacles]
        out = ctrl.step(x, path, seen)
        cmds.append(out.cmd.copy())
        levels.append(out.level)
        valid.append(out.valid)
        times.append(out.solve_time_s)
        iters.append(out.iterations)
        reasons.append(out.reason)
        x = step(x, out.cmd, cfg)
        states.append(x.copy())
        tn = (t + 1) * cfg.dt
        for o in obstacles:
            d = np.hypot(*(x[0:2] - agent_pos(o, tn)))
            dmin = cfg.d_min_person if o.kind == "person" else cfg.d_min_object
            min_dist = min(min_dist, d)
            min_margin = min(min_margin, d - dmin)
        if np.hypot(*(x[0:2] - goal)) <= goal_tol:
            reached = True
            break
    gd = float(np.hypot(*(x[0:2] - goal)))
    return Episode(states=np.array(states), cmds=np.array(cmds).reshape(-1, NU),
                   levels=levels, valid=valid, solve_times=times, iterations=iters,
                   reasons=reasons, min_true_dist=float(min_dist),
                   min_true_margin=float(min_margin), reached_goal=reached, goal_dist=gd)


def summarise(ep: Episode) -> dict:
    counts = {}
    for lv in ep.levels:
        counts[lv] = counts.get(lv, 0) + 1
    times = np.array(ep.solve_times) if ep.solve_times else np.zeros(1)
    return {
        "steps": len(ep.levels),
        "levels": counts,
        "fallback_fraction": float(sum(1 for lv in ep.levels if lv != "nominal") / max(1, len(ep.levels))),
        "reached_goal": ep.reached_goal,
        "goal_distance_m": ep.goal_dist,
        "min_true_distance_m": ep.min_true_dist,
        "min_true_margin_to_dmin_m": ep.min_true_margin,
        "solve_time_mean_s": float(times.mean()),
        "solve_time_p95_s": float(np.percentile(times, 95)),
        "solve_time_max_s": float(times.max()),
        "sandbox_note": "sandbox CPU timing, not target-hardware timing",
    }
