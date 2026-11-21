"""Closed-loop harness and seeded sweep for the NMPC (docs/16 §5–6). The plant is the same model (no mismatch)."""
import json
import sys
import os

import numpy as np

from .config import NMPCConfig
from .controller import NMPC
from .model import rk4_step

GOAL_TOL = 0.05


def random_scene(seed):
    rng = np.random.default_rng(seed)
    goal = np.array([rng.uniform(3.0, 5.0), rng.uniform(-1.0, 1.0)])
    obstacles = []
    if rng.random() < 0.7:
        t = rng.uniform(0.3, 0.7)
        off = rng.uniform(-0.3, 0.3)
        perp = np.array([-goal[1], goal[0]]) / np.hypot(*goal)
        c = t * goal + off * perp
        obstacles.append((c[0], c[1], rng.uniform(0.3, 0.5)))
    return dict(start=np.zeros(3), goal=goal, obstacles=obstacles)


def run(scene, cfg=NMPCConfig(), T=20.0, ctrl=None):
    ctrl = ctrl or NMPC(cfg)
    ctrl.reset()
    x = np.array(scene["start"], dtype=float)
    goal = np.asarray(scene["goal"], dtype=float)
    obs = [tuple(o) for o in scene["obstacles"]]
    u_prev = np.zeros(2)
    traj, us, times, slacks, fails = [x.copy()], [], [], [], 0
    min_clear = np.inf
    steps = int(round(T / cfg.dt))
    for _ in range(steps):
        u, info = ctrl.solve(x, goal, obs, u_prev)
        times.append(info["solve_ms"])
        slacks.append(info["slack_max"])
        fails += 0 if info["success"] else 1
        us.append(u.copy())
        x = rk4_step(x, u, cfg.dt)
        traj.append(x.copy())
        u_prev = u
        for cx, cy, r in obs:
            min_clear = min(min_clear, float(np.hypot(x[0] - cx, x[1] - cy)))
        if np.hypot(*(x[:2] - goal)) < GOAL_TOL and np.hypot(*u) < 0.05:
            break
    err = float(np.hypot(*(x[:2] - goal)))
    return dict(traj=np.array(traj), u=np.array(us), reached=err < GOAL_TOL, final_err=err,
                min_center_dist=min_clear, max_slack=float(max(slacks)) if slacks else 0.0,
                fails=fails, solve_ms=np.array(times))


def sweep(seeds, cfg=NMPCConfig()):
    ctrl = NMPC(cfg)
    rows = []
    for s in seeds:
        sc = random_scene(s)
        res = run(sc, cfg, ctrl=ctrl)
        r = dict(seed=s, reached=bool(res["reached"]), final_err=round(res["final_err"], 4),
                 min_center_dist=None if res["min_center_dist"] == np.inf else round(res["min_center_dist"], 4),
                 max_slack=round(res["max_slack"], 6), fails=res["fails"],
                 solve_ms_p50=round(float(np.median(res["solve_ms"])), 1),
                 solve_ms_p99=round(float(np.percentile(res["solve_ms"], 99)), 1))
        rows.append(r)
    return rows


def summary(rows):
    n = len(rows)
    return dict(n=n, reached=sum(r["reached"] for r in rows), fails=sum(r["fails"] for r in rows),
                max_slack=max(r["max_slack"] for r in rows),
                solve_ms_p50=float(np.median([r["solve_ms_p50"] for r in rows])),
                solve_ms_p99=float(np.median([r["solve_ms_p99"] for r in rows])))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs=2, default=[0, 10], help="start, count")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rows = sweep(range(a.seeds[0], a.seeds[0] + a.seeds[1]))
    out = dict(rows=rows, summary=summary(rows), note="simulation only; solve times are sandbox measurements")
    print(json.dumps(out["summary"], indent=1))
    if a.out:
        os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
        json.dump(out, open(a.out, "w"), indent=1)
