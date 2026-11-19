"""Seeded sweep of the humanoid NMPC on the base model (docs/15 §8).

Usage (from planning/):
    ../.venv-nmpc/bin/python -m humanoid_nmpc.run_demo --seeds 50 --T 200

Each seed fixes: start offset, goal, and the number, start and velocity of people
crossing the path, plus one static object near the path. Agents move at constant
velocity, which is also the controller's prediction. This tests the controller
logic on the base model only. It is not a walking, prediction or hardware result.

Output: a JSON file in runs/ (gitignored) and a short printed summary. Solve times are
sandbox CPU measurements, not target-hardware numbers.
"""
import argparse
import json
import os
from collections import Counter

import numpy as np

from .config import HumanoidNMPCConfig
from .controller import HumanoidNMPC, Obstacle
from .sim import run, summarise


def scenario(seed: int):
    rng = np.random.default_rng(seed)
    y0 = rng.uniform(-0.3, 0.3)
    path = np.array([[0.0, y0], [6.0, 0.0]])
    obs = []
    n_people = int(rng.integers(1, 3))
    for _ in range(n_people):
        x = rng.uniform(2.0, 4.0)
        y = rng.uniform(-2.5, -1.5)
        vy = rng.uniform(0.3, 0.6) * (1 if y < 0 else -1)
        obs.append(Obstacle(pos0=(x, y), vel=(0.0, vy), kind="person"))
    obs.append(Obstacle(pos0=(rng.uniform(4.5, 5.5), rng.uniform(0.8, 1.2)), vel=(0.0, 0.0), kind="object"))
    return path, obs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", type=int, default=0, help="first seed (use 30 for a held-out set)")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--T", type=int, default=200)
    ap.add_argument("--solver", default="quadprog", choices=["quadprog", "osqp"])
    ap.add_argument("--out", default="runs/humanoid_nmpc_demo.json")
    args = ap.parse_args()

    cfg = HumanoidNMPCConfig(solver=args.solver)
    per_seed = []
    levels = Counter()
    times = []
    reached = 0
    min_margin = np.inf
    seeds = range(args.start, args.start + args.seeds)
    for seed in seeds:
        path, obs = scenario(seed)
        ep = run(HumanoidNMPC(cfg), np.zeros(6), path, obs, T=args.T)
        s = summarise(ep)
        s["seed"] = seed
        per_seed.append(s)
        levels.update(ep.levels)
        times.extend(ep.solve_times)
        reached += int(ep.reached_goal)
        min_margin = min(min_margin, ep.min_true_margin)

    total = sum(levels.values())
    t = np.array(times) if times else np.zeros(1)
    fallback = sum(v for k, v in levels.items() if k != "nominal")
    summary = {
        "solver": args.solver,
        "seed_start": args.start,
        "seeds": args.seeds,
        "steps_total": total,
        "levels": dict(levels),
        "fallback_fraction": fallback / max(1, total),
        "reached_goal_seeds": reached,
        "min_true_margin_to_dmin_m": float(min_margin),
        "solve_time_mean_s": float(t.mean()),
        "solve_time_p95_s": float(np.percentile(t, 95)),
        "solve_time_max_s": float(t.max()),
        "config": cfg.__dict__ if hasattr(cfg, "__dict__") else {},
        "sandbox_note": "sandbox CPU timing, not target-hardware timing; base model only",
    }
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump({"summary": summary, "per_seed": per_seed}, f, indent=1, default=float)

    print(f"solver={args.solver} seeds={args.start}..{args.start + args.seeds - 1} steps={total}")
    print(f"levels={dict(levels)}  fallback_fraction={summary['fallback_fraction']:.3f}")
    print(f"reached_goal={reached}/{args.seeds}  min_true_margin_to_dmin={min_margin:.3f} m")
    print(f"solve time (sandbox): mean={t.mean()*1e3:.1f} ms  p95={np.percentile(t,95)*1e3:.1f} ms  max={t.max()*1e3:.1f} ms")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
