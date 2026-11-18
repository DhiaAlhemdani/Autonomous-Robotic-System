"""Step 2 diagnosis: are the infeasible steps real infeasibility or solver failure?

The controller is NOT changed. The episodes run exactly as in run.py; the robot follows
the baseline controller. At each step where the baseline fell back to braking, this
script re-solves the same problem from several different starting guesses with a fresh
solver. If any start returns a feasible solution, the step was a solver failure.
If none does, the problem is probably infeasible, but that is not proven: the search
is only over the starts tried here.

Usage:  PYTHONPATH=planning python -m stage0.diagnose --seeds 20 --out runs/stage0_step2.json
"""
import argparse
import json
from collections import Counter
from datetime import datetime, timezone

import numpy as np

from .config import Stage0Config
from .nmpc import Stage0NMPC
from .scenario import run_episode


def _starts(x0, goal, cfg, rng):
    N = cfg.horizon
    starts = {"zeros": np.zeros((N, 2))}
    to_goal = goal - x0[:2]
    d = np.linalg.norm(to_goal)
    direction = to_goal / d if d > 1e-9 else np.zeros(2)
    starts["toward_goal"] = np.tile(0.5 * cfg.a_max * direction, (N, 1))
    v = x0[2:]
    brake = -v / cfg.dt
    n = np.linalg.norm(brake)
    if n > cfg.a_max:
        brake = brake * cfg.a_max / n
    starts["brake"] = np.tile(brake, (N, 1))
    starts["random"] = rng.uniform(-cfg.a_max, cfg.a_max, size=(N, 2))
    return starts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--out", default="runs/stage0_step2.json")
    args = ap.parse_args()
    cfg = Stage0Config()

    found = Counter()          # per-step: "feasible_found" or "none_found"
    which_start = Counter()    # which start first gave a feasible solution
    best_c_none = []           # best min-constraint over starts, when none feasible
    msgs = Counter()           # solver messages across all re-solves
    rec = []

    def hook(x0, goal, agents, base):
        rng = np.random.default_rng(len(rec))
        best = -np.inf
        winner = None
        for name, init in _starts(x0, goal, cfg, rng).items():
            fresh = Stage0NMPC(cfg)
            r = fresh.solve(x0, goal, agents, init=init)
            msgs[f"{name}: {r.message}"] += 1
            best = max(best, r.min_constraint)
            if r.status == "ok" and winner is None:
                winner = name
        if winner is not None:
            found["feasible_found"] += 1
            which_start[winner] += 1
        else:
            found["none_found"] += 1
            best_c_none.append(best)
        rec.append({"baseline_message": base.message, "winner": winner, "best_min_c": float(best)})

    for seed in range(args.seeds):
        run_episode(seed, cfg, hook=hook)

    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "claim_scope": "simulation only; diagnosis of baseline infeasible steps; "
                       "'none_found' is not a proof of infeasibility",
        "seeds": args.seeds,
        "infeasible_steps": len(rec),
        "classification": dict(found),
        "first_feasible_start": dict(which_start),
        "best_min_c_when_none_found": {
            "p50": float(np.percentile(best_c_none, 50)) if best_c_none else None,
            "max": float(np.max(best_c_none)) if best_c_none else None,
        },
        "solver_messages_on_resolves": dict(msgs.most_common(12)),
    }
    import os
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
