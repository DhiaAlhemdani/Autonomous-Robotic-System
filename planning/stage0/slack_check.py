"""Step 2b runner: for each baseline infeasible step, run the slack check.

The controller is unchanged; episodes follow the baseline exactly.
Usage:  PYTHONPATH=planning python -m stage0.slack_check --seeds 20 --out runs/stage0_step2b.json
"""
import argparse
import json
import os
from collections import Counter
from datetime import datetime, timezone

import numpy as np

from .config import Stage0Config
from .scenario import run_episode
from .slack import FEAS_TOL, slack_check


def _starts(x0, goal, cfg):
    N = cfg.horizon
    d = goal - x0[:2]
    n = np.linalg.norm(d)
    direction = d / n if n > 1e-9 else np.zeros(2)
    return [np.zeros(2 * N), np.tile(0.5 * cfg.a_max * direction, N)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--out", default="runs/stage0_step2b.json")
    args = ap.parse_args()
    cfg = Stage0Config()

    cls = Counter()
    shortfalls_infeasible = []
    msgs = Counter()

    def hook(x0, goal, agents, base):
        best = None
        for init in _starts(x0, goal, cfg):
            r = slack_check(cfg, x0, goal, agents, init=init)
            msgs[r["message"]] += 1
            if best is None or r["shortfall_m"] < best["shortfall_m"]:
                best = r
        if best["feasible_within_tol"]:
            cls["feasible_within_tol (baseline solver failed)"] += 1
        elif best["success"]:
            cls["no margin-feasible point; solver converged"] += 1
            shortfalls_infeasible.append(best["shortfall_m"])
        else:
            cls["unresolved (slack solver failed)"] += 1

    for seed in range(args.seeds):
        run_episode(seed, cfg, hook=hook)

    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "claim_scope": "simulation only; diagnosis; 'no point' is not a proof of infeasibility over all starts",
        "seeds": args.seeds,
        "feasibility_tolerance_m": FEAS_TOL,
        "infeasible_steps": int(sum(cls.values())),
        "classification": dict(cls),
        "shortfall_m_when_converged_no_point": {
            "p50": float(np.percentile(shortfalls_infeasible, 50)) if shortfalls_infeasible else None,
            "p90": float(np.percentile(shortfalls_infeasible, 90)) if shortfalls_infeasible else None,
            "max": float(np.max(shortfalls_infeasible)) if shortfalls_infeasible else None,
        },
        "solver_messages": dict(msgs.most_common(10)),
    }
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
