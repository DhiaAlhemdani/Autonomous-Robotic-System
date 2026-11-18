"""Run the Stage 0 scenario over fixed seeds and write a JSON report.

Usage (from the repository root, inside the sim container or a venv with numpy and scipy):
    PYTHONPATH=planning python -m stage0.run --seeds 50 --out runs/stage0.json

Results describe a simulation with the assumptions in stage0/config.py. They are not
measurements of the physical robot, and solve times are for this machine only.
"""
import argparse
import json
import os
import platform
from dataclasses import asdict
from datetime import datetime, timezone

import numpy as np
import scipy

from .config import Stage0Config
from .scenario import run_episode, summarize


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=50)
    ap.add_argument("--out", default="runs/stage0.json")
    ap.add_argument("--analytic", action="store_true", help="analytic SLSQP gradients")
    ap.add_argument("--jerk", type=float, default=None, help="override w_jerk")
    ap.add_argument("--lat", type=float, default=None, help="override w_lat")
    ap.add_argument("--effort", type=float, default=None, help="override w_effort")
    args = ap.parse_args()

    overrides = {k: v for k, v in (("w_jerk", args.jerk), ("w_lat", args.lat),
                                   ("w_effort", args.effort)) if v is not None}
    cfg = Stage0Config(**overrides)
    results = [run_episode(seed, cfg, analytic=args.analytic) for seed in range(args.seeds)]
    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "claim_scope": "simulation only, planar double integrator, scripted S2, assumed parameters",
        "config": asdict(cfg) | {"z_delta": cfg.z_delta},
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "machine": platform.machine(),
            "cpu_count": os.cpu_count(),
            "platform": platform.platform(),
        },
        "seeds": args.seeds,
        "analytic_gradients": args.analytic,
        "overrides": overrides,
        "summary": summarize(results),
    }
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report["summary"], indent=2))
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
