"""Run wbc_eval.py with overridden wbc.py gain constants (tuning only; simulation only).

Usage (repo root):
    .venv/bin/python sim/t800/wbc_sweep.py --set W_ORI=20 -- --controller wbc --seeds 100 101 102 103 104 --pushes all --com-target -0.07 --out runs/stage_a/sweep_x.json

Everything after `--` is passed to wbc_eval.py. The defaults in wbc.py are not edited. Each --set
overrides one module constant for that run only. Record every run and its settings in sim/t800/README.md.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wbc as W  # noqa: E402
import wbc_eval  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", action="append", default=[], help="NAME=VALUE, a module constant in wbc.py")
    args, rest = ap.parse_known_args()
    if rest and rest[0] == "--":
        rest = rest[1:]
    for item in args.set:
        name, value = item.split("=", 1)
        if not hasattr(W, name):
            sys.exit(f"unknown constant: {name}")
        old = getattr(W, name)
        setattr(W, name, type(old)(float(value)) if isinstance(old, float) else float(value))
        print(f"override {name}: {old} -> {getattr(W, name)}", flush=True)
    sys.argv = [sys.argv[0]] + rest
    return wbc_eval.main()


if __name__ == "__main__":
    sys.exit(main())
