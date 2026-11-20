"""Stage A standing evaluation for the T800 (simulation only, fixed seeds, pre-declared criteria).

Evaluates the existing free-base standing controller (balance.py, unchanged) against criteria that
are fixed in this file before the run. The controller's gains were chosen on pushes of 20-500 N
(README, Stage A). The push magnitudes here are 25-100 N, a set that was not used for gain selection.

Stage A passes only if BOTH hold for every seed:
  1. Quiet standing for QUIET_T s: no fall, final tilt < TILT_OK_DEG, pelvis height > Z_OK_M,
     and at least one floor contact on each foot at the end.
  2. Every push in HELD_OUT_N, in each of the four horizontal directions, is recovered:
     no fall within PUSH_WINDOW_S s after the push, and final tilt < TILT_OK_DEG.

Nothing here claims balance under larger pushes, walking, or hardware transfer. Those are reported
as measured numbers only, and the largest-held push is reported per direction without a pass/fail.

Usage (repo root):  .venv/bin/python sim/t800/stage_a_eval.py --out runs/stage_a/eval.json
"""
import argparse
import json
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "planning"))

import balance as B  # noqa: E402  (the controller under test; not modified here)

# ---- Pre-declared criteria (ASSUMPTION thresholds, fixed before the run) ----
SEEDS = [0, 1, 2, 3, 4]          # fixed seeds for the initial pelvis offset
OFFSET_RAD = 0.02                # uniform initial offset on every actuated joint, +-0.02 rad (ASSUMPTION)
# A pelvis x/y shift was tried first and does not change the outcome: the ankle law uses the CoM
# relative to the feet, and a shift moves both equally. Joint offsets do change the CoM error.
QUIET_T = 20.0                   # s, quiet standing
PUSH_T = 5.0                     # s, push start
PUSH_DUR = 0.2                   # s, push duration (as in balance.py)
PUSH_WINDOW_S = 15.0             # s after the push start to judge recovery
HELD_OUT_N = [25.0, 50.0, 75.0, 100.0]   # N, not used for gain selection
DIRECTIONS = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}
TILT_OK_DEG = 5.0
Z_OK_M = 0.8
FALL_TILT_DEG = 25.0
FALL_Z_M = 0.6
# -------------------------------------------------------------------------------


def episode(seed, quiet=True, fx=0.0, fy=0.0):
    r = B.Robot()
    r.reset()
    rng = np.random.default_rng(seed)
    import mujoco as mj
    r.d.qpos[r.act_q] = r.target + rng.uniform(-OFFSET_RAD, OFFSET_RAD, size=len(r.target))
    mj.mj_forward(r.m, r.d)
    ctrl = B.BalanceController(r)
    body = mj.mj_name2id(r.m, mj.mjtObj.mjOBJ_BODY, "LINK_BASE")
    dt = r.m.opt.timestep
    T = QUIET_T if quiet else PUSH_T + PUSH_WINDOW_S
    n = int(round(T / dt))
    fell_at = None
    max_tilt = 0.0
    for i in range(n):
        t = i * dt
        r.d.xfrc_applied[body, :] = 0
        if not quiet and PUSH_T <= t < PUSH_T + PUSH_DUR:
            r.d.xfrc_applied[body, 0] = fx
            r.d.xfrc_applied[body, 1] = fy
        ctrl.step()
        max_tilt = max(max_tilt, r.tilt_deg())
        if fell_at is None and (r.d.qpos[2] < FALL_Z_M or r.tilt_deg() > FALL_TILT_DEG):
            fell_at = t + dt
            break
    r.d.xfrc_applied[body, :] = 0
    c = B.foot_contacts(r)
    return {
        "fell_at_s": fell_at,
        "max_tilt_deg": float(max_tilt),
        "final_tilt_deg": float(r.tilt_deg()),
        "final_z_m": float(r.d.qpos[2]),
        "feet_contacts_end": c,
    }


def quiet_ok(e):
    return (e["fell_at_s"] is None and e["final_tilt_deg"] < TILT_OK_DEG
            and e["final_z_m"] > Z_OK_M and e["feet_contacts_end"]["L"] > 0
            and e["feet_contacts_end"]["R"] > 0)


def push_ok(e):
    return e["fell_at_s"] is None and e["final_tilt_deg"] < TILT_OK_DEG


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    per_seed = []
    for seed in SEEDS:
        q = episode(seed, quiet=True)
        pushes = {}
        for dname, (ux, uy) in DIRECTIONS.items():
            for mag in HELD_OUT_N:
                e = episode(seed, quiet=False, fx=ux * mag, fy=uy * mag)
                pushes[f"{dname}_{int(mag)}N"] = dict(e, ok=push_ok(e), dir=dname, mag_n=mag)
        per_seed.append({"seed": seed, "quiet": dict(q, ok=quiet_ok(q)), "pushes": pushes})
        print(f"seed {seed}: quiet ok={quiet_ok(q)} tilt {q['final_tilt_deg']:.2f} | "
              f"pushes ok {sum(p['ok'] for p in pushes.values())}/{len(pushes)}", flush=True)

    # Largest push held per direction, per seed (reported, not pass/fail).
    largest = {}
    for dname in DIRECTIONS:
        vals = []
        for s in per_seed:
            held = [p["mag_n"] for p in s["pushes"].values() if p["dir"] == dname and p["ok"]]
            vals.append(max(held) if held else 0.0)
        largest[dname] = {"per_seed_max_held_n": vals, "min_over_seeds_n": min(vals)}

    stage_a_pass = all(s["quiet"]["ok"] and all(p["ok"] for p in s["pushes"].values()) for s in per_seed)
    result = {
        "criteria": {"seeds": SEEDS, "offset_rad": OFFSET_RAD, "quiet_t_s": QUIET_T,
                     "held_out_push_n": HELD_OUT_N, "push_window_s": PUSH_WINDOW_S,
                     "tilt_ok_deg": TILT_OK_DEG, "z_ok_m": Z_OK_M,
                     "fall_tilt_deg": FALL_TILT_DEG, "fall_z_m": FALL_Z_M},
        "controller": {"file": "sim/t800/balance.py", "kp": 600.0, "kd": 20.0,
                       "cfg": dict(B.DEFAULT_CFG)},
        "per_seed": per_seed,
        "largest_held_n": largest,
        "stage_a_pass": stage_a_pass,
        "scope": "simulation only; no claim of balance under larger pushes, walking, or hardware transfer",
    }
    try:
        from runmeta.collect import collect_meta
        result["meta"] = collect_meta("t800_stage_a_standing", SEEDS[0],
                                      {"controller": result["controller"], "criteria": result["criteria"]},
                                      argv=sys.argv)
    except Exception as exc:  # metadata is required for a reportable result; record the failure
        result["meta_error"] = str(exc)

    text = json.dumps(result, indent=2, default=float)
    print(f"STAGE A {'PASS' if stage_a_pass else 'FAIL'}")
    for dname, v in largest.items():
        print(f"largest held push {dname}: min over seeds {v['min_over_seeds_n']} N")
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            f.write(text + "\n")
    return 0 if stage_a_pass else 1


if __name__ == "__main__":
    sys.exit(main())
