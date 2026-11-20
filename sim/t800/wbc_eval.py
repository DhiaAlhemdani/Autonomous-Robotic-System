"""Held-out evaluation of the whole-body QP (wbc.py) with the Stage A criteria (simulation only).

Uses the same pre-declared criteria and evaluation seeds as stage_a_eval.py (seeds 0-4, pushes of
25-100 N in four directions). Two changes from stage_a_eval, both applied to BOTH controllers so the
comparison is like-for-like:

* Level start. The keyframe pitches both feet about 7 degrees, so only the toes touch the floor.
  Both ankle pitch joints are offset by -0.168 rad, which levels the feet by kinematics. This is a
  setup choice, not a result, and it is fixed here before the run.
* Fixed CoM target. The desired centre of mass is the start position, held for the whole episode.

Usage (repo root):  .venv/bin/python sim/t800/wbc_eval.py --controller wbc --out runs/stage_a/wbc_eval.json
                    .venv/bin/python sim/t800/wbc_eval.py --controller balance --out runs/stage_a/balance_level.json
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
import balance as B  # noqa: E402
import wbc as W      # noqa: E402
import stage_a_eval as SA  # noqa: E402  (criteria only; not run from here)

ANKLE_LEVEL = -0.168
SEEDS = SA.SEEDS
QUIET_T, PUSH_T, PUSH_DUR, PUSH_WINDOW_S = SA.QUIET_T, SA.PUSH_T, SA.PUSH_DUR, SA.PUSH_WINDOW_S
HELD_OUT_N = SA.HELD_OUT_N
DIRECTIONS = SA.DIRECTIONS
TILT_OK_DEG, Z_OK_M, FALL_TILT_DEG, FALL_Z_M = SA.TILT_OK_DEG, SA.Z_OK_M, SA.FALL_TILT_DEG, SA.FALL_Z_M


def setup(seed):
    r = B.Robot()
    r.reset()
    rng = np.random.default_rng(seed)
    r.d.qpos[r.act_q] = r.target + rng.uniform(-SA.OFFSET_RAD, SA.OFFSET_RAD, size=len(r.target))
    for n in ["J04_ANKLE_PITCH_L", "J10_ANKLE_PITCH_R"]:
        r.d.qpos[r.act_q[r.idx[n]]] += ANKLE_LEVEL
    mujoco.mj_forward(r.m, r.d)
    return r


def episode(seed, controller, quiet=True, fx=0.0, fy=0.0, com_target="start"):
    r = setup(seed)
    if controller == "wbc":
        ctl = W.WBC(r)
        pts, _ = ctl._corner_points()
        r.d.qpos[2] += -pts[:, 2].min()          # lowest sole corner on the floor
        mujoco.mj_forward(r.m, r.d)
        ctl = W.WBC(r)
        ctl.z_des = float(r.d.subtree_com[1][2])
        step = lambda com: ctl.step(com)
        fails = lambda: ctl.failures
    else:
        ctl = B.BalanceController(r)
        step = lambda com: ctl.step()
        fails = lambda: 0
    if com_target == "start":
        com0 = r.d.subtree_com[1][:2].copy()
    else:
        com0 = np.array(com_target, dtype=float)   # fixed xy, e.g. the sole centre
    body = mujoco.mj_name2id(r.m, mujoco.mjtObj.mjOBJ_BODY, "LINK_BASE")
    dt = r.m.opt.timestep
    T = QUIET_T if quiet else PUSH_T + PUSH_WINDOW_S
    fell_at = None
    max_tilt = 0.0
    for i in range(int(round(T / dt))):
        t = i * dt
        r.d.xfrc_applied[body, :] = 0
        if not quiet and PUSH_T <= t < PUSH_T + PUSH_DUR:
            r.d.xfrc_applied[body, 0] = fx
            r.d.xfrc_applied[body, 1] = fy
        step(com0)
        max_tilt = max(max_tilt, r.tilt_deg())
        if fell_at is None and (r.d.qpos[2] < FALL_Z_M or r.tilt_deg() > FALL_TILT_DEG):
            fell_at = t + dt
            break
    r.d.xfrc_applied[body, :] = 0
    c = B.foot_contacts(r)
    return {"fell_at_s": fell_at, "max_tilt_deg": float(max_tilt), "final_tilt_deg": float(r.tilt_deg()),
            "final_z_m": float(r.d.qpos[2]), "feet_contacts_end": c, "qp_failures": fails()}


def quiet_ok(e):
    return (e["fell_at_s"] is None and e["final_tilt_deg"] < TILT_OK_DEG and e["final_z_m"] > Z_OK_M
            and e["feet_contacts_end"]["L"] > 0 and e["feet_contacts_end"]["R"] > 0)


def push_ok(e):
    return e["fell_at_s"] is None and e["final_tilt_deg"] < TILT_OK_DEG


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", choices=["wbc", "balance"], required=True)
    ap.add_argument("--out", default="")
    ap.add_argument("--seeds", type=int, nargs="*", default=None, help="evaluation seeds (default: 0-4)")
    ap.add_argument("--com-target", default="start", help="'start', 'sole' (world xy (-0.11, 0)), or a numeric world x (y = 0)")
    ap.add_argument("--pushes", default="all", help="'all' or 'forward' (+x 75 and 100 N only, for tuning)")
    args = ap.parse_args()
    seeds = args.seeds if args.seeds else SEEDS
    if args.com_target == "start":
        com_target = "start"
    elif args.com_target == "sole":
        com_target = (-0.11, 0.0)
    else:
        com_target = (float(args.com_target), 0.0)   # numeric world x, y = 0
    if args.pushes == "forward":
        push_list = [("+x", 1, 0, m) for m in (75.0, 100.0)]
    else:
        push_list = [(d, ux, uy, m) for d, (ux, uy) in DIRECTIONS.items() for m in HELD_OUT_N]
    per_seed = []
    for seed in seeds:
        q = episode(seed, args.controller, quiet=True, com_target=com_target)
        pushes = {}
        for dname, ux, uy, mag in push_list:
            e = episode(seed, args.controller, quiet=False, fx=ux * mag, fy=uy * mag, com_target=com_target)
            pushes[f"{dname}_{int(mag)}N"] = dict(e, ok=push_ok(e), dir=dname, mag_n=mag)
        per_seed.append({"seed": seed, "quiet": dict(q, ok=quiet_ok(q)), "pushes": pushes})
        print(f"seed {seed}: quiet ok={quiet_ok(q)} fell={q['fell_at_s']} tilt {q['final_tilt_deg']:.2f} | "
              f"pushes ok {sum(p['ok'] for p in pushes.values())}/{len(pushes)}", flush=True)
    largest = {}
    for dname in DIRECTIONS:
        vals = [max([p["mag_n"] for p in s["pushes"].values() if p["dir"] == dname and p["ok"]] or [0.0])
                for s in per_seed]
        largest[dname] = {"per_seed_max_held_n": vals, "min_over_seeds_n": min(vals)}
    passed = all(s["quiet"]["ok"] and all(p["ok"] for p in s["pushes"].values()) for s in per_seed)
    result = {"controller": args.controller, "level_start_ankle_rad": ANKLE_LEVEL,
              "com_target": args.com_target, "seeds": seeds, "pushes": args.pushes,
              "criteria_source": "sim/t800/stage_a_eval.py", "per_seed": per_seed,
              "largest_held_n": largest, "stage_a_pass": passed,
              "scope": "simulation only; no claim of balance under larger pushes, walking, or hardware transfer"}
    try:
        from runmeta.collect import collect_meta
        result["meta"] = collect_meta("t800_stage_a_eval_" + args.controller, SEEDS[0],
                                      {"controller": args.controller, "level_start_ankle_rad": ANKLE_LEVEL,
                                       "criteria": SA.__doc__[:0] or "stage_a_eval.py constants"}, argv=sys.argv)
    except Exception as exc:
        result["meta_error"] = str(exc)
    print(f"STAGE A ({args.controller}, level start): {'PASS' if passed else 'FAIL'}")
    for dname, v in largest.items():
        print(f"largest held push {dname}: min over seeds {v['min_over_seeds_n']} N")
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            f.write(json.dumps(result, indent=2, default=float) + "\n")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
