"""T800 with a virtual planar base driven by the Stage 0 NMPC (docs/13; option 2).

The T800's pelvis is welded (MuJoCo equality constraint) to a mocap body. The mocap body is
moved in the plane by the NMPC. The legs hold the upstream standing pose with PD torques.
Upright orientation is enforced by the weld, so this is NOT balance or locomotion.

Closed loop: the NMPC receives the pelvis's measured planar state from the physics, so the
weld lag and the leg dynamics are inside the loop. Persons and objects are the Stage 0 scene
(ground truth via the same random walk); they are drawn as non-colliding visuals.

Everything here runs in MuJoCo 3.2.3 in the sandbox. It is simulation only.

Usage:  python sim/t800/planar_base.py --seed 0 --out runs/t800_planar_seed0.json [--kp 600 --kd 20]
"""
import argparse
import json
import os
import sys
import time

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "planning"))

import mujoco  # noqa: E402

from stage0.config import Stage0Config  # noqa: E402
from stage0.nmpc import Agent, Stage0NMPC  # noqa: E402
from stage0.predictor import predict_constant_velocity  # noqa: E402
from stage0.scenario import PERSON_ACCEL_NOISE, VEL_OBS_NOISE, GOAL_TOL, _place_scene  # noqa: E402

XML = os.path.join(REPO, "third_party", "T800-PROJECT", "assets", "resource", "robot", "t800",
                   "xml", "serial_t800.xml")
PELVIS_Z = 0.87          # m, upstream standing keyframe base height
# Standing pose from the upstream keyframe (commented out upstream; used as the hold target)
HOME = [0, 0, PELVIS_Z, 1, 0, 0, 0,
        0, 0, -0.12, 0.24, -0.12, 0, 0, 0, -0.12, 0.24, -0.12,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
N_AGENTS = 3
N_OBJECTS = 2


def build_model(solref_weld):
    if not os.path.exists(XML):
        raise SystemExit(f"T800 model not found at {XML}; run: git submodule update --init --depth 1 third_party/T800-PROJECT")
    spec = mujoco.MjSpec()
    spec.from_file(XML)

    floor = spec.worldbody.add_geom()
    floor.name = "floor"
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [20, 20, 0.1]
    floor.friction = [1.0, 0.005, 0.0001]

    target = spec.worldbody.add_body()
    target.name = "base_target"
    target.mocap = True
    target.pos = [0.0, 0.0, PELVIS_Z]

    eq = spec.add_equality()
    eq.name = "base_weld"
    eq.type = mujoco.mjtEq.mjEQ_WELD
    eq.objtype = mujoco.mjtObj.mjOBJ_BODY
    eq.name1 = "base_target"
    eq.name2 = "LINK_BASE"
    eq.data = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1.0])
    eq.solref = np.array(solref_weld)

    for i in range(N_AGENTS + N_OBJECTS):
        b = spec.worldbody.add_body()
        b.name = f"vis_{i}"
        b.mocap = True
        b.pos = [0, 0, 0.5]
        g = b.add_geom()
        g.type = mujoco.mjtGeom.mjGEOM_SPHERE
        g.size = [0.3, 0, 0]
        g.contype = 0
        g.conaffinity = 0
        g.rgba = [0.9, 0.3, 0.2, 0.8] if i < N_AGENTS else [0.3, 0.6, 0.9, 0.8]
    return spec.compile()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--kp", type=float, default=600.0)
    ap.add_argument("--kd", type=float, default=20.0)
    ap.add_argument("--weld-solref", type=float, nargs=2, default=[0.02, 1.0])
    ap.add_argument("--settle", type=float, default=2.0, help="s of hold before the NMPC starts")
    ap.add_argument("--max-time", type=float, default=30.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--fd", action="store_true",
                    help="finite-difference SLSQP gradients (baseline); default is analytic")
    args = ap.parse_args()
    res = run_episode(args)
    from dataclasses import asdict
    from runmeta.collect import collect_meta  # docs/02 §7: every result carries a run record
    cfg_dict = asdict(Stage0Config())
    cfg_dict.update({"sim": {"kp": args.kp, "kd": args.kd, "weld_solref": args.weld_solref,
                              "settle_s": args.settle, "max_time_s": args.max_time,
                              "solver": "slsqp", "analytic_gradients": not args.fd}})
    res["meta"] = collect_meta("stage0_planar_scene", args.seed, cfg_dict)
    text = json.dumps(res, indent=2)
    print(text)
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            f.write(text + "\n")


def run_episode(args, on_step=None, stop=None):
    """Run one episode. on_step(dict) is called after every control step; stop() ends it early."""
    cfg = Stage0Config()
    m = build_model(args.weld_solref)
    d = mujoco.MjData(m)

    # Joint and actuator maps (PD on the 25 torque motors)
    jnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
    q_home = {}
    k = 7
    for j in range(1, m.njnt):
        q_home[jnames[j]] = HOME[k]
        k += 1
    act_q, act_dof, act_tgt = [], [], []
    for a in range(m.nu):
        jid = m.actuator_trnid[a, 0]
        act_q.append(m.jnt_qposadr[jid])
        act_dof.append(m.jnt_dofadr[jid])
        act_tgt.append(q_home.get(jnames[jid], 0.0))
    act_q = np.array(act_q)
    act_dof = np.array(act_dof)
    act_tgt = np.array(act_tgt)
    lo = m.actuator_ctrlrange[:, 0]
    hi = m.actuator_ctrlrange[:, 1]

    mocap_t = m.body("base_target").mocapid[0]
    mocap_v = [m.body(f"vis_{i}").mocapid[0] for i in range(N_AGENTS + N_OBJECTS)]

    def set_pelvis_pose():
        d.qpos[0:3] = [0.0, 0.0, PELVIS_Z]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[act_q] = act_tgt
        mujoco.mj_forward(m, d)

    mujoco.mj_resetData(m, d)
    set_pelvis_pose()
    d.mocap_pos[mocap_t] = [0.0, 0.0, PELVIS_Z]
    d.mocap_quat[mocap_t] = [1, 0, 0, 0]

    def pd_ctrl():
        q = d.qpos[act_q]
        qd = d.qvel[act_dof]
        d.ctrl[:] = np.clip(args.kp * (act_tgt - q) - args.kd * qd, lo, hi)

    substeps = int(round(cfg.dt / m.opt.timestep))

    def advance(n):
        for _ in range(n):
            pd_ctrl()
            mujoco.mj_step(m, d)

    # Scene (same generator as Stage 0)
    rng = np.random.default_rng(args.seed)
    goal, persons, objects = _place_scene(rng)
    p_pos = [p.copy() for p, _ in persons]
    p_vel = [v.copy() for _, v in persons]
    for i, o in enumerate(objects):
        d.mocap_pos[mocap_v[N_AGENTS + i]] = [o[0], o[1], 0.5]

    # Settle with the target held at the pelvis height
    settle_steps = int(round(args.settle / cfg.dt))
    for _ in range(settle_steps):
        d.mocap_pos[mocap_t] = [0.0, 0.0, PELVIS_Z]
        advance(substeps)
    pre = {"pelvis_z": float(d.qpos[2]), "tilt_deg": None}
    w, x_, y_, z_ = d.qpos[3:7]
    pre["tilt_deg"] = float(np.degrees(2 * np.arccos(min(1.0, abs(w)))))

    ctrl = Stage0NMPC(cfg, analytic=not args.fd)
    res = {"seed": args.seed, "kp": args.kp, "kd": args.kd, "weld_solref": args.weld_solref,
           "settle": pre, "goal_reached": False, "steps": 0, "violation_steps": 0,
           "min_dist_person": float("inf"), "min_dist_object": float("inf"),
           "max_tilt_deg": 0.0, "min_pelvis_z": float("inf"),
           "track_err_m": [], "solve_ms": [], "infeasible": 0, "fell": False}

    planner = None
    t0 = time.perf_counter()
    max_steps = int(round(args.max_time / cfg.dt))
    for step in range(max_steps):
        # Observe the physical pelvis state (closed loop)
        px, py = d.qpos[0], d.qpos[1]
        vx, vy = d.qvel[0], d.qvel[1]
        x = np.array([px, py, vx, vy])

        agents = []
        for i in range(N_AGENTS):
            obs_p = p_pos[i] + rng.normal(0.0, cfg.agent_sigma0, 2)
            obs_v = p_vel[i] + rng.normal(0.0, VEL_OBS_NOISE, 2)
            pred, sig = predict_constant_velocity(obs_p, obs_v, cfg.horizon, cfg.dt,
                                                  cfg.agent_sigma0, cfg.agent_sigma_rate)
            agents.append(Agent(pred=pred, sigma=sig, d_min=cfg.d_min_person))
        for o in objects:
            obs_p = o + rng.normal(0.0, cfg.agent_sigma0, 2)
            pred, sig = predict_constant_velocity(obs_p, np.zeros(2), cfg.horizon, cfg.dt,
                                                  cfg.agent_sigma0, 0.0)
            agents.append(Agent(pred=pred, sigma=sig, d_min=cfg.d_min_object))

        r = ctrl.solve(x, goal, agents)
        res["solve_ms"].append(r.solve_time_s * 1000.0)
        if r.status == "infeasible":
            res["infeasible"] += 1
        a = r.accel

        # Next target: NMPC's one-step prediction of the planar position
        # Planner state: integrates the NMPC command. The weld target follows the planner, not
        # the measured pelvis, so the commanded acceleration accumulates (the NMPC still reads
        # the measured state each step).
        if planner is None:
            planner = np.array([px, py, vx, vy])
        start_t = d.mocap_pos[mocap_t].copy()
        planner = np.array([planner[0] + planner[2] * cfg.dt + 0.5 * a[0] * cfg.dt ** 2,
                            planner[1] + planner[3] * cfg.dt + 0.5 * a[1] * cfg.dt ** 2,
                            planner[2] + a[0] * cfg.dt,
                            planner[3] + a[1] * cfg.dt])
        nxt = np.array([planner[0], planner[1], PELVIS_Z])
        for s in range(substeps):
            frac = (s + 1) / substeps
            d.mocap_pos[mocap_t] = start_t + frac * (nxt - start_t)
            pd_ctrl()
            mujoco.mj_step(m, d)

        # Ground truth of persons and objects
        for i in range(N_AGENTS):
            p_vel[i] = p_vel[i] + rng.normal(0.0, PERSON_ACCEL_NOISE * cfg.dt, 2)
            p_pos[i] = p_pos[i] + p_vel[i] * cfg.dt
            d.mocap_pos[mocap_v[i]] = [p_pos[i][0], p_pos[i][1], 0.5]
        for i, o in enumerate(objects):
            d.mocap_pos[mocap_v[N_AGENTS + i]] = [o[0], o[1], 0.5]

        # Metrics from the physics state after the step
        qx, qy = d.qpos[0], d.qpos[1]
        wq = d.qpos[3]
        tilt = float(np.degrees(2 * np.arccos(min(1.0, abs(wq)))))
        res["max_tilt_deg"] = max(res["max_tilt_deg"], tilt)
        res["min_pelvis_z"] = min(res["min_pelvis_z"], float(d.qpos[2]))
        res["track_err_m"].append(float(np.hypot(qx - nxt[0], qy - nxt[1])))
        dp = min(np.hypot(qx - p[0], qy - p[1]) for p in p_pos)
        do = min(np.hypot(qx - o[0], qy - o[1]) for o in objects)
        res["min_dist_person"] = min(res["min_dist_person"], float(dp))
        res["min_dist_object"] = min(res["min_dist_object"], float(do))
        if dp < cfg.d_min_person or do < cfg.d_min_object:
            res["violation_steps"] += 1

        res["steps"] = step + 1
        if on_step is not None:
            on_step({
                "step": step + 1, "t": round((step + 1) * cfg.dt, 3),
                "pelvis": [round(float(qx), 3), round(float(qy), 3)],
                "planner": [round(float(planner[0]), 3), round(float(planner[1]), 3)],
                "speed": round(float(np.hypot(d.qvel[0], d.qvel[1])), 3),
                "accel": [round(float(a[0]), 3), round(float(a[1]), 3)],
                "tilt_deg": round(tilt, 2), "pelvis_z": round(float(d.qpos[2]), 3),
                "solve_ms": round(r.solve_time_s * 1000.0, 1), "nmpc_status": r.status,
                "dist_person": round(float(dp), 3), "dist_object": round(float(do), 3),
                "violation": bool(dp < cfg.d_min_person or do < cfg.d_min_object),
                "track_err_m": round(res["track_err_m"][-1], 4),
                "persons": [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in p_pos],
                "objects": [[round(float(o[0]), 3), round(float(o[1]), 3)] for o in objects],
                "goal": [round(float(goal[0]), 3), round(float(goal[1]), 3)],
            })
        if stop is not None and stop():
            res["stopped"] = True
            break
        if step % 50 == 0:
            res.setdefault("trace", []).append([round(float(qx), 3), round(float(qy), 3), round(float(x[2]), 3), round(float(x[3]), 3), round(float(a[0]), 3), round(float(a[1]), 3)])
        if d.qpos[2] < 0.6 or tilt > 20.0:
            res["fell"] = True
            break
        if np.hypot(qx - goal[0], qy - goal[1]) < GOAL_TOL:
            res["goal_reached"] = True
            break

    res["wall_s"] = time.perf_counter() - t0
    res["final_xy"] = [float(d.qpos[0]), float(d.qpos[1])]
    res["goal_xy"] = [float(goal[0]), float(goal[1])]
    res["final_speed"] = float(np.hypot(d.qvel[0], d.qvel[1]))
    sm = np.array(res["solve_ms"])
    te = np.array(res["track_err_m"])
    res["summary"] = {
        "solve_p50_ms": float(np.percentile(sm, 50)) if sm.size else None,
        "solve_p99_ms": float(np.percentile(sm, 99)) if sm.size else None,
        "infeasible_fraction": res["infeasible"] / max(1, len(sm)),
        "track_err_p50_m": float(np.percentile(te, 50)) if te.size else None,
        "track_err_p99_m": float(np.percentile(te, 99)) if te.size else None,
    }
    del res["solve_ms"], res["track_err_m"]
    return res



if __name__ == "__main__":
    main()
