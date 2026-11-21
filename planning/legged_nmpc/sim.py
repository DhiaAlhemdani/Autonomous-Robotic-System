"""Closed-loop simulation (docs/17 §7, docs/18 §4-7). Plant = the reduced model (with yaw) + an external push.

The whole-body layer (wbik.py) runs kinematically on the NMPC's CoM and yaw at every control step.
Run the seeded sweep: python -m planning.legged_nmpc.sim   (writes runs/legged_nmpc_sweep_{tuning,heldout}.json)
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

from .config import LeggedNMPCConfig
from .controller import LeggedNMPC
from .model import (ZMP_ROWS, contact_state, discrete_matrices, reference, swing_pose, yaw_torque,
                    zmp, zmp_rows)
from .wbik import WholeBody


def run_episode(scn, cfg=None, wb=None):
    """scn: gait, v, w (yaw rate), offset (2,), psi0, push_N, push_dir, push_t, push_dur, T (s), seed."""
    cfg = cfg or LeggedNMPCConfig()
    Ad, Bd, cd = discrete_matrices(cfg)
    ctrl = LeggedNMPC(cfg)
    wb = wb or WholeBody()
    gait, v, w = scn["gait"], scn["v"], scn.get("w", 0.0)
    steps = int(round(scn["T"] / cfg.dt))
    x = np.zeros(8)
    x[:3] = [scn["offset"][0], scn["offset"][1], cfg.Z_NOM]
    x[6] = scn.get("psi0", 0.0)
    push_vec = None
    if scn.get("push_N", 0.0) > 0:
        d = scn["push_dir"]
        push_vec = scn["push_N"] * np.array([np.cos(d), np.sin(d), 0.0])
    q = wb.q0.copy()

    rec = dict(fail=0, zmp_viol=0.0, fric_viol=0.0, bound_viol=0.0, solve_s=[], ik_s=[],
               com_res=[], feet_res_stance=[], feet_res_swing=[], clamps=0, dev=[], vx=[], psi_err=[])
    xs = []
    for i in range(steps):
        t = i * cfg.dt
        u, ok, el = ctrl.solve(t, x, v, w, gait)
        rec["fail"] += 0 if ok else 1
        rec["solve_s"].append(el)

        contacts = contact_state(cfg, t, v, w, gait)
        n, off = zmp_rows(cfg, contacts)
        rec["zmp_viol"] = max(rec["zmp_viol"], float(np.max(n @ zmp(x, u) + off)))
        for f in range(2):
            fx, fy, fz = u[3 * f], u[3 * f + 1], u[3 * f + 2]
            rec["fric_viol"] = max(rec["fric_viol"], abs(fx) - cfg.MU * fz, abs(fy) - cfg.MU * fz, -fz)
            if not contacts[f][0]:
                rec["bound_viol"] = max(rec["bound_viol"], float(np.max(np.abs(u[3 * f:3 * f + 3]))))
            rec["bound_viol"] = max(rec["bound_viol"], float(fz - cfg.FZ_MAX), float(-fz))

        # Whole-body layer on the current CoM and yaw, feet from the swing/stance reference.
        feet = []
        for foot in (0, 1):
            xy, z, psi_f = swing_pose(cfg, t, v, w, foot)
            feet.append((xy, z, psi_f))
        import time as _t
        t0 = _t.perf_counter()
        res = wb.solve(q, x[:3].copy(), feet, x[6])
        rec["ik_s"].append(_t.perf_counter() - t0)
        q = res["q"]
        rec["clamps"] += res["clamps"]
        rec["com_res"].append(res["com_res"])
        for foot in (0, 1):
            (rec["feet_res_stance"] if contacts[foot][0] else rec["feet_res_swing"]).append(res["feet_res"][foot])

        dev_xy = x[:2] - reference(cfg, t, v, w, gait)[0, :2]
        rec["dev"].append(float(np.linalg.norm(dev_xy)))
        rec["vx"].append(float(x[3]))
        rec.setdefault("vel_w", []).append(x[3:5].copy())
        rec.setdefault("vel_ref", []).append(reference(cfg, t, v, w, gait)[0, 3:5].copy())
        rec["psi_err"].append(float(abs(x[6] - (w * t if gait == "walk" else 0.0))))
        xs.append(x.copy())

        # Plant: the same reduced model with the yaw torque, plus the external push.
        tau = yaw_torque(x, u, _feet_centres(cfg, t, v, w, gait))
        xn = np.zeros(8)
        xn[0:6] = Ad @ x[0:6] + Bd @ u + cd
        if push_vec is not None and scn["push_t"] <= t < scn["push_t"] + scn["push_dur"]:
            xn[0:6] += np.concatenate([0.5 * cfg.dt ** 2 * push_vec / cfg.MASS, cfg.dt * push_vec / cfg.MASS])
        xn[6] = x[6] + cfg.dt * x[7] + 0.5 * cfg.dt ** 2 * tau / cfg.IZ
        xn[7] = x[7] + cfg.dt * tau / cfg.IZ
        x = xn

    half = steps // 2
    win = int(round(5.0 / cfg.dt))
    dev = np.array(rec["dev"])
    vx = np.array(rec["vx"])
    vel_w = np.array(rec["vel_w"])
    vel_ref = np.array(rec["vel_ref"])
    psi_err = np.array(rec["psi_err"])
    final_xy = xs[-1][:2]
    return dict(
        seed=scn.get("seed"), gait=gait, v=v, w=w, solver_failures=rec["fail"],
        infeasible_step_fraction=rec["fail"] / steps,
        zmp_violation_max_m=max(rec["zmp_viol"], 0.0),
        friction_violation_max_N=max(rec["fric_viol"], 0.0),
        force_bound_violation_max_N=max(rec["bound_viol"], 0.0),
        vel_mean_error_last_5s=float(np.linalg.norm(np.mean(vel_w[-win:], axis=0) - np.mean(vel_ref[-win:], axis=0))),
        heading_error_mean_last_5s=float(np.mean(psi_err[-win:])),
        max_deviation_m=float(dev.max()), frac_within_0p1m=float(np.mean(dev <= 0.1)),
        com_res_max_m=float(np.max(rec["com_res"][1:])) if steps > 1 else 0.0,
        feet_res_stance_max_m=float(np.max(rec["feet_res_stance"][int(round(1.0 / cfg.dt)):])) if rec["feet_res_stance"] else 0.0,
        feet_res_swing_max_m=float(np.max(rec["feet_res_swing"][int(round(1.0 / cfg.dt)):])) if rec["feet_res_swing"] else 0.0,
        joint_clamps=rec["clamps"],
        solve_s_p50=float(np.percentile(rec["solve_s"], 50)), solve_s_p99=float(np.percentile(rec["solve_s"], 99)),
        ik_s_p50=float(np.percentile(rec["ik_s"], 50)), ik_s_p99=float(np.percentile(rec["ik_s"], 99)),
        final_state=[float(a) for a in x],
    )


def _feet_centres(cfg, t, v, w, gait):
    """Sole centres used for the yaw torque at time t (stance centre, or next placement for a swing foot)."""
    out = np.zeros(4)
    for f, (ok, c, _) in enumerate(contact_state(cfg, t, v, w, gait)):
        out[2 * f:2 * f + 2] = c
    return out


def scenario(seed):
    """Seeded scene: speed, yaw rate, initial offset, push (magnitude, direction, timing). Walking."""
    rng = np.random.default_rng(seed)
    return dict(
        seed=seed, gait="walk", v=float(rng.uniform(0.1, 0.4)), w=float(rng.uniform(-0.2, 0.2)),
        offset=[float(a) for a in rng.uniform(-0.02, 0.02, size=2)],
        push_N=float(rng.uniform(0.0, 60.0)), push_dir=float(rng.uniform(0.0, 2 * np.pi)),
        push_t=3.0, push_dur=0.2, T=10.0,
    )


def summarise(results, cfg=None):
    cfg = cfg or LeggedNMPCConfig()
    steps_total = sum(round(r_T / cfg.dt) for r_T in [10.0] * len(results))
    fails = sum(r["solver_failures"] for r in results)
    return dict(
        episodes=len(results), solver_failures=fails, infeasible_step_fraction=fails / steps_total,
        zmp_violation_max_m=max(r["zmp_violation_max_m"] for r in results),
        friction_violation_max_N=max(r["friction_violation_max_N"] for r in results),
        vel_mean_error_last_5s_mean=float(np.mean([r["vel_mean_error_last_5s"] for r in results])),
        heading_error_mean_last_5s_mean=float(np.mean([r["heading_error_mean_last_5s"] for r in results])),
        max_deviation_m=max(r["max_deviation_m"] for r in results),
        frac_within_0p1m_mean=float(np.mean([r["frac_within_0p1m"] for r in results])),
        com_res_max_m=max(r["com_res_max_m"] for r in results),
        feet_res_stance_max_m=max(r["feet_res_stance_max_m"] for r in results),
        feet_res_swing_max_m=max(r["feet_res_swing_max_m"] for r in results),
        joint_clamps_total=sum(r["joint_clamps"] for r in results),
        solve_p50_median_s=float(np.median([r["solve_s_p50"] for r in results])),
        ik_p50_median_s=float(np.median([r["ik_s_p50"] for r in results])),
    )


def main():
    out = Path("runs")
    out.mkdir(exist_ok=True)
    wb = WholeBody()
    for name, seeds in [("tuning", range(0, 10)), ("heldout", range(10, 20))]:
        t0 = time.time()
        res = [run_episode(scenario(s), wb=wb) for s in seeds]
        payload = dict(set=name, seeds=list(seeds), episodes=res, summary=summarise(res),
                       wall_s=round(time.time() - t0, 1),
                       note="sandbox simulation; reduced model + kinematic whole-body layer; no hardware claim")
        (out / f"legged_nmpc_sweep_{name}.json").write_text(json.dumps(payload, indent=1))
        print(name, json.dumps(payload["summary"], indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
