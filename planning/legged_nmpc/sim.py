"""Closed-loop simulation of the reduced model (docs/17 §7). Plant = the same model plus an external push.

Run the seeded sweep:  python -m planning.legged_nmpc.sim
Tuning seeds 0-9 and held-out seeds 10-19. Results are written to runs/legged_nmpc_sweep_*.json.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

from .config import LeggedNMPCConfig
from .controller import LeggedNMPC
from .model import contact_state, discrete_matrices, reference, zmp, zmp_rows


def plant_step(x, u, push, cfg, Ad, Bd, cd):
    xn = Ad @ x + Bd @ u + cd
    if push is not None:
        Fp = push
        xn = xn + np.concatenate([0.5 * cfg.dt ** 2 * Fp / cfg.MASS, cfg.dt * Fp / cfg.MASS])
    return xn


def run_episode(scn, cfg=None):
    """scn: dict with gait, v_cmd, offset (2,), push_N, push_dir (rad), push_t, push_dur, T (s)."""
    cfg = cfg or LeggedNMPCConfig()
    Ad, Bd, cd = discrete_matrices(cfg)
    ctrl = LeggedNMPC(cfg)
    gait, v = scn["gait"], scn["v_cmd"]
    steps = int(round(scn["T"] / cfg.dt))
    p0 = np.array([scn["offset"][0], scn["offset"][1], cfg.Z_NOM])
    x = np.concatenate([p0, np.zeros(3)])
    push_vec = None
    if scn.get("push_N", 0.0) > 0:
        d = scn["push_dir"]
        push_vec = scn["push_N"] * np.array([np.cos(d), np.sin(d), 0.0])

    rec = dict(fail=0, zmp_viol=0.0, fric_viol=0.0, bound_viol=0.0, dev=[], verr=[], vx=[], solve_s=[], steps=steps)
    xs = []
    for i in range(steps):
        t = i * cfg.dt
        u, ok, el = ctrl.solve(t, x, v, gait)
        rec["fail"] += 0 if ok else 1
        rec["solve_s"].append(el)
        contacts = contact_state(cfg, t, v, gait)
        n, off = zmp_rows(cfg, contacts)
        z = zmp(x, u)
        rec["zmp_viol"] = max(rec["zmp_viol"], float(np.max(n @ z + off)))
        for f in range(2):
            fx, fy, fz = u[3 * f], u[3 * f + 1], u[3 * f + 2]
            rec["fric_viol"] = max(rec["fric_viol"], abs(fx) - cfg.MU * fz, abs(fy) - cfg.MU * fz, -fz)
            if not contacts[f][0]:
                rec["bound_viol"] = max(rec["bound_viol"], float(np.max(np.abs(u[3 * f:3 * f + 3]))))
            rec["bound_viol"] = max(rec["bound_viol"], float(fz - cfg.FZ_MAX), float(-fz))
        pref = np.array([v * t, 0.0, cfg.Z_NOM]) if gait == "walk" else np.array([0.0, 0.0, cfg.Z_NOM])
        rec["dev"].append(float(np.linalg.norm(x[:2] - pref[:2])))
        rec["vx"].append(float(x[3]))
        rec["verr"].append(float(np.linalg.norm(x[3:5] - np.array([v if gait == "walk" else 0.0, 0.0]))))
        xs.append(x.copy())
        push = push_vec if (push_vec is not None and scn["push_t"] <= t < scn["push_t"] + scn["push_dur"]) else None
        x = plant_step(x, u, push, cfg, Ad, Bd, cd)

    dev = np.array(rec["dev"])
    verr = np.array(rec["verr"])
    half = steps // 2
    win = int(round(5.0 / cfg.dt))
    vel_mean_err_5s = float(abs(np.mean(rec["vx"][-win:]) - (v if gait == "walk" else 0.0)))
    return dict(
        seed=scn.get("seed"), gait=gait, v_cmd=v, solver_failures=rec["fail"],
        zmp_violation_max_m=max(rec["zmp_viol"], 0.0), friction_violation_max_N=max(rec["fric_viol"], 0.0),
        velocity_error_mean_last_half=float(verr[half:].mean()),
        vel_mean_error_last_5s=vel_mean_err_5s,
        force_bound_violation_max_N=max(rec["bound_viol"], 0.0),
        max_deviation_m=float(dev.max()), frac_within_0p1m=float(np.mean(dev <= 0.1)),
        solve_s_p50=float(np.percentile(rec["solve_s"], 50)), solve_s_p99=float(np.percentile(rec["solve_s"], 99)),
        final_pos_error_m=float(np.linalg.norm(xs[-1][:2] - np.array([v * (steps - 1) * cfg.dt if gait == "walk" else 0.0, 0.0]))),
        final_state=[float(a) for a in x],
    )


def scenario(seed):
    """Seeded scene: speed command, initial offset, push (magnitude, direction, timing)."""
    rng = np.random.default_rng(seed)
    return dict(
        seed=seed, gait="walk", v_cmd=float(rng.uniform(0.1, 0.4)),
        offset=[float(a) for a in rng.uniform(-0.02, 0.02, size=2)],
        push_N=float(rng.uniform(0.0, 60.0)), push_dir=float(rng.uniform(0.0, 2 * np.pi)),
        push_t=3.0, push_dur=0.2, T=10.0,
    )


def sweep(seeds):
    return [run_episode(scenario(s)) for s in seeds]


def summarise(results):
    fails = sum(r["solver_failures"] for r in results)
    steps = sum(round(10.0 / 0.05) for _ in results)
    times = np.concatenate([[r["solve_s_p50"]] for r in results])
    return dict(
        episodes=len(results), solver_failures=fails,
        infeasible_step_fraction=fails / steps,
        zmp_violation_max_m=max(r["zmp_violation_max_m"] for r in results),
        friction_violation_max_N=max(r["friction_violation_max_N"] for r in results),
        velocity_error_mean=float(np.mean([r["velocity_error_mean_last_half"] for r in results])),
        vel_mean_error_last_5s_mean=float(np.mean([r["vel_mean_error_last_5s"] for r in results])),
        max_deviation_m=max(r["max_deviation_m"] for r in results),
        frac_within_0p1m_mean=float(np.mean([r["frac_within_0p1m"] for r in results])),
        solve_p50_median_over_episodes_s=float(np.median(times)),
    )


def main():
    out = Path("runs")
    out.mkdir(exist_ok=True)
    for name, seeds in [("tuning", range(0, 10)), ("heldout", range(10, 20))]:
        t0 = time.time()
        res = sweep(seeds)
        payload = dict(set=name, seeds=list(seeds), episodes=res, summary=summarise(res),
                       wall_s=round(time.time() - t0, 1), note="sandbox simulation; reduced model; no hardware claim")
        path = out / f"legged_nmpc_sweep_{name}.json"
        path.write_text(json.dumps(payload, indent=1))
        print(name, json.dumps(payload["summary"], indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
