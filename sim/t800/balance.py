"""Free-base standing test for the T800 in MuJoCo (no weld, no mocap).

Stage A of the whole-body plan: can a feedback law keep the free-standing robot upright?
The robot starts in the upstream standing pose. Every joint is torque-controlled by PD toward
a target. The ankle pitch and ankle roll targets are corrected by the whole-body centre-of-mass
error relative to the feet (ankle strategy). Hip and waist are held at the standing pose.

Everything here is sandbox simulation. Gains and signs are swept, not tuned for hardware.

Usage:  python sim/t800/balance.py                  (sweep, prints a table)
        python sim/t800/balance.py --sign 1 --kc 2.0 --T 5 --out runs/balance.json
"""
import argparse
import json
import os

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
XML = os.path.join(REPO, "third_party", "T800-PROJECT", "assets", "resource", "robot", "t800",
                   "xml", "serial_t800.xml")
PELVIS_Z = 0.87
HOME = [0, 0, PELVIS_Z, 1, 0, 0, 0,
        0, 0, -0.12, 0.24, -0.12, 0, 0, 0, -0.12, 0.24, -0.12,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
ANKLE_PITCH = ["J04_ANKLE_PITCH_L", "J10_ANKLE_PITCH_R"]
ANKLE_ROLL = ["J05_ANKLE_ROLL_L", "J11_ANKLE_ROLL_R"]


def build():
    if not os.path.exists(XML):
        raise SystemExit(f"T800 model not found at {XML}; run: git submodule update --init --depth 1 third_party/T800-PROJECT")
    spec = mujoco.MjSpec()
    spec.from_file(XML)
    fl = spec.worldbody.add_geom()
    fl.name = "floor"
    fl.type = mujoco.mjtGeom.mjGEOM_PLANE
    fl.size = [20, 20, 0.1]
    fl.friction = [1.0, 0.005, 0.0001]
    return spec.compile()


class Robot:
    def __init__(self):
        self.m = build()
        self.d = mujoco.MjData(self.m)
        m = self.m
        jnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
        self.q_home = {}
        k = 7
        for j in range(1, m.njnt):
            self.q_home[jnames[j]] = HOME[k]
            k += 1
        self.act_q, self.act_dof, self.act_name = [], [], []
        for a in range(m.nu):
            jid = m.actuator_trnid[a, 0]
            self.act_q.append(m.jnt_qposadr[jid])
            self.act_dof.append(m.jnt_dofadr[jid])
            self.act_name.append(jnames[jid])
        self.act_q = np.array(self.act_q)
        self.act_dof = np.array(self.act_dof)
        self.target = np.array([self.q_home[n] for n in self.act_name])
        self.idx = {n: i for i, n in enumerate(self.act_name)}
        self.lo = m.actuator_ctrlrange[:, 0]
        self.hi = m.actuator_ctrlrange[:, 1]
        self.reset()

    def reset(self):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[0:3] = [0, 0, PELVIS_Z]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[self.act_q] = self.target
        mujoco.mj_forward(m, d)

    def com_error(self):
        """Whole-body CoM minus the mid-point of the two feet, in the plane (m)."""
        d, m = self.d, self.m
        com = d.subtree_com[1].copy()  # LINK_BASE subtree = whole robot
        fl = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "LINK_FOOT_L")]
        fr = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "LINK_FOOT_R")]
        feet = 0.5 * (fl + fr)
        return com[:2] - feet[:2]

    def pitch_rad(self):
        """Signed pelvis pitch about the y axis (rad)."""
        w, x, y, z = self.d.qpos[3:7]
        return float(2 * np.arctan2(y, w))

    def tilt_deg(self):
        w = self.d.qpos[3]
        return float(np.degrees(2 * np.arccos(min(1.0, abs(w)))))

    def step(self, kp, kd, ankle_cmd=None):
        d, m = self.d, self.m
        tgt = self.target.copy()
        if ankle_cmd is not None:
            for name, delta in ankle_cmd.items():
                tgt[self.idx[name]] += delta
        q = d.qpos[self.act_q]
        qd = d.qvel[self.act_dof]
        d.ctrl[:] = np.clip(kp * (tgt - q) - kd * qd, self.lo, self.hi)
        mujoco.mj_step(m, d)


HIP_PITCH = ["J00_HIP_PITCH_L", "J06_HIP_PITCH_R"]


def run(sign, kc, kv, kp, kd, T, robot=None, hip_sign=0.0, kh=0.0, ki=0.0):
    r = robot or Robot()
    r.reset()
    n = int(round(T / r.m.opt.timestep))
    prev = r.com_error()
    dt = r.m.opt.timestep
    integ = np.zeros(2)
    fell_t = None
    min_z = np.inf
    max_tilt = 0.0
    for i in range(n):
        e = r.com_error()
        ev = (e - prev) / dt  # CoM error rate (finite difference)
        prev = e
        # ankle strategy: forward CoM error -> more ankle pitch (sign chooses the direction)
        integ = integ + e * dt
        cmd_pitch = sign * (kc * e[0] + kv * ev[0] + ki * integ[0])
        cmd = {}
        for name in ANKLE_PITCH:
            cmd[name] = cmd_pitch
        # hip strategy: correct the signed pelvis pitch
        hip = hip_sign * kh * r.pitch_rad()
        for name in HIP_PITCH:
            cmd[name] = hip
        r.step(kp, kd, ankle_cmd=cmd)
        z = float(r.d.qpos[2])
        tilt = r.tilt_deg()
        min_z = min(min_z, z)
        max_tilt = max(max_tilt, tilt)
        if z < 0.6 or tilt > 25.0:
            fell_t = i * dt
            break
    final_tilt = r.tilt_deg()
    final_z = float(r.d.qpos[2])
    # "held" = no fall AND settled upright at the end (tilt < 5 deg, pelvis at standing height)
    held = fell_t is None and final_tilt < 5.0 and final_z > 0.8
    return {"sign": sign, "kc": kc, "kv": kv, "ki": ki, "hip_sign": hip_sign, "kh": kh,
            "kp": kp, "kd": kd, "T": T,
            "held": held, "fell_at_s": fell_t, "min_pelvis_z": float(min_z),
            "max_tilt_deg": max_tilt, "final_tilt_deg": final_tilt, "final_z": final_z}


def foot_contacts(r):
    """Number of floor contacts on each foot (body name prefix match on the contact's geoms)."""
    m, d = r.m, r.d
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    counts = {"L": 0, "R": 0}
    for i in range(d.ncon):
        c = d.contact[i]
        g = [c.geom1, c.geom2]
        if floor not in g:
            continue
        other = g[1] if g[0] == floor else g[0]
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[other]) or ""
        if "FOOT_L" in body or "ANKLE_ROLL_L" in body:
            counts["L"] += 1
        if "FOOT_R" in body or "ANKLE_ROLL_R" in body:
            counts["R"] += 1
    return counts


def push_test(force_x, force_y, t_push=3.0, dur=0.2, T=15.0, cfg=None):
    """Apply a horizontal force to the pelvis for `dur` seconds at `t_push`, then check recovery."""
    r = Robot()
    cfg = cfg or dict(sign=+1.0, kc=3.0, kv=0.5, hip_sign=+1.0, kh=2.0, ki=0.0)
    r.reset()
    body = mujoco.mj_name2id(r.m, mujoco.mjtObj.mjOBJ_BODY, "LINK_BASE")
    dt = r.m.opt.timestep
    integ = np.zeros(2)
    prev = r.com_error()
    max_tilt = 0.0
    fell = None
    for i in range(int(round(T / dt))):
        t = i * dt
        r.d.xfrc_applied[body, :] = 0
        if t_push <= t < t_push + dur:
            r.d.xfrc_applied[body, 0] = force_x
            r.d.xfrc_applied[body, 1] = force_y
        e = r.com_error()
        ev = (e - prev) / dt
        prev = e
        integ = integ + e * dt
        cmd = {n: cfg["sign"] * (cfg["kc"] * e[0] + cfg["kv"] * ev[0] + cfg["ki"] * integ[0]) for n in ANKLE_PITCH}
        hip = cfg["hip_sign"] * cfg["kh"] * r.pitch_rad()
        for n in HIP_PITCH:
            cmd[n] = hip
        r.step(600.0, 20.0, ankle_cmd=cmd)
        max_tilt = max(max_tilt, r.tilt_deg())
        if r.d.qpos[2] < 0.6 or r.tilt_deg() > 25.0:
            fell = t
            break
    r.d.xfrc_applied[body, :] = 0
    return {"force_xy": [force_x, force_y], "fell_at_s": fell, "max_tilt_deg": max_tilt,
            "final_tilt_deg": r.tilt_deg(), "final_z": float(r.d.qpos[2]),
            "recovered": fell is None and r.tilt_deg() < 5.0,
            "feet_contacts_end": foot_contacts(r)}


def validate(T=20.0):
    r = Robot()
    cfg = dict(sign=+1.0, kc=3.0, kv=0.5, hip_sign=+1.0, kh=2.0, ki=0.0)
    out = run(cfg["sign"], cfg["kc"], cfg["kv"], 600.0, 20.0, T, r,
              hip_sign=cfg["hip_sign"], kh=cfg["kh"], ki=cfg["ki"])
    out["feet_contacts_end"] = foot_contacts(r)
    out["pushes"] = [push_test(fx, fy) for fx, fy in [(20, 0), (-20, 0), (0, 20), (0, -20), (40, 0), (0, 40)]]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sign", type=float, default=None)
    ap.add_argument("--kc", type=float, default=None)
    ap.add_argument("--kv", type=float, default=0.0)
    ap.add_argument("--kp", type=float, default=600.0)
    ap.add_argument("--kd", type=float, default=20.0)
    ap.add_argument("--T", type=float, default=10.0)
    ap.add_argument("--hip-sign", type=float, default=0.0)
    ap.add_argument("--kh", type=float, default=0.0)
    ap.add_argument("--ki", type=float, default=0.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--validate", action="store_true")
    args = ap.parse_args()
    if args.validate:
        out = validate(args.T)
        print(json.dumps(out, indent=2, default=float))
        if args.out:
            os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
            with open(args.out, "w") as f:
                json.dump(out, f, indent=2, default=float)
        return
    robot = Robot()
    if args.sign is not None and args.kc is not None:
        res = [run(args.sign, args.kc, args.kv, args.kp, args.kd, args.T, robot,
                   hip_sign=args.hip_sign, kh=args.kh, ki=args.ki)]
    else:
        res = []
        for hip_sign, kh in [(0.0, 0.0), (1.0, 2.0), (1.0, 6.0), (-1.0, 2.0), (-1.0, 6.0)]:
            for ki in (0.0, 0.3):
                res.append(run(+1.0, 3.0, 0.5, args.kp, args.kd, args.T, robot,
                               hip_sign=hip_sign, kh=kh, ki=ki))
    for r in res:
        print(f"hip {r['hip_sign']:+.0f} kh {r['kh']:4.1f} ki {r['ki']:.1f} | held {r['held']!s:5} "
              f"fell_at {r['fell_at_s']} | final_tilt {r['final_tilt_deg']:.1f} final_z {r['final_z']:.3f} max_tilt {r['max_tilt_deg']:.1f}")
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(res, f, indent=2)


if __name__ == "__main__":
    main()
