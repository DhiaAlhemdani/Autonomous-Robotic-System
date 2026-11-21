"""Full-dynamics walking with the floating-base whole-body QP (docs/18 §10.2). Simulation only.

Reduced NMPC (sim.py model) gives the CoM, yaw and footstep plan. The QP tracks the CoM and yaw, places the
swing foot on its planned sole target (kinematic swing reference), and realises the stance forces through the
floating-base dynamics. Joint posture is held to the standing configuration. No walking-policy weights used.
"""
import mujoco
import numpy as np

from .config import LeggedNMPCConfig
from .controller import LeggedNMPC
from .model import contact_state, swing_pose
from .wbc_qp import WBCQP
from .wbik import LEG_WAIST
from .fullsim import FullWalk


def build(cfg=None):
    fw = FullWalk(cfg)
    fw.reset_standing()
    m = fw.m
    sole_local = []
    for off, bot in zip(fw.wb.origin_to_sole, fw.wb.origin_to_sole_bottom):
        pts = [np.array([off[0] + sx * 0.12, off[1] + sy * 0.055, -bot]) for sx in (-1, 1) for sy in (-1, 1)]
        sole_local.append(pts)
    act = [(int(m.jnt_dofadr[j]), int(a)) for j, a in zip(range(1, m.njnt), range(m.nu))]
    tlim = [m.actuator_ctrlrange[a][1] for _, a in act]
    qp = WBCQP(m, fw.wb.feet, sole_local, act, tlim)
    q_stand = {j: float(fw.wb.q0[m.jnt_qposadr[j]]) for j in range(1, m.njnt)}
    for k, j in enumerate(fw.joint_ids):
        q_stand[j] = float(fw.q_target[k])
    return fw, qp, act, q_stand


def origin_target(fw, sole_xy, sole_z, yaw, foot):
    """Foot-origin world target from the sole reference (sole plane at sole_z)."""
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    sole = np.array([sole_xy[0], sole_xy[1], sole_z])
    return sole - Rz @ fw.wb.origin_to_sole[foot]


def run(v=0.0, w=0.0, T=3.0, gait=None, push=None, push_t=None, push_dur=0.2, kc=10.0, dc=4.0,
        ka=40.0, da=8.0, kf=100.0, df=20.0, kp_post=60.0, kd_post=6.0, log_every=0.5):
    cfg = LeggedNMPCConfig()
    fw, qp, act, q_stand = build(cfg)
    m, d = fw.m, fw.d
    nmpc = LeggedNMPC(cfg)
    gait = gait or ("stand" if (v == 0 and w == 0) else "walk")
    w_ = {"com": 1.0, "att": 1.0, "foot": 20.0, "post": 0.05, "tau": 1e-6, "force": 1e-6}
    dt_ctrl = cfg.dt
    sub = int(round(dt_ctrl / m.opt.timestep))
    n_ticks = int(round(T / dt_ctrl))
    log = []
    fell = False
    t_ctrl = 0.0
    stat_fail = 0
    for k in range(n_ticks):
        t = k * dt_ctrl
        x = np.zeros(8)
        mujoco.mj_subtreeVel(m, d)
        x[0:3] = d.subtree_com[0]
        x[3:6] = d.subtree_linvel[0]
        R = d.xmat[1].reshape(3, 3)
        x[6] = np.arctan2(R[1, 0], R[0, 0])
        x[7] = d.qvel[5]
        u, ok, _ = nmpc.solve(t, x, v, w, gait)
        stat_fail += 0 if ok else 1
        Xp = nmpc._prev[0]
        com_ref, vel_ref, psi_ref = Xp[0:3, 1], Xp[3:6, 1], Xp[6, 1]
        contacts = contact_state(cfg, t + dt_ctrl, v, w, gait)
        active = [bool(c[0]) for c in contacts]
        foot_acc = []
        for fi in (0, 1):
            if not active[fi]:
                xy, z, yaw_f = swing_pose(cfg, t + dt_ctrl, v, w, fi)
                tgt = origin_target(fw, xy, z, yaw_f, fi)
                cur = d.xpos[fw.wb.feet[fi]]
                curv = np.zeros(3)
                jp = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, d, jp, None, fw.wb.feet[fi])
                curv = jp @ d.qvel
                foot_acc.append((fi, kf * (tgt - cur) - df * curv))
        com_acc = np.array([kc * (com_ref[0] - x[0]) + dc * (vel_ref[0] - x[3]),
                            kc * (com_ref[1] - x[1]) + dc * (vel_ref[1] - x[4]),
                            kc * (com_ref[2] - x[2]) + dc * (vel_ref[2] - x[5])])
        roll = np.arctan2(R[2, 1], R[2, 2]); pitch = np.arcsin(np.clip(-R[2, 0], -1, 1))
        yaw_err = np.arctan2(np.sin(psi_ref - x[6]), np.cos(psi_ref - x[6]))
        om = d.qvel[3:6]
        att_acc = np.array([-ka * roll - da * om[0], -ka * pitch - da * om[1], ka * yaw_err - da * om[2]])
        post = np.zeros(len(act))
        for i, (dof, a) in enumerate(act):
            j = [jj for jj in range(1, m.njnt) if m.jnt_dofadr[jj] == dof][0]
            post[i] = kp_post * (q_stand[j] - d.qpos[m.jnt_qposadr[j]]) - kd_post * d.qvel[dof]
        tgt = {"active": active, "com_acc": com_acc, "att_acc": att_acc, "post_acc": post,
               "foot_acc": foot_acc, "w": w_}
        tau, st = qp.solve(d, tgt)
        for i, (dof, a) in enumerate(act):
            d.ctrl[a] = float(tau[i])
        for s in range(sub):
            if push is not None and push_t is not None and push_t <= d.time < push_t + push_dur:
                d.xfrc_applied[1, :3] = push
            else:
                d.xfrc_applied[1, :3] = 0.0
            mujoco.mj_step(m, d)
            if d.qpos[2] < 0.5 or abs(np.degrees(np.arccos(np.clip(d.xmat[1].reshape(3, 3)[2, 2], -1, 1)))) > 40:
                fell = True
                break
        if k % max(1, int(round(log_every / dt_ctrl))) == 0:
            log.append(dict(t=round(d.time, 2), com=np.round(d.subtree_com[0], 3).tolist(),
                            com_ref=np.round(com_ref, 3).tolist(), psi=round(float(x[6]), 3),
                            ncon=int(d.ncon), active=active))
        if fell:
            break
    return dict(fell=fell, t_end=round(float(d.time), 3), nmpc_fail=stat_fail, qp=qp.stats,
                com_final=d.subtree_com[0].copy().tolist(), log=log)


if __name__ == "__main__":
    import json
    import sys
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    T = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    r = run(v=v, w=0.0, T=T)
    print(json.dumps({k: v_ for k, v_ in r.items() if k != "log"}, indent=1))
    for e in r["log"][:12]:
        print(e)
