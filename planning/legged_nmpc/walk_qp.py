"""Full-dynamics walking with the floating-base whole-body QP (docs/18 §10.2). Simulation only.

Reduced NMPC (sim.py model) gives the CoM, yaw and footstep plan. The QP tracks the CoM and yaw, places the
swing foot on its planned sole target (kinematic swing reference), and realises the stance forces through the
floating-base dynamics. Joint posture is held to the standing configuration. No walking-policy weights used.

QPWalker steps the loop one control tick at a time (plan_and_actuate, then substep per physics step), so the
MuJoCo viewer can drive it. run() is the batch wrapper used by the tests.
"""
import mujoco
import numpy as np

from .config import LeggedNMPCConfig
from .controller import LeggedNMPC
from .model import contact_state, swing_pose, _stance_index
import math
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


class QPWalker:
    """One NMPC + QP walking run, stepped by the caller. Attributes after each tick: fell, k, stat_fail, log."""

    def __init__(self, v=0.0, w=0.0, gait=None, push=None, push_t=None, push_dur=0.2, kc=10.0, dc=4.0,
                 ka=40.0, da=8.0, kf=100.0, df=20.0, kp_post=60.0, kd_post=6.0, log_every=0.5, nmpc_every=None, dcm_k=0.0):
        cfg = LeggedNMPCConfig()
        self.cfg = cfg
        self.fw, self.qp, self.act, self.q_stand = build(cfg)
        self.m, self.d = self.fw.m, self.fw.d
        self.nmpc = LeggedNMPC(cfg)
        self.v, self.w = v, w
        self.gait = gait or ("stand" if (v == 0 and w == 0) else "walk")
        self.push, self.push_t, self.push_dur = push, push_t, push_dur
        self.kc, self.dc, self.ka, self.da, self.kf, self.df = kc, dc, ka, da, kf, df
        self.kp_post, self.kd_post = kp_post, kd_post
        self.dcm_k = dcm_k  # docs/19 §10: capture-point correction of the next footstep (0 = off)
        self.w_ = {"com": 1.0, "att": 1.0, "foot": 20.0, "post": 0.05, "tau": 1e-6, "force": 1e-6}
        self.dt_ctrl = cfg.dt
        self.sub = int(round(self.dt_ctrl / self.m.opt.timestep))
        self.log_every = log_every
        self.nmpc_every = max(1, int(nmpc_every if nmpc_every is not None else cfg.nmpc_every))
        self.k_solve = None          # tick index of the last NMPC solve
        self.Xp = None               # its predicted states (8 x N+1)
        self.k = 0
        self.stat_fail = 0
        self.fell = False
        self.log = []
        self.com_ref = np.zeros(3)
        self.psi_ref = 0.0
        self.psi = 0.0
        self.active = [True, True]

    def n_ticks(self, T):
        return int(round(T / self.dt_ctrl))

    def plan_and_actuate(self):
        """Compute NMPC reference, foot/CoM/attitude/posture accelerations, solve the QP, set actuator commands."""
        m, d, cfg = self.m, self.d, self.cfg
        t = self.k * self.dt_ctrl
        x = np.zeros(8)
        mujoco.mj_subtreeVel(m, d)
        x[0:3] = d.subtree_com[0]
        x[3:6] = d.subtree_linvel[0]
        R = d.xmat[1].reshape(3, 3)
        x[6] = np.arctan2(R[1, 0], R[0, 0])
        x[7] = d.qvel[5]
        if self.k_solve is None or self.k - self.k_solve >= self.nmpc_every:
            shift = self.k - self.k_solve if self.k_solve is not None else 1
            u, ok, _ = self.nmpc.solve(t, x, self.v, self.w, self.gait, shift=shift)
            self.stat_fail += 0 if ok else 1
            self.Xp = self.nmpc._prev[0]
            self.k_solve = self.k
        # Stage j of the last solve is j*dt after its solve time; tick k is (k - k_solve) ticks after it.
        j = min(self.k - self.k_solve + 1, self.Xp.shape[1] - 1)
        Xp = self.Xp
        com_ref, vel_ref, psi_ref = Xp[0:3, j], Xp[3:6, j], Xp[6, j]
        contacts = contact_state(cfg, t + self.dt_ctrl, self.v, self.w, self.gait)
        active = [bool(c[0]) for c in contacts]
        foot_acc = []
        for fi in (0, 1):
            if not active[fi]:
                xy, z, yaw_f = swing_pose(cfg, t + self.dt_ctrl, self.v, self.w, fi)
                if self.dcm_k:
                    xy = xy + self._dcm_blend(t + self.dt_ctrl, fi) * self._dcm_delta(x, com_ref, vel_ref)
                tgt = origin_target(self.fw, xy, z, yaw_f, fi)
                cur = d.xpos[self.fw.wb.feet[fi]]
                jp = np.zeros((3, m.nv))
                mujoco.mj_jacBody(m, d, jp, None, self.fw.wb.feet[fi])
                curv = jp @ d.qvel
                foot_acc.append((fi, self.kf * (tgt - cur) - self.df * curv))
        kc, dc = self.kc, self.dc
        com_acc = np.array([kc * (com_ref[0] - x[0]) + dc * (vel_ref[0] - x[3]),
                            kc * (com_ref[1] - x[1]) + dc * (vel_ref[1] - x[4]),
                            kc * (com_ref[2] - x[2]) + dc * (vel_ref[2] - x[5])])
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arcsin(np.clip(-R[2, 0], -1, 1))
        yaw_err = np.arctan2(np.sin(psi_ref - x[6]), np.cos(psi_ref - x[6]))
        om = d.qvel[3:6]
        ka, da = self.ka, self.da
        att_acc = np.array([-ka * roll - da * om[0], -ka * pitch - da * om[1], ka * yaw_err - da * om[2]])
        post = np.zeros(len(self.act))
        for i, (dof, a) in enumerate(self.act):
            j = [jj for jj in range(1, m.njnt) if m.jnt_dofadr[jj] == dof][0]
            post[i] = self.kp_post * (self.q_stand[j] - d.qpos[m.jnt_qposadr[j]]) - self.kd_post * d.qvel[dof]
        tgt = {"active": active, "com_acc": com_acc, "att_acc": att_acc, "post_acc": post,
               "foot_acc": foot_acc, "w": self.w_}
        tau, st = self.qp.solve(d, tgt)
        for i, (dof, a) in enumerate(self.act):
            d.ctrl[a] = float(tau[i])
        self.com_ref, self.psi_ref, self.psi, self.active = com_ref, psi_ref, float(x[6]), active

    def _dcm_delta(self, x, com_ref, vel_ref):
        """K * (measured DCM - reference DCM) in the xy plane. DCM xi = c + cdot / omega."""
        omega = math.sqrt(9.81 / self.cfg.Z_NOM)
        xi_meas = x[0:2] + x[3:5] / omega
        xi_ref = com_ref[0:2] + vel_ref[0:2] / omega
        return self.dcm_k * (xi_meas - xi_ref)

    def _dcm_blend(self, t, foot):
        """Weight of the correction in the swing pose: 0 in stance, rises to 1 at touchdown (same blend as swing_pose)."""
        cfg = self.cfg
        _, into = _stance_index(cfg, t, foot)
        if into < cfg.STANCE_T:
            return 0.0
        s = (into - cfg.STANCE_T) / cfg.SWING_T
        return 0.5 * (1.0 - math.cos(math.pi * s))

    def substep(self):
        """One physics step with the push applied if due. Returns True if the robot has fallen."""
        d = self.d
        if self.push is not None and self.push_t is not None and self.push_t <= d.time < self.push_t + self.push_dur:
            d.xfrc_applied[1, :3] = self.push
        else:
            d.xfrc_applied[1, :3] = 0.0
        mujoco.mj_step(self.m, d)
        if d.qpos[2] < 0.5 or abs(np.degrees(np.arccos(np.clip(d.xmat[1].reshape(3, 3)[2, 2], -1, 1)))) > 40:
            self.fell = True
        return self.fell

    def log_tick(self):
        """Record a log entry on the log_every cadence, then advance the tick counter."""
        d = self.d
        if self.k % max(1, int(round(self.log_every / self.dt_ctrl))) == 0:
            self.log.append(dict(t=round(d.time, 2), com=np.round(d.subtree_com[0], 3).tolist(),
                                 com_ref=np.round(self.com_ref, 3).tolist(), psi=round(self.psi, 3),
                                 ncon=int(d.ncon), active=self.active))
        self.k += 1

    def result(self):
        d = self.d
        return dict(fell=self.fell, t_end=round(float(d.time), 3), nmpc_fail=self.stat_fail, qp=self.qp.stats,
                    com_final=d.subtree_com[0].copy().tolist(), log=self.log)


def run(v=0.0, w=0.0, T=3.0, gait=None, push=None, push_t=None, push_dur=0.2, kc=10.0, dc=4.0,
        ka=40.0, da=8.0, kf=100.0, df=20.0, kp_post=60.0, kd_post=6.0, log_every=0.5, dcm_k=0.0):
    walker = QPWalker(v=v, w=w, gait=gait, push=push, push_t=push_t, push_dur=push_dur, kc=kc, dc=dc,
                      ka=ka, da=da, kf=kf, df=df, kp_post=kp_post, kd_post=kd_post, log_every=log_every, dcm_k=dcm_k)
    for _ in range(walker.n_ticks(T)):
        walker.plan_and_actuate()
        for _ in range(walker.sub):
            if walker.substep():
                break
        walker.log_tick()
        if walker.fell:
            break
    return walker.result()


if __name__ == "__main__":
    import json
    import sys
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    T = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    r = run(v=v, w=0.0, T=T)
    print(json.dumps({k: v_ for k, v_ in r.items() if k != "log"}, indent=1))
    for e in r["log"][:12]:
        print(e)
