"""Full-dynamics T800 walking experiment (MuJoCo, simulation only).

Loop: the reduced NMPC (sim.py model) runs on the measured CoM and yaw; its CoM, yaw and the swing/stance
reference go through the whole-body IK (wbik.py); the IK joint targets drive the real MuJoCo joints with
PD + gravity feedforward (qfrc_bias), clipped to the actuator torque limits. Ground: a floor plane.
Nothing here uses the walking-policy weights. No hardware claim.
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

from .config import LeggedNMPCConfig
from .controller import LeggedNMPC
from .model import swing_pose
from .wbik import LEG_WAIST, ROOT, WholeBody

XML = ROOT / "vendor" / "t800" / "assets" / "resource" / "robot" / "t800" / "xml" / "serial_t800.xml"


def build_model():
    spec = mujoco.MjSpec()
    spec.from_file(str(XML))
    fl = spec.worldbody.add_geom()
    fl.name = "floor"
    fl.type = mujoco.mjtGeom.mjGEOM_PLANE
    fl.size = [20, 20, 0.1]
    fl.friction = [1.0, 0.005, 0.0001]
    return spec.compile()


class FullWalk:
    KP = {"HIP": 400.0, "KNEE": 600.0, "ANKLE": 200.0, "WAIST": 150.0}
    KD = {"HIP": 25.0, "KNEE": 35.0, "ANKLE": 15.0, "WAIST": 8.0}

    # Ankle attitude feedback (ankle strategy): torque on ankle pitch/roll proportional to base pitch/roll.
    ATT_PITCH = 0.0   # N m per rad (set by experiments)
    ATT_ROLL = 0.0
    # Task-space balance (Jacobian transpose), on the actuated leg and waist joints only.
    BAL = False
    KC = 1500.0       # N/m, CoM horizontal stiffness
    DC = 300.0        # N s/m, CoM damping
    KA = 500.0        # N m/rad, base roll/pitch stiffness
    DA = 50.0         # N m s/rad, base roll/pitch damping

    def __init__(self, cfg=None, kp_scale=1.0, kd_scale=1.0):
        self.cfg = cfg or LeggedNMPCConfig()
        self.m = build_model()
        self.d = mujoco.MjData(self.m)
        self.wb = WholeBody()           # kinematics for IK (same model without the floor)
        self.ctrl = LeggedNMPC(self.cfg)
        self.kp_scale, self.kd_scale = kp_scale, kd_scale
        # Joint -> dof, actuator index, PD gains.
        self.joint_ids = self.wb.jids
        self.dofs = [int(self.m.jnt_dofadr[j]) for j in self.joint_ids]
        self.qadr = [int(self.m.jnt_qposadr[j]) for j in self.joint_ids]
        self.act = []
        for j in self.joint_ids:
            a = [i for i in range(self.m.nu) if self.m.actuator_trnid[i][0] == j][0]
            self.act.append(a)
        self.tlim = np.array([self.m.actuator_ctrlrange[a][1] for a in self.act])
        self.kp = np.array([self._gain(n, self.KP) for n in LEG_WAIST]) * kp_scale
        self.kd = np.array([self._gain(n, self.KD) for n in LEG_WAIST]) * kd_scale
        self.q_init = None

    @staticmethod
    def _gain(name, table):
        for key in ("WAIST", "KNEE", "ANKLE", "HIP"):
            if key in name or (key == "WAIST" and "TORSO" in name):
                return table[key]
        return table["HIP"]

    def reset_standing(self):
        cfg = self.cfg
        feet = [(np.array([0.0, cfg.FOOT_Y]), 0.0, 0.0), (np.array([0.0, -cfg.FOOT_Y]), 0.0, 0.0)]
        r = self.wb.solve(self.wb.q0, np.array([0.0, 0.0, cfg.Z_NOM]), feet, 0.0)
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:] = r["q"]
        # Joints not in the IK keep their default angles.
        mujoco.mj_forward(self.m, self.d)
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        self.q_target = np.array([self.d.qpos[a] for a in self.qadr])

    def _measured_state(self):
        mujoco.mj_subtreeVel(self.m, self.d)
        com = self.d.subtree_com[0].copy()
        vcom = self.d.subtree_linvel[0].copy()
        R = self.d.xmat[1].reshape(3, 3)
        psi = float(np.arctan2(R[1, 0], R[0, 0]))
        x = np.zeros(8)
        x[0:3] = com
        x[3:6] = vcom
        x[6] = psi
        x[7] = float(self.d.qvel[5])
        return x

    def tick(self, t, v, w, gait="walk"):
        """NMPC + whole-body IK for the next interval. Returns (ok, IK residuals)."""
        x = self._measured_state()
        u, ok, _ = self.ctrl.solve(t, x, v, w, gait)
        # Predicted next state from the NMPC: first solved state (warm-start stored).
        Xp, _ = self.ctrl._prev
        x_next = Xp[:, 1]
        feet = []
        t_next = t + self.cfg.dt
        for foot in (0, 1):
            xy, z, psi_f = swing_pose(self.cfg, t_next, v, w, foot)
            feet.append((xy, z, psi_f))
        res = self.wb.solve(self.d.qpos.copy(), x_next[0:3], feet, x_next[6])
        q = res["q"]
        self.q_target = np.array([q[a] for a in self.wb.qadr])
        return ok, res

    def run(self, scn, T=None, verbose=False):
        cfg = self.cfg
        self.reset_standing()
        T = T or scn["T"]
        v, w = scn.get("v", 0.0), scn.get("w", 0.0)
        gait = "stand" if v == 0.0 and w == 0.0 else "walk"
        n_ticks = int(round(T / cfg.dt))
        sim_per_tick = int(round(cfg.dt / self.m.opt.timestep))
        q_prev = np.array([self.d.qpos[a] for a in self.qadr])
        fail = 0
        ik_res = []
        fell = False
        log = []
        for i in range(n_ticks):
            t = i * cfg.dt
            ok, res = self.tick(t, v, w, gait)
            fail += 0 if ok else 1
            ik_res.append(res["com_res"])
            q_start = np.array([self.d.qpos[a] for a in self.qadr])
            q_goal = self.q_target
            for s in range(sim_per_tick):
                alpha = (s + 1) / sim_per_tick
                q_des = q_start + alpha * (q_goal - q_start)
                qd_des = (q_goal - q_start) / cfg.dt
                roll, pitch = self._attitude()
                tau_task = self._balance_torque(q_des) if self.BAL else np.zeros(len(self.dofs))
                for k in range(len(self.dofs)):
                    qk = self.d.qpos[self.qadr[k]]
                    vk = self.d.qvel[self.dofs[k]]
                    tau = self.kp[k] * (q_des[k] - qk) + self.kd[k] * (qd_des[k] - vk)
                    name = LEG_WAIST[k]
                    if "ANKLE_PITCH" in name:
                        tau += -self.ATT_PITCH * pitch
                    if "ANKLE_ROLL" in name:
                        tau += -self.ATT_ROLL * roll
                    tau = tau + tau_task[k]
                    self.d.ctrl[self.act[k]] = float(np.clip(tau + self.d.qfrc_bias[self.dofs[k]],
                                                             -self.tlim[k], self.tlim[k]))
                mujoco.mj_step(self.m, self.d)
                if self.d.qpos[2] < 0.5 or abs(self._tilt()) > 0.6:
                    fell = True
                    break
            log.append(dict(t=round(t + cfg.dt, 3), com=[round(float(c), 4) for c in self.d.subtree_com[0]],
                            base_z=round(float(self.d.qpos[2]), 4)))
            if fell:
                break
        return dict(fell=fell, t_fall=(log[-1]["t"] if fell else None), solver_failures=fail,
                    com_res_max=float(np.max(ik_res)) if ik_res else None,
                    com_final=log[-1]["com"] if log else None, duration=log[-1]["t"] if log else 0.0,
                    log=log)

    def _balance_torque(self, q_des):
        """Torque on the actuated joints: CoM PD force through the CoM Jacobian, plus base attitude torque
        through the base rotation Jacobian. The base is unactuated, so only the actuated columns are used."""
        m, d = self.m, self.d
        jp = np.zeros((3, m.nv))
        mujoco.mj_jacSubtreeCom(m, d, jp, 1)
        jr = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, None, jr, 1)
        mujoco.mj_subtreeVel(m, d)
        com = d.subtree_com[0]
        vcom = d.subtree_linvel[0]
        # Horizontal CoM target: the stance centre (origin of the sole frame, x = y = 0).
        F = np.array([-self.KC * com[0] - self.DC * vcom[0],
                      -self.KC * com[1] - self.DC * vcom[1], 0.0])
        roll, pitch = self._attitude()
        omega = d.qvel[3:6]
        M = np.array([-self.KA * roll - self.DA * omega[0], -self.KA * pitch - self.DA * omega[1], 0.0])
        tau_full = jp[:, self.dofs].T @ F + jr[:, self.dofs].T @ M
        return tau_full

    def _attitude(self):
        R = self.d.xmat[1].reshape(3, 3)
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        pitch = float(np.arcsin(np.clip(-R[2, 0], -1, 1)))
        return roll, pitch

    def _tilt(self):
        R = self.d.xmat[1].reshape(3, 3)
        return float(np.arccos(np.clip(R[2, 2], -1, 1)))


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--v", type=float, default=0.0)
    p.add_argument("--w", type=float, default=0.0)
    p.add_argument("--T", type=float, default=3.0)
    p.add_argument("--kp", type=float, default=1.0)
    p.add_argument("--kd", type=float, default=1.0)
    p.add_argument("--out", type=str, default="")
    a = p.parse_args(argv)
    fw = FullWalk(kp_scale=a.kp, kd_scale=a.kd)
    r = fw.run(dict(v=a.v, w=a.w, T=a.T))
    summary = {k: v for k, v in r.items() if k != "log"}
    print(json.dumps(summary, indent=1))
    if a.out:
        Path(a.out).write_text(json.dumps(r, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
