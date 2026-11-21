"""Kinematic whole-body layer on vendor/t800 (docs/18 §5). Damped least squares on MuJoCo Jacobians.

Free base + 12 leg joints + waist yaw. Targets: whole-body CoM, both sole positions (via foot origins),
base yaw, base roll/pitch regularised to zero. Joint limits are enforced by clamping. No torques, no
collision checks, feet orientation not constrained.
"""
import glob
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LEG_WAIST = ["J00_HIP_PITCH_L", "J01_HIP_ROLL_L", "J02_HIP_YAW_L", "J03_KNEE_PITCH_L",
             "J04_ANKLE_PITCH_L", "J05_ANKLE_ROLL_L", "J06_HIP_PITCH_R", "J07_HIP_ROLL_R",
             "J08_HIP_YAW_R", "J09_KNEE_PITCH_R", "J10_ANKLE_PITCH_R", "J11_ANKLE_ROLL_R",
             "J12_TORSO_YAW"]


def load_model(root: Path = ROOT):
    xml = glob.glob(str(root / "vendor" / "t800" / "**" / "serial_t800.xml"), recursive=True)[0]
    return mujoco.MjModel.from_xml_path(xml)


class WholeBody:
    def __init__(self, model=None):
        self.m = model or load_model()
        m = self.m
        self.d = mujoco.MjData(m)
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "LINK_BASE")
        self.feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in ("LINK_FOOT_L", "LINK_FOOT_R")]
        self.jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEG_WAIST]
        self.dofs = list(range(6)) + [int(m.jnt_dofadr[j]) for j in self.jids]
        self.qadr = [int(m.jnt_qposadr[j]) for j in self.jids]
        self.lo = np.array([m.jnt_range[j][0] for j in self.jids])
        self.hi = np.array([m.jnt_range[j][1] for j in self.jids])
        self.q0 = m.qpos0.copy()
        self._measure_offsets()

    def _measure_offsets(self):
        """Foot-origin offset from the sole centre and sole height above the origin, from the default pose."""
        m, d = self.m, self.d
        d.qpos[:] = self.q0
        mujoco.mj_forward(m, d)
        self.origin_to_sole = []
        self.origin_to_sole_bottom = []
        for fb in self.feet:
            pts = []
            for g in range(m.ngeom):
                if m.geom_bodyid[g] == fb and m.geom_contype[g] + m.geom_conaffinity[g] > 0:
                    R = d.geom_xmat[g].reshape(3, 3)
                    s = m.geom_size[g]
                    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and d.geom_xpos[g][2] < 0.03:
                        for sx in (-1, 1):
                            for sy in (-1, 1):
                                for sz in (-1, 1):
                                    pts.append(d.geom_xpos[g] + R @ np.array([sx * s[0], sy * s[1], sz * s[2]]))
            P = np.array(pts)
            Ro = d.xmat[fb].reshape(3, 3)
            centre = np.array([(P[:, 0].min() + P[:, 0].max()) / 2, (P[:, 1].min() + P[:, 1].max()) / 2, 0.0])
            centre[2] = P[:, 2].min()
            self.origin_to_sole.append(Ro.T @ (centre - d.xpos[fb]))
            self.origin_to_sole_bottom.append(float(d.xpos[fb][2] - P[:, 2].min()))

    def _yaw(self):
        R = self.d.xmat[self.base].reshape(3, 3)
        return float(np.arctan2(R[1, 0], R[0, 0]))

    # Posture task: bent-knee nominal leg posture (flat feet: ankle = -hip). Weight is small; it only
    # selects among joint solutions, it does not change the CoM or foot targets.
    POSTURE_BEND = 0.25
    TASK_W = 10.0
    POSTURE_W = 0.05

    def _posture(self):
        q = np.zeros(len(LEG_WAIST))
        for i, name in enumerate(LEG_WAIST):
            if "HIP_PITCH" in name:
                q[i] = -self.POSTURE_BEND
            elif "KNEE" in name:
                q[i] = 2.0 * self.POSTURE_BEND
            elif "ANKLE_PITCH" in name:
                q[i] = -self.POSTURE_BEND
        return q

    def solve(self, q_init, com, feet, psi, iters=200, tol=1e-5, lam=1e-2, max_step=0.2):
        """com: (3,) world. feet: [(sole_xy, sole_z, yaw), ...] for L and R. psi: base yaw. Returns result dict."""
        m, d = self.m, self.d
        d.qpos[:] = q_init
        mujoco.mj_forward(m, d)
        nv = m.nv
        clamps = 0
        for _ in range(iters):
            J_rows, e_rows = [], []
            jp = np.zeros((3, nv))
            mujoco.mj_jacSubtreeCom(m, d, jp, self.base)
            J_rows.append(jp)
            e_rows.append(com - d.subtree_com[0])
            for i, fb in enumerate(self.feet):
                sole_xy_z, sole_z, yaw_f = feet[i]
                Rf = np.array([[np.cos(yaw_f), -np.sin(yaw_f), 0], [np.sin(yaw_f), np.cos(yaw_f), 0], [0, 0, 1]])
                origin_target = np.array([sole_xy_z[0], sole_xy_z[1], sole_z]) - Rf @ self.origin_to_sole[i]
                origin_target[2] = sole_z + self.origin_to_sole_bottom[i]
                jp = np.zeros((3, nv))
                mujoco.mj_jacBody(m, d, jp, None, fb)
                J_rows.append(jp)
                e_rows.append(origin_target - d.xpos[fb])
            # Flat soles: foot z-axis driven to world z (roll and pitch of each foot).
            for fb in self.feet:
                jrf = np.zeros((3, nv))
                mujoco.mj_jacBody(m, d, None, jrf, fb)
                Rf_ = d.xmat[fb].reshape(3, 3)
                J_rows.append(np.vstack([jrf[1], -jrf[0]]))
                e_rows.append(-np.array([Rf_[0, 2], Rf_[1, 2]]))
            jr = np.zeros((3, nv))
            mujoco.mj_jacBody(m, d, None, jr, self.base)
            J_rows.append(jr[2:3])
            dpsi = float(np.arctan2(np.sin(psi - self._yaw()), np.cos(psi - self._yaw())))
            e_rows.append(np.array([dpsi]))
            # Base tilt: z-axis components (R02, R12) are driven to zero; d(R02)/dt = w_y, d(R12)/dt = -w_x.
            R = d.xmat[self.base].reshape(3, 3)
            tilt = np.array([R[0, 2], R[1, 2]])
            J_rows.append(np.vstack([jr[1], -jr[0]]))
            e_rows.append(-tilt)
            J = np.vstack([r for r in J_rows if r.size])
            e = np.concatenate([r for r in e_rows if r.size])
            Ja = J[:, self.dofs]
            # Task rows (CoM, feet, yaw, tilt) carry weight TASK_W; the posture row is weighted 1.
            Ja = self.TASK_W * Ja
            e = self.TASK_W * e
            qp = self._posture()
            qcur = np.array([d.qpos[qa] for qa in self.qadr])
            Pj = np.zeros((len(self.dofs) - 6, len(self.dofs)))
            Pj[np.arange(len(self.dofs) - 6), np.arange(6, len(self.dofs))] = 1.0
            Ja = np.vstack([Ja, self.POSTURE_W * Pj])
            e = np.concatenate([e, self.POSTURE_W * (qp - qcur)])
            if np.linalg.norm(e) < tol:
                break
            A = Ja.T @ Ja + lam * np.eye(len(self.dofs))
            dq = np.linalg.solve(A, Ja.T @ e)
            nrm = np.linalg.norm(dq)
            if nrm > max_step:
                dq *= max_step / nrm
            dqf = np.zeros(nv)
            dqf[self.dofs] = dq
            mujoco.mj_integratePos(m, d.qpos, dqf, 1.0)
            for qa, lo, hi in zip(self.qadr, self.lo, self.hi):
                if d.qpos[qa] < lo or d.qpos[qa] > hi:
                    clamps += 1
                    d.qpos[qa] = np.clip(d.qpos[qa], lo, hi)
            mujoco.mj_forward(m, d)
        q = d.qpos.copy()
        mujoco.mj_forward(m, d)
        feet_res = []
        for i, fb in enumerate(self.feet):
            sole_xy_z, sole_z, yaw_f = feet[i]
            Rf = np.array([[np.cos(yaw_f), -np.sin(yaw_f), 0], [np.sin(yaw_f), np.cos(yaw_f), 0], [0, 0, 1]])
            tgt = np.array([sole_xy_z[0], sole_xy_z[1], sole_z]) - Rf @ self.origin_to_sole[i]
            tgt[2] = sole_z + self.origin_to_sole_bottom[i]
            feet_res.append(float(np.linalg.norm(tgt - d.xpos[fb])))
        return dict(q=q, com_res=float(np.linalg.norm(com - d.subtree_com[0])),
                    feet_res=feet_res, yaw_res=float(abs(np.arctan2(np.sin(psi - self._yaw()), np.cos(psi - self._yaw())))),
                    clamps=clamps)
