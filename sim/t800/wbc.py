"""Torque-level whole-body controller (QP) for the free-base T800 in MuJoCo (simulation only).

Each control step:

1. Read the mass matrix M and bias forces h from MuJoCo.
2. Contact points: four corners per sole, at the sole bottom. A corner counts as in contact when
   its world height is below CONTACT_Z. Contact points are required to have zero acceleration.
3. Solve a QP over joint accelerations qdd and contact forces f (world frame, 3 per point):
     minimise  sum of task errors  (weighted least squares)
     subject to
       base rows of   M qdd + h = B tau + Jc^T f   (no torque on the 6 free-base dofs)
       contact:       Jc qdd = -Jdot qd          (points do not accelerate)
       friction:      fz >= 0,  |fx| <= MU fz,  |fy| <= MU fz  (linearised cone)
       torque limits: lo <= tau <= hi,   tau = M_a qdd + h_a - (Jc^T f)_a
4. Apply tau as the actuator control (the actuators are motors, gain 1).

Tasks: CoM x/y position and velocity; pelvis orientation; posture of the actuated joints.
If the QP fails, gravity compensation is applied and the failure is counted.

Every gain and weight is an ASSUMPTION for the sandbox. None is measured on hardware.
"""
import os
import sys

import numpy as np
import mujoco
from quadprog import solve_qp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import balance as B  # noqa: E402

MU = 1.0                     # friction coefficient (floor friction in the model is 1.0)
CONTACT_Z = 0.03             # m, a sole corner below this height is in contact (ASSUMPTION)
SOLE_X = (-0.110, 0.157)     # sole outline in the foot-origin frame (from the foot geoms)
SOLE_Y = (-0.053, 0.057)
SOLE_BOTTOM_Z = -0.064       # sole bottom in the foot-origin frame
FOOT = {"L": "LINK_FOOT_L", "R": "LINK_FOOT_R"}
CORNERS = [(SOLE_X[0], SOLE_Y[0]), (SOLE_X[0], SOLE_Y[1]), (SOLE_X[1], SOLE_Y[0]), (SOLE_X[1], SOLE_Y[1])]

# Task gains and weights (ASSUMPTION)
KP_COM, KD_COM, W_COM = 60.0, 12.0, 10.0
KP_ORI, KD_ORI, W_ORI = 80.0, 14.0, 5.0
KP_POST, KD_POST, W_POST = 30.0, 6.0, 1.0
KP_Z, KD_Z, W_Z = 60.0, 14.0, 10.0   # CoM height task (ASSUMPTION)
KP_FOOT, KD_FOOT, W_FOOT = 80.0, 14.0, 8.0  # foot flat task (ASSUMPTION)
Z_DES_OFFSET = None                 # set from the start pose by the caller
W_REG = 1e-5


class WBC:
    def __init__(self, robot):
        self.r = robot
        self.m, self.d = robot.m, robot.d
        self.body = {k: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, v) for k, v in FOOT.items()}
        self.nv = self.m.nv
        self.nu = self.m.nu
        self.act_dof = np.array([int(self.m.jnt_dofadr[self.m.actuator_trnid[a, 0]]) for a in range(self.nu)])
        self.failures = 0
        self.last_info = {}
        self.z_des = float(self.d.subtree_com[1][2])  # CoM height at start (set before stepping)
        self.foot_rigid = True  # True: feet fixed in orientation (needs flat start); False: foot-flat task

    # ---- kinematics --------------------------------------------------------
    def _corner_points(self):
        """World positions of all sole corners, with the foot that owns each one."""
        pts, owner = [], []
        for k, b in self.body.items():
            R = self.d.xmat[b].reshape(3, 3)
            o = self.d.xpos[b]
            for (x, y) in CORNERS:
                p = o + R @ np.array([x, y, SOLE_BOTTOM_Z])
                pts.append(p)
                owner.append(k)
        return np.array(pts), owner

    def _point_jac(self, pts, owner):
        J = np.zeros((3 * len(pts), self.nv))
        for i, (p, k) in enumerate(zip(pts, owner)):
            jp = np.zeros((3, self.nv))
            mujoco.mj_jac(self.m, self.d, jp, None, p, self.body[k])
            J[3 * i:3 * i + 3] = jp
        return J

    def _jdot_qd_all(self, Jc_full):
        """Jdot * qd for all 8 corners, by a forward difference of J along the current velocity."""
        h = 1e-5
        qn = self.d.qpos.copy()
        mujoco.mj_integratePos(self.m, qn, self.d.qvel, h)
        save = self.d.qpos.copy()
        self.d.qpos[:] = qn
        mujoco.mj_kinematics(self.m, self.d)
        pts2, owner2 = self._corner_points()
        Jc2 = self._point_jac(pts2, owner2)
        self.d.qpos[:] = save
        mujoco.mj_kinematics(self.m, self.d)
        return (Jc2 - Jc_full) @ self.d.qvel / h

    def _foot_rigid(self, k):
        """6 x nv Jacobian of the foot origin (3 linear, 3 angular) and its Jdot*qd by finite difference."""
        b = self.body[k]
        def jac_pair():
            jp = np.zeros((3, self.nv)); jr = np.zeros((3, self.nv))
            mujoco.mj_jacBody(self.m, self.d, jp, jr, b)
            return np.vstack([jp, jr])
        J = jac_pair()
        h = 1e-5
        qn = self.d.qpos.copy()
        mujoco.mj_integratePos(self.m, qn, self.d.qvel, h)
        save = self.d.qpos.copy()
        self.d.qpos[:] = qn
        mujoco.mj_kinematics(self.m, self.d)
        J2 = jac_pair()
        self.d.qpos[:] = save
        mujoco.mj_kinematics(self.m, self.d)
        return J, (J2 - J) @ self.d.qvel / h

    def com_jac(self):
        jp = np.zeros((3, self.nv))
        mujoco.mj_jacSubtreeCom(self.m, self.d, jp, 1)
        return jp

    # ---- one control step -------------------------------------------------
    def step(self, com_des_xy, feet_contact=None):
        m, d = self.m, self.d
        mujoco.mj_forward(m, d)
        M = np.zeros((self.nv, self.nv))
        mujoco.mj_fullM(m, M, d.qM)
        h = d.qfrc_bias.copy()

        pts, owner = self._corner_points()
        in_contact = np.array([p[2] < CONTACT_Z for p in pts])
        if feet_contact is not None:
            in_contact &= np.array([feet_contact[k] for k in owner])
        idx = np.where(in_contact)[0]
        n_pts = len(idx)
        Jc_full = self._point_jac(pts, owner)
        Jc = np.vstack([Jc_full[3 * i:3 * i + 3] for i in idx]) if n_pts else np.zeros((0, self.nv))
        owner_c = [owner[i] for i in idx]
        pts_c = pts[idx] if n_pts else np.zeros((0, 3))
        jdq_all = self._jdot_qd_all(Jc_full)
        jdq = np.concatenate([jdq_all[3 * i:3 * i + 3] for i in idx]) if n_pts else np.zeros(0)

        nq = self.nv + 3 * n_pts
        # Decision vector z = [qdd (nv), f (3 per contact point)]
        Jcom = self.com_jac()
        com = d.subtree_com[1].copy()
        comd = Jcom @ d.qvel
        # Jdot for CoM by the same finite-difference scheme
        h_fd = 1e-5
        qn = d.qpos.copy()
        mujoco.mj_integratePos(m, qn, d.qvel, h_fd)
        save = d.qpos.copy()
        d.qpos[:] = qn
        mujoco.mj_kinematics(m, d)
        Jcom2 = self.com_jac()
        d.qpos[:] = save
        mujoco.mj_kinematics(m, d)
        jcom_dqd = (Jcom2 - Jcom) @ d.qvel / h_fd

        # Equalities: base dynamics (6 rows) and contact acceleration (3 per point)
        JcT = Jc.T if n_pts else np.zeros((self.nv, 0))  # nv x 3n
        # base rows: M_b qdd - (Jc^T f)_b = -h_b   (no actuator torque on the free base)
        Ab = np.zeros((6, nq))
        Ab[:, :self.nv] = M[0:6]
        Ab[:, self.nv:] = -JcT[0:6]
        rows_eq = [Ab]
        rhs_eq = [-h[0:6]]
        # contact feet: rigid-body acceleration rows per foot (6 each), not per corner.
        # Four corners of one sole are rank 6 together, so per-corner rows make the system inconsistent.
        feet_in = sorted({owner[i] for i in idx})
        foot_ang = {}
        for k in feet_in:
            Jf, jdf = self._foot_rigid(k)
            if self.foot_rigid:
                # rigid foot: origin and orientation do not accelerate (requires flat feet at start)
                Af = np.zeros((6, nq))
                Af[:, :self.nv] = Jf
                rows_eq.append(Af)
                rhs_eq.append(-jdf)
            else:
                Af = np.zeros((3, nq))
                Af[:, :self.nv] = Jf[0:3]      # origin does not accelerate
                rows_eq.append(Af)
                rhs_eq.append(-jdf[0:3])
                foot_ang[k] = (Jf[3:6], jdf[3:6])  # orientation is a soft task
        Aeq = np.vstack(rows_eq)
        beq = np.concatenate(rhs_eq)

        # Inequalities G z >= g: torque limits and friction cone
        G_rows, g_rows = [], []
        # tau_a = M_a qdd + h_a - (Jc^T f)_a
        Ma = M[6:, :]
        ha = h[6:]
        Ta = np.zeros((self.nu, nq))
        Ta[:, :self.nv] = Ma
        Ta[:, self.nv:] = -JcT[6:]
        lo, hi = self.r.lo, self.r.hi
        G_rows.append(Ta)
        g_rows.append(lo - ha)
        G_rows.append(-Ta)
        g_rows.append(-(hi - ha))
        # friction cone per point: fz >= 0, mu*fz -/+ fx >= 0, mu*fz -/+ fy >= 0
        for i in range(n_pts):
            c0 = self.nv + 3 * i
            F = np.zeros((5, nq))
            F[0, c0 + 2] = 1.0
            F[1, c0 + 2] = MU
            F[1, c0 + 0] = -1.0
            F[2, c0 + 2] = MU
            F[2, c0 + 0] = 1.0
            F[3, c0 + 2] = MU
            F[3, c0 + 1] = -1.0
            F[4, c0 + 2] = MU
            F[4, c0 + 1] = 1.0
            G_rows.append(F)
            g_rows.append(np.zeros(5))
        G = np.vstack(G_rows)
        g = np.concatenate(g_rows)

        # Tasks, least squares ||A z - b||^2
        tasks_A, tasks_b, tasks_w = [], [], []
        # CoM x,y: jcom qdd = kp (des - com) - kd comd
        A = np.zeros((2, nq))
        A[:, :self.nv] = Jcom[0:2]
        b = KP_COM * (np.asarray(com_des_xy) - com[0:2]) - KD_COM * comd[0:2] - jcom_dqd[0:2]
        tasks_A.append(A); tasks_b.append(b); tasks_w.append(np.full(2, W_COM))
        # Foot orientation: angular acceleration towards a flat foot (sole normal = world z)
        for k in ([] if self.foot_rigid else feet_in):
            Jr, jdr = foot_ang[k]
            R = d.xmat[self.body[k]].reshape(3, 3)
            zf = R[:, 2]
            e_flat = np.cross(zf, np.array([0.0, 0.0, 1.0]))
            omega = Jr @ d.qvel
            A = np.zeros((3, nq))
            A[:, :self.nv] = Jr
            b = KP_FOOT * e_flat - KD_FOOT * omega - jdr
            tasks_A.append(A); tasks_b.append(b); tasks_w.append(np.full(3, W_FOOT))
        # CoM height: hold the start height so the legs cannot collapse
        A = np.zeros((1, nq))
        A[0, :self.nv] = Jcom[2]
        b = np.array([KP_Z * (self.z_des - com[2]) - KD_Z * comd[2] - jcom_dqd[2]])
        tasks_A.append(A); tasks_b.append(b); tasks_w.append(np.array([W_Z]))
        # Pelvis orientation: angular acceleration (body dofs 3..5) towards upright
        qw, qx, qy, qz = d.qpos[3:7]
        rot_err = 2.0 * np.sign(qw if qw != 0 else 1.0) * np.array([qx, qy, qz])
        A = np.zeros((3, nq))
        A[:, 3:6] = np.eye(3)
        b = -KP_ORI * rot_err - KD_ORI * d.qvel[3:6]
        tasks_A.append(A); tasks_b.append(b); tasks_w.append(np.full(3, W_ORI))
        # Posture on actuated joints
        A = np.zeros((self.nu, nq))
        A[:, self.act_dof] = np.eye(self.nu)
        q_act = d.qpos[self.r.act_q]
        qd_act = d.qvel[self.act_dof]
        b = -KP_POST * (q_act - self.r.target) - KD_POST * qd_act
        tasks_A.append(A); tasks_b.append(b); tasks_w.append(np.full(self.nu, W_POST))

        TA = np.vstack(tasks_A)
        Tb = np.concatenate(tasks_b)
        Tw = np.concatenate(tasks_w)
        W = Tw
        H = (TA.T * W) @ TA + W_REG * np.eye(nq)
        c = -(TA.T * W) @ Tb
        # quadprog: min 0.5 z'Hz + c'z  s.t.  Aeq z = beq (equalities first), G z >= g
        Amat = np.vstack([Aeq, G]).T
        bvec = np.concatenate([beq, g])
        # make H positive definite (small ridge)
        H = H + 1e-9 * np.eye(nq)
        try:
            sol = solve_qp(H, -c, Amat, bvec, meq=Aeq.shape[0])[0]
            ok = True
        except Exception:
            sol = None
            ok = False
        if ok:
            qdd = sol[:self.nv]
            f = sol[self.nv:]
            tau = Ma @ qdd + ha - JcT[6:] @ f
        else:
            self.failures += 1
            qdd = np.zeros(self.nv)
            tau = ha.copy()  # gravity-compensation fallback
        self.last_info = dict(ok=ok, n_contacts=n_pts, com=com[:2].copy())
        self.last_qdd = qdd.copy()
        self.last_f = (f.copy() if ok else None, pts_c.copy() if n_pts else None)
        d.ctrl[:] = np.clip(tau, lo, hi)
        mujoco.mj_step(m, d)
        return ok
