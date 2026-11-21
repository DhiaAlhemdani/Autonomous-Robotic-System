"""Floating-base whole-body torque QP (docs/18 §10.2). Simulation only.

Per call, for the current MuJoCo state, with an active-contact mask per foot:
  unknowns  x = [qdd (nv), tau (na), f (3 per sole corner, 4 corners per foot)]
  equality  M qdd + qfrc_bias = S tau + Jc^T f            floating-base dynamics with contact forces
  equality  Jc_active qdd = 0                              active contact points do not accelerate (Jdot qd ignored)
  equality  f_inactive = 0                                 swing foot carries no force
  bounds    |tau_i| <= torque limit; |fx|,|fy| <= MU fz; fz >= 0
  cost      weighted least squares: CoM acceleration, base attitude/yaw acceleration, swing-foot
            position acceleration, joint posture acceleration, small regularisation on tau and f.
Solved with OSQP (pinned in the repo's README).
"""
import mujoco
import numpy as np
import osqp
from scipy import sparse

N_CORNERS = 4


class WBCQP:
    MU = 0.6

    def __init__(self, m, feet_bodies, sole_local, act_map, tlim):
        """sole_local: per foot, (4,3) corner offsets in the foot body frame. act_map: [(dof, actuator)]."""
        self.m = m
        self.feet = list(feet_bodies)
        self.sole_local = sole_local
        self.act_dofs = [int(a[0]) for a in act_map]
        self.act_ids = [int(a[1]) for a in act_map]
        self.tlim = np.asarray(tlim, float)
        self.nv = m.nv
        self.na = len(self.act_dofs)
        self.nfeet = len(self.feet)
        self.npt = N_CORNERS * self.nfeet
        self.nf = 3 * self.npt
        self.nx = self.nv + self.na + self.nf
        self.S = np.zeros((self.nv, self.na))
        for i, dof in enumerate(self.act_dofs):
            self.S[dof, i] = 1.0
        self.x_prev = None
        self.stats = {}

    def _corner_jacobians(self, d):
        m = self.m
        Jc = np.zeros((self.npt, 3, self.nv))
        pts = np.zeros((self.npt, 3))
        for fi, fb in enumerate(self.feet):
            R = d.xmat[fb].reshape(3, 3)
            for ci, c in enumerate(self.sole_local[fi]):
                k = fi * N_CORNERS + ci
                p = d.xpos[fb] + R @ c
                mujoco.mj_jac(m, d, Jc[k], None, p, fb)
                pts[k] = p
        return Jc, pts

    def solve(self, d, targets):
        """targets: 'active' (bool per foot), 'com_acc' (3,), 'att_acc' (3,), 'post_acc' (na,),
        'foot_acc' (list of (foot index, (3,) desired origin acceleration)), 'w' weights."""
        m, nv, na, nf, nx = self.m, self.nv, self.na, self.nf, self.nx
        M = np.zeros((nv, nv))
        mujoco.mj_fullM(m, M, d.qM)
        bias = d.qfrc_bias.copy()
        Jcc, _ = self._corner_jacobians(d)
        Jc = Jcc.reshape(self.npt * 3, nv)                           # rows: corner k, component c -> 3k+c
        active = np.repeat(np.asarray(targets["active"], bool), N_CORNERS * 3)   # per Jc row
        w = targets["w"]

        # Equality rows: dynamics (nv), active contact acceleration (sum active*3), inactive forces (sum inactive*3).
        n_act_rows = int(active.sum())
        n_inact = int((~active).sum())
        neq = nv + n_act_rows + n_inact
        Aeq = np.zeros((neq, nx))
        Aeq[:nv, :nv] = M
        Aeq[:nv, nv:nv + na] = -self.S
        Aeq[:nv, nv + na:] = -Jc.T
        beq = np.zeros(neq)
        beq[:nv] = -bias[:nv]
        Aeq[nv:nv + n_act_rows, :nv] = Jc[active]
        r = nv + n_act_rows
        for k in np.where(~active)[0]:
            Aeq[r, nv + na + k] = 1.0
            r += 1

        # Inequalities: friction per corner, fz >= 0, torque bounds.
        rows, cols, vals, ub = [], [], [], []
        rr = 0
        f0 = nv + na
        for p in range(self.npt):
            fx, fy, fz = f0 + 3 * p, f0 + 3 * p + 1, f0 + 3 * p + 2
            for sgn in (1, -1):
                rows += [rr, rr]; cols += [fx, fz]; vals += [sgn, -self.MU]; ub.append(0.0); rr += 1
                rows += [rr, rr]; cols += [fy, fz]; vals += [sgn, -self.MU]; ub.append(0.0); rr += 1
            rows += [rr]; cols += [fz]; vals += [-1.0]; ub.append(0.0); rr += 1
        A_fr = sparse.csc_matrix((vals, (rows, cols)), shape=(rr, nx))
        A_t = sparse.lil_matrix((2 * na, nx))
        for i in range(na):
            A_t[i, nv + i] = 1.0
            A_t[na + i, nv + i] = -1.0
        A_in = sparse.vstack([A_fr, A_t.tocsc()]).tocsc()
        l_in = np.concatenate([-np.inf * np.ones(rr), -self.tlim, -self.tlim])
        u_in = np.concatenate([np.array(ub), self.tlim, self.tlim])

        # Cost: sum_i w_i ||J_i x - a_i||^2.
        P = np.zeros((nx, nx))
        q = np.zeros(nx)

        def add(J, a, weight):
            nonlocal P, q
            P += weight * J.T @ J
            q += -weight * J.T @ a

        mujoco.mj_subtreeVel(m, d)
        Jcom = np.zeros((3, nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, 1)
        Jw = np.zeros((3, nv)); mujoco.mj_jacBody(m, d, None, Jw, 1)
        Jcom_x = np.zeros((3, nx)); Jcom_x[:, :nv] = Jcom
        add(Jcom_x, targets["com_acc"], w["com"])
        Jw_x = np.zeros((3, nx)); Jw_x[:, :nv] = Jw
        add(Jw_x, targets["att_acc"], w["att"])
        Jp_x = np.zeros((na, nx)); Jp_x[:, self.act_dofs] = np.eye(na)
        add(Jp_x, targets["post_acc"], w["post"])
        for fi, a in targets.get("foot_acc", []):
            jp = np.zeros((3, nv)); mujoco.mj_jacBody(m, d, jp, None, self.feet[fi])
            Jf_x = np.zeros((3, nx)); Jf_x[:, :nv] = jp
            add(Jf_x, a, w["foot"])
        Jt_x = np.zeros((na, nx)); Jt_x[:, nv:nv + na] = np.eye(na)
        add(Jt_x, np.zeros(na), w["tau"])
        Jf_x = np.zeros((nf, nx)); Jf_x[:, nv + na:] = np.eye(nf)
        add(Jf_x, np.zeros(nf), w["force"])
        P += 1e-9 * np.eye(nx)

        A = sparse.vstack([sparse.csc_matrix(Aeq), A_in]).tocsc()
        l = np.concatenate([beq, l_in])
        u = np.concatenate([beq, u_in])
        prob = osqp.OSQP()
        prob.setup(sparse.csc_matrix(P), q, A, l, u, verbose=False, polish=False,
                   eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, warm_starting=True)
        if self.x_prev is not None and len(self.x_prev) == nx:
            prob.warm_start(x=self.x_prev)
        res = prob.solve()
        if res.x is None or not np.all(np.isfinite(res.x)):
            self.stats["failed"] = self.stats.get("failed", 0) + 1
            return np.zeros(na), "failed"
        self.x_prev = res.x
        st = "solved" if res.info.status in ("solved", "solved inaccurate") else res.info.status
        self.stats[st] = self.stats.get(st, 0) + 1
        return res.x[nv:nv + na].copy(), st
