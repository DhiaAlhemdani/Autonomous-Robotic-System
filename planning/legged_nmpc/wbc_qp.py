"""Floating-base whole-body torque QP (docs/18 §10.2, §10.14). Simulation only.

Per call, for the current MuJoCo state, with an active-contact mask per foot:
  unknowns  x = [qdd (nv), tau (na), f (3 per sole corner, 4 corners per foot)]
  equality  M qdd + qfrc_bias = S tau + Jc^T f            floating-base dynamics with contact forces
  equality  Jc_active qdd = -Jdot_active qd                active contact points do not accelerate
  equality  f_inactive = 0                                 swing foot carries no force
  bounds    |tau_i| <= torque limit; |fx|,|fy| <= MU fz; fz >= 0

Tasks (acceleration targets): base attitude (roll, pitch, yaw), swing-foot position, CoM, joint posture.
Two modes:
  "weighted"  one weighted least-squares cost over all tasks (docs/18 §10.2).
  "priority"  strict priority: tasks are solved in the given order; the achieved value of each solved task
              is fixed as an equality for the lower-priority tasks (docs/18 §10.14).
Solved with OSQP (pinned in the repo's README).
"""
import mujoco
import numpy as np
import osqp
from scipy import sparse

N_CORNERS = 4
DEFAULT_ORDER = ("att", "foot", "com", "post")
TASK_TOL = 1e-3   # band (m/s^2, rad/s^2) around each solved task value for lower priorities


class WBCQP:
    MU = 0.6

    def __init__(self, m, feet_bodies, sole_local, act_map, tlim, mode="weighted", order=DEFAULT_ORDER):
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
        self.mode = mode
        self.order = tuple(order)
        self.x_prev = None
        self.stats = {}

    def _corner_jacobians(self, d):
        m = self.m
        Jc = np.zeros((self.npt, 3, self.nv))
        pts = np.zeros((self.npt, 3))
        self.Jdq_corner = np.zeros((self.npt, 3))
        for fi, fb in enumerate(self.feet):
            R = d.xmat[fb].reshape(3, 3)
            for ci, c in enumerate(self.sole_local[fi]):
                k = fi * N_CORNERS + ci
                p = d.xpos[fb] + R @ c
                mujoco.mj_jac(m, d, Jc[k], None, p, fb)
                Jd = np.zeros((3, self.nv))
                mujoco.mj_jacDot(m, d, Jd, None, p, fb)
                self.Jdq_corner[k] = Jd @ d.qvel
                pts[k] = p
        return Jc, pts

    def _body_jdq(self, d, body, point=None):
        """(Jdot qd) of a body's point (translational) and rotational part, as in mj_jacDot."""
        p = d.xpos[body] if point is None else point
        jp = np.zeros((3, self.nv)); jr = np.zeros((3, self.nv))
        mujoco.mj_jacDot(self.m, d, jp, jr, p, body)
        return jp @ d.qvel, jr @ d.qvel

    def _constraints(self, d, targets):
        """Equality (dynamics, contact) and inequality (friction, torque) blocks."""
        m, nv, na, nf, nx = self.m, self.nv, self.na, self.nf, self.nx
        M = np.zeros((nv, nv))
        mujoco.mj_fullM(m, M, d.qM)
        bias = d.qfrc_bias.copy()
        Jcc, _ = self._corner_jacobians(d)
        Jc = Jcc.reshape(self.npt * 3, nv)
        active = np.repeat(np.asarray(targets["active"], bool), N_CORNERS * 3)
        n_act_rows = int(active.sum())
        n_inact = int((~active).sum())
        neq = nv + n_act_rows + n_inact
        Aeq = np.zeros((neq, nx))
        Aeq[:nv, :nv] = M
        Aeq[:nv, nv:nv + na] = -self.S
        Aeq[:nv, nv + na:] = -Jc.T
        beq = np.zeros(neq)
        beq[:nv] = -bias[:nv]
        beq[nv:nv + n_act_rows] = -self.Jdq_corner.reshape(-1)[active]
        Aeq[nv:nv + n_act_rows, :nv] = Jc[active]
        r = nv + n_act_rows
        for k in np.where(~active)[0]:
            Aeq[r, nv + na + k] = 1.0
            r += 1
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
        return Aeq, beq, A_in, l_in, u_in

    def _tasks(self, d, targets):
        """Task Jacobians (in x space) and accelerations: name -> (J (k,nx), a (k,))."""
        m, nv, na, nf, nx = self.m, self.nv, self.na, self.nf, self.nx
        T = {}
        Jcom = np.zeros((3, nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, 1)
        Jc_x = np.zeros((3, nx)); Jc_x[:, :nv] = Jcom
        T["com"] = (Jc_x, np.asarray(targets["com_acc"], float))
        Jw = np.zeros((3, nv)); mujoco.mj_jacBody(m, d, None, Jw, 1)
        Jw_x = np.zeros((3, nx)); Jw_x[:, :nv] = Jw
        _, jdq_w = self._body_jdq(d, 1)
        T["att"] = (Jw_x, np.asarray(targets["att_acc"], float) - jdq_w)
        Jp_x = np.zeros((na, nx)); Jp_x[:, self.act_dofs] = np.eye(na)
        T["post"] = (Jp_x, np.asarray(targets["post_acc"], float))
        Js, As = [], []
        for fi, a in targets.get("foot_acc", []):
            jp = np.zeros((3, nv)); mujoco.mj_jacBody(m, d, jp, None, self.feet[fi])
            Jf_x = np.zeros((3, nx)); Jf_x[:, :nv] = jp
            jdq_f, _ = self._body_jdq(d, self.feet[fi])
            Js.append(Jf_x); As.append(np.asarray(a, float) - jdq_f)
        if Js:
            T["foot"] = (np.vstack(Js), np.concatenate(As))
        return T

    def _cost(self, d, targets, names):
        """Weighted least squares over the named tasks plus regularisation (weights from targets['w'])."""
        nx = self.nx
        w = targets["w"]
        T = self._tasks(d, targets)
        P = np.zeros((nx, nx)); q = np.zeros(nx)
        wmap = {"com": w["com"], "att": w["att"], "post": w["post"], "foot": w["foot"]}
        for nm in names:
            if nm not in T:
                continue
            J, a = T[nm]
            P += wmap[nm] * J.T @ J
            q += -wmap[nm] * J.T @ a
        Jt_x = np.zeros((self.na, nx)); Jt_x[:, self.nv:self.nv + self.na] = np.eye(self.na)
        P += w["tau"] * Jt_x.T @ Jt_x
        Jf_x = np.zeros((self.nf, nx)); Jf_x[:, self.nv + self.na:] = np.eye(self.nf)
        P += w["force"] * Jf_x.T @ Jf_x
        P += 1e-9 * np.eye(nx)
        return P, q, T

    def _qp(self, P, q, A, l, u, warm):
        prob = osqp.OSQP()
        prob.setup(sparse.csc_matrix(P), q, A, l, u, verbose=False, polish=False,
                   eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, warm_starting=True)
        if warm is not None and len(warm) == self.nx:
            prob.warm_start(x=warm)
        res = prob.solve()
        if res.x is None or not np.all(np.isfinite(res.x)) or res.info.status not in ("solved", "solved inaccurate"):
            return None, res.info.status
        return res.x, res.info.status

    def solve(self, d, targets):
        """targets: 'active' (bool per foot), 'com_acc' (3,), 'att_acc' (3,), 'post_acc' (na,),
        'foot_acc' (list of (foot index, (3,) desired origin acceleration)), 'w' weights."""
        Aeq, beq, A_in, l_in, u_in = self._constraints(d, targets)
        if self.mode == "weighted":
            names = ("com", "att", "post", "foot")
            P, q, _ = self._cost(d, targets, names)
            A = sparse.vstack([sparse.csc_matrix(Aeq), A_in]).tocsc()
            l = np.concatenate([beq, l_in]); u = np.concatenate([beq, u_in])
            x, st = self._qp(P, q, A, l, u, self.x_prev)
        else:
            x, st = self._solve_priority(d, targets, Aeq, beq, A_in, l_in, u_in)
            if x is None:
                # Priority levels conflict with the contact model on this tick: fall back to the weighted solve.
                self.stats["priority_fallback"] = self.stats.get("priority_fallback", 0) + 1
                P, q, _ = self._cost(d, targets, ("com", "att", "post", "foot"))
                A = sparse.vstack([sparse.csc_matrix(Aeq), A_in]).tocsc()
                l = np.concatenate([beq, l_in]); u = np.concatenate([beq, u_in])
                x, st = self._qp(P, q, A, l, u, self.x_prev)
        if x is None:
            self.stats["failed"] = self.stats.get("failed", 0) + 1
            return np.zeros(self.na), "failed"
        self.x_prev = x
        st = "solved" if st in ("solved", "solved inaccurate") else st
        self.stats[st] = self.stats.get(st, 0) + 1
        return x[self.nv:self.nv + self.na].copy(), st

    def _solve_priority(self, d, targets, Aeq, beq, A_in, l_in, u_in):
        """Strict priority: each solved task's value is kept within a band (TASK_TOL) for lower levels."""
        fixA, fixv = [], []
        x = None
        st = "solved"

        def assemble(extra_P=None):
            if fixA:
                Af = np.vstack(fixA)
                vals = np.concatenate(fixv)
                A = sparse.vstack([sparse.csc_matrix(Aeq), sparse.csc_matrix(Af), A_in]).tocsc()
                l = np.concatenate([beq, vals - TASK_TOL, l_in])
                u = np.concatenate([beq, vals + TASK_TOL, u_in])
            else:
                A = sparse.vstack([sparse.csc_matrix(Aeq), A_in]).tocsc()
                l = np.concatenate([beq, l_in]); u = np.concatenate([beq, u_in])
            return A, l, u

        for nm in self.order:
            P, q, T = self._cost(d, targets, [nm])
            if nm not in T:
                continue
            A, l, u = assemble()
            x, st = self._qp(P, q, A, l, u, x)
            if x is None:
                return None, st
            J, _ = T[nm]
            fixA.append(J)
            fixv.append(J @ x)
        P, q, _ = self._cost(d, targets, [])
        A, l, u = assemble()
        x, st = self._qp(P, q, A, l, u, x)
        return x, st
