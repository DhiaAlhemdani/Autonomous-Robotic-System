"""Capture-point stepping for the free-base T800 in MuJoCo (simulation only).

Extends the Stage A standing law in balance.py with one stepping rule:

* Capture point. xi = c + cdot / omega, with c the whole-body CoM (x, y), cdot its velocity, and
  omega = sqrt(g / z_c) (linear inverted pendulum). omega uses z_c = CoM height above the stance
  foot, and g = 9.81 m/s^2.
* Trigger. When xi leaves the support outline (both soles in double support, one sole in single
  support), a step starts. The swing foot is the one on the side of the capture error.
* Swing. The swing foot follows a smoothstep path to the capture point, with a sinusoidal lift
  of STEP_H. Its joints follow damped least-squares IK on the foot origin.
* Stance. Stance legs use the balance.py ankle/hip law, referenced to the stance foot origin
  during single support.

Every tunable value here is an ASSUMPTION chosen for the sandbox. None is measured on hardware.

Usage:  .venv/bin/python sim/t800/stepper.py --selftest
"""
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import balance as B  # noqa: E402

G = 9.81
STEP_T = 0.40          # s, swing duration (ASSUMPTION)
STEP_H = 0.04          # m, swing foot lift (ASSUMPTION)
TRIG_MARGIN = 0.02     # m, capture point must be this far inside the outline to avoid a step (ASSUMPTION)
MAX_STEP_X = 0.25      # m, longest forward/back step from the stance foot (ASSUMPTION)
MAX_STEP_Y = 0.20      # m, widest lateral step from the stance foot (ASSUMPTION)
MIN_FOOT_SEP_Y = 0.16  # m, feet must not cross or touch (ASSUMPTION)
# Sole outline in the foot-origin frame (from the foot geoms in the model): x in [-0.11, 0.157],
# y in [-0.053, 0.057].
SOLE_X = (-0.110, 0.157)
SOLE_Y = (-0.053, 0.057)

LEG = {
    "L": dict(foot="LINK_FOOT_L", joints=["J00_HIP_PITCH_L", "J01_HIP_ROLL_L", "J02_HIP_YAW_L",
                                          "J03_KNEE_PITCH_L", "J04_ANKLE_PITCH_L", "J05_ANKLE_ROLL_L"]),
    "R": dict(foot="LINK_FOOT_R", joints=["J06_HIP_PITCH_R", "J07_HIP_ROLL_R", "J08_HIP_YAW_R",
                                          "J09_KNEE_PITCH_R", "J10_ANKLE_PITCH_R", "J11_ANKLE_ROLL_R"]),
}


class Stepper:
    """Balance law plus one capture-point step at a time. Owns a balance.Robot instance."""

    def __init__(self, robot, cfg=None, kp=600.0, kd=20.0):
        self.r = robot
        self.m, self.d = robot.m, robot.d
        self.cfg = dict(B.DEFAULT_CFG, **(cfg or {}))
        self.kp, self.kd = kp, kd
        self.dt = self.m.opt.timestep
        self.body = {k: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, v["foot"]) for k, v in LEG.items()}
        self.jid = {}
        self.dof = {}
        self.qadr = {}
        for k, v in LEG.items():
            self.jid[k] = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in v["joints"]]
            self.dof[k] = [int(self.m.jnt_dofadr[j]) for j in self.jid[k]]
            self.qadr[k] = [int(self.m.jnt_qposadr[j]) for j in self.jid[k]]
        self.jrange = {k: np.array([self.m.jnt_range[j] for j in self.jid[k]]) for k in LEG}
        self.swing = None      # dict when a step is in progress
        self.stance = "both"   # "both" or "L"/"R" (the stance foot in single support)
        self.ik_target = {k: None for k in LEG}  # joint targets for the swing leg
        self.steps_taken = 0
        self.prev_c = self.com()
        self.cd = np.zeros(2)

    # ---- state -------------------------------------------------------------
    def com(self):
        return self.d.subtree_com[1][:2].copy()

    def foot_origin(self, k):
        return self.d.xpos[self.body[k]].copy()

    def com_height_above_stance(self):
        z_feet = min(self.foot_origin(k)[2] for k in LEG)
        return float(self.d.subtree_com[1][2] - z_feet)

    def capture_point(self):
        c = self.com()
        self.cd = (c - self.prev_c) / (self.dt * 10)  # updated every 10 steps by step()
        omega = np.sqrt(G / max(0.3, self.com_height_above_stance()))
        return c + self.cd / omega

    def support_outline_contains(self, xi, feet, margin=None):
        """Axis-aligned box covering the given feet's soles (world xy), shrunk by margin."""
        if margin is None:
            margin = TRIG_MARGIN
        xs, ys = [], []
        for k in feet:
            o = self.foot_origin(k)
            xs += [o[0] + SOLE_X[0], o[0] + SOLE_X[1]]
            ys += [o[1] + SOLE_Y[0], o[1] + SOLE_Y[1]]
        return (min(xs) + margin <= xi[0] <= max(xs) - margin
                and min(ys) + margin <= xi[1] <= max(ys) - margin)

    # ---- IK ----------------------------------------------------------------
    def ik_step(self, k, target_xyz, iters=5, lam=0.05):
        """Damped least squares on the 6 leg joints so that the foot origin reaches target_xyz.

        Returns the joint targets (not applied to the model). The model is not changed.
        """
        d, m = self.d, self.m
        q_save = d.qpos.copy()
        dq_targets = None
        q = np.array([d.qpos[a] for a in self.qadr[k]], dtype=float)
        if self.ik_target[k] is not None:
            q = self.ik_target[k].copy()
        for _ in range(iters):
            # evaluate the foot with the trial joints written into qpos
            for i, a in enumerate(self.qadr[k]):
                d.qpos[a] = q[i]
            mujoco.mj_kinematics(m, d)
            x = d.xpos[self.body[k]].copy()
            err = target_xyz - x
            jacp = np.zeros((3, m.nv))
            mujoco.mj_jacBody(m, d, jacp, None, self.body[k])
            J = jacp[:, self.dof[k]]
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(3), err)
            q = q + dq
            q = np.clip(q, self.jrange[k][:, 0], self.jrange[k][:, 1])
        d.qpos[:] = q_save
        mujoco.mj_kinematics(m, d)
        self.ik_target[k] = q.copy()
        return q, float(np.linalg.norm(target_xyz - self._foot_at(k, q)))

    def _foot_at(self, k, q):
        d, m = self.d, self.m
        q_save = d.qpos.copy()
        for i, a in enumerate(self.qadr[k]):
            d.qpos[a] = q[i]
        mujoco.mj_kinematics(m, d)
        x = d.xpos[self.body[k]].copy()
        d.qpos[:] = q_save
        mujoco.mj_kinematics(m, d)
        return x

    # ---- control -----------------------------------------------------------
    def begin_step(self, k, target_origin_xy):
        p0 = self.foot_origin(k)
        self.swing = dict(leg=k, p0=p0, p1=np.array([target_origin_xy[0], target_origin_xy[1], p0[2]]),
                          t=0.0)
        self.ik_target[k] = np.array([self.d.qpos[a] for a in self.qadr[k]], dtype=float)
        self.stance = "L" if k == "R" else "R"

    def plan_step(self, xi):
        """Choose the swing leg and the foot target from the capture point."""
        fl, fr = self.foot_origin("L"), self.foot_origin("R")
        # side of the capture error decides the swing leg; forward error -> step with the rear foot
        if abs(xi[1] - 0.5 * (fl[1] + fr[1])) > 0.0 and abs(xi[1] - 0.5 * (fl[1] + fr[1])) > abs(xi[0] - 0.5 * (fl[0] + fr[0])):
            k = "L" if xi[1] > 0.5 * (fl[1] + fr[1]) else "R"
        else:
            k = "R" if fr[0] < fl[0] else "L"
        other = "R" if k == "L" else "L"
        stance = self.foot_origin(other)
        # foot origin target so that the sole centre lands on the capture point, clamped
        sole_c = np.array([0.5 * (SOLE_X[0] + SOLE_X[1]), 0.5 * (SOLE_Y[0] + SOLE_Y[1])])
        tgt = xi - sole_c
        dx = np.clip(tgt[0] - stance[0], -MAX_STEP_X, MAX_STEP_X)
        dy = np.clip(tgt[1] - stance[1], -MAX_STEP_Y, MAX_STEP_Y)
        # keep the feet apart on the correct side
        side = 1.0 if k == "L" else -1.0
        dy = max(dy, MIN_FOOT_SEP_Y * side) if side > 0 else min(dy, -MIN_FOOT_SEP_Y)
        return k, np.array([stance[0] + dx, stance[1] + dy])

    def step(self, ext_force=None):
        """One physics step: choose the stance/swing control, then advance the simulation."""
        d, m, dt = self.d, self.m, self.dt
        if self.steps_taken and self.swing is None and self.ik_target["L"] is None:
            pass
        # refresh velocity every 10 steps
        if int(round(d.time / dt)) % 10 == 0:
            c = self.com()
            self.cd = (c - self.prev_c) / (dt * 10)
            self.prev_c = c
        xi = self.capture_point()
        tgt = self.r.target.copy()

        if self.swing is None:
            both_ok = self.support_outline_contains(xi, ["L", "R"])
            if not both_ok:
                k, p1 = self.plan_step(xi)
                self.begin_step(k, p1)
                self.steps_taken += 1
        if self.swing is not None:
            s = self.swing
            s["t"] += dt
            u = min(1.0, s["t"] / STEP_T)
            su = u * u * (3 - 2 * u)
            p = s["p0"] + su * (s["p1"] - s["p0"])
            p[2] = s["p0"][2] + STEP_H * np.sin(np.pi * u)
            q, _ = self.ik_step(s["leg"], p)
            for i, n in enumerate(LEG[s["leg"]]["joints"]):
                tgt[self.r.idx[n]] = q[i]
            if u >= 1.0:
                self.swing = None
                self.stance = "both"

        # balance law on the stance legs (ankle + hip pitch), referenced to the CoM error
        if self.stance == "both":
            e = self.r.com_error()
        else:
            e = self.com() - self.foot_origin(self.stance)[:2]
        ev = self.cd if self.stance == "both" else self.cd
        cmd = {n: self.cfg["sign"] * (self.cfg["kc"] * e[0] + self.cfg["kv"] * ev[0]) for n in B.ANKLE_PITCH}
        hip = self.cfg["hip_sign"] * self.cfg["kh"] * self.r.pitch_rad()
        for n in B.HIP_PITCH:
            cmd[n] = hip
        for n, delta in cmd.items():
            tgt[self.r.idx[n]] = self.r.target[self.r.idx[n]] + delta
        # the swing leg keeps its IK targets (already written above); stance legs use tgt as well
        q = d.qpos[self.r.act_q]
        qd = d.qvel[self.r.act_dof]
        d.ctrl[:] = np.clip(self.kp * (tgt - q) - self.kd * qd, self.r.lo, self.r.hi)
        mujoco.mj_step(m, d)


def _selftest():
    r = B.Robot()
    st = Stepper(r)
    # 1. IK: move the left foot origin 5 cm forward and 2 cm up while the robot is held in place
    r.reset()
    p0 = st.foot_origin("L")
    target = p0 + np.array([0.05, 0.0, 0.02])
    q, err = st.ik_step("L", target, iters=40)
    print(f"IK L foot: start {np.round(p0, 3)} target {np.round(target, 3)} residual {err * 1000:.2f} mm")
    print(f"IK joint targets (deg): {np.round(np.degrees(q), 1)}")
    # 2. capture point at rest
    print("capture point (rest):", np.round(st.capture_point(), 3), "CoM:", np.round(st.com(), 3))


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
