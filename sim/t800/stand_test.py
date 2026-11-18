import sys
import numpy as np
import mujoco

XML = "/home/user/Autonomous-Robotic-System/third_party/T800-PROJECT/assets/resource/robot/t800/xml/serial_t800.xml"
KP = float(sys.argv[1])
KD = float(sys.argv[2])
MODE = sys.argv[3]  # "pd" or "none"
T = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0

spec = mujoco.MjSpec()
spec.from_file(XML)
fl = spec.worldbody.add_geom()
fl.name = "floor"
fl.type = mujoco.mjtGeom.mjGEOM_PLANE
fl.size = [10, 10, 0.1]
fl.friction = [1.0, 0.005, 0.0001]
m = spec.compile()
d = mujoco.MjData(m)

# standing pose from the (commented-out) upstream keyframe: root(7) then 25 joints
home = [0, 0, 0.87, 1, 0, 0, 0,
        0, 0, -0.12, 0.24, -0.12, 0, 0, 0, -0.12, 0.24, -0.12,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
jnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
q0 = {}
k = 7
for j in range(1, m.njnt):
    q0[jnames[j]] = home[k]
    k += 1

act_q, act_dof, act_target = [], [], []
for a in range(m.nu):
    jid = m.actuator_trnid[a, 0]
    act_q.append(m.jnt_qposadr[jid])
    act_dof.append(m.jnt_dofadr[jid])
    act_target.append(q0.get(jnames[jid], 0.0))
act_q = np.array(act_q)
act_dof = np.array(act_dof)
act_target = np.array(act_target)

mujoco.mj_resetData(m, d)
d.qpos[0:3] = [0, 0, 0.87]
d.qpos[3:7] = [1, 0, 0, 0]
d.qpos[act_q] = act_target
mujoco.mj_forward(m, d)

lo = m.actuator_ctrlrange[:, 0]
hi = m.actuator_ctrlrange[:, 1]
nsteps = int(round(T / m.opt.timestep))
every = max(1, int(round(0.1 / m.opt.timestep)))
zs = []
for s in range(nsteps):
    if MODE == "pd":
        q = d.qpos[act_q]
        qd = d.qvel[act_dof]
        d.ctrl[:] = np.clip(KP * (act_target - q) - KD * qd, lo, hi)
    mujoco.mj_step(m, d)
    if s % every == 0:
        zs.append(d.qpos[2])

w, x, y, z = d.qpos[3:7]
tilt = np.degrees(2 * np.arccos(min(1.0, abs(w))))
print(f"MODE={MODE} KP={KP} KD={KD} T={T}s | z start {zs[0]:.3f} end {zs[-1]:.3f} min {min(zs):.3f} | tilt {tilt:.1f} deg | upright={zs[-1] > 0.6 and tilt < 20}")
