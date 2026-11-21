"""Free-base T800 in MuJoCo with an optional 3D window. No controller: the robot is passive.

The model is the vendored upstream model (vendor/t800), with a floor added. Starting from the
model's default pose, the robot falls under gravity. Pushes are horizontal forces on the pelvis.

    python sim/t800/viewer.py                          # 3D window
    python sim/t800/viewer.py --push-x -150 --push-t 5 # 150 N backward push at 5 s
    python sim/t800/viewer.py --headless --T 10        # no window; status printed every second

Pushes are held for --push-dur seconds (default 0.2 s). The window runs at real time, which is
the simulation rate, not a measurement. Closing the window ends the run.

Simulation only. Nothing here is a controller or a balance claim.
"""
import argparse
import math
import os
import sys
import time

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
XML = os.path.join(REPO, "vendor", "t800", "assets", "resource", "robot", "t800", "xml", "serial_t800.xml")
PELVIS_START_Z = 0.87


def build():
    if not os.path.exists(XML):
        raise SystemExit(f"T800 model not found at {XML}")
    spec = mujoco.MjSpec()
    spec.from_file(XML)
    fl = spec.worldbody.add_geom()
    fl.name = "floor"
    fl.type = mujoco.mjtGeom.mjGEOM_PLANE
    fl.size = [20, 20, 0.1]
    fl.friction = [1.0, 0.005, 0.0001]
    return spec.compile()


def tilt_deg(d):
    w, x, y, z = d.qpos[3:7]
    zz = 1 - 2 * (x * x + y * y)   # z component of the body z-axis in world coordinates
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=0.0, help="stop after T seconds (0 = until Ctrl+C)")
    ap.add_argument("--push-x", type=float, default=0.0, help="pelvis force along x (N)")
    ap.add_argument("--push-y", type=float, default=0.0, help="pelvis force along y (N)")
    ap.add_argument("--push-t", type=float, default=5.0, help="push start time (s)")
    ap.add_argument("--push-dur", type=float, default=0.2, help="push duration (s)")
    ap.add_argument("--headless", action="store_true", help="no 3D window; print status every second")
    args = ap.parse_args()

    m = build()
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    d.qpos[2] = PELVIS_START_Z
    mujoco.mj_forward(m, d)
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "LINK_BASE")
    dt = m.opt.timestep

    def step(t):
        d.xfrc_applied[body, :] = 0
        if args.push_t <= t < args.push_t + args.push_dur:
            d.xfrc_applied[body, 0] = args.push_x
            d.xfrc_applied[body, 1] = args.push_y
        mujoco.mj_step(m, d)

    def status(t):
        print(f"t={t:6.2f} s  pelvis z={d.qpos[2]:.3f} m  tilt={tilt_deg(d):5.1f} deg", flush=True)

    if args.headless:
        n = int(round((args.T or 10.0) / dt))
        for i in range(n):
            t = i * dt
            step(t)
            if i % int(round(1.0 / dt)) == 0:
                status(t)
        status(n * dt)
        return

    if not os.environ.get("DISPLAY"):
        # MuJoCo's C code exits the process when GLFW cannot open a display, so check first.
        print("no DISPLAY: 3D window not opened (use scripts/run_viewer.sh, or --headless)", flush=True)
        sys.exit(1)

    from mujoco import viewer as mviewer

    t = 0.0
    last = -1
    wall0 = time.perf_counter()
    try:
        with mviewer.launch_passive(m, d) as viewer:
            print("3D window: open (closing it ends the run)", flush=True)
            while viewer.is_running():
                if args.T and t >= args.T:
                    break
                step(t)
                viewer.sync()
                t += dt
                if int(t) != last:
                    last = int(t)
                    status(t)
                ahead = t - (time.perf_counter() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
            print("3D window closed; run ended." if not args.T or t < args.T else "run ended.", flush=True)
    except KeyboardInterrupt:
        print("stopped (Ctrl+C)", flush=True)


if __name__ == "__main__":
    main()
