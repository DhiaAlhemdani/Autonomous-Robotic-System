"""Free-standing T800 in MuJoCo, with an optional 3D window.

The balance controller (balance.py) runs on the free-base T800. The 3D window is MuJoCo's own viewer.

Run on a laptop with a display (native, or the Linux container with X11, see docker-compose.yml):

    python sim/t800/viewer.py                          # 3D window
    python sim/t800/viewer.py --push-x -150 --push-t 5 # 150 N backward push at 5 s
    python sim/t800/viewer.py --headless --T 10        # no window; status printed every second

Pushes are horizontal forces on the pelvis, held for --push-dur seconds (default 0.2 s).
The window runs at real time. That is the simulation rate, not a measurement of the controller.
Closing the window ends the run. Stop with Ctrl+C or --T.

Simulation only. The controller is not a validated balance controller and does not handle
pushes above about 150 N.
"""
import argparse
import importlib.util
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=0.0, help="stop after T seconds (0 = until Ctrl+C)")
    ap.add_argument("--push-x", type=float, default=0.0, help="pelvis force along x (N)")
    ap.add_argument("--push-y", type=float, default=0.0, help="pelvis force along y (N)")
    ap.add_argument("--push-t", type=float, default=5.0, help="push start time (s)")
    ap.add_argument("--push-dur", type=float, default=0.2, help="push duration (s)")
    ap.add_argument("--headless", action="store_true", help="no 3D window; print status every second")
    args = ap.parse_args()

    B = _load("balance", os.path.join(HERE, "balance.py"))
    import mujoco

    r = B.Robot()
    r.reset()
    ctrl = B.BalanceController(r)
    body = mujoco.mj_name2id(r.m, mujoco.mjtObj.mjOBJ_BODY, "LINK_BASE")
    dt = r.m.opt.timestep

    def apply_push(t):
        r.d.xfrc_applied[body, :] = 0
        active = args.push_t <= t < args.push_t + args.push_dur
        if active:
            r.d.xfrc_applied[body, 0] = args.push_x
            r.d.xfrc_applied[body, 1] = args.push_y
        return active

    def status(t):
        c = B.foot_contacts(r)
        print(f"t={t:6.2f} s  pelvis z={r.d.qpos[2]:.3f} m  tilt={r.tilt_deg():5.1f} deg  "
              f"feet contacts L/R={c['L']}/{c['R']}", flush=True)

    fell_at = None
    t = 0.0

    def run_loop_step(t):
        nonlocal fell_at
        apply_push(t)
        ctrl.step()
        if fell_at is None and (r.d.qpos[2] < 0.6 or r.tilt_deg() > 25.0):
            fell_at = t + dt
            print(f"FELL at t={fell_at:.2f} s (tilt {r.tilt_deg():.1f} deg)", flush=True)

    if args.headless:
        n = int(round((args.T or 10.0) / dt))
        for i in range(n):
            t = i * dt
            run_loop_step(t)
            if i % int(round(1.0 / dt)) == 0:
                status(t)
            if fell_at is not None:
                break
        if fell_at is None:
            print(f"held to t={t + dt:.2f} s; final tilt {r.tilt_deg():.1f} deg", flush=True)
        return

    if not os.environ.get("DISPLAY"):
        # MuJoCo's C code exits the process when GLFW cannot open a display, so check first.
        print("no DISPLAY: 3D window not opened (use scripts/run_viewer.sh, or --headless)", flush=True)
        sys.exit(1)

    import mujoco.viewer  # noqa: F401  (needs a display)

    # Closing the 3D window ends the run. The window is not reopened.
    cm = None
    viewer = None

    def open_window():
        nonlocal cm, viewer
        print(f"3D window: opening on DISPLAY={os.environ.get('DISPLAY')}", flush=True)
        cm = mujoco.viewer.launch_passive(r.m, r.d)
        viewer = cm.__enter__()
        print("3D window: open (closing it ends the run)", flush=True)

    def close_window():
        nonlocal cm, viewer
        if cm is not None:
            try:
                cm.__exit__(None, None, None)
            except Exception:
                pass
        cm = viewer = None

    wall0 = time.perf_counter()
    last_print = -1
    try:
        open_window()
        while True:
            if args.T and t >= args.T:
                break
            if not viewer.is_running():
                print("3D window closed; run ended.", flush=True)
                break
            run_loop_step(t)
            viewer.sync()
            t += dt
            if int(t) != last_print:
                last_print = int(t)
                status(t)
            ahead = t - (time.perf_counter() - wall0)
            if ahead > 0:
                time.sleep(ahead)
    except KeyboardInterrupt:
        print("stopped (Ctrl+C)", flush=True)
    finally:
        close_window()


if __name__ == "__main__":
    main()
