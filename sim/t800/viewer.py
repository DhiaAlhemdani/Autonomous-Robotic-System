"""Native MuJoCo 3D viewer for the free-standing T800 (sim/t800/balance.py controller).

Run this on your laptop, outside Docker: the viewer needs a display. Example:

    python sim/t800/viewer.py                       # stand; no pushes
    python sim/t800/viewer.py --push-x -150 --push-t 5     # 150 N backward push at 5 s
    python sim/t800/viewer.py --headless --T 10     # no window: checks the loop (used in the sandbox)

Pushes are horizontal forces on the pelvis, held for --push-dur seconds (default 0.2 s).
The viewer runs at real time (one physics step per wall-clock step), so the rate you see is the
simulation rate, not a measurement of the controller.

Simulation only. The controller is the free-base standing law from balance.py (ankle CoM and hip
pitch feedback). It is not a validated balance controller and does not handle large pushes.
"""
import argparse
import importlib.util
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _load_balance():
    spec = importlib.util.spec_from_file_location("balance", os.path.join(HERE, "balance.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=0.0, help="stop after T seconds (0 = until the window closes)")
    ap.add_argument("--push-x", type=float, default=0.0, help="pelvis force along x (N)")
    ap.add_argument("--push-y", type=float, default=0.0, help="pelvis force along y (N)")
    ap.add_argument("--push-t", type=float, default=5.0, help="push start time (s)")
    ap.add_argument("--push-dur", type=float, default=0.2, help="push duration (s)")
    ap.add_argument("--headless", action="store_true", help="no window; print a status line every second")
    args = ap.parse_args()

    B = _load_balance()
    import mujoco

    r = B.Robot()
    r.reset()
    ctrl = B.BalanceController(r)
    body = mujoco.mj_name2id(r.m, mujoco.mjtObj.mjOBJ_BODY, "LINK_BASE")
    dt = r.m.opt.timestep

    def apply_push(t):
        r.d.xfrc_applied[body, :] = 0
        if args.push_t <= t < args.push_t + args.push_dur:
            r.d.xfrc_applied[body, 0] = args.push_x
            r.d.xfrc_applied[body, 1] = args.push_y

    def status(t):
        print(f"t={t:6.2f} s  pelvis z={r.d.qpos[2]:.3f} m  tilt={r.tilt_deg():5.1f} deg  "
              f"feet contacts L/R={B.foot_contacts(r)['L']}/{B.foot_contacts(r)['R']}", flush=True)

    t = 0.0
    if args.headless:
        n = int(round((args.T or 10.0) / dt))
        for i in range(n):
            t = i * dt
            apply_push(t)
            ctrl.step()
            if i % int(round(1.0 / dt)) == 0:
                status(t)
            if r.d.qpos[2] < 0.6 or r.tilt_deg() > 25.0:
                print(f"FELL at t={t:.2f} s (tilt {r.tilt_deg():.1f} deg)")
                return
        print(f"held to t={t + dt:.2f} s; final tilt {r.tilt_deg():.1f} deg")
        return

    import mujoco.viewer  # noqa: F401  (needed for launch_passive; needs a display)

    with mujoco.viewer.launch_passive(r.m, r.d) as viewer:
        wall0 = time.perf_counter()
        last_print = -1.0
        while viewer.is_running():
            if args.T and t >= args.T:
                break
            apply_push(t)
            ctrl.step()
            t += dt
            viewer.sync()
            if int(t) != last_print:
                last_print = int(t)
                status(t)
            if r.d.qpos[2] < 0.6 or r.tilt_deg() > 25.0:
                print(f"FELL at t={t:.2f} s (tilt {r.tilt_deg():.1f} deg). Close the window to exit.")
            # real-time pacing
            ahead = t - (time.perf_counter() - wall0)
            if ahead > 0:
                time.sleep(ahead)


if __name__ == "__main__":
    main()
