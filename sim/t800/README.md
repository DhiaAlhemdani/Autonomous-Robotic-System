# T800 standing-pose test (sandbox, not validated on hardware)

`stand_test.py` loads the pinned upstream model (`third_party/T800-PROJECT`, `serial_t800.xml`) with a floor added through `MjSpec`, sets the upstream keyframe pose, and applies a PD torque law to all 25 actuators. The torque limits come from the model.

Usage: `python stand_test.py KP KD pd|none [T_seconds]`

Results (MuJoCo 3.2.3, dt 0.002 s, 3 s, free base):

| Mode | KP | KD | Pelvis z end (m) | Tilt (deg) | Upright |
| --- | --- | --- | --- | --- | --- |
| none | 0 | 0 | 0.46 | 116 | no |
| pd | 150 | 5 | 0.10 | 86 | no |
| pd | 300 | 10 | 0.12 | 90 | no |
| pd | 600 | 20 | 0.10 | 89 | no |
| pd | 1200 | 40 | 0.10 | 88 | no |
| pd | 2000 | 60 | 0.10 | 88 | no |

Conclusion: with a free base, a fixed standing pose under per-joint PD does not hold, at any tested gain. A balance or base-support controller is needed, or the base must be constrained. The gains are sweep values, not tuned. The test does not establish anything about walking.

## Planar base under NMPC (option 2)

`planar_base.py` drives the T800 in the MuJoCo scene with the Stage 0 NMPC.

- **Virtual base.** The pelvis (`LINK_BASE`) is welded to a mocap body. The NMPC moves that body in the plane. The weld holds the orientation upright. The legs hold the upstream standing pose with PD torques (`kp` 600, `kd` 20: ASSUMPTION, chosen by the standing-pose sweep, not tuned). Nothing balances the robot, and it does not walk.
- **Planner and feedback.** A planner state integrates the NMPC command, and the weld target follows the planner. The NMPC reads the measured pelvis state each step. A first version that set the target from the measured state stalled the base at 0.13 m/s, because it never accumulated the commanded acceleration. That version is not reported.
- **Scene.** The Stage 0 scenario S2 (goal, 3 moving persons, 2 static objects, same random seeds). Persons and objects are non-colliding visuals. Separation is measured from ground truth, as in Stage 0. Persons do not react to the robot.
- **Physics.** MuJoCo 3.2.3, dt 0.002 s, control at 10 Hz (`Stage0Config.dt`). Floor added in the script, not in the submodule.

Run: `python sim/t800/planar_base.py --seed 0 --max-time 30 --out runs/t800_planar_seed0.json` (needs `numpy`, `scipy`, `mujoco==3.2.3`), then `python sim/t800/aggregate.py` from the repo root.

Results (sandbox, seeds 0–9, all 10 episodes run to completion):

| Measure | Result |
| --- | --- |
| Goal reached | 10 / 10 (time to goal p50 9.7 s) |
| Fell (pelvis < 0.6 m or tilt > 20°) | 0 / 10 |
| Max tilt | 4.5° (max across episodes) |
| Min pelvis height | 0.87 m |
| Separation violations (episodes) | 0 / 10 |
| Min distance to a person (p5 / p50) | 1.27 m / 1.95 m |
| Infeasible NMPC steps (mean over episodes) | 5.0% (brake fallback applied) |
| NMPC solve p50 / p99 (median over episodes) | 389 ms / 764 ms |
| Tracking error, planner vs pelvis (p50 / p99) | 37 mm / 39 mm |

**What this supports:** the NMPC can steer a T800 base in simulation, with the pelvis staying upright and the legs holding under this weld. The loop is closed through the physics.

**What this does not support:**

- No balance or walking. The base is kinematically driven by the weld, which carries the robot's weight.
- The solve times are sandbox numbers, and the sandbox was shared with the background batch. They are not target-hardware figures and are not an NMPC rate.
- Infeasible steps occur (5%). Each uses the brake fallback, and the cause is not diagnosed in this scene.
- Persons don't react to the robot, as in Stage 0. The 37 mm tracking lag is a weld-compliance measurement, not a comfort result.

## Stage A: free-base standing (no weld)

`balance.py` tests whether a feedback law can keep the free-standing T800 upright. There is no weld and no mocap body. Every joint is PD-controlled (kp 600, kd 20, both assumptions from the sweep). The controller adds:

- **Ankle strategy:** ankle pitch correction from the whole-body centre-of-mass error relative to the feet (sign +1, kc 3, kv 0.5 on the CoM error rate). The sign was chosen by sweep.
- **Hip strategy:** hip pitch correction from the signed pelvis pitch (sign +1, kh 2).

Ankle-only feedback held the robot, but only with a steady lean of about 9° and the pelvis at 1.01 m. Adding the hip term brought the final tilt to about 3.4°.

Results (MuJoCo 3.2.3, sandbox, dt 0.002 s):

| Test | Result |
| --- | --- |
| Quiet standing, 10 s and 20 s | Held. Final tilt 3.4°, max tilt 4.4°. Both feet on the floor (8 contacts each). |
| 20 N and 40 N pushes (0.2 s) | Recovered. |
| 150 N pushes, forward, sideways | Recovered (tilt 4–5°). |
| 150 N push backward (−x) | Fell at 4.7 s (max tilt 24°). |
| 300 N pushes (x, y) and 500 N (x) | Fell at 3.8–4.2 s. |
| Sweep of 18 gain sets (kc 2–4, kv 0.5–1, kh 1–3) | None held quiet standing and both the −150 N and +300 N pushes. |

Open questions and caveats:

- **Not a balance controller that handles pushes.** It holds quiet standing and small pushes only. Larger pushes need a different strategy (stepping, or a fuller whole-body controller). I haven't tried those.
- **Standing height is 1.01 m, not 0.87 m.** The keyframe height (0.87 m) was for the welded case. The reason for the taller stance is not checked.
- **Ankle torque limit: inconclusive.** During the −150 N push the ankle command saturated at its 160 N·m limit for 313 steps, and the hips did not saturate. But doubling the ankle limit in the model gave identical results, so I can't attribute the fall to the limit. The result is not settled.
- **Gains are selected on the same pushes used to report them.** Treat the push results as a check on the chosen gains, not as held-out evidence.
- Only 1 contact-free test per push direction; no noise, delay or model error. This is simulation only.

## Native 3D viewer (run on your laptop)

`viewer.py` opens MuJoCo's own 3D window and runs the Stage A standing controller on the free-standing T800. Run it natively, not in Docker, because the window needs a display.

```
git submodule update --init --depth 1 third_party/T800-PROJECT
# Ubuntu/Debian first, if venv is missing:  sudo apt install python3-venv python3-pip
python3 -m venv .venv-viewer && . .venv-viewer/bin/activate
python -m pip install mujoco==3.2.3 numpy==1.26.4
python sim/t800/viewer.py                            # stand; no push
python sim/t800/viewer.py --push-x -150 --push-t 5    # 150 N backward push at 5 s
```

Options: `--push-x`, `--push-y` (N), `--push-t` (start, s), `--push-dur` (s, default 0.2), `--T` (stop after T s). Close the window to exit.

Expected behaviour, from the sandbox headless run: quiet standing holds with a final tilt of 3.4°. The 150 N backward push at 5 s makes the robot fall at about 6.6 s. The window is not checked in the sandbox (no display there), so please report anything that looks wrong in the window.

Headless check (no window, runs anywhere): `python sim/t800/viewer.py --headless --T 10`.


## Linux laptop: viewer in a container (X11) with the dashboard

Requires a Linux desktop session (X11, not Wayland-only) and Docker with the compose plugin.

```bash
scripts/run_viewer.sh
```

The dashboard opens once the 3D window has opened (the viewer logs `3D window: open`). The script allows local X connections (`xhost +local:`), starts the `viewer` service (MuJoCo 3D window on your display, plus the Dash dashboard on port 8050), and opens http://localhost:8050 in your browser. Stop with `docker compose --profile sim-only stop viewer`.

The dashboard's balance panel shows the live state written by the viewer: pelvis tilt and height, centre-of-mass error with the ankle command, and floor contacts per foot. Red bands mark the push. After the run ends, the dashboard stays up until you stop the container.

Simulation only. Stage A is a free-standing balance test, not walking, and no claim is made about pushes beyond the table above or about hardware.

Not tested in the build sandbox: the window path (no display there) and the X11 mount on a real laptop. The headless path and the dashboard startup were tested.
