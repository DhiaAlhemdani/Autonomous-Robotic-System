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

