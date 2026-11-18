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
