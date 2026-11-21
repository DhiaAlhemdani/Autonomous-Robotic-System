# 18 — Legged NMPC extensions: footsteps, swing legs, yaw, whole-body layer

**Status: design declaration, written before any code. Simulation only. Not validated on hardware. Not a walking or balance claim for the T800.**

Builds on `docs/17`. The same reduced model, the same caveats, and the same rule: thresholds below are declared before the code exists, and a failing test is reported as a failure.

## 1. Correction to the constants in `docs/17` (made before any code here)

Measured from `vendor/t800/` (MuJoCo, default pose, collision geoms of `LINK_FOOT_L`/`LINK_FOOT_R`):

| Constant | `docs/17` value | Measured value | Source |
|---|---|---|---|
| `FOOT_Y` (sole centre y) | 0.12 | 0.121 | union of the flat sole blocks (geoms 12, 13), y range 0.066–0.176 |
| `FOOT_HX` (support half-length) | 0.12 | 0.120 | convex hull of the two flat blocks in x: −0.108 to 0.132 |
| `FOOT_HY` (support half-width) | 0.05 | 0.055 | same y range |
| `FOOT_OFFSET_X` (new) | — | 0.037 | sole centre x (0.012) minus foot-origin x (−0.025) |
| `Z_NOM` (CoM height, sole on ground) | 0.967 | 0.9536 | CoM z 0.967 minus sole-bottom height 0.0134 above z = 0 in the default pose |

The `docs/17` values put the CoM height and the support box in the wrong place. Results in `docs/17` §9 were produced with the old values and are superseded by the rerun in §7. They are not deleted.

## 2. Footstep planner

- Inputs: forward speed `v` and yaw rate `w` commands.
- Heading `psi(t) = w t`. Reference CoM (closed form): for `w != 0`, `p(t) = (v/w)(sin(wt), 1 - cos(wt))`; for `w = 0`, `p(t) = (vt, 0)`. Reference height `Z_NOM`.
- Gait: unchanged from `docs/17` (period 0.9 s, stance 0.6 s, feet offset by half a period).
- Placement: each stance is placed at the sole centre `p(t_mid) + R(psi(t_mid)) (0, ±FOOT_Y)`, with yaw `psi(t_mid)`. The sole centre is the support-box centre. The foot origin is `sole centre − R(psi)(FOOT_OFFSET_X, 0)` for the whole-body layer.
- Declared properties (tests): feet alternate; same-foot step length `<= STEP_MAX = 0.5 m` (ASSUMPTION); lateral separation of the two sole centres `= 2 FOOT_Y ± 1 mm`, and yaw of each placement matches `psi` at its mid-stance time.

## 3. Swing-leg control

- Swing window: the complement of each foot's stance interval, `PERIOD - STANCE_T = 0.3 s`.
- Trajectory from the previous placement to the next placement, with `s` = normalised swing time in [0, 1]:
  - xy: cosine ease, `xy(s) = xy0 + (xy1 - xy0)(1 - cos(pi s))/2`;
  - z: `z(s) = SWING_H sin(pi s)`, `SWING_H = 0.04 m` (ASSUMPTION);
  - yaw: linear interpolation of the placement yaw (unwrapped).
- Declared properties (tests): endpoints exact to 1e-9 m; `z >= 0` everywhere; `max z = SWING_H` to 1e-6 m.
- The swing is a kinematic reference. The NMPC does not model swing dynamics; the swing foot carries zero force (as in `docs/17`).

## 4. Yaw dynamics

- State extended to `x = [px, py, pz, vx, vy, vz, psi, w]` (8 states). Inputs unchanged: six forces.
- Yaw torque about the CoM from the stance feet, with `r_i` the sole centres (parameters per stage):
  `tau_z = sum_i [ (r_ix - px) fy_i - (r_iy - py) fx_i ]`
- `psi_{k+1} = psi_k + dt w_k + dt^2/2 * tau_z,k / IZ`, `w_{k+1} = w_k + dt tau_z,k / IZ`, with `IZ = 1.7 kg m^2`, the total yaw inertia about the whole-body CoM. Measured by summing body inertias with parallel-axis terms (`vendor/t800`). Declared as a measurement; the test recomputes it.
- Support boxes rotate with each foot's yaw (`R(psi_foot)`). ZMP and friction constraints are unchanged in form.
- Cost: adds `Q_PSI (psi - psi_ref)^2 + Q_W (w - w_ref)^2`, with `psi_ref = w_cmd t`, `w_ref = w_cmd`.
- Limitation: yaw torque is produced only by tangential forces within the friction cones. No actuator or joint-torque limit is modelled.

## 5. Whole-body layer (kinematic, MuJoCo)

- Model: `vendor/t800` (MuJoCo). Free base plus 13 joints: 12 leg joints and the waist yaw. Arms and head fixed at their default values.
- Method: damped least squares (Levenberg–Marquardt style) on MuJoCo Jacobians, with joint limits enforced by clamping after each step.
- Targets per control step:
  - whole-body CoM position = NMPC CoM;
  - stance-foot sole positions and the swing-foot position (from §2–3, converted to foot origins);
  - base yaw = NMPC `psi`;
  - base roll and pitch regularised to zero.
- Outputs: joint targets, the residuals (CoM, feet, yaw), and the count of joint-limit clamps.
- Not checked: torques, velocities, self-collision, foot orientation (feet are not constrained to be flat), contact forces, and any dynamic feasibility of the joint trajectory. The NMPC forces are not mapped to joint torques.

## 6. Tests and thresholds (declared)

1. Footstep properties (§2) hold for a 10 s walk with `v = 0.25 m/s`, `w = 0.2 rad/s`.
2. Swing properties (§3) hold for every swing window of the same run (tolerances as stated in §3).
3. Yaw tracking: with `v = 0.2 m/s`, `w = 0.2 rad/s`, over 10 s, the mean `|psi - psi_ref|` over the last 5 s is below 0.05 rad, and there are zero solver failures.
4. Standing (rerun with the corrected constants): final CoM error below 1 mm after 3 s (declared in `docs/17`).
5. Walking (rerun): windowed mean velocity error below 0.02 m/s over the last 5 s (declared in `docs/17`).
6. Whole-body IK: for a 10 s walk at the reference speed, the CoM residual is below 5 mm and the stance-foot residual is below 5 mm at each control step after the first second. Joint-limit clamps are counted and reported; the test does not require zero clamps.
7. Determinism (same seed, same state).

## 7. Seeded sweep (declared, reported, no gate)

- Tuning seeds 0–9 and held-out seeds 10–19 (as in `docs/17` §7). Scenes now also draw a yaw-rate command in `[-0.2, 0.2] rad/s`.
- Report, per set: solver failures and infeasible-step fraction, ZMP violation max, friction violation max, windowed velocity error, heading error (mean over the last 5 s), max CoM deviation from reference, IK residuals and joint-limit clamp counts, and solve time p50/p99 (sandbox).
- The `docs/17` sweep is rerun with the corrected constants and reported next to the old numbers.

## 8. What this does not claim

- No walking, balance or push-recovery claim for the T800. The plant is the reduced model plus the kinematic whole-body layer.
- No claim that the footstep plan is kinematically reachable beyond the joint-limit and residual checks above.
- No swing-dynamics, torque, or contact-impact claim.
- No real-time guarantee. Solve and IK times are sandbox measurements.
- No robustness claim against model mismatch, sensor noise, or terrain.
- Solver failures are counted and reported, not diagnosed in this document.

## 9. Results

_To be filled in after implementation, with the commit and exact outputs._
