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
- Declared properties (tests): feet alternate; same-foot step length `<= STEP_MAX = 0.5 m` (ASSUMPTION); each sole centre's lateral offset from the CoM reference at its mid-stance time is `±FOOT_Y ± 1 mm`; yaw of each placement matches `psi` at its mid-stance time.
- Amendment (made before the tests were run): the wording "lateral separation of the two sole centres = 2 FOOT_Y" was replaced by the per-foot lateral offset above. The two feet are placed at different times, so a same-time separation is not the quantity the plan defines.

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

## 9. Results (sandbox, simulation only)

Implementation: `planning/legged_nmpc/` (config, model, controller, sim, wbik). Tests: `planning/tests/test_legged_nmpc.py`. Sweep: `python -m planning.legged_nmpc.sim`, output `runs/legged_nmpc_sweep_{tuning,heldout}.json` (git-ignored).

**Corrections made to `docs/17` results.** The old `docs/17` §9 numbers used the wrong support box and CoM height. Those numbers are superseded. The new `docs/17` tests (standing, walking) were rerun with the corrected constants below. The old sweep JSON was overwritten by the rerun. The old numbers remain only in `docs/17` §9.

**Whole-body envelope (before the sweep, on the default pose).** The IK required a posture term and a task weight to converge. Final setting: task rows weighted 10, posture row weighted 0.05 toward a bent-knee nominal posture, damped least squares (λ = 1e-2) with a 0.2 rad step cap. Measured results at fixed poses (feet at y = ±0.121, with dx = forward offset of the second foot):

| CoM height | dx | CoM residual | Foot residual | Clamps |
|---|---|---|---|---|
| 0.9536 (measured nominal) | 0.00 | 0.1 mm | 0.6 mm | 0 |
| 0.9536 | 0.15 | 1.1 mm | 0.8 mm | 0 |
| 0.9536 | 0.25 | 6.0 mm | 19.7 mm | 0 |
| 0.93 | 0.00 | 1.3 mm | 9.4 mm | 396 |
| 0.90 | 0.00 | 390 mm | 1.15 m (diverged) | 1072 |

The layer converges at the measured height for forward offsets up to about 0.15 m, and it does not converge at 0.93 m or lower. This is an IK limit, not a physical one. The CoM was not lowered to get reach. A 0.93 m walking height is not claimed.

**Tests (§6).**

| Test | Result |
|---|---|
| 0. Vendor constants (mass, Z_NOM, FOOT_Y, FOOT_OFFSET_X, IZ) | pass |
| 1. Discrete model matches ODE (1e-9) | pass |
| 2. Standing: final CoM error < 1 mm after 3 s (`docs/17`) | **FAIL: 3.7 mm** |
| 3. Walking 0.3 m/s: windowed mean velocity error < 0.02 m/s (`docs/17`) | pass |
| 4. Yaw tracking: heading error < 0.05 rad over the last 5 s | pass |
| 5. Footstep properties (alternation, step ≤ 0.5 m, lateral ±FOOT_Y ± 1 mm, yaw at mid-stance) | pass |
| 6. Swing properties (endpoints, z ≥ 0, max z = SWING_H) | pass |
| 7. Whole-body: CoM residual < 5 mm and stance residual < 5 mm (walk v = 0.25, w = 0.2) | **FAIL: CoM 6.6 mm** (stance residual not reached; the test stops at the first failed assert) |
| 8. Determinism | pass |
| 9. Applied forces within friction and bounds | pass |

**Seeded sweep (§7), run once, no tuning.** Scenes drawn from each seed (speed in [0.1, 0.4] m/s, yaw rate in [-0.2, 0.2] rad/s, offset ±0.02 m, push 0–60 N for 0.2 s at 3 s, 10 s episode).

| Metric | Tuning 0–9 | Held-out 10–19 | Declared limit |
|---|---|---|---|
| Solver failures (infeasible-step fraction) | 1 (0.0005) | 1 (0.0005) | reported |
| ZMP violation max | 0 m | 0 m | — |
| Friction violation max | 3.6e-8 N | 2.0e-8 N | IPOPT tolerance |
| Windowed velocity error (mean) | 0.0014 m/s | 0.0015 m/s | — |
| Heading error, last 5 s (mean) | 0.0005 rad | 0.0005 rad | < 0.05 rad (test 4) |
| Max CoM deviation from reference | 0.059 m | 0.051 m | — |
| Fraction of steps within 0.1 m | 1.00 | 1.00 | — |
| CoM residual max (whole-body, after 1 s) | 8.4 mm | 9.0 mm | **5 mm: FAIL** |
| Stance-foot residual max | 15.0 mm | 14.4 mm | **5 mm: FAIL** |
| Swing-foot residual max | 8.6 mm | 8.7 mm | not declared |
| Joint clamp events (all IK iterations) | 217 | 0 | reported |
| Solve p50 (median over episodes) | 0.12 s | 0.11 s | sandbox |
| IK p50 (median over episodes) | 37 ms | 37 ms | sandbox |

**What this does not show.**
- The whole-body layer does not meet its declared 5 mm residual limit in the walking sweep. Walking with yaw is not reference-tracking-clean at the joint level.
- Solver failures remain undiagnosed, as in `docs/17`.
- The plant is the reduced model plus kinematic whole-body. No torque, dynamic, collision, or hardware claim. The 0.9536 m CoM height is kinematically reachable only for forward offsets up to about 0.15 m.
- Solve and IK times are sandbox measurements.

## 10. Full-dynamics walking attempt (gate lifted for this attempt)

The gate in the project plan (walking blocked until standing passes) was lifted by the user for this attempt. The attempt used the full MuJoCo T800 model with a floor (`planning/legged_nmpc/fullsim.py`). The NMPC drives the whole-body IK, and the IK targets drive the 25 motors through PD with gravity feedforward (`qfrc_bias`), clipped to each motor's torque limit.

**Environment correction.** The rebuilt `.venv` had picked up MuJoCo 4.0.1 and CasADi 3.8.2. The repo pins MuJoCo 3.2.3 (README, Dockerfile). MuJoCo 4.0.1 could not load the model through `MjSpec` (no joints), and the viewer was broken. The venv was rebuilt with the pinned versions: mujoco 3.2.3, casadi 3.6.7, numpy 1.26.4, scipy 1.13.1, pytest 8.3.3. The viewer runs again. The planning suite under the pinned versions gives 36 passed, 2 failed (standing and whole-body CoM residual, as in §9).

**Outcome: the robot does not stand, so it does not walk.** Measured in simulation, not claimed for hardware:

| Experiment | Result |
|---|---|
| Standing with NMPC + IK + PD (kp = 1) | falls at 1.4–1.8 s |
| Standing with ankle attitude feedback (±50, ±200 N·m/rad) | falls at 1.65–1.9 s in every case |
| Fixed IK pose held with PD + gravity feedforward | pitch reaches 8.7° at 0.8 s |
| Same, after adding a flat-sole constraint to the IK | feet flat (z-axis vertical); still pitches, 10.5° at 1.0 s |
| Same, gains × 1, 3, 6 | still drifts forward; pitch 4° at 0.9 s even at ×6 |

**What the measurements show.**
- The base tilts and yaws in the full model. The IK constrains base orientation only kinematically, and no actuator controls the trunk's attitude directly.
- The centre of pressure sits about 3 cm behind the CoM even at the IK pose, so the robot is out of balance from the start. Its contacts span x = ±0.121 m, so the sole geometry is in the expected place.
- The restoring stiffness needed about the feet is roughly m·g·h ≈ 800 N·m/rad. Gains of ×1 to ×6 did not remove the drift. Ankle torque limits (160 N·m) are small relative to the hip (415 N·m), so the ankle alone cannot balance.
- The fixed-pose drift at high gain is not yet explained. The likely candidates are torque saturation at the ankle and hip, or contact compliance. Neither was isolated.

**What this does not show.** No standing, no walking, no balance claim for the T800. The reduced NMPC never produced the forces that act on the robot; the full-dynamics loop uses only the IK joint targets derived from it. The `docs/17` NMPC forces are not applied through contact.

**What would be needed.** A balance controller in joint space (CoM/attitude feedback with ankle and hip strategies, torque-aware), a foot-contact model that is verified before walking, and an explanation of the fixed-pose drift. These are the first steps before any walking experiment can be meaningful.

### 10.1 Task-space balance attempt (Jacobian transpose): rejected

Added `BAL` in `fullsim.py` (off by default): CoM PD force mapped through the CoM Jacobian, plus base attitude torque mapped through the base rotation Jacobian, both restricted to the actuated joints.

- CoM feedback alone (KC 1500–6000 N/m, with attitude): the CoM still drifts forward, 5 cm by 0.6 s and 25 cm by 1.2 s. The robot falls.
- Attitude feedback alone (KA 500–8000 N·m/rad): results are identical for every gain, so the term is zero. The base body's rotation Jacobian with respect to the actuated joints is zero, because those joints are children of the base. Joint torques cannot rotate the base in this formulation.

**Why this approach is wrong.** The base is unactuated and its motion comes from ground contact. Joint torques change the CoM and trunk attitude only through the contact forces at the feet. A correct controller must account for that. The standard formulation is a whole-body torque QP on the floating-base dynamics: choose contact forces (friction-cone constrained) and joint torques so that the base accelerates as desired. That is the next step if the balance work continues. It is not implemented.

**Status.** No standing, no walking. The walk attempt in §10 stands as the result.

## 10.2 Floating-base whole-body torque QP (full dynamics, simulation only)

Implementation: `planning/legged_nmpc/wbc_qp.py` (OSQP 1.1.3; active-contact mask; corner friction MU 0.6; torque bounds; swing-foot task) and `planning/legged_nmpc/walk_qp.py` (NMPC-driven loop). Tests: `planning/tests/test_walk_qp.py`. MuJoCo 3.2.3.

Results (single runs, not acceptance):
- Standing, no push, 3 s: CoM xy error 0.75 mm (declared < 1 mm). 0 QP failures.
- Standing push 30 N x, 0.2 s at t=1 s, 4 s run: CoM xy error 1.04 mm (declared < 2 mm). Earlier probe: 60 N y also recovers in about 1.5 s. Push timing differs from `docs/17` (t=3 s), not yet matched.
- Walking v=0.2 m/s under NMPC reference, 3 s: falls at about 1.66 s in the 6 s run. Gain variants (kf/df, ka/da, kc/dc) fall between 1.03 and 2.42 s. First-step drift about 5 cm laterally, yaw diverges to about -1.45 rad, stance contacts drop to 3-5 points.

Status: walking is NOT achieved. `test_walk_3s_upright` fails and is kept as a declared failure. Diagnosis of the first step (stance force vs friction, swing-foot targets, CoM reference feasibility, yaw sign) is the next step. Gain sweeps are not expected to fix it.

§10.1 causal claim (base Jacobian zero for actuated joints) is still unverified; the attitude-identical trajectories may indicate a wiring fault in the torque path. Treat §10.1 as open.

## 10.3 Real-time NMPC: horizon and tolerance

Problem: the full-dynamics viewer ran at about 0.6x real time on a laptop. The NMPC (IPOPT, one solve per 50 ms control tick) took about 40 ms median and up to 81 ms at N=24, which is most of the tick budget. Physics and the QP together were under 0.5 s per 3 s run.

Benchmark (sandbox, standing reference, 60 solves each, 0 failures):

| Setting | p50 | max |
|---|---|---|
| N=24, default tol (before) | 40.5 ms | 81.0 ms |
| N=24, tol 1e-6 | 34.7 ms | 77.3 ms |
| N=16, default tol | 24.9 ms | 51.9 ms |
| N=16, tol 1e-6 (now) | 24.1 ms | 33.6 ms |
| N=12, tol 1e-6 | 19.6 ms | 34.9 ms |
| N=8, tol 1e-6 | 13.7 ms | 17.6 ms |

Change: `N` 24 -> 16 (horizon 1.2 s -> 0.8 s) and `ipopt_tol` 1e-6 added to `config.py` and used in `controller.py`. Horizon is the main lever; tolerance helps little.

Results after the change (single runs unless stated):
- Standing, no push, 3 s: CoM xy error 0.76 mm (was 0.75 mm). `test_walk_qp` standing and push tests pass.
- Walking v=0.2 m/s, 6 s: falls at 1.40 s (was 1.66 s). The walking test still fails. The shorter horizon is shorter than the 0.9 s gait period, which is a likely cause, but not tested.
- `planning/tests/test_legged_nmpc.py`: 8 passed, 2 failed (test 2 final CoM error 4.7 mm vs 1 mm; test 7 CoM residual 7.7 mm vs 5 mm). These are the same two declared-limit misses as before this change.

Not verified: the wall-clock timing of the viewer on the user's laptop. In the sandbox the headless 3 s run takes about 6 s wall, of which about 3 s is model build and startup. This is not a real-time claim.

Trade-off: smoother simulation, but the walking attempt is worse and the horizon no longer covers a full gait cycle. Recovering walking may need a longer horizon at a lower control rate, or warm-start changes. Not attempted.

## 10.4 NMPC at a lower rate than the QP

Change: the NMPC solves every `nmpc_every` = 2 control ticks (0.1 s); the QP runs every 0.05 s tick. Between solves the walker uses the reference from the last solve's prediction (stage `ticks_since_solve + 1`). The warm start is shifted by the number of ticks since the last solve. Horizon N=16 (0.8 s), `ipopt_tol` 1e-6, as in §10.3.

Results (single runs, sandbox, MuJoCo 3.2.3):
- Standing v=0, 3 s: no fall, 60 QP solves, CoM xy error 0.8 mm, 0 NMPC failures.
- Walking v=0.2 m/s, 6 s: falls at 2.14 s with 4 NMPC failures (43 QP solves). Previous: 1.40 s with NMPC every tick at N=16 (§10.3), 1.66 s at N=24 (§10.2). The walking test still fails.
- `planning/tests/test_walk_qp.py`: 2 passed, 1 failed (walking, expected).
- `planning/tests/test_legged_nmpc.py`: 8 passed, 2 failed (tests 2 and 7, the same declared-limit misses).

Not verified: viewer smoothness on the user's laptop; the cause of the 4 NMPC failures during the walk (likely the reference moving faster than the solve period, not tested).
