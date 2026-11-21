# 19. Legged controller: full design (T800, simulation only)

Status: design. Nothing in this document is a walking, balance or push-recovery result. Each layer below is either implemented and measured (standing), implemented and failing (walking), or specified and not built. Thresholds are declared here before any run that uses them.

## 1. Scope and claims

- Target: the vendored T800 (`vendor/t800`, 84.917 kg, Z_NOM 0.9536 m), MuJoCo 3.2.3, flat floor, simulation only.
- Goal: stand, recover from declared pushes, then walk at declared speeds. Walking is not claimed until §7 gates pass.
- Out of scope now: hardware, real-time guarantees, learned policies, weights from any repository without a licence.

## 2. Layers

| # | Layer | Input | Output | Code | Status |
|---|---|---|---|---|---|
| L1 | Gait and footstep plan | v, w, gait period | stance schedule, sole targets, swing poses | `planning/legged_nmpc/model.py` | implemented |
| L2 | Lateral and yaw reference | L1 | CoM and yaw reference | `model.reference` | committed version is y = 0; LIP version tried (docs/18 §10.7) |
| L3 | Reduced NMPC (CoM, yaw, forces) | state, reference, ZMP half-planes | forces, predicted CoM | `controller.py` (IPOPT via CasADi) | implemented; N=16, dt 0.05 s, solve every 2 ticks |
| L4 | Whole-body torque QP | L3 reference, swing targets, posture | joint torques | `wbc_qp.py` (OSQP) | implemented; every tick |
| L5 | Swing control | sole target and height | foot accelerations | `walk_qp.py` | kinematic, gains kf 100, df 20 |
| L6 | Yaw control | yaw reference | yaw acceleration via L4 | `walk_qp.py` attitude term | implemented; **fails** (§6) |
| L7 | State estimation | sim state | x (8-vector) | `walk_qp.py` | perfect state from simulation (no estimator) |

Data flow each control tick (0.05 s): L7 state → L3 (every 2nd tick) → reference (held between solves, indexed by ticks since solve) → L4 with foot targets from L1 → torques → 25 physics steps at 0.002 s.

## 3. Interfaces (fixed)

- L3 → L4: CoM position and velocity, yaw and yaw rate, per tick (held reference).
- L1 → L4: active contact flags per foot, sole target per swing foot (xy, z, yaw).
- L4 → plant: 13 leg and waist torques, clipped to actuator ranges (ankle 160 N·m, hip 415 N·m).
- Friction: corner model, MU 0.6, normal force 0 to FZ_MAX 1700 N per foot.

## 4. Measured facts that constrain the design

- Standing holds with NMPC + QP: CoM xy error 0.76 mm over 3 s; 30 N push recovers, 1.04 mm after 4 s (docs/18 §10.2).
- The NMPC is feasible at every solve for the committed reference and for the LIP reference (docs/18 §10.5, §10.7).
- Lateral dynamics: with the foot-centre ZMP square wave at period 0.9 s and z = 0.9536 m, the steady-state CoM amplitude is about 0.027 m. A fixed y = 0 is outside the single-support box (docs/18 §10.5).
- NMPC solve: about 24 ms median and 34 ms max at N=16, tol 1e-6 (sandbox, docs/18 §10.3). Physics and QP together under 0.5 s per 3 s run.

## 5. Known failures (measured)

| Setting | Walk at v = 0.2 m/s | First observed failure |
|---|---|---|
| Committed (y = 0 reference, N=16, solve every 2 ticks) | falls 2.14 s | lateral drift, then NMPC failures from 1.8 s (§10.5) |
| LIP lateral reference | falls 1.23 s, 0 NMPC failures | yaw divergence from 0.85 s, stance contact drops to 1–2 points (§10.7) |
| Lateral cost Q_PY 0–5 | falls 2.14–2.36 s | NMPC failures remain (§10.6) |
| Gait period 1.8–2.0 s | falls 1.5–1.6 s | NMPC failures (§10.5) |

## 6. Open problem: yaw and single-foot contact (blocks walking)

The LIP run shows the first failure is yaw: psi goes 0.06 rad at 0.6 s to −1.0 rad at 1.15 s while the stance has one contact point. Candidate causes, not yet tested:

1. Yaw authority: the yaw moment comes from the stance friction forces through the floating-base QP. Check whether friction and the QP bounds leave yaw torque where it is needed.
2. Swing-leg yaw: the swing foot's yaw is interpolated between placements (`swing_pose`), and its inertia is not in the QP model.
3. Posture coupling: hip-yaw posture gains (`kp_post` 60, `kd_post` 6) may fight the attitude term.
4. Single-foot support: when one foot is down, the stance support box may not contain the CoM through the swing; the contact count drop suggests the stance foot is tilting (roll/pitch), not only yawing.

Test order (each a single run; no seed sweeps): log per-tick yaw torque from the QP against its bound; log friction utilisation of the stance forces; freeze the swing foot yaw at zero and compare; log roll and pitch of the stance foot.

## 7. Acceptance gates (declared before any walking run)

- G0 standing: CoM xy error < 1 mm after 3 s with no push (current: 0.76 mm, passes).
- G1 push standing: 30 N x and 60 N y for 0.2 s at t = 3 s (`docs/17` timing, not the t = 1 s used so far); no fall, CoM xy error < 2 mm after 6 s.
- G2 walking v = 0.2 m/s: no fall over 10 s; yaw |psi| < 0.05 rad over the last 5 s; forward speed error < 0.02 m/s over the last 5 s; 0 NMPC solve failures; at least 90% of QP solves feasible.
- G3 walking v = 0.2 m/s with a declared push: no fall after the push.
- Walking is claimed only after G2 passes on the tuning seeds and G3 passes on held-out seeds (docs/17 seed rule).

Current status (docs/18 §10.20): G0 passes (0.77 mm). G1 passes at the declared timing (30 N x and 60 N y, 0.2 s at t = 3 s; no fall; CoM xy error 0.87 mm and 0.83 mm after 6 s; single runs). G2 fails (falls at 1.2–2.4 s depending on variant). G3 not run. Walking work is stopped (see docs/18 §10.20); walking is not claimed.

## 8. Build order

1. Resolve §6 (yaw and single-foot contact) with the four tests above. Gate: each test gives a single diagnosis, recorded in docs/18 §10.8.
2. Re-run G1 with the declared timing (t = 3 s).
3. Choose the lateral reference (committed y = 0 is infeasible, LIP is feasible) and record the choice with its gate result.
4. Run G2. If it fails, stop and record; do not add gain sweeps as the fix.
5. Run G3 only after G2 passes.

## 9. Limits

- Reduced model: no swing-leg dynamics, joint torques, or contact impacts in L3 (docs/18 §8). L4 uses full floating-base dynamics; L3 does not.
- Perfect state (no estimator), so robustness results would not transfer to hardware.
- Solve times are sandbox measurements, not real-time claims.
- No walking-policy weights are used.
