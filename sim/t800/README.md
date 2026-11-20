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

### Stage A evaluation with pre-declared criteria (`stage_a_eval.py`)

The earlier table reused the pushes that picked the gains. `stage_a_eval.py` sets the criteria before the run and uses held-out pushes of 25, 50, 75 and 100 N in four directions. It starts each seed from small random joint offsets of ±0.02 rad (seeds 0–4, ASSUMPTION). Stage A passes only if quiet standing holds and every held-out push recovers for every seed. The controller is unchanged.

Result: **FAIL** (`python sim/t800/stage_a_eval.py`; output in `runs/stage_a/eval.json`, git-ignored).

| Seed | Quiet standing | Max tilt before fall or end (deg) | Held-out pushes |
| --- | --- | --- | --- |
| 0 | **Fell at 2.16 s** | 19.95 | 0 / 16 (already fallen) |
| 1 | Held, final tilt 3.5° | 4.68 | 16 / 16 |
| 2 | Held, final tilt 4.3° | 5.58 | 16 / 16 |
| 3 | Held, final tilt 4.6° | 5.51 | 16 / 16 |
| 4 | **Fell at 2.12 s** | 18.78 | 0 / 16 (already fallen) |

Diagnosis, not a fix. Seeds 0 and 4 start with the whole-body centre of mass about 0.12 m ahead of the feet, produced by the joint offsets. In seed 0 the ankle pitch command sits at its 160 N·m limit for about 630 control steps in the first 3 s, against about 50 for seed 1. The ankle authority looks exhausted, but the controller was not changed and this is not proven. Seeds 2 and 3 pass with little margin, reaching 5.5° during standing. Changing the controller needs a decision first (e.g. a stepping strategy, a different ankle/hip split, or a narrower initial-offset scope).

Follow-up sweeps on tuning seeds 100–104 (separate from evaluation seeds 0–4; quiet standing only, same ±0.02 rad offsets). None passes:
- Adding a centre-of-mass term to the hip pitch (either sign, gains 2–10): every seed falls.
- Ankle gain kc 0.5–3 with hip gain kh 2: 2–3 of 5 seeds fall, and the rest end at 3–20° tilt. The fall angle stays near 19–20° for all kc, so ankle authority is not the binding limit. The earlier saturation reading was not confirmed as the cause.
- kh 4–6: no fall, but a steady lean of 8–14°, which fails the criterion.
- Integral gain ki 0.3 and 1.0 on the ankle: no improvement.

Conclusion: the ankle-CoM plus hip-pitch law cannot meet the criterion on these offsets. Stepping (or a different balance formulation) is needed. The controller was not changed in the repository.

### Stepping attempt (`stepper.py`, experimental, not in the Stage A evaluation)

`stepper.py` adds a capture-point step to the balance law: xi = c + cdot/omega, checked against the sole outline. A step puts the swing foot on xi, with a damped least-squares IK swing and a stance leg on the balance law. The IK converges (residual 0 mm for a 5 cm / 2 cm target).

Result: **does not work yet.** Sweep on tuning seeds 100–104, quiet standing, 15 s:

| Trigger margin (m) | Max step (m) | Step time (s) | Falls (of 5) |
| --- | --- | --- | --- |
| +0.02 | 0.12 / 0.25 | 0.25 / 0.40 | 5 |
| −0.03 | 0.12 / 0.25 | 0.25 / 0.40 | 3 |
| −0.08 | any | any | 0 (no step ever taken; final tilts 3.1–6.7°, this is the balance law alone) |

Why it fails, from the trace: the robot starts 0.02 m from the toe edge of the sole, and already drifts forward at about 0.15 m/s from the joint offsets. The step unloads one foot while the lean is still growing, and the 0.25–0.4 s swing is slower than the fall, which comes at about 0.45 s. The balance law has no single-support mode: its reference is the midpoint of the two feet. A usable stepper needs a single-support stance law and a faster capture trigger. Those are not built.

Stage A stays FAIL. The stepper is not used in any evaluation or claim.

### Follow-up diagnosis (still no pass)

- **A forced in-place swing falls even with no step.** With the trigger disabled, a zero-displacement swing of the left foot (1.0–1.4 s) falls on all 5 tuning seeds at 2.2–3.5 s. The pelvis does not drop. Pitch grows during double support afterwards. So the stance law, not the swing, is marginal.
- **The ankle saturates.** The ankle command is kc × CoM error × kp 600. At 0.1 m error that is about 0.36 rad of target, which hits the 160 N·m limit within 0.6 s, and the centre of mass moves about 0.2 m forward before it recovers.
- **Ankle gain sweep, tuning seeds 100–104, quiet standing 20 s.** Ankle kp 600, 300, 150, 75: all settings with kp below 600 fall on all 5 seeds. kp 1200 with kc 3, kd 20: 0 falls, final tilt 3.4–7.0°. kp 2400: 0–2 falls.
- **Best tuning-set setting:** kp_ankle 1200, kc 3, kv 0.5, kh 3, kd 20. No falls on the tuning seeds, but seed 104 ends at 6.0°, above the 5° criterion. This setting has not been run on the held-out evaluation seeds, and was not promoted.

Stage A stays FAIL. The stance family (ankle CoM error plus hip pitch, PD per joint) does not give a robust standing law under these offsets, and no setting passes the tuning seeds. A stepper would sit on top of a stance law that is itself marginal.

### Model-based whole-body QP (`wbc.py`, experimental, not in the Stage A evaluation)

`wbc.py` solves, every 2 ms, a QP over joint accelerations and contact forces (four corners per sole, linearised friction, torque limits from the model). It uses tasks for CoM x/y and height, pelvis orientation, foot flatness and posture. The dynamics come from MuJoCo (M and bias), and the actuators are torque motors. Standing from the ±0.02 rad offsets on the tuning seeds 100–104 is **not achieved**: 4–5 of 5 seeds fall within 1–2 s.

Faults found and fixed while building it:
- **Rank-deficient contact rows.** Per-corner acceleration rows of one sole are rank 6 together. With finite-difference Jdot terms they were inconsistent and the QP returned no solution. Replaced by three linear rows per foot. The QP then solves at every step.
- **Start pose inside the floor.** The reset pelvis height (0.87 m) puts the soles 0.15 m below the floor. The pelvis is now raised so the lowest corner is on the floor.
- **Infeasible CoM demand.** With the feet fixed, the sideways CoM acceleration is limited by the zero-moment point to about g × 0.13 / 0.96 ≈ 1.3 m/s². A gain of 60 asked for about 5 m/s², and the QP correctly refused. Reduced gains did not stand.
- **Start pose on the toes.** The keyframe has both feet pitched about 8°, so only the toe corners touch the floor. Making the feet fixed in their current orientation held them tilted. A foot-flat task was added, but at gains 5–80 the feet oscillate or diverge, and the robot still falls.

Open: the foot-flat task needs its gains and weights designed against the foot and ankle dynamics, and the height and posture weights need the same treatment. The QP is not yet a standing controller.

**Resolved in a later pass:** the foot-flat task was the wrong fix. Feet pitched 7° can be levelled by the setup instead: an offset of −0.168 rad on both ankle pitch joints, found by forward kinematics (`wbc_eval.py`, `ANKLE_LEVEL`). With the feet rigid and the start levelled, the QP stands on the tuning seeds 100–104 with no falls and final tilt below 0.5°.

### Held-out evaluation of the QP (`wbc_eval.py`, Stage A criteria, seeds 0–4)

These seeds were not used to tune the QP. The criteria are the ones in `stage_a_eval.py` (quiet standing 20 s; pushes 25, 50, 75, 100 N, four directions, 0.2 s, recovery judged 15 s after the push). Both controllers start from the same levelled pose. The centre-of-mass target is fixed at the start position.

| Controller (level start) | Quiet standing | Pushes held (of 16 per seed) | Largest push held in every seed |
| --- | --- | --- | --- |
| Whole-body QP (`wbc.py`) | Held on 5/5, final tilt 0.1–0.5° | 14, 15, 15, 15, 14 | −x 100 N, ±y 100 N; +x 50 N |
| Balance law (`balance.py`) | Seed 0 ends at 8.7°; seed 3 at 5.0° | 0, 16, 16, 11, 16 | 0 N on some seeds |

**Verdict: Stage A FAIL under the declared criteria.** The QP falls on forward pushes of 75 N and 100 N in four of five seeds (seed 0 falls on 75 N and 100 N, seeds 1–4 on 100 N). All other pushes up to 100 N are held. The criteria require every push in every direction, so this does not pass.

### Stage A with the final QP configuration (`wbc_eval.py --com-target -0.08`)

The forward-push failure came from the centre-of-mass target. The start position sits 0.02 m from the toe edge, so a forward push leaves the support. Targets were tried on tuning seeds 100–104 only, with the full criteria:

| CoM target x (m, world) | Tuning seeds 100–104 |
| --- | --- |
| start (about −0.02) | forward pushes fail on 3 seeds (earlier verdict) |
| −0.06 | 1 push fails (+x 100 N, seed 101) |
| −0.08 | all 80 pushes and quiet standing pass |
| −0.11 (sole centre) | backward 100 N fails on 2 seeds |

Final check, with the target fixed at −0.08 m:

| Seeds | Role | Quiet standing (final tilt) | Pushes held | Result |
| --- | --- | --- | --- | --- |
| 5–9 | fresh, never used | held, 0.3–0.6° | 16/16 on every seed | **PASS** |
| 0–4 | used for the earlier FAIL, not for tuning | held, 0.4–0.6° | 16/16 on every seed | PASS |

Seeds 10–14 (never used for tuning or earlier evaluation; run once with the target fixed at −0.08 m, no further tuning; `runs/stage_a/wbc_heldout_10_14_m008.json`, git-ignored): quiet standing held on all five seeds (tilt 0.34–0.67°); 16/16 pushes held on every seed, 25–100 N in ±x and ±y. These seeds are **not** the held-out seeds named in `stage_a_eval.py` (0–4), so this result is a separate check, not the declared verdict.

Re-run of seeds 0–4 at the same settings (`runs/stage_a/wbc_heldout_m008.json`, git-ignored): all five seeds held quiet standing (tilt 0.40–0.59°) and 16/16 pushes; **PASS** again. Seeds 0–4 were run before the target was fixed (start-target FAIL), so this is a reproduction on a previously seen seed set, not a fresh held-out test.

**Fresh set 15–19 (run once, −0.08 m, no tuning; `runs/stage_a/wbc_heldout_15_19_m008.json`, git-ignored): FAIL.** Seeds 15–18 hold quiet standing and all 16 pushes (final quiet tilt 0.43–0.57°). Seed 19 does not: its quiet tilt peaks at 4.2° and ends at 3.9° with only two left-foot contacts. It falls forward under the +x 75 N push (at 7.1 s) and the +x 100 N push (at 6.6 s). It holds the other 12 pushes. Stage A fails on this set.

**Tuning round 2 (seeds 100–104 only; all 16 pushes; `wbc_tune_m007.json`, `wbc_tune_m009.json`, git-ignored).** Rule fixed before the runs: keep configurations that hold all 80 pushes with no falls on 100–104; among those, pick the lowest maximum quiet tilt. Results: −0.07 m holds 80/80 (max quiet tilt 0.52°); −0.09 m holds 80/80 (max quiet tilt 0.67°). The rule selects **−0.07 m**. Next: one run of −0.07 m on unused seeds 20–24, with no further tuning. That run is the verdict on this round.

**Round 2 verdict: −0.07 m on fresh seeds 20–24 (run once, no tuning, after the selection rule above was committed; `wbc_heldout_20_24_m007.json`, git-ignored): PASS.** All five seeds hold quiet standing (final tilt 0.39–0.51°) and all 16 pushes (25–100 N, ±x and ±y), with no falls. This is one unused set and one configuration chosen on 100–104. It is not a separate test of −0.08 m, which failed seed 19 on 15–19. Seeds 15–19 were not re-run with −0.07 m.
Still open: the declared Stage A criteria name seeds 0–4 as held-out. That question is for the project owner and is not decided by this result.

**Summary of the −0.08 m configuration on fresh seeds:** seeds 10–14 PASS, seeds 15–19 FAIL (seed 19). Seeds 0–4 were used for the start-target FAIL before the target was fixed, so they are not clean.

**Verdict: Stage A not passed under the declared criteria.** The declared criteria name seeds 0–4 as the held-out set. Those seeds were used for the start-target FAIL before the −0.08 m target was chosen, so they cannot give a clean verdict on this configuration. Whether the seeds 10–14 run counts as the held-out verdict, or the criteria must be changed, is the project owner's decision and is not yet recorded. Until it is, Stage A is not claimed to pass.

A result for seeds 5–9 is recorded in commit `f0cf235`. It was not re-run for this README and is not relied on here.

What this supports, and what it does not:
- Supports (simulation only): on the seeds run above, standing from ±0.02 rad joint offsets with pelvis pushes of 25–100 N, 0.2 s, in four directions, with no falls. Recovery time was not measured.
- Does not support: pushes above 100 N, walking, whole-body control beyond this QP, balance on the original (non-levelled) start, or any hardware transfer. The balance law from the same levelled start still FAILS (above).
- The levelled start and the −0.08 m target are setup and design choices. The Stage A criteria themselves are unchanged.

Commands: `.venv/bin/python sim/t800/wbc_eval.py --controller wbc --seeds 5 6 7 8 9 --pushes all --com-target -0.08` (results to `runs/stage_a/`, git-ignored).

Caveats:
- The setup changes (levelled start, fixed CoM target) are disclosed above and were applied to both controllers.
- Seeds 0–4 have now been used for this evaluation, so they cannot be used again as a clean held-out set. Any further tuning must use other tuning seeds.
- Simulation only. Pushes are applied to the pelvis for 0.2 s. No claim is made about larger pushes, walking, or hardware transfer.

Scope: simulation only. No claim of balance under larger pushes, walking, or hardware transfer.
- Only 1 contact-free test per push direction; no noise, delay or model error. This is simulation only.

## Native 3D viewer (run on your laptop)

`viewer.py` opens MuJoCo's own 3D window and runs the Stage A standing controller on the free-standing T800. Run it natively, not in Docker, because the window needs a display.

```
git submodule update --init --depth 1 third_party/T800-PROJECT
# One shared venv at the repo root; see the "Python environment" section of README.md
.venv/bin/python -m pip install mujoco==3.2.3 numpy==1.26.4   # first time only
.venv/bin/python sim/t800/viewer.py                          # stand; no push
.venv/bin/python sim/t800/viewer.py --push-x -150 --push-t 5 # 150 N backward push at 5 s
```

Options: `--push-x`, `--push-y` (N), `--push-t` (start, s), `--push-dur` (s, default 0.2), `--T` (stop after T s). Closing the window ends the run. Stop with Ctrl+C or `--T`.

Expected behaviour, from the sandbox headless run: quiet standing holds with a final tilt of 3.4°. The 150 N backward push at 5 s makes the robot fall at about 6.6 s. The window is not checked in the sandbox (no display there), so please report anything that looks wrong in the window.

Headless check (no window, runs anywhere): `.venv/bin/python sim/t800/viewer.py --headless --T 10`.


## Linux laptop: viewer in a container (X11)

Requires a Linux desktop session (X11, not Wayland-only) and Docker with the compose plugin.

```bash
scripts/run_viewer.sh
```

The script allows local X connections (`xhost +local:`) and starts the `viewer` service, which opens the MuJoCo 3D window on your display. Stop with `docker compose --profile sim-only stop viewer`.

Simulation only. Stage A is a free-standing balance test, not walking, and no claim is made about pushes beyond the table above or about hardware.

Not tested in the build sandbox: the window path (no display there) and the X11 mount on a real laptop. The headless path was tested.
