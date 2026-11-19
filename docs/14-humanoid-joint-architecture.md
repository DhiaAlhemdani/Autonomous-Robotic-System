# 14 — Joint-Level Architecture for the T800 Humanoid

**Status: design proposal, not frozen.** This document specifies the joint-level and locomotion layers for the T800 platform decided in `docs/11` §7. It follows the change rule in `docs/02` §8. Interfaces here are **drafts**: nothing is frozen, and no value is validated. All statements about the T800 come from the upstream repository at the pinned commit `8bfd5b4` (submodule `third_party/T800-PROJECT`). Simulation only. No hardware has been run.

Scope: the layers below the navigation and planning layers of `docs/02`. Navigation, perception and the learned mission policy are out of scope here.

---

## 1. What the T800 model provides (sourced facts)

Sources are paths inside the submodule. Values were read from the files; none was measured on a robot.

| Item | Value | Source |
| --- | --- | --- |
| Actuated joints | **25**: legs 12 (J00–J11), torso yaw 1 (J12), arms 10 (J13–J22), head 2 (J23–J24) | `assets/resource/robot/t800/xml/serial_links.xml`, `serial_actuators.xml` |
| Joint torque limits (`ctrlrange`, N·m assumed) | Hip pitch ±415, knee ±415, hip roll ±370, hip yaw ±222, ankle pitch and roll ±160; torso yaw ±222; shoulder and elbow pitch/roll ±160; elbow yaw ±52; head ±52 | `serial_actuators.xml` (**unit not confirmed upstream**) |
| Link mass | **74.8 kg**, computed as the sum of link inertials in `serial_links.xml` (29 bodies). Not an official total mass. | sandbox sum of `mass=` fields |
| Base state | Position, velocity and orientation of the base site, **from the simulator**. | `serial_sensors.xml` (`base_link_*`) |
| IMU | Orientation, linear acceleration, angular velocity. | `serial_sensors.xml` (`imu_*`) |
| Foot contact | Force and torque on each foot (`force_left_foot`, `torque_left_foot`, and the right foot). | `serial_sensors.xml` |
| Ankle | **Parallel linkage**: motor-to-joint transform is a fitted polynomial (`xoffset`, `gain`, `b1`, `b2`). | `assets/config/t800/parallel_ankle/default.yaml` |
| Locomotion | RL walking policy (MLP, `.mnn` file, 4.3 MB), 100 Hz control step (`control_dt` 0.01 s). | `assets/config/t800/rl_walking_example/default.yaml`, `policy/` |
| Operating modes | `sim` and `robot` mode maps; includes `pd_stand`, `passive`, `stand_up`, and the RL modes. | `assets/config/t800/mode.yaml` |

**Consequences for the design:**

- **Base pose from the simulator is ground truth.** The base pose sensor is not a sensing result. Our estimator must not use it. It can be used only to score an estimator in simulation (doc 11 §7 says the same).
- **The walking policy has no position input.** It takes only proprioception (joint offsets and velocities, previous action, body-frame angular velocity, projected gravity) and a 3-value velocity command. Navigation has to be built above it, through the velocity command.
- **The ankle is a parallel mechanism.** Motor commands and joint states differ from joint commands and states. The transform must be applied on both sides, and tested.

---

## 2. Layer model for the humanoid

Joint-level control is split into five layers. The rate rule of `docs/02` §1 applies: each layer is at least 5–10× slower than the one below it.

```
  L5  Navigation & command shaping      ~10–20 Hz   path → (vx, vy, ω) command   [docs/02 L3–L4, revised later]
  ───────────────────────────────────────────────────────────────────────────────
  L4  Locomotion (balance & gait)        100 Hz      RL walking policy (baseline)
                                                     or pd_stand; joint targets
  ───────────────────────────────────────────────────────────────────────────────
  L3  Joint servo / actuation            ≥ 1 kHz*    PD on each joint; torque limits;
                                                     parallel-ankle transform
  ───────────────────────────────────────────────────────────────────────────────
  L2  State estimation                   100–1000 Hz IMU + joint states + foot contact
                                                     → base state with covariance
  ───────────────────────────────────────────────────────────────────────────────
  L1  Safety supervisor (independent)    50–200 Hz   fall detection, degradation ladder,
                                                     stop path; veto over L4–L5
```

\* The servo rate is not given upstream. The rate shown is the target for a real joint loop and is **not yet verified**. In simulation the physics step is 0.002 s (`balance.py`, our Stage A setup); the upstream policy steps at 0.01 s.

### 2.1 L3 — Joint servo and actuation

- **Input:** joint position targets from L4 (22 controlled joints for the walking policy; see §3.2).
- **Output:** joint torques, bounded by the `ctrlrange` limits in §1.
- **Gains:** the upstream walking policy sets PD gains per joint (`joint_kp`, `joint_kd` in its default config). Those values are the baseline. They are not tuned for our simulation or for hardware.
- **Parallel ankle:** the transform in `parallel_ankle/default.yaml` maps between motor and joint space. Its validity range is not stated in the upstream file. The design requires a test of the transform's round trip and of its range before the ankle is commanded.

### 2.2 L4 — Locomotion (balance and gait)

- **Baseline:** the upstream RL walking policy, adopted as a baseline per `docs/11` §7. It is **not evaluated** by this project.
- **Other modes:** `pd_stand` (hold a standing pose) and `passive` (no torque). `passive` is a safe-state candidate only after §4 has decided what a fall does.
- **Input:** a velocity command `(vx, vy, ω)` in the body frame, from L5. The upstream command comes from a gamepad, with per-axis scaling (`command_scale_pos`, `command_scale_neg`) and a low-pass filter (`remote_command_cut_off_frequency` 0.1 Hz at 40 Hz sampling).
- **Output:** joint position targets for the 22 controlled joints.
- **Observation** (per step, 72 values, 15 history steps, plus the 3-value command): joint offset from default (22), joint velocity (22), previous action (22), body-frame angular velocity (3), projected gravity (3). The composition is in `rl_walking_example_runner.cc`.
- **Limitation for our use:** the policy has no base velocity or position observation. Its velocity tracking is not measured by this project.

### 2.3 L2 — State estimation

- **Inputs:** IMU, joint positions and velocities, foot force/torque.
- **Outputs:** base orientation, base velocity, and contact state for each foot. Each output carries a covariance.
- **Requirements:**
  - Not use the simulator base pose (§1).
  - Use foot contact to decide stance and swing, with a stated threshold.
  - Report drift against the simulator base pose **only in evaluation**, not in the estimator.
- **Upstream reference:** `motion_state_estimator/default.yaml` fuses joint velocity (weight 0.8) with IMU (weight 0.2) and uses per-joint velocity thresholds. Its joint labels do not match the model: the config numbers the twelve leg joints J01–J12, while the model numbers them J00–J11, and some names differ (for example `J03_HIP_PITCH_L` in the config, where the model has `J03_KNEE_PITCH_L`). This must be resolved before the config is used. Until then, the estimator is ours to design, and the upstream file is a reference only.

### 2.4 L5 — Navigation and command shaping

- **Output:** `(vx, vy, ω)` at 10–20 Hz, bounded by the limits in §4.
- **Input:** the path and goal from the navigation layer (`docs/02` L3–L4). That layer is not revised here.
- **Shaping:** a limit on acceleration and on yaw rate, so the walking policy receives commands inside its training range. The range is not documented upstream and must be measured.

### 2.5 L1 — Safety supervisor

Independent of L2–L5 in logic and in its stop path (`docs/01` SF-01). The design is under the rules of `docs/02` §6, but a humanoid changes the stop behaviour: removing torque makes the robot fall. Section 4 covers this.

---

## 3. Interfaces (draft, not frozen)

Names and units are proposed. Ranges marked **TBD** must be measured in simulation before they are frozen.

### 3.1 Velocity command (L5 → L4)

| Field | Unit | Range | Source of range |
| --- | --- | --- | --- |
| `vx` | m/s | TBD | upstream scale only: forward +1.0, backward ×0.6 |
| `vy` | m/s | TBD | upstream scale only: lateral ×0.4 both ways |
| `yaw_rate` | rad/s | TBD | upstream scale only: ×1.0 both ways |
| `stamp` | s | — | sensor-independent monotonic clock |
| `valid` | bool | — | false → L4 holds the last safe pose |

### 3.2 Joint command (L4 → L3)

- 22 joint position targets (the walking policy's controlled set), in the order of `active_joint_names`.
- Torso yaw and head joints (J12, J23, J24) are **not** in the walking policy's controlled set. The policy config does not say how they behave in walking mode. The design assumes they hold their default pose, and this must be confirmed by test.

### 3.3 Base state (L2 → L4, L5, L1)

| Field | Unit | Notes |
| --- | --- | --- |
| `orientation` | quaternion | base frame, with covariance |
| `angular_velocity` | rad/s | body frame |
| `linear_velocity` | m/s | body frame; covariance required |
| `contact[2]` | bool + confidence | per foot |
| `valid` | bool | false if any input is stale |

### 3.4 Safety state (L1 → all)

- `level`: `nominal`, `degraded`, `protective_stop`, `safe_stop`, `recovery` (`docs/02` §6).
- `reason`: the trigger, logged as an event.

---

## 4. Safe state for a fall (proposal, decision open)

**Status:** proposal. The safe-state choice is the project owner's decision (O4). Every number in this section is an **ASSUMPTION** until tested in simulation. Nothing here is a validated safety property.

### 4.1 Why the existing rule does not transfer

Doc 02 §6 defines a safe stop as torque removed and brakes engaged. A humanoid has no brakes. Removing torque from a standing robot lets it fall under gravity, so "torque off" is itself a fall. The safe state therefore has to be chosen per situation, not set once.

### 4.2 Safe states

| State | What the joints do | Allowed when | Exit |
| --- | --- | --- | --- |
| **S0 Nominal** | L4 locomotion follows L5 commands | All triggers in §4.3 clear | Trigger → S1 or S2 |
| **S1 Degraded** | L5 commands limited to zero; L4 stops the gait and returns to `pd_stand` | Single soft trigger (e.g. command timeout, estimator warning) | Clears for T_hold → S0 (only with operator reset); escalates → S2 |
| **S2 Hold stance** | `pd_stand` at reduced gains; no stepping | Robot upright and both feet in contact; tilt under θ_hold | Tilt over θ_hold or loss of contact → S3 |
| **S3 Fall mitigation** | Fall posture (arms forward, knees bent, torso flexed) commanded for the predicted fall direction | Fall predicted (§4.3) and S2 cannot recover it | Impact detected → S4; or posture reached and stable → S4 |
| **S4 Torque off (limp)** | Joint torques set to zero | Only after impact (contact on a non-foot body, or joint load spike), or when S3 has not reached its posture within T_posture | Stays until operator reset and a stand-up procedure (not in scope here) |

Key constraints:

- **Torque removal is never the first response to a robot that is upright.** Torque is removed only in S4, and only after S3 has run or an impact has already happened.
- **S2 is not a stop from walking.** A walk is stopped by ramping the command to zero (L5) and letting L4 settle into `pd_stand`. A stop that cuts the gait abruptly is treated as a trigger for S3.
- **S3 must know its fall direction.** Direction comes from the estimator's tilt and angular velocity. If the estimator is invalid, the robot goes straight to S4 under this rule, and this is a known weak case (see §4.6).

### 4.3 Triggers and thresholds

Each trigger is a sensor-derived signal. None uses the simulator base pose.

| Trigger | Signal | Threshold (ASSUMPTION) | Used for |
| --- | --- | --- | --- |
| Tilt | Projected gravity, body frame | θ_warn = 10°, θ_hold = 20°, θ_fall = 35° | S1 → S2 → S3 |
| Tilt rate | Body angular velocity | ω_fall = 2.0 rad/s, sustained for 0.1 s | Predicts a fall to S3 |
| Loss of contact | Both foot force below F_min | F_min = 20 N, for 0.1 s | S2 → S3 |
| Command timeout | L5 `valid` false, or no command for T_cmd | T_cmd = 0.3 s | S0 → S1 |
| Estimator invalid | Stale or non-finite base state | any, for 0.05 s | S0 → S1; S3 uses last valid direction |
| Joint saturation | Joint torque at `ctrlrange` for a sustained period | T_sat = 0.2 s | S2 → S3 |
| Impact | Non-foot contact or joint torque spike | spike > 2× the nominal peak (to be measured) | S3 → S4 |
| Posture timeout | S3 has not reached posture | T_posture = 0.5 s | S3 → S4 |

The thresholds are placeholders to be **measured** in simulation (§4.5). They are not chosen from any robot data.

### 4.4 Options considered

1. **Hold only.** Keep `pd_stand` and stop. Works only for small tilts. Does nothing for a push that exceeds the balance margin.
2. **Fall mitigation, then torque off (the proposal above).** Lowers impact energy and avoids damage to joints. Needs a posture that is designed and tested. The posture is **not yet designed**.
3. **Torque off immediately.** Simple and does not depend on the estimator. The robot falls uncontrolled and the impact is largest.

**Recommendation (proposal):** use option 1 for S1 and S2, and option 2 for falls. Option 3 is used only in S4, after impact or posture timeout. This is not a recommendation for hardware. It is a simulation design to be tested.

### 4.5 Evaluation in simulation (adds E7–E9 to §5)

| Test | Procedure | Metrics | Gate |
| --- | --- | --- | --- |
| E7 Push ramp | Push the pelvis at increasing force, from standing and during walking | Force at which S2 fails; state sequence; time in each state | Reported; no gate |
| E8 Stop from walk | Command `(vx,0,0)` then zero; zero-command ramp length varied | Stops upright (tilt < θ_hold); time to stance | Reported |
| E9 Fall posture | Forced fall in each direction (forward, backward, lateral) | Peak pelvis and head impact, peak joint torque, self-collision count, state sequence | Reported; posture is accepted only if the impact is lower than option 3 |

Fixed seeds, the commit, and the MuJoCo version are recorded, as in §5. The simulator base pose is used only to score the outcome of each test, never as an input to the state machine.

### 4.6 Known gaps

- **No posture is designed yet.** S3 is a state with no tested motion behind it.
- **Estimator failure.** If the estimator is invalid during a fall, direction is unknown and S3 cannot choose a posture. The fallback is S4, with the impact it causes.
- **Stand-up is out of scope.** The upstream `stand_up` plan exists in `mode.yaml` but is not evaluated here. Recovery from S4 needs a separate design.
- **Sim-to-real.** None of the thresholds, posture or impact figures transfer to hardware without evidence (O8).

### 4.7 Decisions needed from the project owner

1. Confirm the S2/S3/S4 split above, or choose option 1 or 3 only.
2. Decide whether a robot with an invalid estimator should fall through to S4 or hold.
3. Approve E7–E9 as the evaluation before any interface is frozen.

---

## 5. Simulation and evaluation plan

Simulation first, in MuJoCo (`docs/11` §2.5), using the T800 model at the pinned commit. Each step has a measurable outcome.

| Step | Test | Metric | Gate |
| --- | --- | --- | --- |
| E1 | `pd_stand` for 20 s | final tilt, pelvis height, foot contacts | Matches the upstream result, or the difference is explained |
| E2 | Walking policy, command `(0,0,0)` | fall rate, drift from start pose (simulator, evaluation only) | Stands for 60 s with no fall |
| E3 | Walking policy, step commands `vx`, `vy`, `ω` | command tracking error (mean, p95), fall rate | Measured; no gate yet |
| E4 | Estimator vs simulator base pose | orientation error, velocity error, drift over 60 s | Reported; no gate yet |
| E5 | Parallel-ankle transform round trip | max error in joint and motor space | Under a stated tolerance, to be set |
| E6 | Fall detection on pushes | detection latency (sim time), false positives in E2 | Reported |

All results record the commit, the seed, the MuJoCo version, and the time step (`docs/01` SR-05). They are sandbox simulation results, not robot results.

The Stage A standing controller (`sim/t800/balance.py`) is a separate experiment. It is a free-base, two-strategy test and is not the locomotion layer. Its results are not carried into this design.

---

## 6. Open items and risks

| # | Item | Why it matters | Owner |
| --- | --- | --- | --- |
| O1 | **Licence and provenance** of the walking policy weights and of the EngineAI SDK | Adoption as a baseline (`docs/11` §7) is conditional on it. Not checked. | project owner |
| O2 | Torque unit in `serial_actuators.xml` (N·m assumed) | Limits and gains depend on it | verify upstream |
| O3 | Joint-label offset in `motion_state_estimator/default.yaml` | Wrong joint mapping would corrupt the estimate | fix or document before use |
| O4 | Safe state for a humanoid (§4): S2/S3/S4 split proposed, decision open | Doc 02 §6 assumes torque removal is safe; it is not for a humanoid | project owner |
| O5 | Independent supervisor (SF-01) is not in the upstream design | Architecture requirement; upstream process layout not checked | design |
| O6 | Velocity-command ranges (§3.1) | Policy behaviour outside its training range is unknown | measure in E3 |
| O7 | Mass and inertia | The 74.8 kg is a sandbox sum, not an official value | verify with the model owner |
| O8 | Real-robot match to the model | Unknown; sim results do not transfer without evidence | later |

**Explicit non-claims.** This document does not claim that the walking policy works in our simulation, on the real T800, or in any environment outside the upstream example. It does not claim a safety property. It does not claim the estimator, the supervisor, or the stop path meets any requirement in `docs/01`. Each of those needs the evaluation in §5.

---

## 7. Change record

- **Basis:** `docs/11` §2.1 and §2.2 (superseded wheeled items), §5 (open decisions), §7 (T800 decision). Revision of `docs/02` §§1–8 remains the platform revision of `docs/02` §10.
- **Added:** this document. No existing interface is changed. No frozen interface exists yet for the humanoid.
- **Next:** resolve O1–O4 with the project owner, then run the §5 evaluation before any interface is frozen.
