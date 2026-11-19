# 02 — System Architecture (T800 humanoid)

> **Platform.** The platform is the **T800 humanoid** (`docs/00` §4, `docs/01` §8). The wheeled-robot architecture has been removed from this document. Where a value is not yet known for the humanoid it is marked **TBD**, not carried over from the wheeled design.
>
> **Owner decisions recorded here:** the NMPC stays at L3, and the locomotion layer (L1 outer loop) and the joint servo (L1 inner loop) run on the MCU. The walking policy running on the MCU is **not yet shown feasible**. The fall-posture controller and the safe-stop design are **open** (§6).
>
> **Not yet specified:** the joint-level interface, the MCU frame format, and the humanoid safety-supervisor checks. No interface is frozen until they are (§8).

## 1. Layer model and rate bands

The system is organised as five layers. Each layer runs at least 5–10× slower than the layer below it, so the inner loop is transparent to the outer one.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ L5  MISSION POLICY ................................................ 1–5 Hz   │
│     RL policy: goal sequencing, behaviour selection, speed policy           │
│     In: map, goal list, battery, perception summary, safety status          │
│     Out: behaviour command + goal waypoint          [learning-enabled]      │
├─────────────────────────────────────────────────────────────────────────────┤
│ L4  GLOBAL PLANNING ............................................ 0.5–2 Hz   │
│     A*/Dijkstra on inflated occupancy grid → global path                    │
│     In: map + goal          Out: path in world frame                        │
├─────────────────────────────────────────────────────────────────────────────┤
│ L3  LOCAL PLANNING & TRACKING (NMPC) ........................... 20–50 Hz   │
│     NMPC on the base: obstacle-constrained trajectory optimisation          │
│     In: path, local costmap, state estimate, safety field                   │
│     Out: body-frame velocity reference (vx, vy, ω)                          │
├─────────────────────────────────────────────────────────────────────────────┤
│ L2  SAFETY SUPERVISOR & FILTER ................................ 100–200 Hz  │
│     Feasibility monitor, speed limiter, watchdog, veto over L3 and L5       │
│     In: L3/L5 command, obstacle distance, health signals                    │
│     Out: admissible command              [independent of L5]                │
├─────────────────────────────────────────────────────────────────────────────┤
│ L1  LOCOMOTION (outer, 100 Hz) + JOINT SERVO (inner, ≥ 1 kHz target) ...    │
│     Both run on the MCU (owner decision; walking-policy feasibility open)   │
│     In: body velocity reference, IMU, joint states, foot contact            │
│     Out: joint torque/position commands                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ L0  PERCEPTION & ESTIMATION (runs in parallel, feeds L3–L5)                 │
│     LiDAR → scan matching/SLAM (10–20 Hz)   Camera → detection/depth (30 Hz)│
│     IMU + joint states + foot contact → state estimate (100–1000 Hz)        │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Key architectural rule:** L5 (learned) and L4 cannot command the actuators directly. All commands pass through L2, which is not learning-enabled and holds veto power over L3 and L5.

Rates marked as targets (the joint servo at ≥ 1 kHz, the NMPC at 20–50 Hz) are not measured on the target hardware. Estimation has no wheel odometry: the state comes from IMU, joint states and foot contact.

## 2. Compute allocation

| Unit | Runs | Rationale |
| --- | --- | --- |
| **MCU** (STM32-class, FPU, ≥ 100 MHz) | L1 locomotion (100 Hz) and joint servo (≥ 1 kHz target), IMU sampling, joint-state capture, watchdog, serial/CAN bridge, stop path | Hard real-time, deterministic. Owner decision: L1 runs here. The processor budget for the walking policy is **not yet measured** |
| **Edge GPU computer** (Jetson-class, ≥ 8 GB) | L0 estimation and perception, L2 supervisor, L3 NMPC, L4 planning, L5 policy, logging | Floating-point and neural inference; soft real-time with measured jitter |

The interface between them is a **bounded, versioned serial or CAN protocol**: fixed-length frames, sequence numbers, CRC, explicit limits, and a heartbeat. The frame format is not frozen (§8).

The edge computer may fail at any time. What the robot does then depends on the MCU-side rule in §6, which is **open**. No claim is made that the robot is safe without the edge computer.

## 3. ROS 2 node graph (humanoid)

| Node | Layer | Subscribes | Publishes | Rate | QoS |
| --- | --- | --- | --- | --- | --- |
| `lidar_driver` | L0 | — | `scan` | 10–20 Hz | SensorDataQoS (best-effort) |
| `camera_driver` | L0 | — | `image_raw`, `depth`, `camera_info` | 30 Hz | SensorDataQoS |
| `imu_driver` | L0 (MCU source) | — | `imu/data_raw` | 200–1000 Hz | SensorDataQoS |
| `joint_state_driver` | L0 (MCU source) | — | `joint_states` | 100–1000 Hz | SensorDataQoS |
| `state_estimator` | L0 | `imu/data`, `joint_states`, foot contact, `scan` | `odom/fused`, TF `odom→base_link` | 100–200 Hz | Reliable |
| `slam` | L0 | `scan`, `odom/fused` | `map`, TF `map→odom` | 10–20 Hz | Reliable + transient local |
| `perception_detector` | L0 | `image_raw`, `depth` | `detections`, `obstacles` | 15–30 Hz | Reliable |
| `costmap_builder` | L0 | `scan`, `obstacles`, `map` | `local_costmap` | 10–20 Hz | Volatile |
| `global_planner` | L4 | `map`, goal | `global_path` | on request | Reliable |
| `nmpc_controller` | L3 | `global_path`, `local_costmap`, `odom/fused` | `body_vel_ref` (vx, vy, ω) | 20–50 Hz | Reliable |
| `safety_supervisor` | L2 | `body_vel_ref`, `safety_field`, health | `body_vel_safe`, `safety_state`, `estop` | 100–200 Hz | Reliable |
| `rl_policy` | L5 | `map`, summary, goal list | `behavior_cmd` | 1–5 Hz | Reliable |
| `mcu_bridge` | L2 (MCU path) | `body_vel_safe` | serial/CAN frame | 100 Hz | Reliable |
| `logger` | all | selected topics | MCAP bag + JSONL events | event/periodic | Reliable, buffered |

**QoS discipline:** sensor streams are best-effort with a shallow queue. Commands are reliable with a depth of 1 (the newest command only; a stale command is worse than a dropped one). The map and any single-writer topic use transient-local durability.

**Executor discipline:** drivers and the control path run in separate callback groups with a multi-threaded executor. No control-path callback performs allocation, I/O or logging. Image transport uses zero-copy shared memory where available.

**Humanoid estimation.** The estimator fuses IMU, joint states and foot contact. Its covariance is required by §6. Its accuracy on the T800 is not yet measured.

## 4. Timing and synchronisation

| Item | Requirement |
| --- | --- |
| Clock | All nodes use the monotonic ROS clock; sensor timestamps are taken at acquisition (hardware), never at publish |
| Time sync | MCU ↔ computer over PTP or a compensated round-trip estimate; drift monitored and logged |
| Latency budget | Sensor acquisition → perception → costmap → NMPC → safety supervisor → MCU → locomotion → joint servo → actuators, measured end-to-end and reported as p50/p99/max (PR-06) |
| Jitter | Control-path period jitter measured and reported. A controller whose timing is unmeasured is not considered validated |
| Bagging | MCAP with per-topic recording and an event log (state transitions, safety interventions) |

## 5. Control architecture

```
        goal list
            │
            ▼
   ┌─────────────────┐   behaviour, waypoint
   │  L5 RL policy   │───────────────┐            [learned, unverified]
   └─────────────────┘               │
            ▲                        ▼
            │              ┌──────────────────────┐
     map, perception ───── │ L4 global planner    │
     summary, health       └──────────────────────┘
                                      │ global path
                                      ▼
                            ┌──────────────────────┐   body_vel_ref (vx, vy, ω)
                            │ L3 NMPC (base)       │──────────┐   [optimal, model-based]
                            └──────────────────────┘          │
                                      ▲                       ▼
                            state, costmap      ┌──────────────────────────┐
                                                │ L2 SUPERVISOR & FILTER   │
                                                │ + feasibility monitor    │
                                                │ + speed limiter          │
                                                └──────────────────────────┘
                                                             │ body_vel_safe
                                                             ▼
                                                   ┌──────────────────────┐
                                                   │ L1 locomotion (MCU)  │
                                                   │ → joint servo (MCU)  │
                                                   └──────────────────────┘
```

The rule that L5 and L4 cannot command actuators directly holds. The safety filter method for the humanoid (replacing the wheeled CBF-QP on `(v, ω)`) is **TBD**. The L5 choice of behaviour is bounded (§9.4).

## 6. Degradation ladder

The system never fails silently. Each level is a state published on `safety_state`, logged with its trigger, and covered by a test.

| Level | Trigger | Response |
| --- | --- | --- |
| **Nominal** | All health signals good | Full autonomy; L1 follows the L3 body-velocity reference |
| **Degraded sensing** | One perception modality lost or stale | Reduced speed limit, larger margins; map cells from the lost sensor marked unknown |
| **Reduced autonomy** | Localisation uncertainty above threshold, or map inconsistency | Stop exploring; L1 stops the gait and holds stance; speed limited |
| **Protective stop** | Safety field violated, or NMPC infeasible beyond the recovery window | Hold stance. The gait is stopped by ramping the velocity command to zero, **not** by removing torque |
| **Fall mitigation** | Stance cannot be held (tilt or loss of foot contact beyond a threshold) | Fall posture, **if** a fall-posture controller exists. **Open**: not designed, and the MCU may not be able to run it without the edge computer |
| **Safe stop** | MCU heartbeat loss, watchdog timeout, E-stop, critical fault | **Open.** Removing torque on a humanoid is a fall, and no brake exists. The MCU-side rule is undecided (see below) |
| **Recovery** | Operator clearance | Stand-up is **out of scope** until it has its own design |

**Edge-computer loss (open).** The current proposal is: the MCU keeps the locomotion and joint servo running and holds stance, rather than removing torque while upright. If it cannot hold, the only MCU action left is torque removal, which is a fall. Whether a fall-posture controller runs on the MCU is undecided. The heartbeat timeout is **TBD** and must be set from measured hold time in simulation.

**Feasibility rule:** if the NMPC problem is infeasible or the solve exceeds its deadline, the controller falls back in this order: (1) re-solve with slack-relaxed constraints, (2) a conservative reactive controller with a verified stopping envelope (**TBD** for the humanoid), (3) protective stop. The fallback chain is implemented and tested *before* the NMPC is tuned.

## 7. Data and logging architecture

| Artifact | Format | Purpose |
| --- | --- | --- |
| Sensor/state streams | MCAP bags, per-run directory | Reprocessing, debugging, training data |
| Event log | JSONL (state transitions, safety interventions, solver stats, errors) | Timeline reconstruction, safety audit |
| Per-run metadata | JSON (git commit, image digest, config hash, seed, hardware, scenario ID) | Reproducibility (SR-05) |
| Run report | JSON + Markdown summary generated by `ci/` tooling | Comparison across scenarios and versions |
| Datasets / weights | External object store, referenced by checksum in-repo | Keeps the repository small (SR-07) |

**Rule:** a result is only reportable if the run directory contains a complete metadata record. Tooling rejects runs without it.

> The MCU frame format is not frozen for the humanoid (§8).

## 8. Interface freeze

Before implementation begins, these interfaces are written down and frozen: the MCU serial/CAN frame format (fields, units, scaling, CRC, limits, heartbeat), all ROS 2 message types and topic names with QoS, the configuration file schema (for controller, policy, perception and scenario parameters), the run-metadata schema, and the container entrypoints. Interfaces change only through an explicit change record appended to this document — never silently during development.

## 9. Layer decisions for perception, planning and decision making (proposed, under review)

**Status:** proposal, not yet agreed. Recorded as a change record under the rule in §8. Nothing in §§1–8 is changed by this section. The sim-to-real layer is deliberately excluded and will be specified separately.

**Motivation.** The mission environment is unmapped and contains static and movable objects and people. The dominant nonlinearity has two sources: known dynamics and constraints (foot contact, actuator limits, obstacle geometry), and unknown interaction (how a movable object moves when contacted, how a person reacts). These are handled by different tools.

### 9.1 Perception (L0): output contract

Perception produces a typed, bounded interface. Pixels never reach L3.

- **Obstacle list:** each object with class (static, movable, person, unknown), position, velocity, a covariance on each, and a detector confidence.
- **Local costmap:** built from LiDAR, with an unknown-cell layer that is distinct from free space.
- **Person class:** a separate class with a larger minimum separation (value set in scenario S2, not here).
- **Validity:** every item carries an age. Items older than the staleness limit are removed, and the affected region is treated as unknown, not free.

### 9.2 Interaction prediction (new sub-layer, L0–L3 boundary)

A learned prediction model estimates the future positions of movable objects and people over the NMPC horizon, with uncertainty. Its output enters L3 only as constraints, never as a command. If its uncertainty is above a threshold, L3 uses worst-case constraints for that object.

### 9.3 Planning (L3–L4)

- **L4 global planner:** unchanged (A*/Dijkstra on the inflated occupancy grid), rerun on request or when the map changes significantly.
- **L3 local planner:** NMPC for known dynamics and constraints, solved with the real-time iteration scheme at 20–50 Hz. It tracks the global path and handles obstacles from §9.1 and §9.2.
- **Fallback:** §6 feasibility rule applies unchanged.

### 9.4 Decision making (L5)

The learned policy at 1–5 Hz chooses among a bounded set of behaviours (for example: continue, slow near people, wait, re-route, request assistance). It does not output trajectories or velocities. L3 executes the chosen behaviour, and L2 holds veto power over everything.

Deterministic logic decides the following, not the learned policy: mode changes, the degradation ladder (§6), return-to-start, and emergency stop.

### 9.5 Rationale and rejected alternatives

- **End-to-end learned control** was rejected. It has no explicit constraint handling, and the safety argument would rest on the learned policy alone.
- **A learned estimator as the sole state source** was rejected. Covariance is required by §6 and by the degradation ladder, and a learned estimator does not provide a checkable one.
- **Learned perception feeding the NMPC directly as pixels or raw detections** was rejected. The interface would be untyped and its failure modes unbounded.

### 9.6 Alternatives to evaluate before freezing

These are open alternatives, not yet compared on evidence:

- **Sampling-based MPC (e.g. MPPI)** as a replacement for NMPC at L3. It handles non-convex obstacle geometry without a gradient and parallelises on the GPU. Its weaker points are hard constraint satisfaction, certifiability and tuning sensitivity. Decide only after a benchmark on the scenarios S1–S5.
- **Robust safety filter with a learned barrier** (Q-CBF style, or gameplay-filter style) as an additional layer around L2. It treats moving objects as adversarial disturbances. It complements NMPC rather than replacing it.
- **Interaction-aware planning with explicit multi-agent prediction**, as an alternative to §9.2. It is more expensive and needs a validated prediction model, which we do not have.

### 9.7 Evaluation criteria for choosing between alternatives

All candidates are compared on identical runs of scenarios S1–S5 (`docs/00` §2), with these measured outcomes: tracking error against a classical baseline; minimum distance to people and to movable objects, and the count of safety interventions; p99 solve time on the target hardware; and the fraction of commands overridden by L2. No candidate is adopted as "optimal" without these measurements.

### 9.8 Open items

- Confirm the output contract in §9.1 with the perception design (`docs/theory/07`).
- Decide whether §9.2 is needed in the first milestone or deferred.
- Specify the behaviour set for §9.4.
- Run the §9.6 comparison before any NMPC tuning begins.

## 10. Change record: platform revision

- **Platform:** the T800 humanoid (`docs/00` §4, `docs/01` §8, `docs/11`). The wheeled architecture is removed from §§1–6 and from the ROS graph.
- **Removed (wheeled):** the `(v, ω)` references, the wheel-drive FOC cascade, wheel odometry and encoders, the wheel bridge (`bridge_mcu` / `wheel_cmd`), `wheel_odom`, the wheel-torque L1, the wheeled safe-stop rule (torque removal with brakes), and the CBF-QP filter on `(v, ω)`.
- **Folded in (from the former §11 draft):** the NMPC stays at L3; L1 locomotion and the joint servo run on the MCU; the walking-policy feasibility on the MCU and the edge-loss rule are open. The former §11 is now in §§1–6.
- **Not yet specified:** the joint-level interface, the MCU frame format, the humanoid safety-supervisor checks, and the fall-posture controller. No interface is frozen for the humanoid, and the interface freeze in §8 applies once they are.
- **Run record (§7, schema v1, implemented):** `planning/runmeta/collect.py` writes `meta` into every result: scenario ID, seed, git commit, dirty flag, container digest (from `ARS_IMAGE_DIGEST`; `not-in-container` otherwise), config and its SHA-256, hardware, package versions, command, and UTC time. `planning/runmeta/validate.py` rejects records that are missing, dirty, or from a non-git checkout. The schema is in code; changes to it need a new entry here.
- **Configuration schema v1 (§8, implemented):** `planning/config_schema.py` defines the controller and simulation parameters for the planar base. Each field has a type, a unit, a range, and an ASSUMPTION flag. Unknown keys and out-of-range values are errors. `planar_base.py` validates its effective config before running and records `config_schema_version` in the run record. Covers the Stage 0 controller and simulation only. Schemas for the MCU frame, the humanoid joint interface, and the perception and planning outputs are not yet written.
- **L2 supervisor logic (§6, implemented, not integrated):** `planning/safety/supervisor.py` implements the ladder states nominal, degraded sensing, reduced autonomy, protective stop and safe stop. It has a command watchdog (SF-05), a distance-based speed limit (SF-06), an infeasibility counter, and a latched safe stop released only by operator clearance. Its parameters are ASSUMPTIONS. Safe stop only zeroes the velocity command. What the humanoid does on safe stop is still open (§6), so no torque-removal behaviour is implemented or claimed. Not yet connected to the planar simulation.
- **Unchanged:** the layer model structure, the logging architecture (§7, except the frame format), the timing rules (§4), the feasibility rule (§6), and the perception and planning contract in §9, which remains a proposal under review.

