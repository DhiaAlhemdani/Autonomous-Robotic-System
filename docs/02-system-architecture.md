# 02 — System Architecture

## 1. Layer model and rate bands

The system is organised as five layers spanning four orders of magnitude in bandwidth. The rate separation is not cosmetic: each layer must be at least 5–10× slower than the layer below it, so that the inner loop is transparent to the outer one (a standard cascade-control requirement).

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
│     NMPC: obstacle-constrained, kinodynamic trajectory optimisation         │
│     In: path, local costmap, state estimate, safety field                   │
│     Out: (v, ω) or wheel torque references                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│ L2  SAFETY FILTER & SUPERVISOR ................................ 100–200 Hz  │
│     CBF-QP shield + feasibility monitor + speed limiter + watchdog          │
│     In: RL/MPC command, obstacle distance, health signals                   │
│     Out: admissible command              [independent of L5]                │
├─────────────────────────────────────────────────────────────────────────────┤
│ L1  DRIVE: PMSM FIELD-ORIENTED / MPC CURRENT CONTROL .......... 10–20 kHz   │
│     FOC cascade (or NMPC current loop): i_d, i_q regulation, SVPWM          │
│     In: torque/speed reference     Out: PWM to inverter                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ L0  PERCEPTION & ESTIMATION (runs in parallel, feeds L3–L5)                 │
│     LiDAR → scan matching/SLAM (10–20 Hz)   Camera → detection/depth (30 Hz)│
│     IMU 200–1000 Hz + encoders 1–10 kHz → EKF fusion → odometry 100–200 Hz  │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Key architectural rule:** L5 (learned) and L4 cannot command actuators directly. All commands pass through L2, which is not learning-enabled and holds veto power. This is what makes a neural policy admissible on physical hardware.

## 2. Compute allocation

| Unit | Runs | Rationale |
| --- | --- | --- |
| **MCU** (STM32-class, FPU) | L1 current control, encoder capture, IMU sampling, watchdog, safety stop path, serial/CAN bridge | Hard real-time, deterministic, no OS scheduling jitter; survives a crash of the high-level computer |
| **Edge GPU computer** (Jetson-class) | L0 perception/estimation, L3 NMPC, L4 planning, L5 policy, logging | Floating-point and neural inference; soft real-time with measured jitter |

The interface between them is a **bounded, versioned serial or CAN protocol**: fixed-length frames, sequence numbers, CRC, explicit limits, and a heartbeat. Loss of heartbeat ⇒ MCU ramps torque to zero. The high-level computer may fail at any moment without endangering the robot — this is a hard architectural requirement (SF-01, SF-02).

## 3. ROS 2 node graph

| Node | Layer | Subscribes | Publishes | Rate | QoS |
| --- | --- | --- | --- | --- | --- |
| `lidar_driver` | L0 | — | `scan` | 10–20 Hz | SensorDataQoS (best-effort) |
| `camera_driver` | L0 | — | `image_raw`, `depth`, `camera_info` | 30 Hz | SensorDataQoS |
| `imu_driver` | L0 → MCU | — | `imu/data_raw` | 200–1000 Hz | SensorDataQoS |
| `wheel_odom` | L0 → MCU | `joint_states` | `odom` | 50–100 Hz | Reliable |
| `ekf_fusion` | L0 | `imu/data`, `odom`, `scan` | `odom/fused`, TF `odom→base_link` | 100–200 Hz | Reliable |
| `slam` | L0 | `scan`, `odom/fused` | `map`, TF `map→odom` | 10–20 Hz | Reliable + transient local |
| `perception_detector` | L0 | `image_raw`, `depth` | `detections`, `obstacles` | 15–30 Hz | Reliable |
| `costmap_builder` | L0 | `scan`, `obstacles`, `map` | `local_costmap` | 10–20 Hz | Volatile |
| `global_planner` | L4 | `map`, goal | `global_path` | on request | Reliable |
| `nmpc_controller` | L3 | `global_path`, `local_costmap`, `odom/fused` | `cmd_vel_raw` | 20–50 Hz | Reliable |
| `safety_filter` | L2 | `cmd_vel_raw`, `safety_field`, health | `cmd_vel_safe` | 100–200 Hz | Reliable |
| `rl_policy` | L5 | `map`, summary, goal list | `behavior_cmd` | 1–5 Hz | Reliable |
| `safety_supervisor` | L2 | health topics, heartbeat | `safety_state`, `estop` | 50 Hz | Reliable |
| `bridge_mcu` | L2 (MCU path) | `cmd_vel_safe` | `wheel_cmd` (serial frame) | 100 Hz | Reliable |
| `logger` | all | selected topics | MCAP bag + JSONL events | event/periodic | Reliable, buffered |

**QoS discipline:** sensor streams are best-effort with a shallow queue (never block a driver on a consumer); commands are reliable with a depth of 1 (always the newest command — a stale command is worse than a dropped one). The map and any single-writer topic use transient-local durability so late-joining nodes get the current state.

**Executor discipline:** drivers and the control path run in separate callback groups with a multi-threaded executor; no control-path callback performs allocation, I/O or logging. Image transport uses zero-copy shared memory where available — a copy of a 1080p frame costs more than the NMPC solve.

## 4. Timing and synchronisation

| Item | Requirement |
| --- | --- |
| Clock | All nodes use the monotonic ROS clock; sensor timestamps are taken at acquisition (hardware), never at publish |
| Time sync | MCU ↔ computer over PTP or a compensated round-trip estimate; drift monitored and logged |
| Latency budget | Sensor acquisition → perception → costmap → NMPC → safety filter → serial → PWM, measured end-to-end and reported as p50/p99/max (PR-06) |
| Jitter | Control-path period jitter measured and reported; a controller whose timing is unmeasured is not considered validated |
| Bagging | MCAP with per-topic recording and an event log (state transitions, safety interventions) |

## 5. Control architecture: RL + NMPC + safety

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
                            ┌──────────────────────┐   u_mpc = (v, ω)
                            │ L3 NMPC              │──────────┐   [optimal, model-based]
                            └──────────────────────┘          │
                                      ▲                       ▼
                            state, costmap      ┌──────────────────────────┐
                                                │ L2 SAFETY FILTER (CBF-QP)│
                                                │ + feasibility monitor    │
                                                │ + speed limiter          │
                                                └──────────────────────────┘
                                                             │ u_safe
                                                             ▼
                                                   ┌──────────────────┐
                                                   │ L1 FOC (MCU)     │
                                                   └──────────────────┘
```

Three candidate integration topologies are analysed in [`theory/06-rl-mpc-integration-and-safety.md`](theory/06-rl-mpc-integration-and-safety.md). The baseline architecture is **hierarchical (RL above MPC) with an MPC/CBF safety filter below** — the only combination in which every layer's failure mode is bounded, and the one that can be defended without a proof of the policy's behaviour.

## 6. Degradation ladder

The system never fails silently. Each level is a state published on `safety_state`, logged with its trigger, and covered by a test.

| Level | Trigger | Response |
| --- | --- | --- |
| **Nominal** | All health signals good | Full autonomy |
| **Degraded sensing** | One perception modality lost/stale | Continue with reduced speed limit and enlarged safety margins; mark map cells from the lost sensor as unknown |
| **Reduced autonomy** | Localisation uncertainty above threshold, or map inconsistency | Stop exploring, retrace the known-good path, limit speed |
| **Protective stop** | Safety field violated, or NMPC infeasible beyond the recovery window | Controlled stop, hold, request operator |
| **Safe stop** | MCU heartbeat loss, watchdog timeout, E-stop, critical fault | Torque removed by MCU, brakes engaged; no software dependency |
| **Recovery** | Operator clearance / fault cleared | Re-initialise estimator, re-plan from current pose, log the event |

**Feasibility rule:** if the NMPC problem is infeasible or the solve exceeds its deadline, the controller falls back in this exact order — (1) re-solve with slack-relaxed constraints, (2) switch to a conservative reactive controller with a verified stopping envelope, (3) protective stop. The fallback chain is implemented and tested *before* the NMPC is tuned, because the fallback is what the robot actually relies on.

## 7. Data and logging architecture

| Artifact | Format | Purpose |
| --- | --- | --- |
| Sensor/state streams | MCAP bags, per-run directory | Reprocessing, debugging, training data |
| Event log | JSONL (state transitions, safety interventions, solver stats, errors) | Timeline reconstruction, safety audit |
| Per-run metadata | JSON (git commit, image digest, config hash, seed, hardware, scenario ID) | Reproducibility (SR-05) |
| Run report | JSON + Markdown summary generated by `ci/` tooling | Comparison across scenarios and versions |
| Datasets / weights | External object store, referenced by checksum in-repo | Keeps the repository small (SR-07) |

**Rule:** a result is only reportable if the run directory contains a complete metadata record. Tooling rejects runs without it.

## 8. Interface freeze

Before implementation begins, these interfaces are written down and frozen: the MCU serial/CAN frame format (fields, units, scaling, CRC, limits, heartbeat), all ROS 2 message types and topic names with QoS, the configuration file schema (for controller, policy, perception and scenario parameters), the run-metadata schema, and the container entrypoints. Interfaces change only through an explicit change record appended to this document — never silently during development.

## 9. Layer decisions for perception, planning and decision making (proposed, under review)

**Status:** proposal, not yet agreed. Recorded as a change record under the rule in §8. Nothing in §§1–8 is changed by this section. The sim-to-real layer is deliberately excluded and will be specified separately.

**Motivation.** The mission environment is unmapped and contains static and movable objects and people. The dominant nonlinearity has two sources: known dynamics and constraints (wheel slip, actuator limits, obstacle geometry), and unknown interaction (how a movable object moves when contacted, how a person reacts). These are handled by different tools.

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
