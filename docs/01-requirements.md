# 01 — Requirements Specification

Numbered requirements with verification methods. Format: `ID · Requirement · Verification · Status`. Verification methods: **T** = test, **A** = analysis/derivation, **D** = demonstration, **I** = inspection of design record. This document is the contract; the architecture in doc 02 must satisfy it, and the roadmap in doc 04 must deliver it.

## 1. Platform and mission requirements

| ID | Requirement | Verification | Status |
| --- | --- | --- | --- |
| FR-01 | Operate indoors with **no GPS, no external infrastructure** (no markers, no beacons) | D (S1 run) | Specified |
| FR-02 | Build a map of an unknown environment and localise within it, with no prior map | T | Specified |
| FR-03 | Navigate to a sequence of commanded interest points and return to start | T | Specified |
| FR-04 | Detect and avoid static obstacles with a minimum clearance of **0.15 m** at commanded speeds ≤ 0.5 m/s | T | Specified |
| FR-05 | Detect and avoid **people** and moving obstacles; never violate the human safety field | T (S2) | Specified |
| FR-06 | Recover from deadlock/permanent blockage within a bounded time or declare failure explicitly | T | Specified |
| FR-07 | Detect loss of operator communication and continue the mission autonomously (S4) | T | Specified |
| FR-08 | Capture and log inspection data with pose and timestamp for every interest point | T | Specified |
| FR-09 | Continue or degrade safely on loss of any single sensor (S3) | T (fault injection) | Specified |
| FR-10 | Return to a safe state (controlled stop) on any unrecoverable condition | T | Specified |

## 2. Performance requirements

| ID | Requirement | Target | Verification |
| --- | --- | --- | --- |
| PR-01 | Maximum ground speed | 1.0 m/s (autonomous), 1.5 m/s (bench test) | T |
| PR-02 | Trajectory tracking error on a reference path at 0.5 m/s | **RMSE ≤ 0.05 m** cross-track, ≤ 5° heading | T |
| PR-03 | Localisation accuracy in a mapped space | **≤ 0.10 m** position, ≤ 3° heading (1σ) | T |
| PR-04 | Map coverage of a reachable unknown space in a 10-minute mission | ≥ 90% | T |
| PR-05 | NMPC solve time | **p50 ≤ 20 ms, p99 ≤ 45 ms**, hard limit 50 ms (20 Hz minimum) | T (instrumented) |
| PR-06 | Perception end-to-end latency (sensor to obstacle in costmap) | p99 ≤ 150 ms | T (instrumented) |
| PR-07 | Mission success rate in nominal scenario suite | ≥ 90% over ≥ 30 runs | T |
| PR-08 | Collision rate in nominal + dynamic scenario suite | **0 collisions** in ≥ 50 runs | T |
| PR-09 | Minimum distance to a human during S2 | ≥ 0.35 m, never < 0.20 m | T |
| PR-10 | Runtime on battery | ≥ 90 minutes continuous autonomy | T |
| PR-11 | Drive current-loop control rate | ≥ 10 kHz | T |
| PR-12 | Speed regulation bandwidth (wheel) | ≥ 100 Hz, measured by step response | T |
| PR-13 | Regulatory/safety stop reaction time | Measured; target ≤ 100 ms from trigger to torque removal | T (instrumented) |

## 3. Software and reproducibility requirements

| ID | Requirement | Verification |
| --- | --- | --- |
| SR-01 | The complete simulation stack starts with one command on a clean machine | D |
| SR-02 | The complete stack is reproducible from **pinned container images** (by digest) and a versioned configuration | I + D |
| SR-03 | GPU training and CPU-only development share one reproducible environment definition | I |
| SR-04 | Headless scenario suite runs in CI on every change, and its report is a build artifact | T |
| SR-05 | Every experiment records: container digest, git commit, configuration file, random seed, hardware, and result | I |
| SR-06 | All random processes are seeded and seed-averaged results are reported (≥ 5 seeds for learning results) | T |
| SR-07 | No dataset, model weight or log is committed to git; all are versioned externally with checksums | I |
| SR-08 | All topics, message types, rates and QoS policies are documented before implementation | I |
| SR-09 | Public interfaces are frozen before implementation begins; changed only by an explicit interface change record | I |

## 4. Safety requirements

| ID | Requirement | Verification |
| --- | --- | --- |
| SF-01 | The safety function is **independent of the learning-enabled stack** (separate compute, separate code path) | I + fault injection |
| SF-02 | Loss of any single component (perception, planner, MPC, policy, comms) results in a safe state within its deadline | T (fault injection) |
| SF-03 | A physical emergency stop removes actuator torque independently of software | T |
| SF-04 | The safety function has a **measured** worst-case reaction time with evidence | T (instrumented) |
| SF-05 | Watchdog coverage: no actuator command is issued unless a fresh, validated setpoint is available | T |
| SF-06 | Speed is limited as a function of measured distance to the nearest obstacle or human | T |
| SF-07 | Hazard analysis follows ISO 12100 methodology; AMR-specific requirements follow ISO 3691-4; safety-related control functions follow ISO 13849 methodology | I (design record) |
| SF-08 | Every safety claim is traceable to a verification artifact; unverified claims are marked as such | I |

## 5. Boundary conditions and constraints

| ID | Constraint | Value / statement |
| --- | --- | --- |
| BC-01 | Mass | ≤ 25 kg complete |
| BC-02 | Footprint | ≤ 0.60 × 0.50 m |
| BC-03 | Operating environment | Indoor, dry, 5–40 °C, non-condensing |
| BC-04 | Floor | Smooth concrete / vinyl / tile; thresholds ≤ 15 mm; ramps ≤ 5° |
| BC-05 | Onboard compute | One edge GPU module (Jetson-class, ≥ 8 GB) + one MCU (STM32-class, FPU, ≥ 100 MHz) |
| BC-06 | Power | 24 V nominal battery, ≥ 300 Wh |
| BC-07 | Cost ceiling | Tiered — see doc 04. Core tier target ≤ $1,500; advanced tier ≤ $3,000 |
| BC-08 | Development constraint | This phase is documentation-only; implementation begins when the development laptop is available |

## 6. Assumption register (violations invalidate requirements)

| ID | Assumption | If violated |
| --- | --- | --- |
| AS-01 | The floor is level and non-slip; slip is limited and detectable | Slip-robust estimation and traction control move from stretch to required |
| AS-02 | The environment has sufficient geometric structure for LiDAR SLAM (walls, corners) | Camera/depth-based localisation becomes mandatory, and PR-03 must be re-verified |
| AS-03 | Lighting is adequate for RGB-D in nominal operation | S3 requires active illumination and low-light retraining |
| AS-04 | Humans are cooperative but not instrumented (no tags, no beacons) | Detection must be purely onboard — this is a hard constraint, deliberately |
| AS-05 | The robot never operates on public roads or in contact with the public | If violated, the entire safety and regulatory envelope changes |

## 7. Requirement-to-test traceability (extract)

Every requirement must appear in at least one scenario, test or analysis. The initial matrix:

| Requirement | Test artifact (to be created in M1–M3) | Scenario |
| --- | --- | --- |
| FR-02, PR-03, PR-04 | `test_slam_accuracy.py`, `mission_s1.yaml` | S1 |
| FR-04, FR-06, PR-02 | `test_tracking_rmse.py`, `test_recovery.py` | S1 |
| FR-05, PR-08, PR-09 | `test_dynamic_avoidance.py`, `scenario_s2.yaml` | S2 |
| FR-09, PR-07 | `fault_inject.py`, `test_degradation.py` | S3 |
| FR-07 | `test_comms_loss.py` | S4 |
| SR-04, PR-05, PR-06 | `ci/run_scenarios.sh`, `bench_report.json` | CI |
| SF-01 … SF-06 | `safety/test_stop_timing.py`, hazard analysis record | Bench |

**Change control:** any change to a requirement requires a note in the repository's change record stating what invalidated the previous version. Requirements are versioned; this document is not edited silently.
