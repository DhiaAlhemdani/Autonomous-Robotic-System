# Theory 08 — Safety, Verification and Validation

**Purpose:** define the safety architecture, follow the relevant standards' methodology, and specify how every claim in this repository will be evidenced. This document carries the requirements SF-01…SF-08 and is the reason the rest of the system can be trusted even though a neural network is in the loop.

---

## 1. Hazard analysis (ISO 12100 methodology)

Method: identify hazards → estimate risk (severity × probability × avoidability) → reduce by design (inherently safe design → safeguarding → information for use) → re-evaluate. Performed **before** implementation so that safety requirements come out of the analysis rather than being bolted on.

| ID | Hazard | Cause | Severity | Mitigation (design order) |
| --- | --- | --- | --- | --- |
| H-01 | Impact with a person | Perception miss, controller error, policy error | High | Independent perception modality; CBF margin; speed limit vs. clearance; MCU heartbeat; torque-off; E-stop; physical guards in the test cell |
| H-02 | Crushing between robot and wall | Rear/side approach not covered by navigation | Medium | 360° coverage; reverse-motion speed limit; padded chassis; contact sensor stop |
| H-03 | Runaway (unbounded speed) | Control bug, communication loss commanding stale values | High | Serial watchdog with torque-off; firmware-enforced speed limit; independent MCU gate |
| H-04 | Loss of braking / uncontrolled stop on a ramp | Loss of power, thermal overload | Medium | Mechanical brake; gravity-safe firmware behaviour; graded speeds on slopes |
| H-05 | Fire / thermal event | Battery or motor thermal runaway | High | Fusing, BMS limits, thermal monitoring, charge supervision only while attended |
| H-06 | Electrical shock during maintenance | Contact with live bus | Medium | Keyed connectors, bus discharge display, service procedure, tool discipline |
| H-07 | Pinch points in mechanical assembly | Wheels, gears, servo linkages | Low–Medium | Guards, covers, de-energise before maintenance |
| H-08 | Data loss / corrupted autonomy state | Crashed node, inconsistent TF, bad loop closure | Medium | Watchdogs, health monitors, validated transform tree, conservative stop on inconsistency |

## 2. Applicable standards (methodology, not certification)

| Standard | Applies to | What the project takes from it |
| --- | --- | --- |
| **ISO 12100** | All machinery | Hazard identification and risk-reduction methodology (used above) |
| **ISO 3691-4** | Driverless industrial trucks and their systems | Safety requirements for AMRs: protective fields, speed/distance control, stop categories, operating modes, restart behaviour |
| **ISO 13849-1** | Safety-related parts of control systems | Performance-level (PL) reasoning; architecture, diagnostics and MTTFd; how the stop path's reliability is argued |
| **IEC 61508 / ISO 26262** (reference only) | Functional safety (general / automotive) | Vocabulary and V-model discipline; not applied in full |
| **IEC 62443** | Industrial cyber-security | Basic securing of containers, keys, update path, network exposure |
| **ISO/IEC 2382-x / safety-of-AI guidance** (emerging) | AI in safety functions | The pattern of *learning-enabled components behind non-learned safety layers* — which this architecture implements by design |

**Framing that must appear in the README and every report:** this project follows these standards' **methodology** to structure its safety argument. It does **not** claim certification, conformity or compliance, and a laboratory prototype with a custom chassis is not a certified industrial machine. Overstating this is the single fastest way to destroy the credibility of every other claim in the project.

## 3. Safety architecture and its layers

Restated from doc 06 §4 as a safety artifact with attributed claims:

| Layer | Function | Claim attributed to it | Evidence |
| --- | --- | --- | --- |
| L1 firmware (MCU) | Current control, limits, heartbeat, torque enable | Bounded torque and speed; stop on heartbeat loss | Firmware tests, instrumented stop timing |
| L2 supervisor | Health, consistency, degradation ladder | Detect faults and escalate within defined deadlines | Fault-injection suite |
| L2 safety filter (CBF-QP) | Enforce margins on any commanded input | Commanded input satisfies the barrier condition given the model and estimate | Logged minimum distances, instrumented filter |
| L3 NMPC | Constraint handling, tracking | Bounded error and constraint satisfaction in the operating envelope | Scenario suite, solver logs |
| L4 planner / L5 policy | Mission performance | **No safety claim** — performance only | Scenario suite (performance metrics) |
| Physical | E-stop, guards, brakes | Torque removal independent of software | Bench test |

**Rule:** every safety claim in documentation names its layer and its evidence. No sentence in this repository says "the system is safe."

## 4. Stop categories and reaction times

| Category | Trigger | Behaviour | Target reaction (measured) |
| --- | --- | --- | --- |
| **Category 0** (uncontrolled) | E-stop pressed, power loss | Immediate torque removal, mechanical brake | ≤ 50 ms (hardware path) |
| **Category 1** (controlled) | Safety field violation, supervisor escalation | Controlled deceleration within the traction limit, then torque removal | ≤ 100 ms detection-to-torque-off; stopping distance measured at each speed |
| **Category 2** (operational, not safety) | Planner requests a hold | Stop, retain power, ready to resume | — |
| **Communication loss** | Missed heartbeat (N frames) | Controlled stop, then torque-off | Bounded by frame period × N; **N and the resulting time are recorded constants** |

**Instrumentation requirement (SF-04):** every stop is timestamped at the trigger and at the moment torque is removed, and both are logged. The reaction time is a measured distribution, not a datasheet value, and it appears in the run report.

## 5. Verification and validation plan

### 5.1 Layers of testing

| Level | What it tests | How | Gate |
| --- | --- | --- | --- |
| **Unit** | Math, protocol parsing, safety filter logic, kinematics | pytest / gtest, no simulation | 100% of safety-critical modules covered |
| **Component** | Each node in isolation | Recorded bags replayed into the node; deterministic outputs | Behaviour reproducible from the bag |
| **Integration (simulation)** | Full stack in Gazebo | Scenario suite, seeded | Requirement-by-requirement pass/fail |
| **Fault injection** | Failure handling | Kill nodes, drop topics, delay packets, corrupt frames, occlude sensors | Correct state transitions; no silent failures (SF-02) |
| **Performance** | Rates, latency, jitter, load | Instrumented over ≥ 30 runs | PR-05, PR-06 met with reported percentiles |
| **Hardware-in-the-loop** | Real MCU + real firmware, simulated plant | HIL bench | Stop timing measured; protocol verified |
| **Field (enclosure)** | Full system, real sensors, real dynamics | Supervised runs, increasing speed, human stand-in | FR/PR met; safety envelope respected |
| **Adversarial** | Attempts to break the safety layer | Scripted adversaries: rapid approach, occlusion, reflective obstacle, boxed-in, deadlock | Minimum distances maintained; failures documented |

### 5.2 Metrics to report for every run

```
Mission:      success/failure, time to goal, path length vs. optimal, energy used
Tracking:     cross-track RMSE, heading RMSE, maximum deviation
Safety:       minimum distance to obstacle and to human, number of safety-field entries,
              number of shield interventions, stop reaction times (p50/p99/max)
Real-time:    NMPC solve time (p50/p99/max), filter solve time, perception latency,
              control-loop period jitter
Resources:    CPU/GPU/memory utilisation, thermal state, battery consumption
Estimator:    position/heading error vs. ground truth (simulation), NIS consistency
Learning:     (where applicable) return, success rate, seeds, shield intervention rate
```

**Percentiles and maxima, always.** Averages hide exactly the tail events that determine safety.

### 5.3 Scenario suite (seeded, scripted, automated)

| Suite | Content | Requirement coverage |
| --- | --- | --- |
| `nominal` | Empty and lightly cluttered spaces, static goals | FR-02, FR-03, PR-02 |
| `cluttered` | Narrow gaps, dead ends, tight corridors | FR-04, FR-06, PR-05 |
| `dynamic` | Moving obstacle scripts, human trajectories, head-on and crossing | FR-05, PR-08, PR-09 |
| `degraded` | Darkness, dust, occlusion, sensor dropout, slip | FR-09, PR-07 |
| `faults` | Node kills, topic dropouts, latency injection, corrupted frames, solver stalls | SF-02, SF-05, FR-10 |
| `adversarial` | Exploitation attempts against the safety filter | SF-06, PR-09 |
| `endurance` | 90-minute continuous mission | PR-10, thermal stability |

## 6. The safety case (document structure to be produced at M6)

1. **System description** and operational design domain (what the robot is and is not allowed to do).
2. **Hazard analysis** with risk reduction traceability (H-01 … H-08 → requirements → tests).
3. **Safety requirements** with verification methods.
4. **Architecture argument**: why the layering bounds the consequences of any single failure, including a learning-enabled component.
5. **Verification evidence**: results per requirement, per scenario, with conditions.
6. **Residual risk**: what remains, why it is accepted, and what would be required to reduce it.
7. **Explicit limitations**: what the evidence does *not* cover (e.g. untested surfaces, untested public environments, unreviewed firmware revisions).

## 7. Assumptions and limits

1. Operation is confined to a controlled, supervised environment (a lab or an enclosure). Public-space operation is out of scope (AS-05).
2. A trained operator is present during hardware tests and holds the E-stop.
3. The robot's mass, speed and geometry keep it in the "small AMR" risk class; scaling any of these invalidates the hazard analysis and requires it to be redone.
4. Software safety functions are assumed to be *unreliable* by design intent: the architecture assumes they can fail, and the MCU and physical layers cover that.
5. Cybersecurity is addressed at a basic level (no exposed services, no secrets in images, signed images where practical); a full threat model is out of scope.

## 8. Open questions

1. What performance-level target is realistic for the stop path, and is a redundant channel required for it? — decide after measuring the single-channel reaction time.
2. Is a certified safety relay/controller needed for credibility, or does documented dual-channel reasoning with a physical E-stop suffice for a prototype? — depends on the intended demonstration audience.
3. How should the safety case handle the **continuously retrained** nature of learned components (a policy update invalidates previous evidence)? — proposal: treat any policy change as a configuration change requiring the adversarial suite to be re-run, and record the result with the new model checksum.
