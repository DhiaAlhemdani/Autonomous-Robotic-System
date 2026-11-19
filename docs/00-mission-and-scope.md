# 00 — Mission and Scope

## 1. Problem statement

Autonomous mobile robots work reliably today in structured, mapped, infrastructure-supported environments: warehouses with fiducial markers, factories with fixed layouts, outdoor vehicles with GPS. They fail in the environments where autonomy is most valuable:

- **No GPS.** Indoor, underground, inside vessels, under canopy, or in jammed/denied signal conditions.
- **No prior map.** The environment is unknown at deployment, or changes between visits.
- **Unstructured space.** No lane markings, dynamic clutter, doors, ramps, uneven thresholds.
- **Humans present.** People move unpredictably and must be treated as a hard safety constraint, not an obstacle to be optimised around.
- **Degraded sensing.** Dust, smoke, low light, reflective floors, sensor failure — conditions where a single-perception-modality stack collapses.

The system specified in this repository is an autonomous inspection platform for exactly that regime.

## 2. Mission

> **A robot that enters an unknown, GPS-denied, human-shared facility with no prior map, builds its own representation of the space, moves through it safely and efficiently, and returns an inspection record — using only onboard sensing and onboard compute, with no external infrastructure.**

### Mission scenarios

| ID | Scenario | What it tests |
| --- | --- | --- |
| **S1** | Unknown indoor facility: enter, map, traverse to a list of interest points, record inspection data, return to start | Autonomy without prior map; SLAM; coverage; energy-aware planning |
| **S2** | Same space with humans and moving obstacles present | Socially-aware navigation, hard safety constraints, comfort, deadlock escape |
| **S3** | Degraded conditions: low light, dust/smoke, one sensor failed or occluded | Graceful degradation, sensor fusion robustness, safe reactive fallback |
| **S4** | Communication loss: operator link drops mid-mission | Full onboard autonomy, safe-state policy, recovery and re-sync |
| **S5** | Anomaly mission: inspect a specific asset, hold a stable pose/standoff, capture imagery, log findings | Precision positioning, station-keeping, data quality |

## 3. Why this is a frontier problem, not a tutorial problem

A robot that follows waypoints in a mapped warehouse is solved engineering. What is **not** solved — and what this project deliberately addresses — is *certifiable learning-enabled autonomy*: how to let neural networks and reinforcement-learned policies into the control loop of a physical system that must not injure a human.

Three specific open problems are in scope:

1. **Safety with learning in the loop.** A neural policy has no proof of correctness. Architectures that combine learned policies with formal safety certificates (control barrier functions, reachability analysis, model-predictive safety filters) are an active research area, and the practical question — *does the safety filter remain feasible and non-conservative when the policy is adversarial?* — is unresolved in general.
2. **The reality gap.** Policies trained in simulation fail on hardware because of unmodelled friction, latency, backlash, sensor noise, and actuator saturation. Sim-to-real transfer by domain randomization, system identification and residual learning is effective but not reliable in an a-priori predictable way.
3. **Nonlinear control at real-time rates on embedded compute.** NMPC with obstacle and actuator constraints at 50 Hz, and NMPC on the motor current loop at multi-kHz rates, push against the limits of what an embedded solver can deliver — the tension between horizon length, model fidelity, solver time and the hard real-time deadline is a genuine engineering frontier.

## 4. System concept

**Platform (revised; `docs/11` §1 and §7).** The platform is the **T800 humanoid** (EngineAI). It is simulated from the model in `third_party/T800-PROJECT` (pinned submodule). Its mass, size, joint and actuator description are taken from that model and are **not yet re-derived for a real robot**. The wheeled description that this section previously contained (differential drive, about 0.5 × 0.4 m, 20 kg, BLDC wheel drives, wheel odometry) **no longer applies** (`docs/11` §2.1).

**Retained as design intent, pending re-derivation (`docs/11` §2.3).** The sensing set (LiDAR, RGB-D camera, IMU, proprioception), the edge computer for perception and planning, a microcontroller for hard real-time and safety functions, ROS 2 for the software stack, NMPC (CasADi/acados) for trajectory optimisation, PyTorch-based learning, and containers for the whole toolchain. None of these is confirmed as the T800 hardware. Simulation was specified as Gazebo; for the T800 model it is MuJoCo (`docs/11` §2.5). The hardware parameters of each item are to be re-derived for the humanoid (`docs/11` §2.1, §2.3).

**Locomotion.** The T800 project's RL walking policy is adopted as a **baseline, not yet evaluated**, and is not built in this repository (`docs/11` §2.2, §7). Navigation, perception and planning are built above it.

## 5. Scope discipline (read this before adding anything)

The feature set above — learning-based decision-making **and** advanced nonlinear control **and** custom PMSM drives **and** safety certification **and** full containerization — is a **research-grade programme**, not a single project. It is genuinely more than one person can complete to a high standard in a first pass. The honest structure is:

**Core (must work; to be re-evaluated for the humanoid, `docs/11` §2.4):** S1 + S2 with a LiDAR/camera SLAM stack, an NMPC trajectory tracker, a measured safety filter, and a container that rebuilds the whole system from scratch.

**Advanced (the differentiator):** RL mission policy with a verifiable safety shield; drive-level NMPC replacing cascade PI on the current loop; sim-to-real transfer to hardware (a target to be measured, not yet attempted).

**Stretch (research):** reachability-based guarantees for the composed stack; formal verification of the discrete supervisor; multi-robot coordination.

**Explicitly out of scope:** manipulation, outdoor/GPS operation, long-range perception beyond the sensor envelope, and any claim of regulatory certification (the project produces a *safety case* and follows the relevant standards' methodology — it does not certify a product).

> **Portfolio reality:** the professional value of this project comes from **depth in two or three milestones with measured, honest results**, not from breadth across all seven. A reader believes "NMPC at 50 Hz with a 120 ms worst-case solve time on a Jetson, with the tracking RMSE against a classical baseline" — not a stack diagram listing twelve techniques.

## 6. What "done" looks like

The project is complete when all of the following are true and reproducible from the repository:

1. A single `docker compose up` starts the full simulation stack; a headless variant runs the scenario suite in CI.
2. Every mission scenario is scripted, runs unattended, and produces a machine-readable report.
3. Every controller is evaluated against a baseline on identical scenarios, with percentiles reported.
4. The safety architecture is documented, implements an independent safety function with its own watchdog and stop path, and its reaction time is **measured**, not estimated.
5. Every claimed performance number is accompanied by the conditions under which it was obtained and an explicit statement of what it does not prove.

> **Change record (platform revision, `docs/11` §2).** §4 is rewritten for the T800 humanoid. Legged locomotion is removed from the out-of-scope list (`docs/11` §1). The mission (§2), the scenarios S1–S5, and the success criteria (§6) are unchanged. The platform change and the open items in `docs/11` §5 are recorded there. Docs 00–02 are changed only where `docs/11` requires; the rest is still the original text.
