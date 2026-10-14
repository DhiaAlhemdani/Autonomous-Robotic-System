# 04 — Build Roadmap: Milestones, Gate Criteria and Cost Tiers

The theoretical phase produces the specification. This roadmap is what the specification will be built against, with **gate criteria** — a milestone is not complete because the code runs, but because its named evidence exists.

Two rules govern the whole ladder:

- **Baseline before advanced.** Every advanced component (NMPC, RL, learned model) is built only after its classical baseline exists and is measured, so the comparison is possible.
- **Safety before autonomy.** The stop path, watchdog and supervisor are built at M0–M1, before anything that moves faster than walking pace.

## Milestone ladder

| # | Milestone | Deliverable | Gate criteria (evidence required to pass) | Depends on |
| --- | --- | --- | --- | --- |
| **M0** | Bench & stop path | MCU + motor + encoder on a bench; firmware with heartbeat, limits, torque-off; E-stop wired | Stop reaction time **measured** and logged; heartbeat loss → torque-off verified; firmware unit tests pass | — |
| **M1** | Containerized simulation skeleton | `docker compose up` brings up Gazebo + robot model + bridge; headless CI variant runs; repo skeleton per doc 02 §8 | Reproducible on a clean machine; image digests recorded; CI green; interfaces frozen and documented | — |
| **M2** | Kinematics, drive and odometry | Robot drives in simulation and on the bench; wheel odometry; identified parameters | Straight-line and turn accuracy vs. external measurement (≤ 2% of distance); parameter identification report with residuals; latency measured | M0, M1 |
| **M3** | Localisation and control baseline | LiDAR SLAM + EKF fusion; pure-pursuit/LQR tracker; safety supervisor implementing the degradation ladder | PR-03 met in simulation; tracking RMSE baseline recorded; supervisor fault-injection suite passes (SF-02) | M2 |
| **M4** | NMPC trajectory tracking | NMPC (CasADi/acados) with obstacle constraints, SDF, velocity-dependent margin, fallback chain | PR-02 and PR-05 met (**p99 solve time reported**); constraint-satisfaction log clean; fallback chain verified by forced infeasibility/timeout; comparison table vs. M3 baselines | M3 |
| **M5** | Perception and semantics | Object/person detection, traversability, costmap fusion; latency instrumentation; OOD trigger | PR-06 met (p99 reported per stage); detection accuracy on a held-out diverse set; degraded-sensing scenarios pass (FR-09); unknown-treated-as-occupied verified | M3 |
| **M6** | Safety architecture complete | CBF-QP safety filter; safety case document; adversarial suite; hazard traceability | Zero safety-field violations in nominal+dynamic suites; adversarial scenarios maintain minimum distances; stop times instrumented; safety case reviewed against doc 08 §6 | M4, M5 |
| **M7** | Learning layer and hardware validation | RL mission layer (Topology 2) and residual layer (Topology 3) behind the shield; hardware trials in an enclosure | ≥ 5 seeds; RL+shield compared against NMPC-only and scripted baselines on identical scenarios; shield intervention rate reported; transfer degradation quantified on hardware | M6 |

**Honest sequencing note:** M0–M4 is a strong, publishable project on its own. M5–M7 is where the "futuristic" claims live, and each one is a genuine research effort. Completing M0–M4 with measured results and a documented safety case is a better professional outcome than a shallow pass at all eight.

## Cost tiers

| Tier | Contents | Approximate cost | Covers |
| --- | --- | --- | --- |
| **T0 — Simulation only** | Development machine already owned; everything containerized; no hardware | $0 | M1, M2 (sim), M3, M4, M5, M6 (sim), M7 (sim) |
| **T1 — Core bench** | MCU dev board + driver + motor + encoder + IMU + power supply | $150–300 | M0 (single axis), M2 (bench verification) |
| **T2 — Core robot** | Two hub/BLDC motors + drivers + MCU + encoders + IMU + 2D LiDAR + battery + chassis + fasteners | $700–1,500 | M0–M4 on hardware |
| **T3 — Advanced** | Edge GPU computer + RGB-D camera + better LiDAR + ultrasonic ring + brake/E-stop hardware + safety relay | $1,500–3,000 | M5–M7 on hardware |
| **T4 — Research** | Second GPU for training, dynamometer, calibrated measurement equipment | $3,000+ | Model-based RL at scale, drive characterisation |

**Acquisition order (cheapest path to real evidence):** MCU + one motor + encoder (T1) → proves the stop path and the drive loop → LiDAR + chassis + second motor (T2) → proves navigation and NMPC on hardware → edge computer + camera (T3) → proves perception and learning in the loop.

**Important:** do not buy T3 hardware before M4 passes in simulation. Hardware bought before the controller works becomes a source of confusion (is the failure in the model or the plant?), and the simulation path costs nothing while the design is still moving.

## Interfaces to freeze before implementation (doc 02 §8)

| Interface | Artefact |
| --- | --- |
| MCU ↔ computer protocol | Frame format table: fields, units, scaling, CRC, sequence numbers, heartbeat, limits, error codes |
| ROS 2 graph | Topic/service/action list with types, rates, QoS (doc 02 §3) |
| Configuration schema | YAML/JSON schema for controller, policy, perception, safety and scenario parameters |
| Run metadata schema | JSON schema per doc 03 §7 |
| Container entrypoints | Named commands per image and compose profile (doc 03 §2) |
| TF tree | Frame list with a single authority per transform |

## Repository structure (target)

```
.
├── docs/                  specifications, theory series, ADRs, safety case
├── docker/                Dockerfiles per image + compose profiles
├── config/                controller, policy, perception, safety, scenario configs
├── scenarios/             seeded scenario definitions and expected outcomes
├── src/
│   ├── perception/        detection, fusion, costmap
│   ├── estimation/        EKF, odometry, SLAM integration
│   ├── control/           NMPC, trackers, safety filter
│   ├── learning/          environments, training, evaluation, policies
│   ├── safety/            supervisor, watchdogs, degradation ladder
│   └── bridge/            MCU protocol, drivers
├── firmware/              MCU project (FOC, limits, watchdog, stop path)
├── ros/                   packages, launch files, URDF/xacro, worlds
├── tests/                 unit, component, fault-injection, scenario runners
├── ci/                    pipeline definitions, report generator
├── scripts/               tooling: dataset inventory, benchmark summaries, calibration
└── artifacts/             generated reports and plots (selected, small)
```

`data/`, `models/` and `runs/` are git-ignored by design (SR-07): external storage with checksums, exactly as in the author's graduation project.

## Working agreement for implementation

1. **One interface change at a time**, recorded in the change log, with the affected tests updated in the same commit.
2. **Every experiment is a script**, seeded and config-driven — never a manual session whose result cannot be reproduced.
3. **No number enters documentation without its run metadata.** The tooling enforces this.
4. **The baseline is always measured first.** An advanced component that has not been compared against its baseline is not finished.
5. **Dirty working tree results are not reportable** (doc 03 §7).
6. **Limitations sections are mandatory** in every report — the practice carried over from the graduation project, and the project's principal credibility mechanism.
