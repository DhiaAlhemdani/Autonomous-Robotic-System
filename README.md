# Autonomous Robotic System

Design and engineering record for an autonomous mobile robot with **learning-based perception and decision-making**, **model-predictive control**, and **PMSM drive-level control**, designed to be developed and validated inside a **fully containerized (Docker) toolchain**.

> **Status: theoretical phase.** The system is being specified, derived and documented before implementation. Every document in [`docs/`](docs/) is part of the engineering record. Code will be added only against a frozen interface specification.

## What this system is

An autonomous mobile robot that navigates and performs inspection tasks in **GPS-denied, previously unmapped, human-shared environments** — with no reliance on external infrastructure — using a control stack that spans four orders of magnitude of bandwidth:

| Layer | Rate | Method |
| --- | --- | --- |
| Mission policy | 1–5 Hz | Deep reinforcement learning |
| Local motion planning | 20–50 Hz | Nonlinear MPC with obstacle constraints |
| Body velocity control | 50–200 Hz | NMPC / baseline tracking controller |
| Motor current control | 10–20 kHz | Field-oriented control on PMSM |

The engineering thesis of this project is **vertical integration**: from power electronics to mission policy in one coherent, verified, reproducible system — with an explicit safety architecture that makes a learning-enabled controller admissible on real hardware.

## The dilemma this system attacks

> **An organisation building an autonomous machine with a learning-enabled component must produce evidence that a third-party reviewer accepts — and no accepted method for producing that evidence exists.**

The EU Machinery Regulation (EU) 2023/1230 applies from **20 January 2027** and names machine-learning safety components as a high-risk category requiring third-party conformity assessment, with no route to self-certification. Formal verification of neural networks does not scale, runtime-assurance guarantees rest on preconditions that designs never measure, and the OOD-detection monitoring most systems build checks the wrong thing. See [`docs/07-open-dilemmas.md`](docs/07-open-dilemmas.md) for the full analysis of eighteen open dilemmas, why this one is the target, and what is explicitly out of scope.

## Documentation map

| Document | Contents |
| --- | --- |
| [`docs/07-open-dilemmas.md`](docs/07-open-dilemmas.md) | **Start here.** The 18 open dilemmas, the selection filter, the scoring matrix, the target dilemma, and the honest contribution claim |
| [`docs/08-state-of-the-art-analysis.md`](docs/08-state-of-the-art-analysis.md) | The frontier analysed: seven axes of "advanced", six leading systems (Waymo, Atlas, the VLA frontier, ANYmal X, Perseverance, Chinese volume), their architectures and measured limits, the seven convergent industry patterns, and what it changes in this project |
| [`docs/09-architecture-comparison.md`](docs/09-architecture-comparison.md) | This architecture against the frontier: a verdict matrix (Converged / Different / Behind / Gap / Unproven), the specification gaps it exposes, a proposed-changes table, and a self-audit |
| [`docs/10-industry-case-study.md`](docs/10-industry-case-study.md) | Industry case study: eleven well-known robotics companies (Clearpath, MiR, Boston Dynamics, ANYbotics, Unitree, Agility, Franka, Universal Robots, Tesla Optimus, Figure AI, Amazon Robotics) on capabilities, limitations, controllers, motors, drivers, simulation, and comparison with our theory; 97 sources, with each value tagged primary, secondary, company claim, not published or conflicting |
| [`docs/00-mission-and-scope.md`](docs/00-mission-and-scope.md) | Problem statement, mission scenarios, frontier problems, scope discipline |
| [`docs/01-requirements.md`](docs/01-requirements.md) | Numbered functional and non-functional requirements with verification methods |
| [`docs/02-system-architecture.md`](docs/02-system-architecture.md) | Layered architecture, rates, interfaces, ROS 2 graph, compute allocation, degradation ladder |
| [`docs/03-simulation-and-docker.md`](docs/03-simulation-and-docker.md) | Container architecture, GPU/real-time caveats, reproducible simulation and training |
| [`docs/04-build-roadmap.md`](docs/04-build-roadmap.md) | Milestone ladder M0–M7 with gate criteria and hardware cost tiers |
| [`docs/05-risk-register.md`](docs/05-risk-register.md) | Technical, resource and schedule risks with mitigations |
| [`docs/06-reading-list.md`](docs/06-reading-list.md) | Books and papers mapped to each theory document |
| [`docs/theory/`](docs/theory/) | Kinematics & dynamics · PMSM/FOC · estimation · NMPC · RL · RL+MPC safety integration · perception · safety & verification |

## Design principles

1. **Theory before code.** Every controller is derived and its assumptions written down before it is implemented.
2. **Baseline first, then replace.** Each layer ships as a well-understood classical baseline; the learning-based or advanced component must beat it against a measured metric on the same test set.
3. **Measure, never assume.** Latency, jitter, bandwidth, accuracy and compute load are measured and reported with percentiles — not quoted from datasheets.
4. **Safety is an architecture, not a feature.** The learning-enabled components are never the last line of defence.
5. **Reproducibility is a deliverable.** A result that cannot be rebuilt from a container image and a pinned configuration is not a result.

## License

MIT — see [`LICENSE`](LICENSE).
