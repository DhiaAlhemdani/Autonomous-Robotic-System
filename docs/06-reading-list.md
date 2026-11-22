# 06 — Reading List, Mapped to the Design

Reading is selected by the design decision it unblocks, not by topic interest. Work down this list **in order**; the first three entries are prerequisites for everything else.

## Tier 0 — Prerequisites (do these first, in this order)

| # | Source | Why | Feeds |
| --- | --- | --- | --- |
| 1 | **Modern Robotics: Mechanics, Planning, and Control** — Lynch & Park (book + Coursera, free to audit; the book PDF and the companion libraries are freely available) | Configuration spaces, rigid-body transforms, kinematics, dynamics, trajectory generation, control. The single best foundation for this project | Theory 01, 04 |
| 2 | **ROS 2 official tutorials** (docs.ros.org) + **Gazebo** documentation | The middleware the whole system is built on; URDF/xacro, tf2, launch, QoS | Docs 02, 03 |
| 3 | **Python Robotics / Scientific Computing refresher** — NumPy, SciPy, matplotlib, control systems basics | Everything downstream is implemented in this stack | All |

## Tier 1 — Control (the project's centre of gravity)

| # | Source | Why | Feeds |
| --- | --- | --- | --- |
| 4 | **Predictive Control for Linear and Hybrid Systems** — Borrelli, Bemporad & Morari (free PDF) | The clearest path into MPC: formulation, feasibility, stability, the QP structure. Read chapters 1–4 and 11 carefully | Theory 04 |
| 5 | **CasADi documentation and examples** | The toolchain: symbolic modelling, automatic differentiation, OCP formulation, IPOPT and QP interfaces | Theory 04 |
| 6 | **acados documentation + examples** | Real-time NMPC: multiple shooting, RTI, generated C code, embedded deployment | Theory 04, M4 |
| 7 | **Nonlinear Model Predictive Control: Theory and Algorithms** — Grüne & Pannek (free PDF) | Stability and recursive feasibility properly — including *why they fail* in practical settings | Theory 04 §4 |
| 8 | **Feedback Systems** — Åström & Murray (free PDF) | The control fundamentals that everything else assumes: Bode, margins, cascade design, anti-windup | Theory 02 §3, 04 §6 |
| 9 | **Robust Adaptive Control / Tube MPC survey papers** | What "margin" means when the model is wrong — the basis for constraint tightening | Theory 04 §5 |

## Tier 2 — Electric drives

| # | Source | Why | Feeds |
| --- | --- | --- | --- |
| 10 | **Analysis of Electric Machinery and Drive Systems** — Krause et al. | The reference for the PMSM model in the dq frame and the assumptions behind it | Theory 02 §2 |
| 11 | **Modern Power Electronics and AC Drives** — Bose | FOC implementation, modulation, field weakening, practical inverter effects | Theory 02 §3–2.5 |
| 12 | **Texas Instruments / ST application notes** on PMSM FOC (e.g. TI InstaSPIN, ST MC SDK) | The engineering detail that textbooks omit: sampling synchronisation, dead-time compensation, current sensing, anti-windup, tuning procedures | Theory 02 §3.3, M0 |
| 13 | **Motor Control papers on MPC/Predictive Current Control** (e.g. Rodriguez et al., FCS-MPC literature) | The specific advanced tier: MPC on the current loop, its rate requirements and its honest limitations | Theory 02 §4 |

## Tier 3 — Estimation and SLAM

| # | Source | Why | Feeds |
| --- | --- | --- | --- |
| 14 | **Probabilistic Robotics** — Thrun, Burgard & Fox | The canonical treatment: Bayes filters, EKF/UKF, particle filters, SLAM | Theory 03 |
| 15 | **State Estimation for Robotics** — Barfoot (free PDF) | Better treatment of the Lie-group geometry; essential once frames get confusing | Theory 03 §2–4 |
| 16 | **Probabilistic graphical SLAM literature** (g2o, GTSAM, Cartographer, LIO-SAM) | The practical graph-SLAM back end and robust loop closure | Theory 03 §5 |
| 17 | **The EuRoC / TUM VI / KITTI benchmark papers** | How localisation systems are actually evaluated — metrics, datasets, failure reporting | Theory 03 §9 |

## Tier 4 — Reinforcement learning

| # | Source | Why | Feeds |
| --- | --- | --- | --- |
| 18 | **Spinning Up in Deep RL** (OpenAI, free) | The clearest implementation-first introduction; PPO, SAC, TD3 with code | Theory 05 §3 |
| 19 | **Reinforcement Learning: An Introduction** — Sutton & Barto (free PDF) | The foundations: MDPs, POMDPs, value/policy methods, credit assignment. Required for the terminology | Theory 05 §2 |
| 20 | **Benchmark suites: Gymnasium / MuJoCo / Isaac Lab documentation** | Environment design, vectorised training, domain randomisation mechanics | Theory 05 §4 |
| 21 | **Safe RL surveys** (e.g. constrained MDPs, Lagrangian methods, CPO) | The constraint-handling toolbox used in Theorem 05 §5 | Theory 05 §5 |
| 22 | **Sim-to-real papers** — domain randomisation (Tobin et al.), domain adaptation, residual RL (Johannink et al.), DreamerV3 | The practices that actually transfer, and the evidence for each | Theory 05 §6 |

## Tier 5 — Safety and verification

| # | Source | Why | Feeds |
| --- | --- | --- | --- |
| 23 | **Ames et al., "Control Barrier Functions: Theory and Applications"** (survey, free) | The mathematical core of the safety filter | Theory 06 §3 |
| 24 | **CBF-QP papers** — Ames, Grizzle, Tabuada; plus discrete-time CBF formulations | The exact formulation, feasibility issues and discretisation treatment | Theory 06 §3.4–3.5 |
| 25 | **ISO 12100 / ISO 3691-4 / ISO 13849 overviews** (summaries are enough; the standards themselves are expensive) | Methodology and vocabulary for the safety case | Theory 08 |
| 26 | **Safe control with learned components** — shielding, safety layers, verified supervision literature | The precise question this project answers architecturally | Theory 06 §2 |
| 27 | **Scenario-based verification and falsification** literature | How to generate the scenario suite that actually finds failures rather than confirming success | Theory 08 §5 |

## Tier 6 — Perception and deployment

| # | Source | Why | Feeds |
| --- | --- | --- | --- |
| 28 | **Computer Vision: Algorithms and Applications** — Szeliski (free PDF) | Geometry: calibration, projection, epipolar reasoning — needed to trust projected obstacle positions | Theory 07 |
| 29 | **YOLO / RT-DETR papers and Ultralytics docs** | The detection family used; the author already has applied experience here | Theory 07 §3 |
| 30 | **TensorRT / ONNX Runtime / Jetson deployment guides** | Meeting the latency budget on edge hardware, quantisation, and how accuracy changes when you do | Theory 07 §4 |
| 31 | **NVIDIA Jetson Linux developer guide** | Power modes, thermal behaviour, GPU sharing between inference and the solver — the practical constraint on the whole architecture | Theory 07 §4, doc 03 §3 |

## Practical resources (not papers, but decision-relevant)

| Source | Use |
| --- | --- |
| ROS 2 Nav2 documentation and design articles | The reference implementation to beat — study its costmap and controller architecture before designing your own |
| ROS 2 Control documentation | Hardware abstraction for the drive layer; the standard interface between firmware and control |
| micro-ROS documentation | The other MCU integration path (instead of a custom serial protocol) — compare before choosing |
| Docker + NVIDIA Container Toolkit docs | The containerization requirement; GPU passthrough and its caveats |
| MCAP / rosbag2 documentation | Logging format, replay for component testing |

## Reading discipline

1. **Read to decide, not to finish.** Each entry exists to unblock a specific design decision; read until the decision is made, then write the decision down.
2. **Implement the smallest possible example** of each method before integrating it. Knowledge that has not been run is a guess.
3. **Write down what you expected before running anything.** The disagreement between expectation and result is where the learning is.
4. **Do not collect references you have not read.** A bibliography no one has opened is a liability, not a signal.
