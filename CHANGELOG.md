# Changelog

All notable changes to this design record. Format inspired by Keep a Changelog; the project is pre-implementation, so entries are design records rather than releases.

## [0.5.3] — 2026-10-11 — Theory 09: planning among moving agents

**Added**

- `docs/theory/09-interaction-aware-planning.md`: two documented approaches. A: prediction-conditioned NMPC with chance constraints (Core). B: interaction-aware planning with a learned response model (Advanced). Includes staged plan, failure modes, evaluation protocol and open questions. Nothing implemented or measured.
- `docs/theory/README.md`: row for theory 09.

**Not applied:** docs 00 to 02 unchanged.

## [0.5.2] — 2026-10-11 — Layer decisions for perception, planning and decision making

**Added**

- `docs/02-system-architecture.md` §9: proposed layer decisions for perception (typed output contract), interaction prediction, planning (L3 NMPC with real-time iteration, fixed L4 global planner), decision making (learned choice among bounded behaviours, rule-based mode changes and fallbacks), rejected alternatives, alternatives still to benchmark (sampling-based MPC, robust learned safety filter, interaction-aware planning) and the evaluation criteria.

**Not applied:** §§1–8 of doc 02 are unchanged. §9 is a proposal under review, and the sim-to-real layer is excluded from it.

## [0.5.1] — 2026-10-09 — Industry case study: three more companies

**Added**

- Tesla Optimus, Figure AI (Figure 03, Helix and Helix 02) and Amazon Robotics (Proteus, Vulcan, DeepFleet) added to `docs/10-industry-case-study.md`, each covering the seven requested items. The study now covers eleven companies and 97 sources.
- Reading tag **K** for company claims with no specification or test document behind them (keynotes, executive posts, demonstration videos). K values are never presented as measurements.
- Backlog IG-8 (contact-force and impact-energy budget), IG-9 (battery-safety requirement), BL-5 (demonstration and teleoperation data plan), BL-6 (rule for learned simulators) and BL-7 (refresh doc 08 §4 Helix figures from the primary releases). **Not applied:** docs 00 to 02 are unchanged until the dilemma decision in doc 07 is confirmed.

**Corrected**

- `docs/08-state-of-the-art-analysis.md` §5.2: the statement that only three robots hold ATEX/IECEx Zone 1 certification now names its source and is marked unverified.
- `docs/theory/06-rl-mpc-integration-and-safety.md` §2.1: "evidence this choice is correct" and "settled practice at the top of the field" are reworded. Convergence supports the reasoning; it is not evidence that the design performs.

**Gaps found**

- **IG-8.** No contact-force or impact-energy requirement. Derived whole-body kinetic energy is about 10 J for our 20 kg platform at 1.0 m/s, against about 44 J for Figure 03 at 1.2 m/s. This is a scale comparison only; neither company publishes an impact-energy limit.
- **IG-9.** No battery-safety requirement. BC-06 states voltage and capacity only.
- **Link-fault testing.** Doc 02 §2 specifies CRC, sequence numbers and a heartbeat on the MCU link, and SF-02 names comms, but no link-fault injection test is specified (doc 10 §5.1, §7.5).
- **Learning placement.** Figure's learned policy runs at 200 Hz and its whole-body controller at 1 kHz; our learned layer runs at 1–5 Hz and does not actuate. This is recorded as a different design choice (doc 10 §3.2, §7.3), not as a convergence.
- **Safety publication.** None of the three new companies publishes a safety-rated stop chain or an independent safety function (N).

## [0.5.0] — 2026-10-09 — Industry case study

**Added**

- `docs/10-industry-case-study.md` — eight well-known robotics companies studied on the seven requested dimensions: capabilities, limitations, controllers, motors, drivers, simulation environments, and comparison with our theoretical work. The companies are Clearpath (Jackal), MiR (MiR250), Boston Dynamics (Spot), ANYbotics (ANYmal and the ANYdrive actuator), Unitree (Go2 and the GO-M8010-6 joint motor), Agility Robotics (Digit), Franka Robotics (FR3) and Universal Robots (UR10e). Every figure carries a source ID and a quality tag (primary, secondary, or not published). Conflicting figures are shown, not averaged. 58 sources are listed with URLs.
- Derived comparisons, with inputs shown: the average power implied by our runtime and battery requirements (about 200 W); our 5° ramp requirement against the MiR250's published 5% incline (about 2.9°); and our 100 Hz command rate against PR-12.
- Backlog IG-1 to IG-7 and BL-1 to BL-4 (doc 10 §8). **Not applied:** docs 00 to 02 are unchanged until the dilemma decision in doc 07 is confirmed.

**Gaps found**

- **IG-1.** The 100 Hz wheel-command bridge cannot support PR-12 (speed-regulation bandwidth of at least 100 Hz, measured by step response). It is also the slowest link in the chain.
- **IG-2.** No payload requirement in doc 01.
- **IG-3.** No energy budget. PR-10 and BC-06 together allow about 200 W average for drive, compute and sensors combined.
- **IG-4.** No safety-rated stop chain is specified. The cases specify PL d chains.
- **IG-5.** No training-simulator decision and no sim-to-sim validation step.
- **IG-6.** No ingress or environment rating.
- **IG-7.** The 5° ramp requirement (BC-04) is not tied to a drive-torque budget. A published wheeled reference is rated at about 2.9°.

## [0.4.0] — 2026-10-09 — Architecture comparison

**Added**

- `docs/09-architecture-comparison.md` — a side-by-side matrix of this specification against six comparator groups (Waymo Driver, Atlas, the VLA frontier, ANYmal X, Perseverance, and the Chinese volume makers Unitree, AgiBot and UBTECH) across 23 dimensions. The Chinese sources are added to doc 08 §10. Every row carries a verdict: **Converged**, **Different**, **Behind**, **Gap** or **Unproven**. Every "ours" cell cites its source document and states whether a value is a requirement target or a measurement; no "ours" value is presented as measured. Also: a structural-differences section, a proposed-changes table, and a self-audit of the corrections below.
- Revision backlog for docs 01–04 (in doc 09 §5). **Not applied:** docs 00–02 are unchanged until the dilemma decision in doc 07 is confirmed.

**Corrected**

- **Waymo's validation layer was called "non-learned".** Waymo describes it as AI-based. The pattern (learned proposer, independent verifier with veto) matches ours; the implementation differs. Corrected in doc 08 §2.2 and §8 and theory 06 §2.1.
- **"Safety-critical backbone" was attributed to Boston Dynamics.** It comes from a secondary analysis of Atlas. Corrected in doc 08 §8 and theory 06 §2.1.
- **"Architecture validated by industry convergence" (0.3.0) overstated the evidence.** Convergence supports the reasoning; it is not evidence that the design performs.
- **"Waymo's failures are specification failures, not perception failures" (0.3.0) overstated the evidence.** The flood is a missing declared response. Emergency-scene and school-bus failures include recognition and escalation failures. Corrected in doc 08 §2.4 and theory 06 §2.1.
- **Overclaims narrowed.** "The only system", "exact convergence", "all seven systems" and "nobody has published" now refer to the systems and sources actually reviewed (doc 08 §1, §2.2, §8, §9).
- **Waymo sensor generations mixed up.** The 13-camera, 4-lidar, 6-radar configuration is the 2024 *6th*-generation suite (doc 08 §2.1, §10).
- **Flood explanation attributed to journalism.** NHTSA's filing says the vehicle "detected potentially untraversable flood water and proceeded at reduced speed". The "no hard-stop condition" explanation is the journalist's analysis (doc 08 §2.4).
- **"Only three robots worldwide hold ATEX/IECEx Zone 1" (0.3.0)** rests on one procurement analysis. Treat it as unverified until checked against primary certification records.
- **"Verified stop path" → "tested"** (theory 06 §4). Doc 02 §6 ("verified stopping envelope") is queued in the backlog.
- Mistyped source domain in doc 08 §10 fixed.
- **README overclaim.** The introduction said the system was "developed and validated" in the Docker toolchain. It is a plan and the README's own status line says theoretical phase, so it now reads "designed to be developed and validated".

**Removed**

- **Two items unrelated to the system.** A personal-circumstances risk row (doc 05, R-17) and a dilemma criterion (doc 07, T6), with its scores and one sentence. No dilemma verdict in doc 07 §3 depended on T6.

**Gaps found**

- **G1 — The safety filter does not satisfy SF-01 as specified.** The CBF-QP filter and supervisor run as ROS 2 nodes on the edge computer (doc 02 §3), beside the learning stack. Only the MCU gate runs on separate compute. Proposed: a minimal safety kernel on the MCU, with inputs it does not share with the supervised stack.
- **G2 — No hazard-to-response register.** The degradation ladder responds to generic triggers only. Proposed: E1 in doc 07 §4, with a row for each hazard class, its declared response and its measured worst case.
- **G3 — No change control for learned or tuned components.** The interface freeze covers interfaces only. Doc 03 says run metadata names weight versions, but its metadata schema has no such field. Proposed: E5 and `weights_sha256` / `dataset_sha256` in the schema.
- **G4 — `safety_field` is consumed but never published.** The filter subscribes to it (doc 02 §3); no node produces it. The ring that should produce it is T3 hardware, and the T2 bill of materials has no computer although T2 is where NMPC is proven on hardware. Decisions needed: the T2 near-field input (contact bumper versus ring) and the T2 compute platform.
- **G5 — "Verified" used where the evidence is measurement** (theory 06 §4; doc 02 §6).

## [0.3.0] — 2026-10-08 — State-of-the-art analysis

**Added**

- `docs/08-state-of-the-art-analysis.md` — seven axes of "advanced" instead of a single ranking; six frontier systems deep-dived with architecture and measured limits: **Waymo Driver** (deployed scale + safety case), **Boston Dynamics Atlas** (dynamic whole-body control), the **VLA frontier** (π0.5/π\*0.6, Helix, Gemini Robotics), **ANYbotics ANYmal X** (ATEX/IECEx Zone 1 certification), **NASA Perseverance** (comms- and compute-constrained autonomy), and the **Chinese volume frontier** (AgiBot/Unitree/UBTECH); a synthesis of seven convergent architecture patterns; and source-quality flags distinguishing primary regulatory filings and peer-reviewed work from vendor marketing.

**Key findings recorded**

- **Architecture validated by industry convergence.** Waymo's independent onboard validation layer — a non-learned verifier with hard veto over the learned driver's trajectories — is the industrial form of Topology 2 + CBF-QP from theory 06. Boston Dynamics describes the same split (learned policies, classical controller as safety-critical backbone). The architecture is settled practice; **the evidence pipeline around it is the open problem.**
- **Waymo's failures are specification failures, not perception failures.** The flood was detected and the system slowed but did not stop — "there was no hard-stop condition for water in its path". Construction zones, emergency scenes and school-bus handling fail for the same reason: hazards outside the *declared and enforced* operating scope. This is empirical support for doc 07's dilemma and elevates E1/E2 (capability declaration + bounded-autonomy monitor) to first-class architectural components.
- **Certification is solved for bounded classical autonomy and unsolved for learning-enabled safety functions.** Only three robots worldwide hold ATEX/IECEx Zone 1 certification; none has learning-enabled safety functions — and the EU Machinery Regulation makes exactly that case mandatory from 20 January 2027.
- **Uncertainty-driven conservatism collapses the operating envelope.** Perseverance's localisation uncertainty grew to a 26.91 m radius, expanding hazard zones until the navigable corridor vanished and the drive faulted after 14 m. This is our own deadlock risk (theory 06 §3.4, doc 05 R-04) with a measured number attached. Lesson: attack margins at the estimation source, not by loosening the filter.
- **Reduce the reality gap in hardware, not only in software.** Boston Dynamics' small sim-to-real gap traces to mechanism design — two actuator types, identical limbs, no cables across joints, rotary actuators. Our platform's mechanical simplicity is a control and reproducibility asset to be stated explicitly.
- **Production is not deployment.** ~16,000 humanoids installed globally in 2025 with Chinese makers above 80% of volume, but one leader's revenue is reportedly ~70% research and education, and claimed success rates are unaudited vendor figures.

**Changed**

- `README.md` — documentation map now points to doc 08.
- `docs/theory/06-rl-mpc-integration-and-safety.md` — added §2.1 "Industry convergence", recording that the selected integration topology was independently reached by the most advanced deployed robotic system, and distinguishing the settled architecture from the open evidence problem.

## [0.2.0] — 2026-10-08 — Dilemma selection

**Added**

- `docs/07-open-dilemmas.md` — the foundational document: a six-test filter for judging whether a dilemma is worth attacking; eighteen open dilemmas in four classes (assurance, capability, physical/embedded, interaction & security), each with the evidence it is unsolved and an honest tractability assessment; a scoring matrix; and the selection of the target dilemma.
- **Target dilemma adopted:** the evidence gap for learning-enabled safety components — the EU Machinery Regulation (EU) 2023/1230 applies 20 January 2027 with ML safety components named as a high-risk category requiring Notified Body assessment, while no accepted method for producing the required evidence exists. Supported by two technical mechanisms (runtime-assurance preconditions that designs do not measure; OOD detection mistaken for safety monitoring).
- **Six evidence artifacts specified (E1–E6):** capability declaration, bounded-autonomy monitor, measured monitor characterisation, traceability register, change-and-revalidation record, and the safety case assembled from them.
- Downstream impact table: the changes required in docs 00–04, theory 06, and doc 08 once this dilemma is adopted.
- Explicit out-of-scope list, and a source-quality table that flags which claims come from peer-reviewed work, preprints, industry analysis, or vendor marketing.

**Changed**

- `README.md` — added the dilemma statement and pointed the documentation map at doc 07 as the starting point.

**Rationale**

The previous specification described *what* the system is without establishing *which unsolved problem* justifies its existence. The selected dilemma passes all six selection tests — including the one that eliminates most grand challenges: it can be advanced by one person with a small robot, because the deliverable is a methodology and a measured evidence pipeline, not a proof.

## [0.1.0] — 2026-10-08 — Theoretical phase: system specification

**Added**

- `README.md` — project definition, layer summary, documentation map, design principles.
- `docs/00-mission-and-scope.md` — problem statement, five mission scenarios (S1–S5), three frontier problems, scope discipline (Core / Advanced / Stretch) and the definition of done.
- `docs/01-requirements.md` — 10 functional, 13 performance, 9 software and 8 safety requirements with verification methods; boundary conditions, assumption register, traceability extract.
- `docs/02-system-architecture.md` — five-layer model with rate bands, compute allocation (MCU/high-level split), ROS 2 node graph with QoS, timing and synchronisation rules, RL+NMPC+safety control architecture, six-level degradation ladder, logging and reproducibility architecture, interface freeze list.
- `docs/03-simulation-and-docker.md` — container image architecture (base/sim/control/ml-train/ml-deploy/dev/ci), compose profiles, DDS networking options, GPU/GUI/real-time caveats, volume and artifact policy, CI pipeline with performance-regression gating, deployment path, reproducibility contract.
- `docs/04-build-roadmap.md` — milestones M0–M7 with gate criteria, cost tiers T0–T4 and acquisition order, interface freeze artifacts, target repository structure, working agreement.
- `docs/05-risk-register.md` — 17 technical, resource and schedule risks with mitigations and decision triggers.
- `docs/06-reading-list.md` — 31 mapped sources in six tiers, each tied to the design decision it unblocks.
- `docs/theory/01-kinematics-and-dynamics.md` — frames, differential-drive kinematics, nonholonomic constraint consequences, Lagrangian and reduced-order dynamics, integrator verification, parameter identification plan, model-fidelity test.
- `docs/theory/02-electric-drive-pmsm-foc.md` — PMSM dq model, torque production, inverter and current constraints, FOC cascade design and bandwidth separation, MTPA and field weakening, NMPC on the current loop with an honest comparison to PI, middle-path decomposition, verification and open questions.
- `docs/theory/03-state-estimation.md` — odometry error growth, IMU fusion and bias observability, EKF formulation with consistency testing, localisation method comparison, slip detection, time/delay handling, verification plan.
- `docs/theory/04-advanced-control-nmpc.md` — OCP formulation, velocity-dependent obstacle margins, SDF constraints, path-progress parameterisation, discretisation and solver selection, solve-time budget, stability claim boundaries, robustness and tube MPC, baseline alternatives, verification matrix.
- `docs/theory/05-reinforcement-learning.md` — where RL belongs (and where it does not), MDP/POMDP formulation, mission-layer and residual-layer designs, algorithm selection with seeds, curriculum and domain randomisation, reward discipline, constrained RL, sim-to-real practices, honest claims and verification.
- `docs/theory/06-rl-mpc-integration-and-safety.md` — four integration topologies and the selection rationale, control barrier functions and the CBF-QP safety filter, practical failure modes of the filter, discrete implementation of a continuous guarantee, complete layered safety architecture, claim boundary table.
- `docs/theory/07-perception-dl.md` — sensing suite with redundancy rationale, perception task formulations, architecture choices under an edge-compute budget, latency budget breakdown, data bootstrapping pipeline, evaluation metrics with cost asymmetry, degradation under adverse conditions, verification.
- `docs/theory/08-safety-and-verification.md` — hazard analysis (ISO 12100 methodology, H-01…H-08), applicable standards and the explicit no-certification framing, safety architecture with attributed claims, stop categories and reaction times, V&V plan across eight test levels, scenario suite, safety case structure.

**Notes**

- This is a specification, not an implementation. No code, model weights or datasets are present by design.
- Every numerical value labeled a target is a design target; nothing in this repository is a measured result yet.
- The specification deliberately includes a laptop-free theoretical phase aligned with the author's current development-hardware constraint.
