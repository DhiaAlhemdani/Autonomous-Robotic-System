# 09 — Architecture Comparison: This Project vs. the Frontier

**Scope.** This document compares the architecture as currently specified in the repository (docs [00](00-mission-and-scope.md), [01](01-requirements.md), [02](02-system-architecture.md), [03](03-simulation-and-docker.md), [04](04-build-roadmap.md), and theory [06](theory/06-rl-mpc-integration-and-safety.md) and [07](theory/07-perception-dl.md)) with the frontier systems analysed in [doc 08](08-state-of-the-art-analysis.md).

Two rules apply throughout:

1. **Ours is specified, not built.** No value in the "Ours" column is a measurement. Requirement targets are labelled as targets.
2. **Frontier figures are public claims.** Where a figure comes from a vendor, trade press or a secondary analysis, the cell says so, following doc 08 §10.

**Status:** this pass. The proposed changes in §5 are **not applied** to docs 00–02. That waits for your decision on the dilemma in [doc 07](07-open-dilemmas.md), as planned.

---

## 1. Verdict legend

| Verdict | Meaning |
| --- | --- |
| **Converged** | Same architectural pattern as the frontier. Evidence that the reasoning is sound, not that the design works. |
| **Different** | Deliberately different, with the reason tied to our scope (doc 00 §5). Defensible only while that reason holds. |
| **Behind** | The frontier has it and we do not. Mostly a budget fact. Accepted, and stated as such. |
| **Gap** | A hole in our current specification that the frontier's public failures show is costly. Fix regardless of budget. |
| **Unproven** | Specified, not built or measured. Not evidence. |

The most important distinction is between **Behind** and **Gap**. "Behind" describes a budget. "Gap" describes a design.

---

## 2. The matrix

Our layers follow [doc 02 §1](02-system-architecture.md): L5 policy, L4 global planner, L3 NMPC, L2 safety filter and supervisor, L1 drive.

| Dimension | Ours (as specified) | Waymo Driver | Atlas | VLA (π0.5 / π\*0.6 / Helix) | ANYmal X | Perseverance | Chinese volume makers (Unitree, AgiBot, UBTECH) | Verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Operating domain** | Indoor, GPS-denied, unmapped facility with humans present; ≤1.0 m/s autonomous (FR-01, PR-01) | Public roads, geofenced; driverless in 11 US cities | Factory logistics; pilot availability | Manipulation tasks (tabletop, household, logistics); mobility varies by system | Industrial plants and Zone 1 hazardous areas; predefined routes | Mars surface; up to 40-minute signal delay | Research and education first (Unitree: about 70% of revenue, secondary). Factory and logistics pilots (UBTECH, AgiBot; vendor-reported) | **Different** (scope) |
| **Platform and actuation** | Differential drive, ~20 kg (≤25 kg, BC-01); BLDC/PMSM with FOC on the MCU; current loop ≥10 kHz (PR-11) | Passenger car; drive-by-wire steering, throttle and brake | 90 kg, 56 DoF; electric actuators with planetary roller screws (secondary) | Mostly fixed-base manipulation in the published π experiments | Quadruped; spring-based joints (ANYdrive) | Wheeled rover | G1: 1.32 m, about 35 kg, 23–43 DoF, PMSM joint motors (Unitree page). Walker S2: 1.76 m, about 70 kg, 52 DoF (secondary) | **Different**: our mechanism is deliberately simple and measurable (doc 08 §3.2) |
| **Sensing** | 2D LiDAR 10–20 Hz; RGB-D 15–30 Hz; IMU 200–1,000 Hz; encoders; ultrasonic/IR ring 20–50 Hz (theory 07; RGB-D and ring are T3 in doc 04) | 6th gen (2024): 13 cameras, 4 lidar, 6 radar, audio receivers. 2026: 17 MP imagers, imaging radar | Lidar and stereo vision; depth cameras (secondary) | Multi-view RGB plus proprioception | Visual zoom, thermal, acoustic, gas and lidar | Navcams for visual odometry; orbital imagery as prior | G1: depth camera and 3D LiDAR (Unitree page). Walker S2: stereo vision, LiDAR, six-axis force sensing (secondary) | **Behind**: two main exteroceptive modalities plus a ring (risk R-06) |
| **Localisation and map prior** | No prior map (FR-02); EKF with SLAM; ≤0.10 m target in a mapped space (PR-03) | HD maps used as a prior, "like our sensors" | Not publicly described for navigation | None; scene-relative | Predefined inspection points and routes | Visual odometry plus global localisation to HiRISE: ~25 cm mode, 1.07 m maximum over 264 stops | Not described in our sources | **Different**: harder localisation, but no stale map. Unproven for us (R-07) |
| **Compute partitioning** | MCU for FOC, watchdog, torque gate and bridge; edge GPU for perception, NMPC, planning, policy and logging (doc 02 §2; edge GPU is T3 in doc 04) | Custom 5 nm ASIC above 1,000 TOPS, alongside NVIDIA accelerated computing (trade press on vendor disclosure) | Jetson Thor T5000-class module (secondary) | GPU-hosted models: ~3.3 B parameters (π0.5); Gemma 3 4B backbone and 860 M action expert (π0.6) | Not published | Radiation-hardened, 2010s-era silicon; a repurposed base-station processor about 100× faster, used for localisation | G1: 8-core CPU, optional Jetson Orin NX (Unitree page). Walker S2: compute housed in the torso (secondary). Real-time partition not described | **Converged** in principle: a separate hard-real-time path for actuation |
| **Compute redundancy** | One edge computer. If it fails, the MCU stops the robot (SF-02) | Two independent processing engines; one takes over on a fault (trade press on vendor disclosure) | Not published | Not published | Not published | Not covered here | Not described in our sources | **Behind**: accepted, because a stop is a safe state at ≤1 m/s indoors. Conditional on G1 |
| **Safety-filter compute (SF-01)** | CBF-QP filter and supervisor run as edge-computer nodes beside the policy; only the MCU gate is on separate compute (doc 02 §2–3) | Two independent processing engines; one takes over on a fault (trade press on vendor disclosure) | Not published | Not published | Not published | Not covered here | Not described in our sources | **Gap** (G1): the requirement is met only for the stop path |
| **Independent verifier** | CBF-QP filter at 100–200 Hz; analytic; veto over every command from L3 and L5 (doc 02 §1, theory 06 §3) | Onboard validation layer checks every trajectory against physics and traffic rules, with a hard veto. Waymo describes it as AI-based | Classical whole-body controller as the "backbone" (a secondary analysis, not a Boston Dynamics statement) | No independent verifier described publicly | Explosion-protection certification; autonomy internals not public | Hazard detection and planning; ground review of each drive plan | Not described in our sources. HEIS 2026 requires fault diagnosis and emergency stop (trade press) | **Converged** in pattern. **Different** in implementation: ours is testable in isolation; theirs is learned |
| **Learning placement and authority** | RL at L5 (1–5 Hz) selects behaviour and waypoint; cannot actuate; residual RL (T3) is an advanced experiment | Learned driver distilled to onboard models; RL in simulation scored by a Critic | Learned locomotion and dexterity policies trained in simulation with domain randomisation (secondary) | End-to-end action chunks at 50 Hz (π0.6); RL from experience (π\*0.6); System 1 at 200 Hz (Helix) | Public descriptions do not indicate learned components in certified functions | Classical autonomy; AI-generated waypoints in one 2025 demonstration | Learned policies drive the joints directly: RL locomotion (Unitree RL GYM, vendor code); GO-1's action expert outputs the control signals (AgiBot paper). No bounded layer described | **Different** by design: we bound learned rate and authority; they bound learned capability with a validator |
| **Motion and optimisation** | Explicit-model NMPC (CasADi/acados); SDF obstacle constraints; solve p99 ≤45 ms, hard limit 50 ms (PR-05, target) | Not disclosed; learned trajectory generation | Whole-body QP for the hydraulic Atlas in the DARPA Robotics Challenge (CMU, 2014); today classical control with learned policies (secondary) | No explicit model; action chunks with real-time chunking | Not published | Classical planning; hazard zones expand with localisation uncertainty | RL-trained locomotion, checked sim-to-sim in MuJoCo (Unitree RL GYM). No MPC or optimisation layer described | **Converged** with Atlas's optimisation lineage. **Different** from learned-model approaches |
| **Failure philosophy** | **Degrade, then stop.** Continue at reduced speed on a lost modality; stop when no safe continuation exists; any single component loss leads to a safe state (SF-02) | **Fail-operational.** A second engine takes over | Not published | Not applicable | Explosion-protected design (certified) | Ends the drive early and waits for ground when uncertain | Remote damping mode: the motors enter damping and the robot falls slowly on command (G1 manual). No other failure response described | **Different** by design: correct only for a stop that is itself safe |
| **Independent hardware stop** | MCU watchdog, torque-enable line and physical E-stop, independent of software (SF-03); stop ≤100 ms target, measured (PR-13) | Not covered in our sources | Not published | Not applicable | Certified; stop details not public | Not covered here | No hardware stop chain described. The emergency stop is a remote-control command into damping mode (G1 manual) | **Converged** in principle. The gap is in the filter, not the stop (G1) |
| **Degradation and failure handling** | Six-level ladder; fallback order: slack re-solve, reactive controller, protective stop (doc 02 §6) | Remote assistance escalation. 2026 flood case: hazard detected, vehicle slowed but did not stop | Not published | Not published | Not published | Keep-out zones grew with uncertainty (26.9 m radius); drive faulted after 14 m | Not described in our sources | **Converged** in design; **Unproven** until built. The failure class that matters is G2 |
| **Declared operating envelope** | Scenarios S1–S5; sensing-envelope barrier (theory 06 §3.3). No enumerated hazard table. E1 is proposed in doc 07, not in doc 02 | Geofenced operating domain; 2026 recalls for flooding and construction zones; freeway service restricted May–July 2026 | Not published | Scope defined by benchmarks | Certified Zone 1 IIB boundary; defined inspection routes | Drive plans reviewed by ground before execution | Datasheet limits only, e.g. arm payload about 2 kg (3 kg EDU) (Unitree page). No hazard-linked envelope | **Gap** (G2) |
| **Rates and timing** | Plan 0.5–2 Hz; policy 1–5 Hz; NMPC 20–50 Hz; filter 100–200 Hz; FOC 10–20 kHz; perception p99 ≤150 ms (PR-06) | No rates published | Not published | Helix System 2 at 7–9 Hz, System 1 at 200 Hz (secondary); π\*0.6 actions at 50 Hz | Not published | 2 Mbps radio; up to 40-minute round trip | Control rates not published in our sources | **Converged** on rate separation. Ours states timing as testable requirements |
| **Measurement discipline** | p50, p99 and max latency; jitter measured; an unmeasured controller "is not considered validated" (doc 02 §4) | Detailed incident and safety-impact reporting; latency not published | Little audited data (secondary) | Benchmark saturation: 90–95% across policies of very different real-world quality (secondary) | Third-party certification | Peer-reviewed; 264-stop validation | Vendor figures only, e.g. 99.9% task success, unaudited. No latency or stop-time measurements published | **Converged** in principle. **Unproven** for us |
| **Simulation and training** | Gazebo; digest-pinned Docker images; headless CI with NMPC solve-time regression gating (doc 03); ≥5 seeds (R-10) | Simulator and Critic in the inner learning loop (vendor); "billions" of simulated miles (secondary) | Isaac Lab; millions of simulated hours with randomised weight, friction and grip (secondary) | Large teleoperation and web datasets (Helix: ~500 h of teleoperation) | Not published | Ground-in-the-loop testing (not detailed here) | Isaac Gym training with MuJoCo sim-to-sim checks (Unitree RL GYM). GO-1 trained on AgiBot World real-robot data, over 1M trajectories (secondary) | **Behind** on scale; **Converged** on sim-first; **Different** on priority (reproducibility over volume) |
| **Reproducibility and change control** | Run metadata (commit, image digest, config hash, seed); dirty-tree results non-reportable (doc 03); interface freeze (doc 02 §8). **No change record for models or parameters, and no weight checksum in the metadata schema** | Learning loops deployed only after the safety framework confirms absence of unreasonable risk (vendor). An over-the-air patch to 3,791 vehicles failed within two weeks | Not published | Some checkpoints open (secondary) | Not published | Not covered here | Open code (Unitree) and open weights under a non-commercial licence (GO-1, secondary). No change-control record described | **Gap** (G3) |
| **Evidence volume** | Designed; targets only: ≥30 mission runs (PR-07), ≥50 collision-free runs (PR-08) | 200+ million autonomous miles; NHTSA crash reporting | Pilot only; no public field dataset | Lab and home evaluations. π0.5 success falls to 74% without web data and 49% without cross-embodiment data | Certified; industrial deployments reported (secondary) | 17.7 km in the first Earth year; 699.9 m autonomous segment | Production, not deployment: about 16,000 humanoids installed worldwide in 2025, over 80% in China; AgiBot about 8,400 and Unitree about 5,900 in H1 2026 (secondary). Field evidence not public | **Behind**: unavoidable at our scale. Our route is method and measurement |
| **Certification and regulation** | None claimed (doc 00 §5). ISO 12100, ISO 3691-4 and ISO 13849 methodology (SF-07) | Self-described safety framework and safety case; NHTSA and NTSB oversight | Not publicly documented | None public | Third-party ATEX/IECEx Zone 1 IIB | Agency review (not detailed here) | HEIS 2026 national standard system (MIIT, Feb 2026), safety provisions phased in as group standards first (trade press). No product certificate found in our search; one listing claims CE (unverified) | **Behind** by scope. Directly relevant: third-party assessment applies to learning-enabled safety components from 20 January 2027 (doc 07 D1) |
| **Human proximity** | Velocity-dependent human barrier; ≥0.35 m target, never <0.20 m (PR-09); near-field ring in T3 | Child struck, January 2026 (minor injuries); braked from about 17 mph to under 6 mph (press reports) | Fenceless operation with human detection claimed (secondary) | Household settings; no proximity metric in our sources | Not published | Not applicable | Vendor advises keeping a 'sufficient safe distance' from people (Unitree page). No separation or speed rule described | **Converged** in approach. **Unproven** for us |
| **Remote fallback** | Protective stop requests an operator (doc 02 §6). No remote guidance designed | About 70 remote agents for about 4,000 vehicles, half in the Philippines. They advise; they do not drive | Not published in our sources | Teleoperation is the data source | Plant-software integration (Yokogawa OpreX, February 2026) | Ground-in-the-loop localisation and drive plans | Handheld remote for mode changes and emergency damping (G1 manual). No remote supervision or fleet guidance described | **Different**: remote advice is itself a failure surface (the Waymo school-bus case). We keep it out of the safety path |
| **Cost and scale** | Planned hardware, T2 + T3: $2,200–4,500 (doc 04); two identical drive motors | Custom silicon; thousands of vehicles | Stated plan of 30,000 units a year by 2028 (secondary); no list price | Large GPU and data budgets | Quote-based pricing | Not comparable | G1 from US$13.5k (Unitree page). Walker S2 prices conflict between listings (secondary). Unit volumes are in the evidence-volume row | **Behind** on scale. **Different** on objective: we optimise cost per measured result, not per unit shipped |

---

## 3. What the matrix says

Five structural differences account for most of the verdicts. Each is a choice, not a shortfall, except where §4 says otherwise.

**3.1 Who verifies the learner.** Waymo verifies a learned driver with a learned validator. We verify a learned policy with an analytic filter whose constraint set can be tested alone. The trade is generality for inspectability. Our design moves the certification question into a smaller, non-learned component. That only helps if the constraint set is complete, and §4 shows it is not yet.

**3.2 When is stopping safe?** Waymo's freeway and flood cases involve speed and scale. A stop on a freeway can itself cause a crash. A stop by a 1 m/s robot in a facility usually does not. Fail-operational is therefore the right answer for a car and an unnecessary cost for us. The argument holds only inside a *declared* envelope (speed, environment, hazard classes). Without that declaration, "stopping is safe" is an assumption rather than a specification.

**3.3 Map prior.** Waymo and Perseverance both rely on a prior: HD maps and orbital imagery. We have none. Localisation is harder, and PR-03 and risk R-07 carry the cost. In return we avoid the failure of a stale map. Perseverance's 26.9 m uncertainty case shows what happens when drift is not corrected by a prior. Our mitigation is measurement in S1 and S3, not a claim.

**3.4 Authority of learning.** Our learned components have bounded rate and no actuation authority. Theirs have large authority, bounded by a validator. Ours is smaller and less capable by design, and easier to bound. The difference is one of kind, not degree.

**3.5 Source of evidence.** Waymo produces evidence by volume: 200+ million miles, simulation, and federal reporting. A 20 kg robot cannot. Our method is to specify the evidence, measure a small system completely, and publish where it fails. That is a legitimate position. It is a method claim, not a performance claim.

**3.6 The Chinese volume makers.** They ship more humanoids than any other group here and publish little about verification. Production is not deployment (doc 08 §7). Their public stacks put learned policies directly on the actuators, and no independent verifier is described in the sources we read, which matches the VLA column. Two points are new. First, the emergency stop is a remote command that puts the motors into damping, so the robot falls slowly (G1 manual). The stop path runs through a radio link and a person, which is the failure surface the remote-fallback row flags. Second, the national standard requires emergency-stop and fault-diagnosis provisions (trade press), but we could not find how any product meets them. No verdict in §2 changes on this evidence.

---

## 4. Gaps the comparison exposes in our own specification

The comparison shows where the frontier's public failures would also apply to us. These are the most valuable findings of this document.

### G1 — The safety filter's compute placement does not satisfy SF-01

- **Requirement.** SF-01 states that the safety function is "independent of the learning-enabled stack (separate compute, separate code path)".
- **Specification.** [Doc 02 §2](02-system-architecture.md) assigns L2 to neither computer. In the [doc 02 §3](02-system-architecture.md) node graph, `safety_filter` and `safety_supervisor` are ROS 2 nodes, so they run on the edge computer beside the L5 policy and the L3 NMPC. The CBF-QP is independent in code path but not in compute. Only the MCU gate runs on separate compute.
- **Frontier.** Waymo's answer to compute independence is duplication: two independent processing engines, one of which takes over on a fault (doc 08 §2.1). Public sources do not say which engine hosts the validation layer.
- **Proposal.** Add a **minimal safety kernel on the MCU** with its own inputs. It enforces a speed ceiling against near-field distance (SF-06, once the near-field input in G4 is chosen), hard speed and acceleration limits, the watchdog and the torque gate. The full CBF-QP stays on the edge computer as the performance filter. Independence requires that the kernel's inputs are not produced by the component it supervises.

### G2 — No hazard-to-response register

- **Frontier.** The public record of Waymo's most serious failures includes two kinds. In one, the system detected a hazard and did not stop (the flood). In the others, it failed to recognise or escalate correctly (emergency scenes, school buses). NHTSA's July 2026 letter called emergency-scene failures a "functional insufficiency" and said they "are not rare or extreme 'edge cases'".
- **Specification.** The degradation ladder ([doc 02 §6](02-system-architecture.md)) maps *generic* triggers (sensor loss, uncertainty, infeasibility) to responses. Nothing lists hazard classes with their detection source, declared response, test scenario and measured worst case. E1 in [doc 07 §4](07-open-dilemmas.md) proposes exactly this, but it is not in doc 02.
- **Proposal.** A hazard-to-response register with rows for at least: wet or reflective floor, glass or mirror, stair or drop edge, overhang, cable or thin obstacle, person in a sensor blind zone, low light, occluded or failed sensor, and unknown obstacle class. Each row gets a test in the S3 suite and a measured worst case.

### G3 — No change control for learned or tuned components

- **Frontier.** Waymo's flood patch went to its whole 3,791-vehicle fleet and failed within two weeks, when another vehicle drove into standing water (doc 08 §2.3). No permanent remedy was in place as of May 2026 (NHTSA 26E026).
- **Specification.** The interface freeze ([doc 02 §8](02-system-architecture.md)) governs interfaces. Nothing requires a record when weights, parameters or code change behaviour. A retrained policy could change the robot's behaviour with no trace in the evidence chain.
- **Internal inconsistency.** The volume-policy section of [doc 03](03-simulation-and-docker.md) says every run's metadata "names the exact dataset and weight versions it used". The `run_metadata.json` schema in the same document has no field for weights or datasets. The promise is in the prose; the schema does not deliver it.
- **Proposal.** E5 from [doc 07 §4](07-open-dilemmas.md). Every change to weights, configuration or code is classified as evidence-invalidating or not. The affected tests are listed, and deployment is blocked until they pass. Add `weights_sha256` and `dataset_sha256` to the metadata schema.

### G4 — The safety filter consumes a topic nothing publishes

- **Specification.** [Doc 02 §3](02-system-architecture.md): `safety_filter` subscribes to `safety_field`. No node in the graph publishes it.
- **Cause.** Doc 02 does not say what produces `safety_field` before T3. The near-field ring that should produce it appears in [doc 00 §4](00-mission-and-scope.md) and theory 07, but in [doc 04](04-build-roadmap.md) the ring is T3 hardware (M5–M7). In T2 the only exteroceptive sensor is the 2D LiDAR, which is also the main perception source. If the field derives from it, the safety layer shares its sensors with the perception layer it supervises. Separately, the T2 bill of materials has no computer, although doc 04 says T2 "proves navigation and NMPC on hardware". The compute for NMPC before T3 is unspecified.
- **Proposal.** Define the producer of `safety_field` for each tier, and name the T2 compute platform. Decide the T2 near-field input: a **contact bumper** is cheap, needs no software or fusion, and gives the MCU kernel in G1 a genuinely independent input. A ring can follow in T3.

### G5 — "Verified" used where the evidence is measurement

- Theory 06 §4 ("verified stop path") and [doc 02 §6](02-system-architecture.md) ("verified stopping envelope") read as formal verification. The evidence is measurement and test. SF-08 requires that unverified claims be marked as such. The wording should say what was done.
- Theory 06 is corrected in this pass. Doc 02 is queued with the revision in §5.

### Known risk, not a new gap

Sensing coverage (R-06) belongs in the matrix as **Behind**, not here. A 2D LiDAR cannot see overhangs, glass or stair edges. The RGB-D camera covers some overhangs and the ring covers the near field. Waymo's water-depth limit shows that a sensing blind spot can be structural rather than a tuning problem. Our mitigation is plausible but unmeasured. It should be an S3 fault-injection measurement that reports a miss rate.

---

## 5. Proposed changes (not applied)

| Where | Change | Closes |
| --- | --- | --- |
| doc 02 §2 | Assign L2 explicitly; add the MCU safety kernel and its inputs for each tier | G1, G4 |
| doc 02 §3 | Record each L2 node's host; make the producer of `safety_field` explicit per tier | G1, G4 |
| doc 01, SF-01 | Name the function SF-01 covers; add a requirement for the MCU kernel | G1 |
| doc 02 §6 and doc 01 | Hazard-to-response register as a requirement class | G2 |
| doc 02 §8 | Change-revalidation rule for weights, parameters and code | G3 |
| doc 03 metadata schema | Add `weights_sha256` and `dataset_sha256`, matching the prose | G3 |
| doc 04 | Decide the T2 near-field input (bumper versus ring) | G4 |
| doc 02 §6 | Replace "verified stopping envelope" with "measured" | G5 |
| doc 00 §3 | Re-frame the problem around the selected dilemma (pending your decision) | doc 07 |

**Decision needed from you:** the T2 near-field input for the MCU kernel. The choice is between a contact bumper, which is cheap and needs no software, and the ultrasonic ring, which gives wider near-field coverage but sits in T3. A bumper gives an independent input earlier, at the cost of detecting only contact.

---

## 6. Where we are stronger on paper — not yet evidence

These are design properties. None is evidence until a run exists. The portfolio should describe them that way.

- An **analytic, inspectable** safety filter, not a learned one.
- **Timing targets written as testable requirements** with a measurement method (PR-05, PR-06, PR-13).
- A **reproducibility contract** that rejects dirty-tree results (doc 03).
- A **fallback order tested before the controller is tuned** (doc 02 §6).
- **Learned components with no actuation authority** (doc 02 §1).
- **Mechanical simplicity** as a documented control advantage (doc 08 §3.2).

The frontier's advantage is volume of evidence. Ours, if we earn one, is transparency of method. We should not describe these properties as advantages until they are measured.

---

## 7. Corrections made in this pass

While checking this comparison against the source documents, I found errors in earlier work. All are corrected here.

1. **Waymo's validator was described as non-learned.** Waymo's own description says the validation layer is "AI-based" and incorporates reinforcement learning. Corrected in doc 08 §2.2, §8 and theory 06 §2.1. The pattern matches; the implementation differs.
2. **A phrase was attributed to Boston Dynamics.** "Safety-critical backbone" comes from a secondary analysis of Atlas, not from Boston Dynamics. Corrected in theory 06 §2.1 and doc 08 §8.
3. **Overclaims in doc 08.** "The only system" (now: the only one among the systems reviewed), "the convergence is exact", "our choice is validated", "not perception failures", "every one of these is a failure of the specification", "nobody has published", and "all seven systems" (there are six) all went beyond what the sources support. Each is narrowed.
4. **The flood explanation was journalism's, not Waymo's.** NHTSA's filing says only that the vehicle "detected potentially untraversable flood water and proceeded at reduced speed". The "no hard-stop condition" explanation is the journalist's analysis. Attribution corrected in doc 08 §2.4.
5. **Waymo's sensor generations were mixed up.** The 13-camera, 4-lidar, 6-radar configuration is the 2024 *6th*-generation suite, not the 5th. Corrected in doc 08 §2.1 and §10.
6. **A source domain was mistyped** in doc 08 §10. Fixed.
7. **"Verified stop path"** in theory 06 §4 changed to "tested". Doc 02 is queued (G5).
8. **CHANGELOG 0.3.0** repeated the "not perception failures" claim. Superseded by 0.4.0, which records these corrections.

---

## 8. Sources and limits

- **Ours:** docs 00, 01, 02, 03, 04 and theory 06, 07 as they stand in this pass. Quoted requirement IDs and numbers were checked against the files.
- **Frontier:** the sources listed in [doc 08 §10](08-state-of-the-art-analysis.md), with the quality flags given there. The Chinese volume rows are added to that list. Atlas, Helix, ANYmal X and Chinese volume figures are mostly secondary. For the Chinese column, the primary sources are the Unitree product page, the Unitree RL repository and the GO-1 paper. The January 2026 child-strike speeds come from press reports listed in doc 08 §10 and should be verified before quoting.
- **Limits.** This is a *specification* comparison. No "Ours" cell is measured. Frontier internals are closed, so the table compares what is publicly described. It may understate what the frontier does internally. It would overstate our position if our paper design were read as performance.
