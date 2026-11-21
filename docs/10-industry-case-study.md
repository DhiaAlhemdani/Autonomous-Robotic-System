# 10 — Industry Case Study: Eleven Robotics Companies Measured Against Our Design

**Status.** Research pass completed on 2026-10-09 from public sources. No robot, simulator or measurement was run; laptop access is still unavailable. Everything here describes **published specifications** and compares them with **our specification**. No value in this document is a measurement of any robot. Tesla Optimus, Figure AI and Amazon Robotics were added in a second pass the same day, at the user's request.

**Scope.** Eleven well-known robotics companies, chosen to cover the platforms that most resemble our system (wheeled indoor robots, legged inspection, torque-controlled arms, certified cobots, commercial humanoids, and a warehouse fleet). For each company the study answers the seven questions requested: 1 capabilities, 2 limitations, 3 controllers, 4 motors, 5 drivers, 6 simulation environments, 7 comparison with our theoretical work. Waymo, Atlas, π-class VLA systems and Perseverance are analysed in [doc 08](08-state-of-the-art-analysis.md) and are not repeated here.

**Reading rules.**

- **P**: primary. Manufacturer documentation (datasheet, manual, SDK, company release), or a peer-reviewed paper or preprint.
- **S**: secondary. Retailer, aggregator, trade press, distributor or practitioner guide. Use with care.
- **K**: company claim with no specification or test document behind it. Keynotes, executive posts, demonstration videos and marketing statements fall here. We report them as the company's words, never as measurements. A release with a specification table is P even when its claims are unverified; the cell then says so.
- **N**: not published in the sources we could access. This is not evidence that the value does not exist.
- **C**: sources conflict. Both values are shown.
- **Derived**: our arithmetic from cited values, with the inputs shown.
- Source IDs in square brackets (for example [S01]) are listed with URLs in §9.
- Verdicts use the vocabulary of [doc 09](09-architecture-comparison.md): **Converged**, **Different**, **Behind**, **Gap**, **Unproven**.

## Key findings

1. **The architecture is in the mainstream.** A two-rate control hierarchy, integrated FOC inside each joint, and an independent stop path are how the best-documented companies build. Our layered design matches this pattern (§3 to §5).
2. **The case set exposes gaps in our own specification** (IG-1 to IG-9, §8). The most serious: our 100 Hz wheel-command bridge cannot support our own ≥100 Hz speed-bandwidth requirement (PR-12), and it is the slowest link in the chain.
3. **Learning placement is a choice, not a convergence.** The legged cases put RL in locomotion. The humanoid cases put learned whole-body control at actuator rate (§3.2). We put RL in behaviour (L5), above a model-based NMPC and a CBF filter. That is defensible, but it needs its own evidence.
4. **Learning simulation is GPU-parallel and includes a sim-to-sim step.** Our specification uses Gazebo for everything and has no throughput estimate or sim-to-sim plan (IG-5).
5. **Published numbers are often secondary or conflicting.** Two Digit mass figures differ by 11 kg [S46, S45], and one "8 hour" runtime is not supported by the company's own release [S43, S44]. Labelling every value as target, measured, or unpublished is the right discipline.
6. **The new humanoid cases run learned control at actuator rates.** Figure's published stack has a 7–9 Hz semantic model, a 200 Hz visuomotor policy and a 1 kHz whole-body controller [S74, S75]. Our learned layer runs at 1–5 Hz and does not actuate (doc 02 §1, §5). That difference is a design choice, and it should be recorded as one (§7.3).
7. **Evidence quality differs by company more than capability does.** Figure publishes technical releases, but its BMW post gives targets, not achieved values [S77]. Tesla's hardware figures come from 2022 statements, aggregators that conflict, and an earnings call; its official Optimus page returned 404 when we fetched it. Amazon publishes fleet-scale claims and a technical report, but no per-robot specification for Proteus or Vulcan.

---

## 1. Companies and platforms studied

| Company | Platform | Archetype | Why it is in the study | Main evidence |
| --- | --- | --- | --- | --- |
| Clearpath Robotics | Jackal | Wheeled ~17 kg skid-steer UGV with ROS | Closest to our footprint, mass, drive type and software stack | P (manual, brochure) [S01, S03] |
| MiR (Teradyne) | MiR250 | Indoor industrial AMR, 250 kg payload | Certification-oriented mobile robot; safety scanners; ISO 3691-4 wording | P (specification, user guide) [S07, S08, S09] |
| Boston Dynamics | Spot | Quadruped inspection robot, 32.5 kg | Inspection missions; best-documented production MPC-to-RL transition; topological maps | P (SDK, company blog) [S15 to S18] |
| ANYbotics | ANYmal and ANYmal X; ANYdrive actuator | Quadruped inspection; ATEX Zone 1 variant | Actuator-level detail in peer-reviewed papers; hazardous-area certification | P (papers, datasheet); S (ANYmal X) [S24 to S29] |
| Unitree | Go2; GO-M8010-6 joint motor; open RL pipeline | Low-cost quadruped and humanoid; volume leader | Integrated FOC joint motor; complete open RL pipeline | S (specifications); P (repository summary) [S35, S37, S39] |
| Agility Robotics | Digit | Bipedal logistics humanoid, commercially deployed | Limits of a deployed humanoid; safety architecture | S, with conflicts [S43 to S46] |
| Franka Robotics | FR3 (Franka Research 3) | 7-DoF torque-controlled arm; 1 kHz interface | Joint torque control; research standard | P (datasheet); S (aggregators) [S47, S48] |
| Universal Robots | UR10e | 6-DoF collaborative arm | Certified safety functions; joint servo architecture | P (datasheet, manual); P (interview) [S51 to S53] |
| Tesla | Optimus: Gen 3 hands reported in factory use; full V3 body not publicly shown as of late September 2026 [S61, S66] | Humanoid with in-house actuators and end-to-end learned control | Largest stated ambition and the most public claims; the least primary specification | P: earnings-call transcript [S59] and an executive article [S65]; otherwise K and S: 2022 statements reported by press [S64] and aggregators [S61 to S63, S71 to S73]; tesla.com/optimus returned 404 when fetched |
| Figure AI | Figure 03, with Helix (VLA) and Helix 02 (whole-body control) | Humanoid for manufacturing and the home; learned control at 200 Hz and 1 kHz | Most detailed published control architecture of the three; longest-documented industrial pilot we found (BMW, 11 months) | P for architecture and pilot [S74 to S77]; S for specifications and litigation [S78 to S84] |
| Amazon Robotics | Proteus AMR; Vulcan picking arm (about 75% of stock by item type, Amazon's claim [S91]); DeepFleet fleet model | Warehouse AMR moving carts among people; tactile picking arm; fleet-level learning | Amazon's own claim: the largest operated mobile-robot fleet, over one million robots [S85]; safety in shared space | P for announcements and the technical report [S85, S86, S93]; S for product specifications and injury studies [S87 to S92, S94 to S97] |

Waymo, Atlas, π-class VLA systems and Perseverance are cross-referenced to [doc 08](08-state-of-the-art-analysis.md) and are not re-profiled here. Figure's Helix is profiled above from Figure's own releases.

---

## 2. Capabilities and limitations

### 2.1 Capabilities

Our values are requirement targets (labelled T in [doc 01](01-requirements.md)). Industry values are as published.

| Platform | Mass | Payload | Max speed | Runtime | Ingress | Stated safety or certification | Tags |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **Ours (spec)** | ~20 kg [doc 00 §4]; ≤25 kg (BC-01) | Not specified (IG-2) | ≤1.0 m/s autonomous; 1.5 m/s bench (PR-01) | ≥90 min (PR-10); battery ≥300 Wh, 24 V (BC-06) | Not specified (IG-6) | MCU gate and physical E-stop (SF-03); stop ≤100 ms target, measured (PR-13); no certification claimed | spec |
| Clearpath Jackal | 17 kg [S04] | 20 kg; 10 kg all-terrain [S01, S04] | 2.0 m/s [S01] | 8 h basic, 2 h heavy, on 270 Wh [S01] | IP62 [S01]; −19 to 45 °C [S01] | None found | P |
| MiR250 | 97 kg [S12] | 250 kg [S07, S08] | 2.0 m/s with full payload on flat floor [S07, S08] | Up to 17 h 30 min unloaded, 13 h loaded; charging ratio 1:16 [S07] | Indoor base (IP21 per review) [S11] | "Designed to meet" ISO 3691-4 (listed exceptions), ISO 13849-1, ISO 13850, ISO 12100, B56.5, R15.08-1 [S07] | P, S |
| Boston Dynamics Spot | 32.5 kg [S15] | 14 kg; two payload ports at 150 W each [S15] | 1.6 m/s [S15] | ~90 min typical on 605 Wh; 120 min to charge [S15] | IP54 [S22] | None found | P, S |
| ANYmal X | ~50 kg [S32] | 10 kg nominal; up to 15 kg reduced performance [S31] | ~1 m/s [S33] | 90–120 min [S31]; about 2 h [S33] and 2–4 h [S58] in other reviews [C] | IP67 [S31] | ATEX/IECEx Zone 1 IIB [S31, S34] | S |
| Unitree Go2 (EDU) | ~15 kg [S39] | 8 kg average, 12 kg maximum [S39] | ~3.7 m/s typical, 5 m/s maximum [S39] | 1–2 h (8 Ah) or 2–4 h (15 Ah EDU) [S42, S39] | N | None found | S |
| Agility Digit | ~65 kg [S46] **C** 76 kg [S45] | 16 kg (35 lb) [S43] | ~1.4–1.5 m/s [C: S45, S46] | 4 h with autonomous docking (company release) [S43, S44]; "8 h" in aggregators, unsupported | N | CAT1 stop, Safety PLC at PL d, FSoE, E-stops (company release, reported by press) [S43]; site-level NRTL inspection, not a product certificate [S43] | P, S, C |
| Franka FR3 | ~18 kg arm [S49] | 3 kg [S47] | 2 m/s at the TCP [S48] | Mains powered; ~80 W typical [S47] | Arm IP40; control IP20 [S48, S47] | Collision detection under 2 ms [S47]; TÜV and ISO claims are distributor statements, not checked [S50] | P, S |
| Universal Robots UR10e | 33.3 kg arm [S52] | 12.5 kg; 10 kg in some configurations [S52] | Set by the safety configuration [S55] | Mains powered; 615 W average maximum, ~350 W typical [S52] | IP54 [S52] | 17 safety functions; PL d, Category 3 (EN ISO 13849-1); EN ISO 10218-1 [S51, S52] | P |
| Tesla Optimus (as reported; Gen 3 hands) | 57 kg; 173 cm [S64 K] | 20 kg carry [S64 K] | About 5 mph maximum, about 2.2 m/s (derived) [S64 K]; ~1.6 m/s [S71 S] **C**; ~8 km/h [S73 S] | 8–12 h for light-to-moderate tasks [S71 S] **C** with about 4 h for V2.5 [S72 S]; 2.3 kWh battery [S63 S] | N | None found. Tesla said in 2022 that people could outrun or overpower the robot [S64 K] | K, S, C |
| Figure 03 | 61 kg; 173 cm [S80 S, not on Figure's launch page as fetched]; 9% less mass than Figure 02 [S76 P] | 20 kg [S80 S] | 1.2 m/s [S80 S]; 4.3 km/h [S83 S] | About 5 h [S80 S]; 2 kW wireless charging through the feet [S76 P] | N | UN38.3 achieved for the battery [S76 P]; UL2271 described as in process [S82 S]; no full-machine certification found [S81 S] | P, S |
| Amazon Proteus | Not published by Amazon (N) | Not published by Amazon; 360 kg [S89 S] and 400 kg [S90 S] **C** | 1.5 m/s [S89 S] | 12 h [S89 S], uncorroborated | N | Amazon says Proteus moves safely around employees [S85 P]; sensing with stereo vision and planar lidar [S87 S]; a catalogue lists an R15.08 compliance claim [S88 S]; no certificate found | P, S, C |
| Amazon Vulcan | N | About 3.6 kg per item [S92 S, uncorroborated] | Human-comparable speed claimed; no figure [S91 S] | N | N | Force feedback to avoid damaging goods [S91 S] | S |

**Derived values** (our arithmetic, inputs shown):

- Jackal heavy use: 270 Wh over 2 h, about **135 W** average [S01].
- Spot typical: 605 Wh over 90 min, about **400 W** average [S15].
- Our requirements: 300 Wh over 90 min, at most about **200 W** average for drive, compute and sensors combined, before losses. This is the basis for IG-3.
- Our ramp requirement: BC-04 allows ramps up to 5° [doc 01]. A 5° grade at 20 kg needs about **17 N** of drive force along the slope (20 × 9.81 × sin 5°).
- MiR250 published incline: ±5% at 0.5 m/s [S08]. A 5% grade is about **2.9°**. Our 5° requirement is therefore steeper than the MiR250's published limit. This is IG-7.
- Tesla's 2022 maximum of 5 mph is about **2.2 m/s** (derived: 5 × 0.447). Aggregator values of ~8 km/h (2.2 m/s) and ~1.6 m/s (5.8 km/h) differ by about 0.6 m/s [S71, S73].
- Kinetic energy of a whole moving body, for scale only: ours at 20 kg and 1.0 m/s is about **10 J**; Figure 03 at 61 kg and 1.2 m/s is about **44 J** (0.5 × m × v², derived from the cited values). This is not contact energy, and neither company publishes an impact-energy limit (N). It is the basis for IG-8.

### 2.2 Limitations by platform

- **Jackal.** Two hours under heavy use [S01]. The manual's safety section gives operating warnings; we found no safety-rated function described [S01]. The legacy manual describes a ROS 1 rosserial link between the MCU and the computer [S02].
- **MiR250.** Floor friction of 0.6–0.8 is recommended; steps above 20 mm only at reduced speed; steps above 30 mm not recommended; above 50 mm prohibited [S08]. Acceleration limit 0.3 m/s² at full payload, and 9.5 m to reach top speed [S08]. The standards are stated as "designed to meet", which is a design statement, not a certificate [S07].
- **Spot.** Ninety-minute typical runtime [S15]. Localisation is relative to waypoints. Scene changes or feature-poor areas degrade localisation; the SDK documents "lost" events that need operator action [S16]. A press report, attributed to Boston Dynamics' CTO, puts the fall rate at about one per 50 km [S19]. The company's own RL announcement says its older production controller consumed significant onboard compute and energy [S18].
- **ANYmal X.** Standard ANYmal runtime 90–120 min [S31], against about 2 h [S33] and 2–4 h [S58] in other reviews [C]. A 2026 review reports that the vendor's product page says updated 2026 specifications are "coming soon" [S31], so the current datasheet was not available to us.
- **Go2.** Runtime 1–4 h depending on battery [S39, S42]; the Go2 raised battery capacity over the Go1 [S40]. Payload 8–12 kg [S39]. No certification found.
- **Digit.** Payload 16 kg and runtime 4 h are the company's published current values [S43]. Fence-free "cooperative safety" belongs to the announced next generation, not the shipping robot [S45]. Launch timing for that generation conflicts between sources: late 2026 [S43] versus early access in 2027 [S45].
- **FR3.** Payload 3 kg and reach 855 mm [S47]. The Cartesian force controller has a published −3 dB bandwidth of 10 Hz [S47]. That is a limit for contact tasks, not a joint-level limit.
- **UR10e.** Stopping distance and time depend on payload and joint, and are tabulated in the manual [S52]. The safety configuration can reduce speeds [S55].
- **Tesla Optimus.** Public targets have slipped. Musk's January 2025 aim of about 10,000 robots that year was not met, and he said in January 2026 that none were doing useful work in Tesla's factories [S60 S]. On the Q1 2026 call he said Fremont production would start in late July or August 2026, that output would be "quite slow", and that he could not predict the 2026 rate; the robot has 10,000 unique parts [S59 P]. The V3 reveal, first set for Q1 2026 and then for mid-2026, had not happened by late September 2026 [S63 S, S66 S]. Musk has also said on X that the hand design in a 2026 patent "didn't actually work" and has been changed [S69 S, K]. No safety or certification evidence has been published (N).
- **Figure 03 and Helix.** Figure's release describes a four-minute, fully autonomous dishwasher task with no resets and no human intervention; that is a demonstration claim (K) [S75 P]. A former principal robotic safety engineer filed a wrongful-termination suit on 21 November 2025. He alleges he was fired after warning that the robots were "powerful enough to fracture a human skull". The complaint cites impact tests on Figure 02 with forces about twice those needed to fracture an adult skull, and a ¼-inch gash in a steel refrigerator door [S78 S]. Figure says he was fired for poor performance and calls the claims falsehoods. It countersued in January 2026, and the case was pending as of March 2026 [S79 S]. These are allegations, not findings. On BMW, Figure's post gives targets for cycle time (84 s, of which 37 s is loading), placement accuracy (more than 99% per shift) and interventions (zero per shift). It reports 1,250+ runtime hours and 90,000+ parts, but no achieved values for the targets [S77 P]. Trade coverage reports the targets as met [S84 S]. The forearm, whose microcontroller board relayed communications to the wrist actuators, was the top hardware failure point [S77 P]. No price or sale date for Figure 03 was found (N) [S80 S].
- **Amazon Robotics.** Injury evidence is contested. A 2020 investigation by Reveal, using internal records from 150 warehouses, reported serious-injury rates about 50% higher in the most common centre type when robots were present. Amazon said its figures are high because it records minor incidents, and it denies misleading anyone [S95 S]. A 2024 Senate investigation reported injury rates nearly twice the industry average; Amazon rejected its findings [S96 S]. A 2025 academic study found robotic fulfilment centres had about 40% fewer severe injuries and about 77% more non-severe injuries than traditional ones [S97 S]. One trade site reports Vulcan's per-item weight limit (about 3.6 kg) and difficulty with round items [S92 S, uncorroborated]. The 10% travel-time gain for DeepFleet is Amazon's announcement [S85 P]; the technical report evaluates prediction accuracy, not travel time [S86 P].

---

## 3. Controllers

### 3.1 Controller table

| Platform | Decision and planning layer | Tracking and low-level layer | Learning in the loop | Tags |
| --- | --- | --- | --- | --- |
| **Ours (spec)** | L4 planner 0.5–2 Hz; L5 policy 1–5 Hz; L3 NMPC 20–50 Hz with acados RTI, p99 ≤45 ms target [doc 02 §1, PR-05] | L2 CBF-QP filter 100–200 Hz; L1 FOC 10–20 kHz [doc 02 §1] | Yes, at L5 only, bounded: T2 hierarchical baseline; residual RL as an advanced experiment [theory 06 §2] | spec |
| Clearpath Jackal | ROS 2 control API [S05]; navigation runs on the onboard computer (stack not specified) | MCU does motor control and kinematic conversion [S02]; control rates not published (N) | No | P, N |
| MiR250 | Proprietary map-based navigation; dynamic obstacle avoidance; automatic deceleration for detected objects [S09] | Not published (N) | Not stated (N) | P, N |
| Spot | Production controller scores many MPC candidates in parallel and selects the best; dozens of candidate horizons in under a millisecond; "hundreds of hours" between falls [S18] | Locomotion control with MPC, with an RL policy integrated [S18]; GraphNav for routes [S16, S17] | Yes: RL integrated into locomotion, trained with over a million simulations [S18] | P |
| ANYmal (ANYdrive) | Learned locomotion policies; teacher and student training with privileged information for the teacher [S30] | Learned motion controller at 8 Hz, with an actuation tracker, on ANYmal C [S29]; joint cascade of torque and position PID with FOC inside the actuator; 4 kHz loop [S26] | Yes: RL policies trained in RaiSim; a 100 Hz policy in a payload study [S28] | P |
| Unitree A1 (research, simulated) and Go2 | RL policy at 50 Hz with 20 ms latency; model-based joint controller at 1 kHz [S41] | Joint-level control in the joint module [S37] | Yes: PPO through rsl_rl in Isaac Gym; sim-to-sim in MuJoCo; deployment through unitree_sdk2 [S35, S36] | P, S |
| Agility Digit | Agility's in-house control and planning stack (N) | N | N | N |
| Franka FR3 | User control at 1 kHz through the Franka Control Interface, with joint torque, position or velocity commands [S47] | Impedance parameters: translational 0–3,000 N/m; rotational 0–300 N·m/rad [S47] | Not built in; research use through libfranka [S49] | P |
| UR10e | PolyScope programs; external control through RTDE at up to 500 Hz [S56] | 500 Hz system update [S51]; per-joint servo units [S53] | Not built in (S) | P, S |
| Tesla Optimus | End-to-end neural network from camera video and other inputs to actuator commands, the same approach as its driving stack [S65 P, executive statement]; architecture details not published (N) | Not published (N); body actuator types described by analysts [S62 S] | Imitation learning from human video and demonstrations [S70 S]; reinforcement learning in simulation planned [S70 S]; closed-loop RL at scale claimed [S65 K] | K, S, N |
| Figure 03 (Helix 02) | S2: 7B vision-language model at 7–9 Hz for scene and language [S74 P]; S1: 80M-parameter visuomotor transformer at 200 Hz for full upper-body targets (2025) [S74 P]; in Helix 02, S1 outputs joint targets for the whole body at 200 Hz [S75 P] | S0: 10M-parameter learned whole-body controller at 1 kHz, outputting joint-level actuator commands [S75 P] | Yes, end-to-end from pixels to joint targets; S0 trained in simulation with sim-to-real RL [S75 P] | P |
| Amazon Robotics (DeepFleet, Proteus, Vulcan) | DeepFleet: four fleet-level foundation-model architectures (robot-centric, robot-floor, image-floor, graph-floor) trained on fleet movement data [S86 P]; Proteus: safety-aware path planning, with speed set by payload and proximity [S88 S] | Not published (N) | Yes: learned fleet coordination [S86 P]; learned grasping for Vulcan from force and touch data [S91 S] | P, S, N |

### 3.2 What the controllers show

1. **Two-rate hierarchy is standard.** The learning or planning layer runs slowly (1–50 Hz across the cases, and 1–5 Hz for our L5). A model-based or joint-level layer runs at 400 Hz to 4 kHz beneath it (ANYmal, the A1 research framework, Unitree, UR, Franka). Our L1 at 10–20 kHz and L2 at 100–200 Hz sit inside the same pattern [doc 02 §1].
2. **Low-rate learning has a demonstrated precedent.** The ANYmal C result with an 8 Hz learned motion controller and an actuation tracker [S29] is the closest published evidence to a learned layer over a fast tracker. It is for locomotion, not mission-level behaviour. It supports our layering but does not validate our policy.
3. **Learning placement differs.** Industry puts RL at the locomotion layer, where the signal is fast and the sim-to-real gap is well studied. We put it at behaviour (L5), above a model-based NMPC and a CBF filter [theory 05 §1, theory 06 §2]. This is a deliberate choice, and our evidence for it is still to be produced.
4. **Model predictive control is in production at Boston Dynamics.** This is a company-reported technique [S18]. Solver budgets are problem-specific. Spot's sub-millisecond scoring of MPC candidates [S18] is not comparable to our NMPC budget (PR-05).
5. **Figure's learned stack reaches actuator rates; ours does not.** Figure's 2026 release says a single network maps all sensors to all actuators, with a 200 Hz policy and a 1 kHz whole-body controller under it [S75 P]. Our L5 policy runs at 1–5 Hz and cannot actuate; the CBF filter and NMPC sit between it and the motors [doc 02 §5]. This is a deliberate placement decision. Ours keeps learning out of the actuation path, which makes the safety argument easier to state. It does not show that the approach performs as well (Unproven).
6. **None of the humanoid cases describes an independent, non-learned veto.** Figure says its whole-body controller ensures motion is executed "smoothly, safely, and stably" [S75 P], but it does not describe a separate safety function. Tesla describes its control stack only in general terms [S65 P]. For Figure and Tesla this is not published (N), which is not evidence of absence. Our CBF veto (theory 06 §3) is therefore a design choice to defend, not a copy of industry practice.
7. **Fleet-scale learning is an operations layer, not a robot controller.** DeepFleet coordinates traffic across a fleet of more than a million robots [S85 P]. Its technical report evaluates prediction accuracy [S86 P]. For our single-robot scope it is out of scope (doc 00 §5). It supports one point: a learned component that changes the behaviour of many robots needs its own change control (doc 09 G3).

---

## 4. Motors

| Platform | Actuator | Peak torque and speed | Gearing | Feedback | Tags |
| --- | --- | --- | --- | --- | --- |
| **Ours (spec)** | BLDC or PMSM hub motors in T2 [doc 04]; motor class (surface or interior magnet) open [theory 02 §8, question 1] | Not specified in our docs | Hub-integrated or with reduction; not fixed | Encoders and IMU [doc 04, T2] | spec |
| Clearpath Jackal | Four driven wheels; 500 W total drive power [S03, S04] | N | N | Quadrature encoders, 78,000 pulses per metre [S03] | P, N |
| MiR250 | Drive motors not described in the sources accessed (N) | N | N | Not stated (N) | P, N |
| Spot | 12 actuators: two per hip and one per knee [S15] | N | N | Joint-level API through the RL kit [S20] | P, N |
| ANYdrive (ANYmal) | Brushless motor, harmonic-drive gear and rotational spring in series (series elastic) [S24, S26] | 40 Nm peak, 15 Nm nominal; 12 rad/s peak; 48 V [S24] | Harmonic drive [S26] | 17–18-bit absolute encoders, under 0.025° [S24, S25]; torque from spring deflection [S26] | P |
| Unitree GO-M8010-6 (joint motor used on Go1) | Brushless PMSM with integrated FOC, temperature sensor and absolute encoder [S37, S38] | 23.7 Nm; 30 rad/s at 24 V; 40 A maximum [S37] | 1:6.33 [S37] | 15-bit encoder [S37] | S |
| Unitree Go2 | 12 joint motors [S39] | Up to 45 Nm joint torque [S39] | N | N | S |
| Agility Digit | Electric; type not disclosed (N) | N | N | N | N |
| Franka FR3 | Brushless DC motors with strain-wave gears and cross-roller bearings [S47] | ±87 Nm joints 1 to 4; ±12 Nm joints 5 to 7 [S48] | Strain-wave gears [S47] | Link-side torque sensor in all seven joints; high-resolution position sensors [S47] | P, S |
| UR10e | Permanent-magnet AC synchronous motor per joint, co-designed with the motor maker; harmonic drive [S53, S57] | Not in the datasheet (N) | Harmonic drive, described as zero-backlash [S57] | Dual encoders per joint [S57] | P, S |
| Tesla Optimus (as reported) | Body: 14 rotary and 14 linear actuators; rotary units use frameless torque motors with harmonic drives, linear units use roller screws [S62 S]. Hands: 22 DoF each, 25 actuators per hand assembly [S71 S], 50 across both hands [S61 S]; a 2026 patent summary places the actuators in the forearm and drives the fingers by cable [S68 S] | N (no per-joint torque or speed published; the 68 kg deadlift is a whole-robot claim [S64 K]) | Planetary gearboxes in the hand [S62 S]; harmonic drives and roller screws in the body [S62 S] | N | S, K, N |
| Figure 03 | Frameless brushless DC actuators [S82 S]; actuators designed in-house [S76 P]; Figure says they run at 2× speed with improved torque density, without a baseline or absolute values [S76 P] | N (absolute values not published) | N | N | P, S, N |
| Amazon Proteus | Electric drives for the wheels and the lift mechanism [S88 S] | N | N | N | S, N |
| Amazon Vulcan | N | N | N | Force-feedback sensors in the end-of-arm tool [S91 S] | S, N |

### 4.1 What the motors show

- **Joint motors are integrated.** ANYdrive, the Unitree joint motor, UR and Franka all combine motor, gear, encoder and power electronics in one unit [S24, S37, S53, S47]. Integrated FOC is the norm for joints.
- **Absolute encoders are standard in joints.** ANYdrive, UR and the Unitree motor all use absolute position sensing [S24, S57, S37]. Our encoder type is not yet specified (§7.4).
- **Series elasticity is a choice for legged robots.** ANYdrive's published torque bandwidth is about 70 Hz at small amplitudes, falling to 24 Hz at 10 Nm [S27]. A wheeled hub-motor robot does not need series elasticity, but the amplitude dependence of bandwidth is a limitation to measure on our own motors.
- **Wheel motors are rarely published.** Jackal and MiR publish power and wheel data [S04, S07], but motor types are not published in the sources we accessed (N). We should expect to measure our wheel motors ourselves (theory 02, parameter identification in theory 01 §5).
- **Humanoid hands hold the actuator count, and that is the least-published part.** Tesla's hand is described as 22 DoF with 50 actuators across two hands [S61 S], and as a cable-driven design in a patent [S68 S] that its CEO says was changed [S69 S, K]. Figure publishes only relative actuator gains [S76 P]. We have no hand in scope, so the lesson is narrow: any manipulator we add concentrates actuator count, sensing and bandwidth in the hand, and that should be specified before a purchase.

---

## 5. Drivers and buses

| Platform | Where servo and FOC run | Bus or link, and rate | Safety-relevant function | Tags |
| --- | --- | --- | --- | --- |
| **Ours (spec)** | MCU (STM32-class) runs FOC at 10–20 kHz and drives the inverter [doc 02 §1, §2] | Serial or CAN bridge; wheel command at 100 Hz; odometry at 50–100 Hz [doc 02 §3] | MCU torque-enable line, heartbeat and watchdog; physical E-stop (SF-03) [theory 08] | spec |
| Clearpath Jackal | MCU does motor control; open-loop voltage and wheel-velocity modes [S02, S03] | USB CDC carrying rosserial, in the legacy manual [S02] | MCU monitors power and motor current [S01] | P |
| ANYdrive | Motor electronics inside the joint; torque, position and impedance regulated locally [S24, S26] | CANopen at 1 Mbit/s in the first generation; EtherCAT in the second [S24, S26, S25] | Not stated (N) | P |
| Unitree joint motor | Integrated FOC driver in the joint [S37] | RS-485 at 4 Mbit/s; 6 kHz communication frequency [S37, S38] | Not stated (N) | S |
| Franka FR3 | Joint control in the robot controller; FCI at 1 kHz [S47, S49] | FCI at 1 kHz with joint state, torque and external-force estimates [S47, S48] | Collision detection under 2 ms [S47] | P |
| UR10e | Per-joint servo controllers; safety calculations cross-checked every 8 ms [S53] | 500 Hz system update [S51]; Modbus TCP, PROFINET, EtherNet/IP [S51, S52] | 17 safety functions; PL d, Category 3 [S51] | P |
| Agility Digit | Not disclosed (N) | Not disclosed (N) | CAT1 stop; Safety PLC at PL d; FSoE; on-robot and wireless E-stops [S43] | P, S |
| Spot | Not disclosed (N) | Not disclosed (N) | Not disclosed (N) | N |
| Tesla Optimus | AI inference chip claimed ('AI5') [S61 S, K]; actuator drivers and buses not published (N) | N | N | K, S, N |
| Figure 03 | Embedded low-power GPUs run Helix on the robot [S74 P]; Helix 02 S0 runs at 1 kHz on the robot [S75 P]; on Figure 02, a forearm board relayed commands between the main computer and the wrist actuators [S77 P] | Not named (N); 2 kW inductive charging through the feet [S76 P]; 10 Gbps mmWave for data offload [S76 P] | Battery protection at the BMS, cell, interconnect and pack levels [S76 P]; UN38.3 achieved [S76 P] | P, N |
| Amazon Proteus | Onboard computer and control boards [S88 S] | N | Safety-aware planning, with speed limited by payload and proximity [S88 S] | S, N |
| Amazon Vulcan | Two-arm cell with force-feedback tooling; arms reportedly from Universal Robots [S92 S] | N | Force feedback to avoid crushing goods [S91 S] | S, N |

### 5.1 What the drivers show

- **Our wheel-command link is the slowest link in the chain.** The serial bridge runs at 100 Hz [doc 02 §3]. The industrial and research buses in the cases run at 500 Hz or faster: UR 500 Hz [S51], Franka 1 kHz [S47], ANYdrive's internal loop at 4 kHz [S26], and Unitree's communication at 6 kHz [S37]. Theory 01 §3.3 already calls this link "the real problem" (a 100 Hz frame plus a 50 Hz control period adds about 30 to 40 ms of pure delay). The case set confirms that it is an outlier. See IG-1.
- **In the cases, the stop path is a safety function, not just a wire.** UR runs its safety functions in a safety system that cross-checks joint calculations every 8 ms [S53]. Agility states PL d through a safety PLC and FSoE [S43]. Ours is an MCU torque-enable line, which is a sound independent path but is not specified as a safety function (IG-4). Theory 08 §8, question 1 already asks whether a redundant channel is required; PL d, as stated by UR and Agility, implies a redundant or diagnosed architecture under ISO 13849-1 (Category 3 or 4).
- **Interfaces are documented in every industrial case.** Franka, UR and ANYdrive publish their interfaces, rates and limits [S47, S51, S24]. Our frame format and CRC are to be frozen before implementation [doc 02 §8]. That instinct is right, but the rate is not yet right (IG-1).
- **A distributed link was a humanoid's top hardware failure point.** Figure reports that the forearm, whose microcontroller board relayed commands to the wrist actuators, was its top hardware failure point at BMW [S77 P]. Doc 02 §2 already specifies a versioned protocol with sequence numbers, CRC and a heartbeat, and SF-02 names comms among the faults to inject [doc 01 SF-02]. What is missing is a written test plan for link faults: corrupted, lost and late frames. That is a test gap, not a design gap, and it belongs in the doc 03 §5 CI pipeline.
- **No new case publishes a stop chain.** Tesla, Figure and Amazon describe no safety-rated stop architecture (N). The stop-chain gap (IG-4) therefore stands as the cases with published chains show it, and the three new cases are evidence neither way.

---

## 6. Simulation environments

| Platform | Simulators | Role | Validation evidence | Tags |
| --- | --- | --- | --- | --- |
| **Ours (spec)** | Gazebo (gz-sim LTS) with the ros_gz bridge, in Docker; headless CI [doc 03 §1, §5] | System tests and scenario suite; RL training in Gazebo, "effectively" [theory 05 §2.1] | Domain randomisation planned [theory 05 §4.2]; sim-to-real discussed [theory 05 §6]; no sim-to-sim step | spec |
| Clearpath Jackal | Official ROS 1 Gazebo package, jackal_gazebo [S06]; "simulator integration" in the manual [S01] | System and navigation tests | N | P, S |
| MiR250 | No official simulator found (N); community Gazebo packages for ROS 1 [S14] and ROS 2 Humble [S13] | Navigation tests | N | N, S |
| Spot | Isaac Lab Spot environment with actuator models, released with the RL Researcher Kit [S20, S21] | RL training; sim-to-real | Company reports over a million simulations to train the policy, and more than 2,000 fleet hours a week of robustness testing [S18] | P, S |
| ANYmal | RaiSim (ETH) [S28] | RL training | Payload study on hardware [S28] | P |
| Unitree | Isaac Gym for training, rsl_rl PPO, MuJoCo for sim-to-sim, deployment through unitree_sdk2 [S35, S36] | Train, cross-validate, deploy | An explicit sim-to-sim step in the official pipeline [S35] | S |
| Agility Digit | Not disclosed (N) | N | N | N |
| Franka FR3 | Gazebo resources, MATLAB/Simulink and libfranka [S49, S48] | Research simulation and control | N | P, S |
| UR10e | URSim, the official offline simulator [S54]. No physics for force, no safety board, no servo-loop model [S55] | Program development and offline checks | Scope limits documented [S55] | P, S |
| Tesla Optimus | Neural world simulator: a learned model that generates sensor streams in response to actions, shared with the driving stack [S65 P]; 36 Hz rendering reported [S67 S] | Closed-loop evaluation; Tesla says it supports large-scale closed-loop RL [S65 K] | Not a physics model; no contact or sim-to-real metric published (N) | K, S, N |
| Figure 03 (Helix) | S1 vision backbone initialised from pretraining done entirely in simulation (2025) [S74 P]; S0 trained entirely in simulation across more than 200,000 parallel environments, with domain randomisation and sim-to-real RL (2026) [S75 P] | Policy pretraining; whole-body control training; transfer to hardware | Company says the policies transfer; no sim-to-real error reported (N) | P, N |
| Amazon Robotics | NVIDIA Omniverse and Isaac Sim digital twins of warehouses [S93 P, vendor]; a digital twin of Proteus with sensor emulation [S93 P; S94 S]; 'hundreds of photo-realistic environments' to test sensor processing and AMR behaviour [S94 S] | Design, sensor emulation and behaviour testing | No sim-to-real error reported (N) | P, S, N |

### 6.1 What the simulators show

- **Learning-scale simulation is GPU-parallel.** Isaac Gym or Isaac Lab, RaiSim and MuJoCo appear across the learned-locomotion cases [S20, S28, S35]. A third-party Spot project ran 8,192 parallel environments in Isaac Lab [S23]. Our docs have no throughput estimate for any training simulator.
- **Sim-to-sim validation is a named step.** The Unitree pipeline trains in Isaac Gym, validates in MuJoCo, then deploys [S35]. That is a cheap guard against overfitting one simulator. Our docs do not have it (IG-5).
- **Gazebo is adequate for system tests.** The Jackal and MiR ecosystems use Gazebo for navigation tests [S06, S14], as our doc 03 does. The gap is not the choice of Gazebo. It is that our docs also assign Gazebo the RL training role [theory 05 §2.1] without a throughput estimate.
- **A simulator that cannot raise a safety event cannot test the safety branch.** URSim does not implement the safety board or protective stops [S55]. Our CI asserts SF-02, SF-05 and SF-06 [doc 03 §5]. We need to confirm that each of these assertions can actually trigger a stop in simulation.
- **Learned simulators are a third category.** Tesla's world simulator generates sensor streams from actions [S65 P]. It is not a physics engine, and no validation against real contact is published (N). Our doc 03 uses a physics simulator for system tests. A learned simulator used to judge a safety property would first need validation against logged real rollouts (BL-6).
- **Figure gives the most specific simulation scale in this set.** Its whole-body controller trains across more than 200,000 parallel environments [S75 P]. The figure is company-reported, but it is a concrete planning number that our Gazebo-only plan (IG-5) does not have.
- **Amazon's simulation is a design and test tool.** Its digital twins serve warehouse layout, sensor emulation and behaviour testing [S93 P; S94 S]. No transfer error is published (N).

---

## 7. Comparison with our theoretical work

Each subsection states what the cases show, what our documents specify, a verdict, and the action. Actions are backlog items. Nothing has been applied to docs 00 to 02.

### 7.1 Capabilities

| Metric | Ours (spec) | Industry cases | Verdict |
| --- | --- | --- | --- |
| Mass class | ~20 kg; ≤25 kg (BC-01) | Jackal 17 kg [S04]; Go2 ~15 kg [S39]; FR3 arm ~18 kg [S49]; Spot 32.5 kg [S15]; ANYmal ~50 kg [S32]; MiR250 97 kg [S12] | **Converged** in the Jackal class |
| Speed | ≤1.0 m/s autonomous (PR-01) | Jackal 2.0 m/s [S01]; MiR250 2.0 m/s [S07]; Spot 1.6 m/s [S15]; Digit ~1.4–1.5 m/s [C: S45, S46] | **Different** by scope: facility, humans present (doc 00 §5) |
| Acceleration bounds | Bounds exist in theory (theory 01 §2.3 and §5; theory 04 state constraints) but are not in the requirements | MiR250: 0.3 m/s² at full payload; 9.5 m to top speed [S08] | **Unproven**: move the bound from theory into a requirement |
| Runtime | ≥90 min (PR-10) | Jackal 2 to 8 h [S01]; MiR250 13 to 17.5 h [S07]; Spot ~90 min [S15]; ANYmal X 90 to 120 min [S31]; Go2 1 to 4 h [S39, S42]; Digit 4 h [S43] | **Behind** the wheeled references; **Unproven** until an energy model exists (IG-3) |
| Payload | None in doc 01 | Jackal 20 kg [S01]; MiR250 250 kg [S07]; Spot 14 kg [S15]; ANYmal 10 kg [S31]; Go2 8 to 12 kg [S39]; Digit 16 kg [S43]; UR10e 12.5 kg [S52] | **Gap**: no payload requirement (IG-2) |
| Ramp and slope | Ramps up to 5° (BC-04) | MiR250 ±5% at 0.5 m/s, about 2.9° (derived) [S08] | **Gap**: the requirement is steeper than a published wheeled reference (IG-7) |
| Floor friction | Smooth concrete, vinyl or tile (BC-04) | MiR250 recommends 0.6 to 0.8 [S08] | **Unproven**: traction tests are planned (theory 01 §5) but not yet specified as a pass criterion |
| Ingress and environment | Indoor; dust and smoke in scenario S3 [doc 00 §2] | Jackal IP62 [S01]; Spot IP54 [S22]; ANYmal IP67 [S31]; UR10e IP54 [S52] | **Unproven**: no ingress target (IG-6) |
| Safety wording | "No certification claimed" (doc 00 §5) | MiR250 "designed to meet" [S07]; ANYmal X certified [S31, S34]; UR10e PL d, Category 3 [S51] | **Converged** in the honesty of our wording; **Behind** in certification (budget and scope) |
| Mass and form | ~20 kg base (doc 00 §4) | Optimus 57 kg, 173 cm [S64 K]; Figure 03 61 kg, from a secondary card [S80 S]; Proteus mass not published (N) | **Different** by archetype: a humanoid and a cart mover are outside our class |
| Top speed | ≤1.0 m/s autonomous (PR-01) | Optimus about 2.2 m/s maximum, Tesla 2022 [S64 K]; Figure 03 1.2 m/s [S80 S]; Proteus 1.5 m/s [S89 S] | **Different** by scope; our cap is set for a 20 kg base in a shared facility (doc 00 §5) |
| Payload | None in doc 01 | Optimus 20 kg carry [S64 K]; Figure 03 20 kg [S80 S]; Vulcan about 3.6 kg per item [S92 S]; Proteus 360 kg or 400 kg (C) [S89, S90] | **Gap** remains (IG-2); humanoid and cart payloads are not comparable to a 20 kg base |
| Runtime | ≥90 min (PR-10) | Optimus 8–12 h claimed [S71 S], **C** with about 4 h for V2.5 [S72 S]; Figure 03 about 5 h [S80 S]; Proteus 12 h, uncorroborated [S89 S] | **Different** in mass and energy class; **Unproven** for us until an energy model exists (IG-3) |
| Battery safety | BC-06: 24 V, ≥300 Wh; no safety or transport requirement | Figure 03: layered battery protection and UN38.3 achieved [S76 P]; UL2271 in process [S82 S] | **Gap** (minor): add a battery requirement (IG-9) |
| Deployment evidence | None yet; our plan is simulation and bench tests (doc 03, doc 04) | Figure: BMW pilot, 1,250+ hours, with targets but no achieved values [S77 P]; Amazon: over one million robots [S85 P]; Tesla: factory use, no productive work reported as of January 2026 [S60 S] | **Different** by scope; **Unproven** for us |

### 7.2 Limitations and risks

| Limitation seen in the cases | Our risk register | Verdict and action |
| --- | --- | --- |
| Spot documents degraded localisation when a scene changes or features are sparse, and "lost" events [S16] | R-07, estimator divergence [doc 05]; reduced autonomy on uncertainty [doc 02 §6] | **Converged** on the risk family; **Unproven** for us. Add an explicit "lost" state with relocalisation (BL-4) |
| Spot falls about once per 50 km, per a press report [S19] | No tip-over or fall risk in the requirements or the register | **Gap** (minor): our ramp and slope limits create a tip-over case we do not yet list |
| Digit and UR: safety behaviour depends on configuration and site [S43, S53] | R-11, safety case credibility [doc 05] | **Converged**: configuration belongs in the safety case (theory 08 §6) |
| FR3: 10 Hz Cartesian force-control bandwidth [S47] | No contact-force control in scope | **Different** by scope |
| Jackal and MiR: floor friction and step limits [S08] | S3 reflective floors [doc 00 §2]; BC-04 thresholds of 15 mm | **Unproven**: add friction and step limits to the envelope |
| URSim does not raise safety events [S55] | CI asserts SF-02, SF-05, SF-06 [doc 03 §5] | **Unproven**: confirm each assertion can trigger a stop (IG-5) |
| Figure: a whistleblower suit alleges impact-test forces above skull-fracture levels and a gash in a steel door; Figure denies it; allegations, pending as of March 2026 [S78, S79] | R-11, safety-case credibility [doc 05]; no contact-force or impact-energy requirement in doc 01 or doc 02 | **Gap**: we have no contact-energy budget. Derived whole-body energy: about 10 J for us at 1.0 m/s and 20 kg, about 44 J for Figure 03 at 1.2 m/s and 61 kg. Add IG-8 |
| Amazon: injury studies conflict; one 2025 study finds fewer severe and more non-severe injuries in robotic centres; the company disputes the injury comparisons [S95 to S97] | R-11; no pace or ergonomics requirement | **Converged** on the risk: pace and human-robot contact drive injury patterns; **Unproven** for us. Record interventions and near misses in any test protocol, not only severe incidents |
| Tesla: timelines have slipped repeatedly, and the CEO says a published hand design was changed [S60, S69] | Doc 09 G3 (change control) | **Converged** on the need for change control. We do not use vendor dates in any plan |

### 7.3 Controllers

| Item | Ours | Industry cases | Verdict | Action |
| --- | --- | --- | --- | --- |
| Layer rates | L5 1–5 Hz; L3 20–50 Hz; L2 100–200 Hz; L1 10–20 kHz [doc 02 §1] | Learned motion controller at 8 Hz over a fast tracker [S29]; 50 Hz policy over 1 kHz joint control [S41] | **Converged** on the pattern | Cite both results in doc 06 as evidence for the layering (BL-2) |
| Where learning sits | RL at behaviour (L5), bounded, no actuation [theory 05 §1, theory 06 §2] | RL at the locomotion layer [S18, S28, S35] | **Different** by design; **Unproven** for navigation | Keep. Name the measurement that would falsify the choice (theory 05 §8) |
| Model-based core | NMPC with acados RTI, SDF constraints, p99 ≤45 ms target [theory 04, PR-05] | MPC candidates in production at Boston Dynamics [S18] | **Converged** in approach | None |
| Safety filter | CBF-QP at 100–200 Hz, analytic [theory 06 §3] | Cross-checked safety system at 8 ms [S53]; safety PLC [S43] | **Converged** that safety is separate from the control stack; **Gap** in placement (doc 09 G1) | Doc 09 backlog G1 |
| Compliant or impedance control | Not in scope (wheeled, no contact control) | Franka and ANYdrive expose impedance control [S47, S24] | **Different** by scope | None |
| Learned actuation path | L5 policy 1–5 Hz, bounded, no actuation; CBF 100–200 Hz; FOC 10–20 kHz [doc 02 §1, §5] | Figure: 7–9 Hz semantic, 200 Hz visuomotor and 1 kHz whole-body control, all learned [S74, S75 P]; Tesla: end-to-end, per executive statement [S65 P] | **Different**: learning reaches actuator rate in the humanoid cases | Record our placement as a constraint: no learned layer commands actuators (doc 02 §5, theory 06 §2). Keep |
| Independent veto | CBF-QP, analytic and non-learned [theory 06 §3] | Not described for Figure or Tesla (N); Amazon's planner is safety-aware but is a planner, not a veto [S88 S] | **Different**; **Unproven** whether a separate veto is required for learned humanoid control | Keep. Doc 09 G1 applies |

### 7.4 Motors

| Item | Ours | Industry cases | Verdict | Action |
| --- | --- | --- | --- | --- |
| Motor family | BLDC or PMSM with FOC [theory 02 §1 to §3] | BLDC with integrated FOC in joints [S24, S37, S47, S53] | **Converged** | None |
| Motor class | Open. Hub motors are likely surface-mounted, so MTPA yields nothing [theory 02 §3.4, §8] | Not published for wheel motors (N) | **Unproven**: identify Ld and Lq on the T1 bench | Measure on T1 (theory 02 §8, question 1) |
| Driver ownership | External drivers in the T2 bill of materials [doc 04] | Integrated in the joint [S24, S37] | **Different**: ours is more modular and cheaper; the cost is calibration and cabling | Record the decision explicitly |
| Encoder type | Encoders in the BOM; type not fixed [doc 04] | Absolute encoders in joints [S24, S57, S37] | **Gap** (decision). Absolute encoders remove homing at power-up for joints; the benefit is smaller for free-spinning wheels | Specify absolute or incremental-with-index in doc 02 §2 |
| Torque and speed envelope | Not specified | Joint peaks published: 40 Nm [S24], 23.7 Nm [S37], ±87 Nm [S48] | **Gap** (minor): no wheel torque or speed in the docs | Derive the wheel torque from the ramp and acceleration envelope (IG-7) |
| Actuator family | BLDC or PMSM with FOC [theory 02 §1 to §3] | Tesla: frameless torque motors with harmonic drives and roller screws (S) [S62]; Figure: frameless BLDC (S) [S82] and in-house actuators [S76 P] | **Converged** on brushless, integrated actuators; roller screws are **Different** and not relevant to a wheeled base | None |
| Hand actuator count | Not applicable: no hand in scope | Tesla: 50 actuators across both hands (S) [S61]; Figure: not published (N) | **Different** by scope | Specify before any manipulator is bought (§4.1) |

### 7.5 Drivers and buses

| Item | Ours | Industry cases | Verdict | Action |
| --- | --- | --- | --- | --- |
| Command rate to the drive | Wheel command at 100 Hz [doc 02 §3] | Joint buses at 500 Hz to 6 kHz: UR 500 Hz [S51]; Franka 1 kHz [S47]; Unitree 6 kHz [S37]; ANYdrive internal 4 kHz [S26] | **Gap** (IG-1) | Raise the wheel-command rate to at least 500 Hz, or define timestamped MCU-side interpolation |
| PR-12 versus the interface | PR-12 requires speed-regulation bandwidth ≥100 Hz, measured by step response [doc 01] | Not applicable | **Gap** (IG-1). A 10 ms command period cannot resolve a 100 Hz-bandwidth step (rise time about 3.5 ms, derived). A 100 Hz command also caps reference bandwidth near 50 Hz (Nyquist, derived) | Measure PR-12 at the MCU with a local step; reword PR-12 |
| Current loop | FOC at ≥10 kHz (PR-11) | ANYdrive joint loop at 4 kHz [S26]; FOC rates for other drives not published (N) | **Converged**: our current-loop rate is above the joint-loop rates in the cases | None |
| Stop path | MCU torque-enable line, heartbeat, watchdog and E-stop (SF-03, theory 08) | Safety PLC at PL d and FSoE [S43]; cross-checked safety system [S53] | **Converged** in layering; **Gap** (IG-4): no safety-rated chain specified | Specify the stop chain (IG-4) |
| Protocol documentation | Frame format and CRC to be frozen (doc 02 §8) | Published interfaces for FCI, RTDE and CANopen or EtherCAT [S47, S56, S24] | **Converged** | None |
| Command rate in the humanoid cases | Wheel command at 100 Hz [doc 02 §3] | Figure S0 at 1 kHz on the robot [S75 P] | **Gap** (IG-1), confirmed by a second, different case | Raise the rate (IG-1) |
| Link-fault handling | Sequence numbers, CRC and heartbeat [doc 02 §2]; SF-02 names comms [doc 01] | Figure's forearm board was its top hardware failure point [S77 P] | **Converged** in design; **Unproven** in test: no link-fault injection is specified | Add corrupted, lost and late frame cases to the doc 03 §5 CI |
| Stop-chain publication | MCU heartbeat and E-stop (SF-03; theory 08) | Not published for Tesla, Figure or Amazon (N) | **Unproven** for these cases; **Gap** (IG-4) unchanged | None beyond IG-4 |

### 7.6 Simulation environments

| Item | Ours | Industry cases | Verdict | Action |
| --- | --- | --- | --- | --- |
| System and navigation tests | Gazebo in Docker; headless CI [doc 03] | Gazebo for Jackal and Franka [S06, S48]; community Gazebo for MiR [S14] | **Converged** | None |
| RL training scale | Gazebo "effectively" [theory 05 §2.1]; no throughput estimate | GPU-parallel: Isaac Gym and Isaac Lab, RaiSim, MuJoCo [S20, S28, S35] | **Gap** (IG-5): no training simulator and no throughput estimate | Choose a training simulator and estimate throughput before M5 |
| Sim-to-sim validation | Not in the docs | Unitree pipeline [S35]; Isaac Lab sim-to-real resources [S21] | **Gap** (IG-5) | Add a sim-to-sim step to the doc 03 `train` profile and theory 05 §6 |
| Safety events in simulation | CI asserts SF-02, SF-05, SF-06 [doc 03 §5] | URSim raises no safety events [S55] | **Unproven** | Confirm that CI can trigger each safety stop |
| GPU access | GPU needed for training [doc 03 §3]; laptop unavailable [R-13, R-14] | Cloud or cluster training (P) [S18] | **Unproven** under our constraints | Record the training-hardware plan under R-14 |
| Training scale in the humanoid cases | Gazebo for RL 'effectively' [theory 05 §2.1]; no throughput estimate | Figure S0: more than 200,000 parallel environments [S75 P] | **Gap** (IG-5), confirmed | Estimate throughput for each candidate simulator before M5 (IG-5) |
| Learned simulators | Not in our docs | Tesla's world simulator [S65 P; S67 S] | **Different**; **Unproven** if used | If used for evaluation, validate against logged real rollouts first (BL-6) |
| Digital twins for design | Out of scope (doc 00 §5) | Amazon: Omniverse and Isaac Sim warehouse twins [S93 P] | **Different** by scope | None |

### 7.7 Safety and certification

| Item | Ours | Industry cases | Verdict | Action |
| --- | --- | --- | --- | --- |
| Layered safety | Software filter, MCU gate, E-stop [theory 06 §4, theory 08 §3] | Cross-checked safety system [S53]; safety PLC [S43]; scanners and standards [S07, S10, S12] | **Converged** | None |
| Stop categories | Categories 0, 1 and 2 [theory 08 §4] | Stop functions in the UR and Agility releases [S51, S43]; stop categories not named in our sources | **Converged** | None |
| Safety-rated chain | Not specified. T3 includes a safety relay [doc 04] | PL d, Category 3 chains [S51, S43] | **Gap** (IG-4) | Specify the chain and its architecture category |
| Wording discipline | Avoid "designed to meet" and "certified" unless evidenced [doc 00 §5, SF-08] | MiR "designed to meet" [S07]; ANYmal X certified [S31]; Franka TÜV claim unverified [S50] | **Converged** | Keep. Apply the same rule to every claim (SF-08) |
| Certification evidence | None claimed | ANYmal X Zone 1 IIB [S31]; Digit site-level inspection [S43] | **Behind** by scope and budget | Regulatory timeline in doc 07 D1 |
| Independent safety function | SF-01: separate compute and code path (doc 01 §4) | Not described for Tesla or Figure (N); Amazon's planner is safety-aware (S) [S88] | **Unproven** for the humanoids; our gap is in IG-4 and doc 09 G1 | None beyond existing items |
| Battery safety | BC-06 states voltage and capacity only | Figure 03: layered protection and UN38.3 achieved [S76 P]; UL2271 in process (S) [S82] | **Gap** (minor), IG-9 | Add battery requirement (IG-9) |
| Field evidence of harm | Not in scope at this stage (doc 00 §6) | Figure lawsuit (allegations) [S78]; Amazon injury studies, contested [S95 to S97] | **Converged** on the principle: field data is what regulators and the public weigh; we have none | Keep SF-08; record near misses in test protocols |

### 7.8 Summary of verdicts

| Dimension | Verdict | One-line reason |
| --- | --- | --- |
| Architecture pattern | **Converged** | Two-rate hierarchy, independent stop path, integrated FOC joints |
| Learning placement | **Different** | RL at behaviour, not locomotion; the humanoid cases learn at actuator rate; performance unproven |
| Wheel command rate | **Gap** | 100 Hz bridge cannot support PR-12 and is the slowest link |
| Payload requirement | **Gap** | Absent from doc 01 |
| Energy budget | **Unproven** | PR-10 and BC-06 imply about 200 W average; no budget exists |
| Ramp requirement | **Gap** | 5° exceeds the published wheeled reference of about 2.9° |
| Safety-rated stop chain | **Gap** | Not specified; the cases specify PL d chains |
| Training simulator | **Gap** | Gazebo only; no throughput estimate; no sim-to-sim step |
| Encoder and driver decisions | **Gap** (decision) | Industry joints use integrated absolute encoders |
| Ingress and environment | **Unproven** | No ingress target |
| Certification | **Behind** | Scope and budget; timeline in doc 07 |
| Learned actuation rate | **Different** | Figure's learned policy runs at 200 Hz and 1 kHz; our learned layer runs at 1–5 Hz and does not actuate |
| Contact-energy budget | **Gap** | No contact-force or impact-energy requirement (IG-8) |
| Battery safety | **Gap** (minor) | Requirements state capacity and voltage only (IG-9) |
| Link-fault testing | **Unproven** | Protocol specified; fault injection not yet specified |
| Training-simulator scale | **Gap** | A 200,000-environment humanoid training setup is published; ours is undecided (IG-5) |

---

## 8. What this changes in our plan (backlog; not applied)

Docs 00 to 02 stay unchanged until you confirm the dilemma in [doc 07](07-open-dilemmas.md), as planned.

| ID | Gap | Proposed change | Target | Priority |
| --- | --- | --- | --- | --- |
| IG-1 | The 100 Hz wheel-command bridge cannot support PR-12, and it is the slowest link | Raise the wheel-command rate to at least 500 Hz, using a link technology checked for that rate, or add timestamped MCU-side interpolation. Measure PR-12 at the MCU with a local step | doc 02 §3 (`bridge_mcu`), doc 01 PR-12, theory 01 §3.3 | High |
| IG-2 | No payload requirement | Add a payload requirement: sensors and compute mass budget within BC-01 | doc 01 (new requirement), doc 00 §4 | High |
| IG-3 | No energy budget. PR-10 and BC-06 imply about 200 W average | Derive a power budget per subsystem (drive at expected torque, compute class, sensors). Decide the edge-compute power class against it | doc 04 tiers, doc 02, theory 02 | High |
| IG-4 | No safety-rated stop chain | Specify the stop chain as a safety function: independent stop path, stop categories, and architecture category. Keep "designed to meet" wording unless evidenced | doc 01 SF-03, theory 08 §2 to §4, doc 04 T3 | High |
| IG-5 | No training simulator and no sim-to-sim step | Choose the training simulator (Isaac Lab or MuJoCo/MJX as candidates, against Gazebo). Add a sim-to-sim validation step. Estimate throughput | doc 03 `train` profile, theory 05 §4 and §6, R-14 | Medium |
| IG-6 | No ingress or environment rating | Set an ingress target and dust and cleaning conditions; link to scenario S3 | doc 01, doc 00 §2 | Medium |
| IG-7 | The ramp requirement is not tied to a drive-torque budget | Derive the required wheel torque for a 5° ramp at full mass, with traction limits. Confirm the drive sizing before the T2 purchase | doc 01 BC-04, theory 01 §5 | High |
| BL-1 | Encoder type unspecified | Specify absolute or incremental-with-index encoders for the wheel motors. Record the driver-ownership decision | doc 02 §2, doc 04 T2 | Medium |
| BL-2 | Evidence for low-rate learning | Add the ANYmal 8 Hz result [S29] and the A1 50 Hz over 1 kHz result [S41] to doc 06. Cite them in theory 06 §2.1 as the same layering with a different layer of learning | doc 06, theory 06 §2.1 | Low |
| BL-3 | Cross-checking between computers | Consider a cross-check: the MCU recomputes the speed envelope from encoders and IMU and compares it with the edge computer's command; a mismatch triggers stop. This answers theory 08 §8, question 1 | doc 09 G1 (kernel design), theory 08 §8 | Medium |
| BL-4 | No "lost" localisation state | Add a lost state with relocalisation to the degradation ladder | doc 02 §6 | Medium |
| IG-8 | No contact-force or impact-energy budget. SF-06 limits speed by distance but sets no energy or force bound | Derive a whole-body and contact-energy envelope at the speed cap and payload (about 10 J at 1.0 m/s and 20 kg, derived). Set force and energy limits per body region against a named reference (not checked in this pass). Tie them to the PR-13 stop time and the SF-06 speed curve | doc 01 SF-06 and SF-07, doc 00 §5, theory 08 §5 | High |
| IG-9 | No battery-safety requirement; BC-06 states voltage and capacity only | Add cell and pack protection requirements (over-voltage, over-current, temperature, short circuit), a BMS cut-off path tested as a safety function, and UN38.3 for any shipped pack | doc 01 BC-06, doc 02 §2, doc 04 T2 | Medium |
| BL-5 | No demonstration or teleoperation data plan | Decide whether real demonstrations or teleoperation logs will be collected. Figure's 2025 model used about 500 hours of teleoperated data [S74 P]. Our plan has no real data. If we collect any, version it: the doc 03 §7 metadata schema has no dataset or weights field, although its prose promises them | doc 00 §5, theory 05 §6, doc 03 §7 | Low |
| BL-6 | No rule for learned simulators | If a learned or video-generation simulator is ever used to evaluate a policy, validate it first against logged real rollouts on the same scenarios and metrics. Keep physics-based tests for every safety claim (SF-02, SF-05, SF-06) | theory 05 §6, theory 08 §5, doc 03 §5 | Low |
| BL-7 | doc 08 §4 relies on secondary figures for Helix | Replace the Helix figures in doc 08 §4 with the primary releases [S74, S75], and mark which values are company-reported | doc 08 §4 | Low |

**Decisions needed from you** (none applied):

- The T2 near-field input for the MCU safety kernel (contact bumper or ring), from [doc 09 §5](09-architecture-comparison.md).
- The wheel-command link technology, which IG-1 depends on.
- The training-simulator and GPU plan, which IG-5 depends on, under the current laptop constraint.
- The ingress target, which IG-6 depends on.
- The contact-energy reference and force limits, which IG-8 depends on.
- The battery-safety requirement and its test, which IG-9 depends on.

---

## 9. Sources

Tags: P primary, S secondary, N not published in the sources accessed. Accessed 2026-10-09.

| ID | Source | Platform | Tag | Used for |
| --- | --- | --- | --- | --- |
| S01 | Jackal user manual (current) — https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal/ | Jackal | P | Specs, runtime, IP62, control, safety warnings |
| S02 | Jackal user manual, legacy ROS 1 — https://docs.clearpathrobotics.com/docs_robots/legacy/ros1_robots/outdoor_robots/jackal/user_manual_jackal/ | Jackal | P | MCU and rosserial architecture |
| S03 | Jackal brochure, 2014/15 — https://www.clearpathrobotics.com/wp-content/uploads/2015/06/Jackal_Brochure_2014-web.pdf | Jackal | P | Encoders, control modes, feedback |
| S04 | Jackal specification sheet (distributor copy) — https://static.generation-robots.com/media/fiche-technique-jackal-clearpath-robotics.pdf | Jackal | P | Mass, payload, drive power |
| S05 | Clearpath ROS 2 getting started — https://docs.clearpathrobotics.com/docs/ros/ | Jackal | P | ROS 2 control API |
| S06 | UCR EE106 lab notes (official jackal_gazebo package) — https://ucr-ee106.readthedocs.io/en/latest/lab2.html | Jackal | S | Gazebo simulation package |
| S07 | MiR250 specifications page — https://mobile-industrial-robots.com/products/robots/mir250/specifications | MiR250 | P | Payload, speed, runtime, standards |
| S08 | MiR250 specification sheet (distributor copy) — https://shelley.com/wp-content/uploads/2024/07/MiR250-Specifications-2.81.pdf | MiR250 | P | Acceleration, step limits, friction, incline, I/O, Modbus |
| S09 | MiR250 user guide 1.4 — https://supportportal.mobile-industrial-robots.com/support-files/manuals/html/en/mir250_user_guide_1.4/content/_resources/pdf/mir250%20user%20guide%201.4_en.pdf | MiR250 | P | Navigation behaviour, automatic deceleration, REST API |
| S10 | MiR250 sensor summary (trade press, 2020) — https://www.vision-systems.com/embedded/article/14174999/mir250-goods-transportation-robot-built-for-tight-environments | MiR250 | S | Sensor suite (earlier generation) |
| S11 | MiR250 review, 2026 — https://antonrobots.com/mir250-review/ | MiR250 | S | IP21 base; safety-standard wording |
| S12 | MiR250 quick facts (distributor) — https://roboticinnovation.no/mir250-3/ | MiR250 | S | Mass 97 kg |
| S13 | Community ROS 2 Humble package for MiR250 — https://github.com/giangalv/MIR_250 | MiR250 | S | ROS 2 integration (community) |
| S14 | Community ROS 1 package, DFKI (not affiliated with MiR) — https://github.com/DFKI-NI/mir_robot | MiR250 | S | Gazebo packages (community) |
| S15 | Spot SDK, About Spot — https://dev.bostondynamics.com/docs/concepts/about_spot.html | Spot | P | Mass, actuators, power, payload, speed |
| S16 | Spot SDK, GraphNav technical summary — https://dev.bostondynamics.com/docs/concepts/autonomy/graphnav_tech_summary.html | Spot | P | Topological localisation and lost robots |
| S17 | Spot SDK, components of navigation — https://github.com/boston-dynamics/spot-sdk/blob/master/docs/concepts/autonomy/components_of_autonomous_navigation.md | Spot | P | Waypoint spacing; maps |
| S18 | Boston Dynamics, "Starting on the Right Foot with Reinforcement Learning" (2024) — https://bostondynamics.com/blog/starting-on-the-right-foot-with-reinforcement-learning/ | Spot | P | MPC scoring; RL; simulation scale; fleet hours |
| S19 | IEEE Spectrum, Spot research variant (2024) — https://spectrum.ieee.org/boston-dynamics-research-spot | Spot | S | Fall rate (press, citing company CTO) |
| S20 | Spot RL Researcher Kit (distributor page) — https://www.intuitive-robots.com/spot-robot-payloads-and-accessories/spot-reinforcement-learning-researcher-kit/ | Spot | S | Joint-level API; Isaac Lab environment |
| S21 | Isaac Lab documentation, sim-to-real resources — https://isaac-sim.github.io/IsaacLab/main/source/refs/additional_resources.html | Spot | P | Spot sim-to-real guide |
| S22 | Spot specification aggregator — https://robotoverflow.com/en/robot-database/boston-dynamics-spot/ | Spot | S | IP54 |
| S23 | Third-party Isaac Lab Spot project — https://www.alexkoven.com/project/spot_walking/ | Spot | S | 8,192 parallel environments |
| S24 | ANYdrive datasheet, ETH RSL flyer — https://rsl.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/ANYdriveFlyer.pdf | ANYmal | P | Torque, speed, encoders, bus |
| S25 | ANYdrive product page, archived 2018 — https://web.archive.org/web/20181004114815/https://www.anybotics.com/anydrive/ | ANYmal | P | EtherCAT, IP67, specifications |
| S26 | Hutter et al., Advanced Robotics 2017 — https://www.markobjelonic.com/publications/files/2017_advanced_robotics_hutter.pdf | ANYmal | P | Series elastic design; 4 kHz loop; CANopen to EtherCAT |
| S27 | ANYmal, IROS 2016 paper (mirror copy) — https://www.scribd.com/document/751670565/ANYmal-A-Highly-Mobile-and-Dynamic-Quadrupedal-Robot | ANYmal | P | Bandwidth 70 Hz to 24 Hz; power and autonomy |
| S28 | Collaborative quadrupedal manipulation with RL (CASE 2021) — https://hybrid-robotics.berkeley.edu/publications/CASE2021_RL_QuadrupedPayloadManipulation.pdf | ANYmal | P | RaiSim; 100 Hz policy |
| S29 | Learning low-frequency motion control (ANYmal C at 8 Hz) — https://www.researchgate.net/publication/364025916_Learning_Low-Frequency_Motion_Control_for_Robust_and_Dynamic_Robot_Locomotion | ANYmal | P | Learned controller at 8 Hz with a tracker |
| S30 | Lee et al., learning quadrupedal locomotion over challenging terrain (arXiv 2010.11251), read as a summary — https://www.alphaxiv.org/abs/2010.11251 | ANYmal | S | Teacher and student training |
| S31 | ANYmal review, 2026 — https://antonrobots.com/anybotics-anymal-review/ | ANYmal X | S | Payload, runtime, ATEX, pending 2026 specifications |
| S32 | ANYmal X profile — https://www.humanoidhub.ai/robots/anymal-x | ANYmal X | S | Mass 50 kg |
| S33 | ANYmal specifications aggregator — https://aihumanlove.com/tools/robots/anymal/specs.html | ANYmal | S | Speed; runtime (conflict) |
| S34 | ANYmal X distributor page — https://www.sep.ae/product/anymal-x- | ANYmal X | S | ATEX Zone 1 |
| S35 | unitree_rl_gym repository summary (DeepWiki) — https://deepwiki.com/unitreerobotics/unitree_rl_gym | Unitree | S (summary of P) | Pipeline: Isaac Gym, rsl_rl, MuJoCo, deployment |
| S36 | Unitree RL GYM guide (third-party) — https://www.robotlar.org/en/rl-gym | Unitree | S | Pipeline description; unsupported success rate excluded |
| S37 | GO-M8010-6 specification (retailer) — https://aifitlab.com/products/unitree-go-m8010-6-motor | Unitree | S | Torque, speed, gear, bus, FOC, encoder |
| S38 | GO-M8010-6 specification (retailer; labelled for Go1) — https://robosavvy.co.uk/unitree-8010-motor.html | Unitree | S | Corroborates S37 |
| S39 | Go2 EDU Plus specifications (retailer) — https://www.generationrobots.com/en/404127-go2-quadruped-robot-edu-plus.html | Unitree | S | Mass, payload, speed, torque, runtime |
| S40 | Go2 and Go1 comparison (retailer blog) — https://www.generationrobots.com/blog/en/unitree-go2-a-quadruped-robot-with-built-in-ai-opening-up-infinite-possibilities-of-applications/ | Unitree | S | Go2 generation changes |
| S41 | Learned locomotion with a model-based joint controller (A1, simulated; peer-reviewed) — https://pmc.ncbi.nlm.nih.gov/articles/PMC10813166/ | Unitree (A1) | P | 50 Hz policy; 1 kHz joint control |
| S42 | Go2 versus Go2-W (blog) — https://www.awesomerobots.xyz/blog/unitree-go2-vs-go2-w-comparison | Unitree | S | Battery options, runtime |
| S43 | Agility Digit review with official-release figures (robozaps) — https://blog.robozaps.com/b/agility-robotics-digit-review | Agility | S (quotes P) | Payload, runtime, safety, certification limits |
| S44 | RobotOps Digit deployment blog — https://robotops.pro/blog/agility-digit-warehouse-deployment | Agility | S | Runtime 4 h (company), deployment |
| S45 | Humanoids Daily, Agility profile — https://www.humanoidsdaily.com/companies/agility-robotics | Agility | S | Mass 76 kg (conflict); v5 timing (conflict) |
| S46 | Robotoverflow, Digit specifications — https://robotoverflow.com/en/robot-database/agility-digit/ | Agility | S | Mass 65 kg (conflict) |
| S47 | Franka Emika datasheet (distributor copy, 2020) — https://pkj-robotics.dk/wp-content/uploads/2020/09/Franka-Emika_Brochure_EN_April20_PKJ.pdf | Franka | P | FCI at 1 kHz; torque sensors; force bandwidth; motors and gears; power |
| S48 | FR3 review with joint limits (aggregator) — https://antonrobots.com/franka-research-3-review/ | Franka | S | Joint torque limits; Cartesian speed |
| S49 | The Franka Emika robot: a standard platform (review, preprint copy) — https://www.researchgate.net/publication/384330010_The_Franka_Emika_Robot_A_Standard_Platform_in_Robotics_Research | Franka | P | Torque sensor after gearbox; 1 kHz interface; libfranka |
| S50 | FR3 aggregator (ednexautomation) — https://www.ednexautomation.ai/franka/franka-research-3/ | Franka | S | TÜV claim (unverified) |
| S51 | UR10e technical data (distributor copy) — https://biorob-hub.eu/wp-content/uploads/2025/02/UR10e_TechnicalData.pdf | UR10e | P | Payload, reach, 500 Hz, safety functions, PL d |
| S52 | UR10e user manual, PolyScope 5 — https://www.universal-robots.com/manuals/EN/PDF/SW5_19/user-manual-UR10e-PDF_online/711-039-00_UR10e_User_Manual_en_Global.pdf | UR10e | P | Mass, IP54, power, repeatability, stopping data |
| S53 | IEEE Spectrum, Universal Robots interview (2023) — https://spectrum.ieee.org/universal-robots | UR10e | P (interview) | Per-joint servo units; 8 ms cross-check |
| S54 | URSim offline simulator download page — https://www.universal-robots.com/download/software-ur-series/simulator-non-linux/offline-simulator-ur-series-e-series-ur-sim-for-non-linux-5252/ | UR10e | P | Official offline simulator |
| S55 | URSim scope guide (practitioner) — https://industrialmonitordirect.com/blogs/knowledgebase/deploying-ursim-code-to-ur-robots-six-hardware-modifications | UR10e | S | What URSim does not model |
| S56 | RTDE external control guide (practitioner) — https://industrialmonitordirect.com/blogs/knowledgebase/controlling-universal-robots-axes-from-external-devices | UR10e | S | RTDE at 500 Hz |
| S57 | UR robot summary (robotsguide) — https://robotsguide.com/robots/universal | UR10e | S | Joint design; dual encoders |
| S58 | ANYmal X runtime review (aggregator), 2026 — https://www.oxmaint.com/blog/post/best-anymal-vs-spot-industrial-maintenance-2026 | ANYmal X | S | Runtime 2–4 h (conflicts with S31 and S33) |
| S59 | Tesla Q1 2026 earnings call, third-party full transcript — https://evwire.com/p/tesla-tsla-q1-2026-earnings-call-full-transcript | Tesla Optimus | P (transcript of the company call) | Fremont start timing; "quite slow" ramp; 10,000 unique parts; no 2026 rate forecast; second factory; V3 reveal timing |
| S60 | Electrek, coverage of the Q1 2026 call (22 Apr 2026) — https://electrek.co/2026/04/22/tesla-optimus-production-fremont-model-sx-line/ | Tesla Optimus | S | Missed 2025 target; January 2026 remark that units were not doing useful work |
| S61 | optimusk.blog, status tracker, Apr–May 2026 — https://optimusk.blog/blog/tesla-optimus-status-april-may-2026/ | Tesla Optimus | S | Q2 2026 summary; 22-DoF hands; 50 hand actuators across two hands; AI5 chip claim |
| S62 | optimusk.blog, hardware specifications (Sep 2026) — https://optimusk.blog/blog/tesla-optimus-hardware-specs/ | Tesla Optimus | S | Body actuator counts and types; hand gearboxes |
| S63 | optimusk.blog, latest version (11 Sep 2026) — https://optimusk.blog/blog/tesla-optimus-humanoid-robot-latest-version-2026/ | Tesla Optimus | S | 2.3 kWh battery; V3 reveal still pending |
| S64 | CNBC, Tesla's stated Optimus specifications (21 Apr 2022) — https://www.cnbc.com/2022/04/21/elon-musk-says-optimus-robot-will-be-worth-more-than-tesla.html | Tesla Optimus | S (reports K) | 5 ft 8 in; 125 lb; carry 45 lb; lift 150 lb; 5 mph maximum; "outrun or overpower" statement |
| S65 | Ashok Elluswamy, "Tesla's approach to Autonomy" (X article on an ICCV 2025 talk, Oct 2025) — https://x.com/aelluswamy/status/1981644831790379245 | Tesla Optimus | P (executive article; performance statements K) | End-to-end learning; neural world simulator; use for Optimus; closed-loop RL claim |
| S66 | Humanoids Daily, Tesla Optimus profile and timeline (Sep 2026) — https://www.humanoidsdaily.com/robots/tesla-optimus | Tesla Optimus | S | July 2026 line installation; Optimus Academy; V3 reveal pending |
| S67 | LightMyTesla, CVPR 2026 coverage (Jun 2026) — https://lightmytesla.com/news/tesla-ashok-elluswamy-unified-ai-fsd-optimus-2026 | Tesla Optimus | S | 36 Hz world-simulator rendering (reported) |
| S68 | TeslaNorth, report on international patent "Mechanically Actuated Robotic Hand" (16 Apr 2026) — https://teslanorth.com/2026/04/16/teslas-robot-hand-patent-reveals-the-clever-engineering-inside-optimuss-fingers/ | Tesla Optimus | S (summary of a patent publication) | Cable-driven hand; forearm actuation; 22 DoF; springs return fingers to open |
| S69 | WeTalkTesla, patent coverage and CEO post (20 Apr 2026) — https://wetalktesla.com/2026/04/20/tesla-filed-patent-optimus-v3-hand-then-elon/ | Tesla Optimus | S (quotes a CEO post, K) | "This one didn't actually work"; design changed |
| S70 | TeslaNorth, Optimus learning from human video (21 May 2025) — https://teslanorth.com/2025/05/21/tesla-shows-off-new-optimus-robot-skills-learned-from-human-videos/ | Tesla Optimus | S (reports Tesla statements) | Learning from video; planned RL in real or simulated environments |
| S71 | Robot Couture, Tesla Optimus profile (25 Mar 2026) — https://robocouture.org/robots/tesla-optimus | Tesla Optimus | S | 72+ DoF; 2.3 kWh; 8–12 h (C); about 1.6 m/s (C); "mass production January 2026" (C) |
| S72 | Robotoverflow, Gen 2 versus V2.5 guide (Jun 2026) — https://www.robotoverflow.com/en/guides/tesla-optimus-gen2-vs-tesla-optimus-v25/ | Tesla Optimus | S | V2.5: 28 DoF; 4 h battery (C) |
| S73 | Robozaps, Optimus versus Atlas (Aug 2026) — https://blog.robozaps.com/b/tesla-optimus-vs-boston-dynamics-atlas | Tesla Optimus | S | About 8 km/h walking speed (C) |
| S74 | Figure AI, "Helix: A Vision-Language-Action Model for Generalist Humanoid Control" (20 Feb 2025) — https://www.figure.ai/news/helix | Figure 03 / Helix | P | S2 7B VLM at 7–9 Hz; S1 80M at 200 Hz; end-to-end training; about 500 h teleoperated data; S1 pretraining in simulation; onboard GPUs |
| S75 | Figure AI, "Introducing Helix 02: Full-Body Autonomy" (27 Jan 2026) — https://www.figure.ai/news/helix-02 | Figure 03 / Helix | P (four-minute demonstration: K) | S0 at 1 kHz, 10M parameters, 1,000+ h of motion data, 200,000 parallel environments; S1 inputs and outputs; demonstration claim (K) |
| S76 | Figure AI, "Introducing Figure 03" (9 Oct 2025) — https://www.figure.ai/news/introducing-figure-03 | Figure 03 | P | 9% less mass than Figure 02; UN38.3; 2 kW wireless charging; 3 g fingertip sensing; 10 Gbps; actuator speed claim; BotQ capacity; no absolute specification table |
| S77 | Figure AI, "F.02 Contributed to the Production of 30,000 Cars at BMW" (19 Nov 2025) — https://www.figure.ai/news/production-at-bmw | Figure 02 / 03 | P | Deployment totals; targets for cycle time, accuracy and interventions; forearm as top hardware failure point |
| S78 | Robotics and Automation News, report on the Gruendel complaint (26 Nov 2025) — https://roboticsandautomationnews.com/2025/11/26/former-figure-ai-engineer-at-claims-companys-humanoid-robots-powerful-enough-to-fracture-human-skull/96962/ | Figure 03 | S (reports a complaint) | Allegations; impact-test claims; gash in a refrigerator door; Figure's denial |
| S79 | CNBC, "Meet Figure AI" (26 Mar 2026) — https://www.cnbc.com/2026/03/26/figure-ai-the-robotics-company-hosted-by-melania-trump.html | Figure 03 | S | Case pending; January 2026 countersuit; Figure's "falsehoods" response |
| S80 | Robozaps, Figure 03 review (13 Jul 2026) — https://blog.robozaps.com/b/figure-03-review | Figure 03 | S (cites a spec card not found on Figure's page) | 61 kg; 20 kg payload; about 5 h; 1.2 m/s; 2.3 kWh (unverified); not for sale; no price |
| S81 | Antonrobots, Figure 03 review (15 Jul 2026) — https://antonrobots.com/figure-03-review/ | Figure 03 | S | Degrees of freedom and full-machine safety certification not published; BotQ units |
| S82 | AIWiki, Figure 03 entry — https://aiwiki.ai/wiki/figure_03 | Figure 03 | S (wiki) | Frameless BLDC actuators; UL2271 in process; 44 DoF (C) |
| S83 | Robotico Market, Figure 03 entry — https://robotico.market/figure-ai/figure-03 | Figure 03 | S | 35 DoF (C); 4.3 km/h |
| S84 | IIoT World, BMW deployment article (Apr 2026) — https://www.iiot-world.com/artificial-intelligence-ml/robotics/physical-ai-deployment-roi-humanoid-robots/ | Figure 02 | S | Trade report that the BMW targets were met (C with S77) |
| S85 | Amazon, "Amazon launches a new AI foundation model … and deploys its 1 millionth robot" (1 Jul 2025) — https://www.aboutamazon.com/news/operations/amazon-million-robots-ai-foundation-model | Amazon Robotics | P | Over one million robots; more than 300 facilities; DeepFleet 10% travel-time claim; Proteus safe-operation claim |
| S86 | Amazon Robotics, DeepFleet: Multi-Agent Foundation Models for Mobile Robots, arXiv:2508.08574 (Aug 2025; v3 Apr 2026) — https://arxiv.org/abs/2508.08574 | Amazon Robotics | P (unpublished technical report) | Four architectures; training on fleet data; prediction-based evaluation |
| S87 | IEEE Spectrum, "Amazon Shows Off Impressive New Warehouse Robots" (22 Nov 2022) — https://spectrum.ieee.org/amazon-warehouse-robots | Amazon Robotics | S | Proteus sensing: stereo vision and planar lidars |
| S88 | Robots Guide, Proteus entry — https://robotsguide.com/robots/proteus | Amazon Robotics | S | R15.08 compliance claim; payload-dependent speed; electric drives |
| S89 | The Robots HQ, Amazon Robotics Proteus entry — https://therobotshq.com/products/amazon-robotics-proteus/ | Amazon Robotics | S (uncorroborated) | 360 kg (C); 1.5 m/s; 12 h (uncorroborated) |
| S90 | Humanoid Metric, Amazon Proteus entry (Sep 2026) — https://humanoidmetric.com/robot/amazon-proteus | Amazon Robotics | S (unverified) | 400 kg payload (C) |
| S91 | TechCrunch, "Amazon debuts a warehouse robot with a sense of touch" (7 May 2025) — https://techcrunch.com/2025/05/07/amazon-debuts-a-warehouse-robot-with-a-sense-of-touch/ | Amazon Robotics | S (press with Amazon statements) | Vulcan force sensing; about 75% of stock (Amazon claim); training on touch data; deployment and order count (C) |
| S92 | CryptoBriefing, Vulcan report (May 2025) — https://cryptobriefing.com/amazon-vulcan-robot-tactile-sensing-warehouse/ | Amazon Robotics | S (uncorroborated) | 12,000 test orders (C); about 3.6 kg limit; Universal Robots arms |
| S93 | NVIDIA, "NVIDIA GPUs on AWS to Offer 2x Simulation Leap in Omniverse Isaac Sim" — https://blogs.nvidia.com/blog/gpu-aws-omniverse-isaac-sim-robots/ | Amazon Robotics | P (vendor) | Amazon digital twins of warehouses; Proteus sensor emulation; Amazon quote |
| S94 | The Robot Report, NVIDIA Isaac Sim coverage (25 Mar 2023) — https://www.therobotreport.com/nvidia-is-making-ai-easier-to-use/ | Amazon Robotics | S | Proteus digital twin; "hundreds of photo-realistic environments" |
| S95 | BBC News, report on the Reveal investigation (30 Sep 2020) — https://www.bbc.com/news/technology-54355803 | Amazon Robotics | S | Injury comparison from internal records; Amazon's response |
| S96 | Fortune, "Meet the growing army of Amazon robots …" (27 Mar 2025) — https://fortune.com/2025/03/27/amazon-robotics-worker-safety-proteus-warehouse-upskilling/ | Amazon Robotics | S | 2024 Senate investigation; Amazon rejects its findings |
| S97 | George Mason University, report on a 2025 study of robotic warehouses (26 Aug 2025) — https://business.gmu.edu/news/2025-08/warehouse-automation-hasnt-made-workers-safer-its-just-reshuffled-risk | Amazon Robotics | S (university summary) | About 40% fewer severe and 77% more non-severe injuries; pick-rate pressure |

**Our documents** are cited by path and section: [doc 00](00-mission-and-scope.md), [doc 01](01-requirements.md), [doc 02](02-system-architecture.md), [doc 03](03-simulation-and-docker.md), [doc 04](04-build-roadmap.md), [doc 05](05-risk-register.md), [doc 07](07-open-dilemmas.md), [doc 08](08-state-of-the-art-analysis.md), [doc 09](09-architecture-comparison.md), and theory [01](theory/01-kinematics-and-dynamics.md), [02](theory/02-electric-drive-pmsm-foc.md), [04](theory/04-advanced-control-nmpc.md), [05](theory/05-reinforcement-learning.md), [06](theory/06-rl-mpc-integration-and-safety.md), [08](theory/08-safety-and-verification.md).

---

## 10. Limits of this study

- **Public specifications only.** No industry value was measured or checked on hardware. Each is quoted as published.
- **Many values are secondary.** Retailer, aggregator and practitioner sources dominate the Unitree, Agility, ANYmal X and Franka figures. Where two secondary sources conflict, both are shown (C). Where no source was found, the cell says N, which means not found, not zero.
- **Tesla evidence is mostly secondary or company-stated.** tesla.com/optimus returned 404 when fetched. Hardware figures come from Tesla statements reported by the press (2022), an earnings-call transcript, and aggregator sites that disagree (C). Treat every Tesla hardware value as K or S, and none as measured.
- **Figure's spec card is unverified.** The 61 kg, 20 kg payload, 5 h and 1.2 m/s figures come from a secondary review that describes an official card. We did not find that card on Figure's release pages, and Figure's release gives only relative actuator gains. Degrees of freedom are not published (N), and aggregators disagree (C).
- **Amazon's per-robot figures are not published.** The DeepFleet report evaluates prediction, and the 10% travel-time gain is an announcement. Proteus and Vulcan figures come from trade press and aggregators.
- **Allegations and disputed studies are cited as such.** The Figure complaint, the Amazon injury comparisons and the 2025 injury study are reported as stated by their sources. We did not verify them and draw no conclusion about fault.
- **Eleven companies, a selection.** Not researched in this pass: 1X and other humanoids; Locus Robotics. Atlas and the π-class VLA systems are covered in [doc 08](08-state-of-the-art-analysis.md). Helix is profiled here from Figure's own releases, and doc 08 §4 still relies on secondary figures (BL-7). The omission is a scope choice, not a judgement about these companies.
- **Derived values are our arithmetic** from cited inputs, marked as derived.
- **Safety statements are the vendors' statements.** We did not check certificates. "Designed to meet" is a design claim, not a certification.
- **Prices and procurement are excluded** except where a source states them for context.
- **This document does not change docs 00 to 02.** Every change is listed as a backlog item in §8.
