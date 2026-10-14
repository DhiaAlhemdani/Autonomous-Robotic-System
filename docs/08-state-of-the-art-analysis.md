# 08 — State of the Art: The Most Advanced Robotic Systems and Where They Stop

**Purpose.** Establish the actual frontier, analyse how the leading systems are architected, and document where each one fails. This serves three ends:

1. **Positioning** — it is impossible to claim a contribution without knowing what the frontier already does.
2. **Validation** — if the most advanced system in the world independently converged on our architecture (doc 02, theory 06), that is evidence our design reasoning is sound.
3. **Justification** — the failures catalogued here (§5) are the empirical basis for the dilemma selected in doc 07.

**Status:** analysis of public information, October 2026. Every company's internals are opaque; several figures are vendor claims. §8 flags source quality.

---

## 1. "Most advanced" is not one thing — the seven axes

There is no single most advanced robot. A Mars rover with 1990s-class compute holds the record for autonomous operation in the most hostile environment; a robotaxi holds the record for verified safety at scale; a humanoid holds the record for dynamic whole-body control. Each is "most advanced" on a different axis.

| Axis | What it measures | Current holder |
| --- | --- | --- |
| **A1 Deployed scale with verified safety** | Autonomous operation, in public, with audited incident data | **Waymo Driver** |
| **A2 Dynamic whole-body control** | Coordinated control of a high-DoF body under load and perturbation | **Boston Dynamics Atlas** |
| **A3 Learned generalist intelligence** | Task generality across unseen objects, environments and embodiments | **Physical Intelligence π\*0.6 / Figure Helix / Google DeepMind Gemini Robotics** |
| **A4 Certified safety in hazardous industry** | Third-party certification for operation where a fault kills people | **ANYbotics ANYmal X** (ATEX/IECEx Zone 1) |
| **A5 Autonomy under extreme communication and compute constraints** | Working with 2 Mbps, 40-minute round trips, and radiation-hardened 2010s silicon | **NASA Perseverance (AutoNav + Mars Global Localization)** |
| **A6 Cost and manufacturing volume** | Units built per year at declining unit cost | **AgiBot / Unitree / UBTECH** (China) |
| **A7 Dexterity** | Manipulation of deformable, contact-rich objects | Physically unsolved; no holder (§5, D14) |

**On the question as asked — "the most advanced robotic system worldwide" — the defensible answer is the Waymo Driver.** Among the systems reviewed here, it is the only one that combines genuine Level-4 autonomy, public operation, a published safety framework with a safety-case method, quantitative incident reporting under federal order, 200+ million fully autonomous miles, and custom silicon — all at once. Other Level-4 operators exist and were not analysed here. It is also, and this is the point of this document, **visibly limited in ways that map closely onto our selected dilemma.**

## 2. Deep dive A — Waymo Driver: the most advanced *deployed* system

### 2.1 Architecture

| Layer | What they do |
| --- | --- |
| **Sensing** | 6th-gen (2024 announcement): 13 cameras, 4 lidar, 6 radar + external audio receivers. 2026 post: 17 MP imagers, imaging radar for severe weather, and **less than half the cameras** of the 5th generation. Redundant modalities, each with protective measures for long exposure to weather |
| **Compute** | Custom **5 nm ASIC delivering >1,000 TOPS**, plus NVIDIA accelerated computing, AMD, Micron, Samsung, TSMC. **Two independent processing engines** — normally run in parallel as one, and if one faults the other takes over seamlessly. Raw compute scaled **20× in 8 years**. Engineered for "first pixel to action" latency |
| **Model** | **Waymo Foundation Model** — deliberately neither pure end-to-end nor purely modular. Learned embeddings as the interface *plus* materialised structured representations (objects, semantic attributes, roadgraph) |
| **Deployment** | Teacher models distilled to efficient student models for onboard real-time deployment; the onboard architecture mirrors the foundation model's structure |
| **Validation** | **A separate, rigorous onboard validation layer that verifies the trajectories produced by the generative ML model** — checking plans against hard physics-based constraints and traffic laws, using reinforcement learning and reasoning inspired by generative AI, with **hard veto ("hard backstop")** authority |
| **Learning loops** | Inner loop: RL in the Simulator, scored by the Critic. Outer loop: real-world data → Critic flags suboptimal behaviour → improved alternative behaviours generated → tested in simulation → **only deployed after the safety framework confirms absence of unreasonable risk** |
| **Safety process** | A **Safety Framework** decomposing hazards along dimensions with acceptance criteria, and a **Safety Case** — a formal argument with evidence — determining readiness of every release candidate. Independent analysis by RAND |
| **Maps** | HD maps as a *prior* — "another input, like our sensors, acting as a mental memory", decisive in poor visibility and complex thoroughfares |
| **Fallback** | Remote assistance: ~70 agents on call for ~4,000 vehicles, half in the Philippines; they **advise, and the vehicle may accept or reject** — they do not drive |

### 2.2 The architecturally important decision

Waymo's own account of why they rejected pure end-to-end is worth quoting precisely, because it is the single most consequential engineering statement in this document:

> *"Pure end-to-end neural architectures… run the risk of black box failures. … we've added an independent onboard validation layer. This is a separate, AI-based safety system that monitors every trajectory proposed by the Waymo Driver. It checks these plans against hard physics-based constraints and traffic laws… If the AI proposes a path that violates a limit or risks a collision, the validation layer acts as a hard backstop."*
> — Waymo, *10 AI Lessons from Driving 200+ Million Fully Autonomous Miles*, August 2026

The convergence is on the **pattern**: **propose with a learned model, dispose with an independent verifier that has veto authority** — which is Topology 2 plus the CBF-QP safety filter in [`theory/06`](theory/06-rl-mpc-integration-and-safety.md) §2–3. The implementations differ, and the difference matters: Waymo describes its validator as AI-based, while ours is analytic and can be tested in isolation. They reached the pattern at 200 million miles and billions of simulated ones; we reached it by reasoning about failure modes. That is consistent with our design reasoning. It is not evidence that our design performs, and it tells us the architecture is not the frontier. **The evidence pipeline around it is.**

### 2.3 Limits — documented, and instructive

| Limit | Evidence |
| --- | --- |
| **Cannot estimate water depth** | Detected the flood, slowed, did not stop, was swept into a creek (San Antonio, April 2026). Recall of **3,791 vehicles**. The patch failed within two weeks (Atlanta, May 2026); service halted in five cities. "Neither [lidar nor radar] are particularly well suited to estimate how deep a particular puddle is" (Georgia Tech). Waymo still has **no permanent fix** |
| **Construction zones** | Second recall in two months: **~3,900 vehicles**, 13 known incidents of driving into closed freeway construction zones; freeway service restricted May–July 2026 |
| **Emergency scenes** | NHTSA administrator, July 2026: *"a disturbing trend"* of AVs interfering with first responders; "**Emergency scenes are not rare or extreme 'edge cases'**". San Francisco Fire Department reported 31 incidents in 15 months. One vehicle blocked an ambulance responding to a mass shooting |
| **School buses** | 19–24 documented illegal passes in one district; NHTSA and NTSB investigations. In one case the vehicle **asked a remote operator whether the bus had active signals, and the operator said no** |
| **Pedestrian safety** | Struck a 9-year-old (minor injuries, January 2026) after she emerged from behind a parked SUV — braked from ~17 mph to under 6 mph but could not avoid contact |
| **Power outages** | During the December 2025 San Francisco outage, vehicles halted in traffic and caused gridlock |
| **Behavioural bugs** | Dozens of empty vehicles circling cul-de-sacs for hours for weeks — a **routing-behaviour** problem |
| **ODD dependence** | Strictly geofenced, HD-map-dependent; speed limits (65 mph cap); service suspended in heavy weather; single-origin/destination only |

**Cumulative record:** as of August 2026, **2,308 NHTSA-logged accidents** involving Waymo vehicles since July 2021 — against 200+ million autonomous miles and injury-causing incident rates below the human benchmark.

### 2.4 The insight that matters most for us

Look at the *shape* of Waymo's failures. Several are **not simple perception failures**:

- The flood was **detected**. The system slowed and did not stop. NHTSA's filing says the vehicle "detected potentially untraversable flood water and proceeded at reduced speed". Journalism's explanation is *"there was no hard-stop condition for water in its path"*. Either way, the required response was never declared as a stop, and the sensor did its job.
- Construction zones: mapped long-term construction is handled well; the failures are unmapped and sudden — again, the **declared operating conditions** lag reality.
- Emergency scenes: NHTSA's July 2026 letter describes vehicles that failed to recognise and respond to flashing lights, flares, smoke, fire and traffic cones, and calls the inability "a functional insufficiency". That is partly a recognition failure and partly a missing declared response.
- The school bus case required **human judgment about an ambiguous scene**, and the remote operator answered wrongly — an escalation path in which the vehicle acted on a wrong answer from its remote operator.

> **The flood is a missing declared response, not a missing detection. The emergency-scene and school-bus failures are recognition and escalation failures, which a specification alone would not fix.**

That is precisely the artifact our doc 07 pipeline exists to produce (E1 capability declaration, E2 bounded-autonomy monitor, E5 change-and-revalidation). The most advanced deployed robotic system in the world is demonstrating, in public, with federal recalls, that a significant share of the bottleneck has moved from raw perception to **specification and evidence**.

## 3. Deep dive B — Boston Dynamics Atlas: dynamic whole-body control

### 3.1 Architecture and specifications

| Attribute | Value |
| --- | --- |
| Configuration | 6th-generation, fully electric humanoid; unveiled 5 January 2026 (CES); 56 DoF with fully rotational hip/waist/neck joints exceeding human range |
| Physical | 1.9 m, 90 kg, 50 kg instantaneous / 30 kg sustained payload, 2.3 m reach, **IP67**, −20 °C to +40 °C, sub-10 cm foot placement |
| Actuation | Custom electric actuators (Hyundai Mobis): **planetary roller screws + high-density neodymium magnets**, 85–90% electrical-to-mechanical efficiency; matching prior hydraulic force density without fluid maintenance |
| Energy | 4-hour runtime; **autonomous battery swap in under 3 minutes** at a charging station |
| Compute | **NVIDIA Jetson Thor T5000** (2,070 FP4 TFLOPS) — runs multimodal foundation models alongside the whole-body controller |
| Intelligence | Google DeepMind **Gemini Robotics** foundation models for cognition; Boston Dynamics proprietary whole-body and manipulation control; **Isaac Lab** for policy training |
| Control | Whole-body control: legs, torso, arms solved as one coupled system; uses **proprioception** (not vision alone) to sense load, balance and resistance |
| Learning | Millions of simulated hours with domain randomisation over weight, friction, grip and motor strength; RL on top of a reference animation; trained on 50–70 lb loads, then handled a 100+ lb unbalanced fridge |
| Manufacturability | Two actuator types across the whole body, identical arms, identical legs, repeated structures, **no cables across joints**, field-replaceable units; target 30,000 units/year by 2028; Hyundai plans 25,000+ deployments |

### 3.2 The architecturally important decision

Boston Dynamics' claim of a **small sim-to-real gap** is not a software achievement — it is a **hardware design decision**:

> *Two actuator types. Identical arms. Identical legs. Repeated structures. No cables across joints. Rotary rather than linear actuators. When the digital model closely matches the physical machine, behaviours trained in simulation transfer more reliably.*

They attacked the reality gap (doc 07 D6) by **removing modelling complexity from the mechanism**. This is a transferable principle we can apply cheaply: our differential-drive platform's simplicity — two identical motors, no suspension, no backlash-prone linkages — is a **control and reproducibility asset**, not a limitation to be apologised for. It is one reason a small robot can produce trustworthy sim-to-real measurements.

### 3.3 Limits

- **Not at scale in production.** "Pilot only" availability; the Hyundai 25,000-unit plan is forward-looking. Mass deployment is a plan, not a record.
- **Quantitative transparency is poor.** No official datasheet for many specifications; third-party estimates circulate; some widely repeated numbers refer to the retired *hydraulic* generation. Any performance figure outside §3.1 should be treated as unverified.
- **Dexterity is unfinished.** The 100 lb fridge demo used "workhorse grippers"; a dexterous gripper is "in testing". Lifting is solved; fine manipulation is not.
- **Energy still bounds it.** Four hours per battery, so continuous operation requires swap stations — viable inside a factory, not in a home.
- **Safety certification status is unclear.** Fenceless operation with human detection is claimed, but this is the exact class the EU Machinery Regulation (doc 07 D1) subjects to third-party assessment from January 2027. Publicly, the evidence pipeline is not visible.
- **Verification without field data is the stated weakness** — which is another way of saying the assurance problem is unsolved here too.

## 4. Deep dive C — the VLA frontier: π\*0.6, Helix, Gemini Robotics

### 4.1 Architectures

**π0.5 → π0.6 → π\*0.6 (Physical Intelligence).** A ~3.3 B-parameter stack in three stages: SigLIP vision encoder (~400 M) → Gemma 2B language backbone (~2.6 B) → action expert (~300 M) producing ~50 action tokens via flow matching. π0.6 upgrades the backbone to Gemma 3 4B and the action expert to 860 M. π\*0.6 adds **advantage-conditioned offline RL** with a learned value function, learning from experience *and human corrections*. Key mechanisms: **FAST action tokenisation** (~10× compression, 5× faster training), **Knowledge Insulation** (a stop-gradient so motor gradients cannot damage the semantic backbone), **Real-Time Chunking** (latency treated as part of the control problem), and **heterogeneous co-training** across robots and web data.

Their ablations quantify what generalisation costs: full π0.5 reaches 94% follow / 94% success in out-of-distribution homes; **removing web data drops it to 80% / 74%; removing cross-embodiment data drops it to 67% / 49%.**

**Figure Helix.** A dual-system humanoid VLA: **System 2** (7B VLM, 7–9 Hz) for scene understanding and planning, **System 1** (visuomotor policy, **200 Hz**) for continuous whole-body action — the classic slow-deliberation / fast-reflex split. One set of weights covers all behaviours without task-specific fine-tuning, trained on ~500 hours of teleoperation. Helix 2.5 (September 2026) pretrains on human behaviour data and reports zero-shot transfer to 30 unseen homes.

**Google DeepMind Gemini Robotics.** Built on a Gemini backbone with embodied reasoning; Motion Transfer unifies data across different robot bodies; on-device variants for low-latency control. Now used by Atlas.

### 4.2 The architecturally important decisions

1. **Two timescales, always.** Every serious system separates slow semantic reasoning (1–10 Hz) from fast control (50–200 Hz). This is the same rate separation as our L3/L5 architecture (doc 02 §1) — independently confirmed across the industry.
2. **A pretrained generalist backbone plus a small action head.** Nobody trains a robotics network from scratch; they graft control onto a vision-language model. Ours differs deliberately (see §6.3).
3. **Learning from experience, not just demonstration.** π\*0.6's RECAP — advantage conditioning, value functions, human corrections — is the frontier moving from imitation to RL in deployment. Our doc 05 §2 Topology 3 (residual RL over a competent baseline) is the small-scale analogue of this.
4. **Latency is a control problem, not an IT problem.** Real-Time Chunking exists because action chunks go stale while the robot is still moving. This is our PR-06 latency budget, treated as a design input.

### 4.3 Limits

- **Contact and deformables remain unsolved.** Simulation transfers well for locomotion, navigation and much of perception, and poorly wherever a gripper touches an object. This is the field's consensus, not a vendor claim.
- **Teleoperation data is still the substrate.** Helix is trained on ~500 hours of teleop; π0.5's generality depends on multi-embodiment teleop. The "teleoperation crutch" is acknowledged industry-wide as a data strategy that does not scale as a deployment model.
- **Long-horizon coherence is unsolved.** Helix "lacks explicit long-term world models". π0.5's generalisation depends on exposure to many environments — it is context-hungry, not reasoning-rich.
- **Closed-source.** π, Helix and Gemini Robotics internals are not inspectable; independent verification of their claims is not possible.
- **Benchmarks are saturated and misleading.** Nearly all policies tested on the LIBERO simulation suite scored 90–95% while spanning the full range from strong to weak on real hardware. **A saturated benchmark still returns a number.** This is the same evaluation pathology we guard against in doc 03 §7.
- **None of it is navigation.** This entire frontier is manipulation. It is the loudest area of the field and — for our platform — the least applicable.

## 5. Deep dive D — ANYmal X: the certification frontier (most relevant to us)

### 5.1 Architecture

| Layer | What they do |
| --- | --- |
| Locomotion | Quadruped, IP67, stair climbing, LiDAR/SLAM localisation, autonomous docking and charging |
| Actuation | **ANYdrive**: frameless motor + harmonic reducer + **planar spiral spring** — a series-elastic actuator giving compliant, force-controllable joints |
| Sensing | Visual zoom (20×), thermal, acoustic gas-leak detection, gas sensors, LiDAR; "super-human sensing" for condition monitoring |
| Autonomy | Predefined inspection rounds, positioning sensors at inspection points, reporting measurements into **asset-management workflows** (not free-form exploration) |
| Certification | Separately engineered for explosive atmospheres: **ATEX and IECEx Zone 1 IIB** — the world's first explosion-proof legged inspection robot |
| Integration | Yokogawa OpreX Robot Management Core integration (Feb 2026) for fleet management linked to plant control systems |
| Deployments | Equinor Northern Lights, Petronas Dulang B, Shell, BASF |

### 5.2 The architecturally important fact

A procurement analysis states that only three robots worldwide hold ATEX/IECEx Zone 1 certification: ANYmal X (legged), Taurob Inspector (tracked), ExRobotics ExR-2 (tracked). We have not checked that count against primary certification records, so treat it as unverified. The same analysis summarises the industry as: *"The certification is solved — now solve the data integration."*

This is the single most important finding in this document, and it must be read carefully. Certification for hazardous-area robots **is achievable** — it has been achieved, three times, by companies far smaller than Waymo. But all three are **bounded, mostly classical autonomy**: defined inspection rounds, known sensor positions, teleoperation fallback, and safety functions that are not learning-enabled.

Therefore the frontier is not "can robots be certified" but:

> **Certification is solved for bounded classical autonomy and unsolved for learning-enabled safety functions** — and the EU Machinery Regulation makes the second case mandatory from 20 January 2027.

That is doc 07's dilemma D1, confirmed from the industrial-certification side rather than the academic side.

### 5.3 Limits

- **Endurance and payload:** roughly 90–120 minutes runtime, ~10 kg nominal payload — the certified version is even more constrained.
- **Autonomy is intentionally narrow.** Inspection rounds at known points. Generalising to free-form exploration would invalidate the analysis behind the certification.
- **Cost and access:** quote-based enterprise pricing; hazardous-area certification is expensive and configuration-specific — *"Do not treat standard ANYmal and ANYmal X as interchangeable."*
- **Data integration is the remaining customer-facing problem**, not mobility or sensing.
- **The learning question is unanswered.** Nothing public describes how these platforms would recertify after a learned safety function changed.

## 6. Deep dive E — Perseverance and Ingenuity: autonomy at the extreme

### 6.1 What they did

- **AutoNav** carries the rover: it evaluated or executed **88%** of the distance driven in the first Earth year (~17.7 km), re-planning around obstacles en route to a commanded destination.
- **The bottleneck was localisation uncertainty, not perception or planning.** Wheel encoders and inclinometers accumulate error; slippage compounds it. Over a 655 m drive, position uncertainty grew from 0 to **~33 m**. Visual odometry limits this to ~2% per 100 m *when feature tracking is reliable*.
- **This is the same family of failure as our filter-conservatism risk (R-04), at planetary scale.** On sol 383–385 the uncertainty circle reached a **26.91 m radius**; hazard "keep-out" zones expanded with it until the navigable corridor shrank to nothing. The rover **drove 14 m and faulted.**
- **The fix was a specification of position, not better AI.** *Mars Global Localization* matches onboard navcam panoramas to HiRISE orbital imagery: ~2 minutes of compute, ~25 cm mode accuracy, 1.07 m maximum error, validated on 264 prior stops. It ran on a **repurposed Ingenuity Helicopter Base Station processor — 100× faster than the rover's other compute** — with 25 damaged memory bits worked around.
- Result: autonomous drive segments of **699.9 m over three sols**, and AI-generated waypoints (Claude-based, December 2025) for 456 m over two days. "Potentially unlimited distances without calling home."

### 6.2 The architecturally important lessons

1. **Unmodelled uncertainty creates conservatism, conservatism shrinks the feasible envelope, and the envelope collapses to a stop.** This is precisely the risk we flagged in theory 06 §3.4 (CBF conservatism → deadlock) and doc 05 R-04. Perseverance's sol-385 fault is that phenomenon with a measured number attached: **26.91 m of uncertainty → zero corridor.**
2. **The fix was reducing uncertainty at the source**, not adding margin. They did not make the planner more aggressive; they improved localisation. Our equivalent lever is doc 03: better localisation shrinks the required margins more cheaply than any controller tuning.
3. **Compute is where you find it.** Repurposing a helicopter's base station because it was 100× faster is unglamorous and correct. Our architecture should assume we will do the same kind of reallocation (e.g. time-slicing the edge GPU between perception and the solver).
4. **Communication-denied operation is genuinely solved — for bounded tasks.** Our scenario S4 has a working precedent.

### 6.3 Limits

- **2 Mbps downlink, 40-minute round trip**; autonomous drive plans historically capped at three sols before human review.
- **Radiation-hardened silicon is generations behind.** The rover's main compute is ancient by necessity.
- **No generalisation beyond the mapped body.** Orbital imagery is a prior — the Martian equivalent of Waymo's HD maps. Neither system explores truly unknown space.
- **Nothing transfers cost-effectively.** The engineering per kilometre is astronomical; this is an existence proof of a method, not a product.

## 7. Deep dive F — the Chinese volume frontier

| Metric | Value |
| --- | --- |
| Global humanoid installations, 2025 | ~16,000 units; China >80% of them; 2026 shipments forecast >50,000; MIIT projects >100,000 units produced in 2026 |
| Leaders (H1 2026) | AgiBot ~8,400 units (44%); Unitree ~5,900 (31%); all non-Chinese makers ~3% combined |
| Prices | Unitree R1 from **$4,900**; G1 **$13,500**; AgiBot X2 **$24,240**; Unitree H1 $90,000 list |
| Claimed performance | AgiBot reports 99.9% task success on production lines and 310 units/hour at a workstation (vendor claim) |
| Vertical integration | AgiBot 5,000 → 10,000 units in three months; UBTECH ~5,000/year capacity with confirmed orders >$195 M; XPeng IRON targeting mass production end-2026 |

**What this changes:** unit cost and manufacturing access. A capable humanoid research platform at $13,500, or an entry platform at $4,900, means hardware is no longer the barrier to entry it was a decade ago.

**Limits — and the most important caution in this document:**
- **Production is not deployment.** Unitree's revenue is reportedly **~70% research and education**, not industrial work. UBTECH slipped from third in production to fourth in shipments. The industry's own analysts characterise 2026 as "production boom meets deployment reality check."
- **Claimed success rates are vendor-reported and unaudited** (99.9%). We should treat these exactly as doc 07 treats the sim-to-real marketing figures — as claims, never as evidence.
- **No demonstrated profitable at-scale deployment** by any vendor.
- **Certification and safety evidence are absent from the public record**, which matters because these units are entering workspaces shared with people.

**Implication for us:** cheap access to high-quality actuators, encoders and motor drivers is realistic. Buying a *certified* safety-rated drive or a safety relay remains expensive — which is why our architecture puts the independent stop path on the MCU and physical E-stop (doc 02 §2, SF-03) rather than assuming we can buy a certified subsystem.

## 8. Synthesis: the convergent architecture

Six independent teams, billions of dollars, and every one of them arrives at the same seven patterns. This is what "state of the art" actually looks like:

| # | Convergent pattern | Who does it | Where it appears in our design |
| --- | --- | --- | --- |
| 1 | **A learned proposer behind an independent verifier with veto authority (learned in Waymo's case, analytic in ours)** | Waymo (onboard validation layer, "hard backstop"), Atlas (learned policies + classical whole-body controller; a secondary analysis calls it the "safety-critical backbone"), π\*0.6 (RL specialist around a prior) | **theory 06 Topology 2 + CBF-QP filter** |
| 2 | **Two timescales: slow reasoning, fast reflex** | Helix (7–9 Hz / 200 Hz), π0.5 (subtask then action), Atlas (foundation model + whole-body control), our L5/L3/L1 bands | **doc 02 §1** |
| 3 | **Train in simulation, randomise over *measured* ranges, deploy behind a guardrail** | Atlas (millions of simulated hours, DR over weight/friction/grip), Waymo (billions of simulated miles) | **theory 05 §4.2, doc 05 R-02** |
| 4 | **Reduce the reality gap in hardware, not only in software** | Atlas (two actuator types, no joint cables, rotary actuators) | **§9.2 action item — our mechanical simplicity is an asset** |
| 5 | **Redundant, multiply-modality sensing, and never a single point of failure** | Waymo (camera + lidar + radar + audio, dual compute engines), ANYmal X (visual + thermal + acoustic + gas) | **doc 02 §2, FR-09, SF-01** |
| 6 | **A data flywheel: operate, mine failures, retrain, re-validate, redeploy** | Waymo (Critic, inner and outer loops), π\*0.6 (experience + corrections) | **theory 05 §4, Route C in the career plan** |
| 7 | **A declared operating domain, defended loudly** | Waymo (geofence, ODD, "no hard-stop condition for water"), ANYmal X (Zone 1 boundaries), Perseverance (three-sol plans) | **doc 01 §5, doc 02 §6** |

**Pattern 1 is the headline.** It is not a coincidence, a fashion, or a vendor preference. It is the most common answer the field has found to a hard question: how can a learned component be admitted to a safety-relevant loop without a correctness proof for the component itself? The most advanced deployed system in the world arrived at the same answer independently. That is consistent with our choice. It is not evidence that our choice performs; that requires our own measurements.

**And the counter-pattern is equally clear.** Across the six systems, capability is strong and public evidence is thin. Among those reviewed, Waymo is the only one that describes a safety-case method, and it is still accumulating federal recalls. ANYmal X is the only certified one reviewed here; public descriptions do not indicate learned components in its certified functions. We found no published, complete, general evidence pipeline for a learning-enabled safety function. That is consistent with the gap doc 07 selects; the search was limited, so the claim should be re-checked before publication.

## 9. What this changes in our project

### 9.1 Keep — now with external justification

1. **Topology 2 + safety filter** (theory 06). Waymo's onboard validation layer is the industrial version of the same decision.
2. **Rate-separated layers** (doc 02 §1). Every advanced system separates deliberation from reflex.
3. **Fail loudly, degrade in stages** (doc 02 §6). Waymo's failures are dominated by *undeclared* conditions, which is what the degradation ladder and capability declaration exist to prevent.
4. **Measure latency; never assume it** (PR-06). Industry treats latency as a control problem (Real-Time Chunking).

### 9.2 Change — three concrete revisions

1. **Elevate E1/E2 (capability declaration + bounded-autonomy monitor) to a first-class architectural component.** One of Waymo's most serious failure modes is a specification gap: the flood was *seen*, the system slowed, and no declared rule required a stop. A system that detects a hazard and lacks the authority to declare it out-of-scope is the failure we must design against. Concretely: any condition outside the declared envelope must **force a bounded response regardless of what the policy or planner prefers** — and must log the event. (Feeds doc 01's evidence requirement class and doc 02's missing module.)
2. **Treat localisation uncertainty as the primary conservatism lever** (doc 03, theory 04 §5). Perseverance's 26.91 m uncertainty circle closing a corridor to nothing is our own R-04 deadlock risk, quantified. The lesson is to attack margins at the *estimation* source rather than by making the safety filter more permissive.
3. **Adopt Boston Dynamics' hardware-simplicity principle explicitly.** Our platform's minimal mechanism (two identical motors, no suspension, no cable routing across joints) is a documented control and reproducibility advantage, and it should be stated in doc 00's scope rationale rather than left implicit. It is why a small robot can produce trustworthy sim-to-real numbers.

### 9.3 Avoid — patterns we deliberately do not copy

1. **Foundation-model-first design.** π/Helix/Gemini generality costs data we cannot generate and compute we cannot power. Our learned components stay small, bounded and residual (theory 05 §2).
2. **Volume as a proxy for capability.** Chinese shipment figures are production, not deployment; 70% of one leader's revenue is research and education. Shipped ≠ working.
3. **Vendor-reported performance as evidence.** 99.9% task success, 200 million miles, "small sim-to-real gap" — all claims. Our contribution is partly *about* producing auditable evidence, so our own numbers must meet a stricter standard than the ones we cite.
4. **Saturated benchmarks.** LIBERO returning 90–95% for policies spanning the full real-world range is a direct warning about simulation-only evaluation. Every claim needs a real-hardware counterpart or an explicit statement that it does not have one.

### 9.4 The positioning statement, revised

> The frontier has made large progress on **capability** and is visibly struggling with **evidence**. Waymo has 200+ million autonomous miles, federal reporting obligations and recent recalls, and its public record still shows failures in the categories above. ANYmal X achieved Zone 1 certification; public descriptions do not indicate learned components in its certified functions. We found no published, complete evidence pipeline for a learning-enabled safety function at a scale a small team can execute.
>
> **We are building exactly that: the smallest possible complete example.** Our contribution is not better autonomy; it is a *demonstrated, measured, honestly-bounded evidence pipeline* — E1 through E6 from doc 07 — on a system small enough that every claim can be verified. If it works at 1/10,000th the scale, the methodology is the transferable artifact.

## 10. Sources and quality notes

| Claim | Source | Quality |
| --- | --- | --- |
| Waymo Foundation Model, Driver/Simulator/Critic, independent onboard validation layer, safety framework | [Waymo engineering blog, Dec 2025](https://waymo.com/blog/2025/12/demonstrably-safe-ai-for-autonomous-driving/) | **Primary, vendor.** Self-reported architecture; consistent across posts |
| "You can't build trust with a black box"; validation layer against physics constraints and traffic laws; 200M+ miles; data flywheel | [Waymo, 10 AI Lessons, Aug 2026](https://waymo.com/blog/2026/08/10ailessons/) | **Primary, vendor** — but specific, falsifiable and consistent with independent reporting |
| Safety framework/safety case, absence of unreasonable risk, RAND study | [Waymo Safety Impact](https://waymo.com/safety/impact/) | **Primary, vendor.** Methodological claims |
| 6th-gen suite (13 cameras, 4 lidar, 6 radar; 2024 announcement); 2026 post: 17 MP imagers, fewer than half the cameras of the 5th generation | [Waymo 6th-gen announcement](https://waymo.com/blog/2024/08/meet-the-6th-generation-waymo-driver/), [Feb 2026 operating post](https://waymo.com/blog/2026/02/ro-on-6th-gen-waymo-driver/) | **Primary, vendor** |
| 5 nm ASIC >1,000 TOPS; two independent compute engines; 20× compute scaling | [Robotics & Automation News, Aug 2026](https://roboticsandautomationnews.com/2026/08/21/waymo-reveals-nvidia-powered-compute-system-behind-its-robotaxis/104373/) | **Trade press** reporting vendor disclosure |
| Flood recall (3,791 vehicles), NHTSA filing, no permanent remedy | [NHTSA Part 573 report 26E026](https://static.nhtsa.gov/odi/rcl/2026/RCLRPT-26E026-6527.pdf) | **Primary, federal regulatory filing** — highest quality source in this document |
| Flood patch failure, service halted in five cities, lidar/radar cannot measure depth, "no hard-stop condition" | [TechTimes, May 2026](https://www.techtimes.com/articles/317003/20260522/waymo-halts-service-five-cities-flood-patch-fails-weeks-after-recall-permanent-fix-missing.htm), [CNN investigation](https://www.cnn.com/us/waymo-robotaxis-safety-invs) | **Journalism** with named expert sources (Georgia Tech); CNN cites the "no hard-stop condition" analysis |
| Construction-zone recall (~3,900 vehicles), 13 incidents, freeway restriction | [CNBC, June 2026](https://www.cnbc.com/2026/06/18/waymo-nhtsa-voluntary-recall-robotaxis-entered-freeway-construction-zones.html) | **Journalism**, based on NHTSA filings |
| Emergency-scene "disturbing trend", "not rare or extreme edge cases", 31 SF incidents | [Axios, July 2026](https://www.axios.com/2026/07/15/waymo-accountability-emergencies-nhtsa), [Wikipedia incident log](https://en.wikipedia.org/wiki/Waymo) | **Journalism** (Axios quotes the NHTSA administrator's letter) + aggregator |
| School bus incidents and investigations; child struck Jan 2026; 2,308 cumulative logged accidents | [Reuters, Mar 2026](https://www.reuters.com/technology/ntsb-says-waymo-robotaxis-illegally-passed-stopped-school-buses-new-incidents-2026-03-03/), [Wikipedia](https://en.wikipedia.org/wiki/Waymo) | **Wire service** (high) + aggregator (verify before quoting) |
| Remote assistance (~70 agents, half in Philippines, advise but do not drive) | [CNN](https://www.cnn.com/us/waymo-robotaxis-safety-invs), [Wikipedia](https://en.wikipedia.org/wiki/Waymo) | **Journalism** / aggregator |
| Atlas 2026 specifications, Hyundai Mobis actuators, Jetson Thor, Gemini Robotics, IP67, battery swap | [Robot specification database](https://www.aparobot.com/robots/atlas-2026), [ROBOTS guide](https://robotsguide.com/robots/atlas) | **Secondary aggregators.** Widely consistent, but *not* a Boston Dynamics datasheet — treat as indicative |
| Whole-body control, proprioception, domain randomisation, hardware-simplicity rationale, 100 lb fridge | [Analysis with quote access](https://www.revolutioninai.com/2026/05/what-is-whole-body-control-robotics-atlas-demo-2026.html) | **Secondary commentary** on Boston Dynamics' own statements |
| Atlas: no official datasheet; unverified figures; pilot-only | [Independent spec review](https://shadabchow.com/blogs/robotics/atlas) | **Secondary**, explicitly notes primary-source gaps — useful as a caution |
| π0.5 architecture and ablations; π\*0.6 RECAP; Knowledge Insulation; RTC; FAST | [π\*0.6 paper](https://www.pi.website/download/pistar06.pdf), [technical walkthrough](https://fusheng-ji.github.io/blog/posts/from-pi0-to-pi07-open-pi-series/), [architecture explainer](https://iotdigitaltwinplm.com/physical-intelligence-pi05-vla-robot-foundation-model-2026/) | **Primary (vendor paper)** + technical secondary. The paper is the best source; the ablations are quantitatively specific |
| Helix dual-system (7B VLM 7–9 Hz / 200 Hz S1), 500 h teleop, Helix 2.5 | [Turing Post analysis](https://www.turingpost.com/p/vlaplus), [LeRobot overview](https://learnopencv.com/vision-language-action-models-lerobot-policy/) | **Secondary.** Consistent across sources; vendor blog not directly retrieved |
| "Helix lacks explicit long-term world models"; teleop dependence | [Turing Post](https://www.turingpost.com/p/vlaplus) | **Secondary analysis** |
| LIBERO saturation (90–95% across policies of very different real-world quality); sim transfers for locomotion/navigation, poorly for contact | [Sim2real status](https://objectways.com/blog/why-the-sim2real-gap-is-still-open-in-2026/) | **Industry analysis** citing a study; the finding is important enough to verify against the primary work before publication |
| ANYmal X ATEX/IECEx Zone 1 IIB; only three Zone 1-certified robots; deployments; ANYdrive composition | [Procurement analysis](https://ifactoryapp.com/industries/oil-and-gas/atex-robot-oil-gas-zone-1-zone-2-petchem), [platform review](https://antonrobots.com/anybotics-anymal-review/), [Baidu wiki](https://baike.baidu.com/en/item/ANYmal/2406921) | **Industry/aggregator.** The Zone-1-only-three claim is specific and repeated; verify against ANYbotics' own certification documents before citing |
| Yokogawa partnership | [ARC](https://www.arcweb.com/blog/yokogawa-anybotics-integrate-oprex-robot-management-core-software-anymal-robotic-inspection), [Robotics & Automation News](https://roboticsandautomationnews.com/2026/02/10/yokogawa-and-anybotics-partner-to-integrate-robot-management-software-with-anymal-inspection-robots/98844/) | **Trade press** (primary-ish; quotes the CEO) |
| Perseverance AutoNav 88%; uncertainty growth to ~33 m; sol 383–385 fault, 26.91 m radius, 14 m; Mars Global Localization (2 min, 25 cm mode, 1.07 m max, 264 stops); HBS co-processor 100× faster, 25 damaged bits | [JPL paper, IEEE Aerospace](https://robotics.jpl.nasa.gov/media/documents/2024_Global_Localization_IEEE_Aero.pdf), [NASA/Astrobiology](https://astrobiology.com/2026/02/perseverance-mars-rover-now-autonomously-pinpoints-its-location-using-generative-ai.html), [The Register](https://www.theregister.com/2026/02/23/perseverance_rover_soc_navigation_upgrade/) | **Primary (peer-reviewed JPL paper)** + NASA-derived reporting. Highest-quality technical source in this document |
| AI-generated waypoints (456 m, Dec 2025) | [Universe Today](https://www.universetoday.com/articles/nasa-let-ai-drive-the-perseverance-rover-for-two-days) | **Journalism** |
| Chinese units shipped, prices, MIIT projections, Unitree 70% research/education revenue, UBTECH shipment slippage | [Market analysis](https://pandaily.com/china-humanoid-robot-2026-production-boom-vs-deployment-reality-jul2026), [production picture](https://theresarobotforthat.com/blog/humanoid-robot-in-china/), [market deep dive](https://emag.directindustry.com/2026/03/17/china-humanoid-robots-market-unitree-robotics-agibot-ubtech-leju-xpeng/) | **Industry analysis and trade press**, citing Counterpoint, Omdia, MIR DATABANK. Figures diverge between analysts (Unitree's own claim differs from Omdia's) — report ranges, not point values |
| AgiBot 99.9% task success, 310 units/hour | Vendor claim via [industry analysis](https://ifactoryapp.com/industries/manufacturing-plant/chinese-humanoid-manufacturing-agibot-xpeng-ubtech-byd) | **Vendor marketing.** Unaudited — cite as a claim only |
| HEIS 2026 (MIIT humanoid standard system, Feb 2026): six pillars; safety pillar covers mechanical, electrical and functional safety, emergency stop and fault diagnosis; group standards first | [RobotToday analysis, Mar 2026](https://robottoday.com/article/china-s-humanoid-robot-and-embodied-intelligence-standard-system-heis-2026); [Robotics & Automation News, 31 Mar 2026](https://roboticsandautomationnews.com/2026/03/31/why-chinas-new-humanoid-robot-standards-could-change-the-industry/100263/); [China Minutes, 2 Mar 2026](https://chinaminutes.com/2026/03/02/china-releases-first-national-standard-for-humanoid-robotics/) | **Trade press and analysis.** The standard text was not accessed; these are summaries |
| Unitree G1 specifications (height, mass, DoF, sensing, arm payload, battery, price); advice to keep a safe distance from people | [Unitree G1 product page](https://www.unitree.com/g1) | **Vendor, primary for specifications.** Footnotes say some functions are still in development |
| Unitree G1 emergency stop: remote command to damping mode; the robot falls slowly | [G1 user manual, third-party mirror](https://manuals.plus/m/2e3be6247b40afbe71cf65ec0b1d010457c18c69d878694ae6f7304735dfd4df); [community controls documentation, firmware V1.0.4](https://docs.quadruped.de/projects/g1/html/operation_1.4.html) | **Vendor manual, mirrored.** Button mapping differs by firmware (L1+A in V1.x; L2+B in V1.0.4) |
| Unitree RL training stack: Isaac Gym training, MuJoCo sim-to-sim, deployment code; supports Go2, H1, H1_2 and G1 | [unitree_rl_gym repository](https://github.com/unitreerobotics/unitree_rl_gym) | **Vendor code, primary.** Open source; performance claims not checked |
| GO-1 architecture: InternVL2.5-2B backbone, latent planner over discrete latent action tokens, diffusion action expert producing control signals; 30% average gain over Open X-Embodiment pre-training | [AgiBot World Colosseo, arXiv 2503.06669](https://arxiv.org/pdf/2503.06669) | **Vendor-authored paper, primary.** Results are the authors' own evaluation |
| GO-1 parameter count (about 3 B); AgiBot World scale (over 1 M trajectories, 217 tasks); weight licence (CC BY-NC-SA) | [GO-1 summary page](https://www.ay-robots.com/arena/go-1) | **Secondary.** Check the model card before quoting numbers |
| UBTECH Walker S2 specifications (176 cm, 70–73 kg, 52 DoF, 15 kg payload, 2 m/s, about 3-minute autonomous battery swap); sensing; customer names | [Robotlar specification page](https://www.robotlar.org/en/humanoid-robots/ubtech-walker-s2); [robotsourced listing](https://robotsourced.com/robots/humanoids/ubtech-walker-s2/); [Livium listing](https://livium.com/humanoids/walker-s2) | **Secondary aggregators.** Mass and price conflict between listings. The CE claim and customer names are unverified |

**Discipline note.** This document cites vendor blogs, trade press and aggregators because that is what exists for closed commercial systems; only three sources here are primary and independent (the NHTSA filing, the JPL paper, and the π\*0.6 paper). Where a number comes from marketing, it is labelled. Our project's credibility rests on evidence discipline, so we cannot be careless about other people's evidence — especially when the central argument of doc 07 is that the field is careless about evidence.
