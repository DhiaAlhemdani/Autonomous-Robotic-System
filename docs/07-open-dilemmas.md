# 07 — The Open Dilemmas: What Is Actually Unsolved

> **Read this before docs 00–06.** This document establishes *which problem the system attacks*. The mission statement in doc 00, the requirements in doc 01 and the architecture in doc 02 are all derived from the choice made here. It is numbered last only to preserve existing document identifiers.
>
> **Status:** research orientation, October 2026. Every claim about the state of the field is sourced in §7, with a quality note on each source. Nothing in this document is a claim about our own system.

---

## 1. How to judge a dilemma (the filter)

A dilemma is worth attacking only if it passes all five tests. Most "big problems in robotics" fail at least two.

| Test | Question | Why it eliminates candidates |
| --- | --- | --- |
| **T1 — Real** | Do practitioners hit this in deployment, not just in papers? | Many "open problems" are academic artifacts that nobody deploying a robot actually encounters |
| **T2 — Genuinely unsolved** | Is it unsolved, or merely hard and expensive? | "Engineering effort" problems are not dilemmas; a funded team will solve them without a new idea |
| **T3 — Consequential** | Does something outside research depend on it — a deadline, a regulation, a market, a safety case? | Consequence is what turns a project into a contribution; without it, results are unverifiable opinions |
| **T4 — Tractable at our scale** | Can one person with a small robot and $0–3,000 produce evidence that a competent reviewer takes seriously? | This is the test that kills most grand challenges, including every manipulation problem on this list |
| **T5 — Startable now** | Does meaningful progress require hardware, or can the theoretical core be built immediately? | Determines whether the current phase can be productive at all |

## 2. The dilemma taxonomy

Eighteen dilemmas, grouped into four classes. Each entry states the dilemma, the evidence it is unsolved, and an honest T1–T5 assessment.

### Class A — Assurance: "you can demonstrate it, but you cannot prove it"

This is the class that matters most for our project, and it is the least crowded.

---

**D1 — The certification gap for learning-enabled safety components** ⭐

**The dilemma.** A machine learning component that performs a safety function cannot be certified by any method that currently exists at industrial scale — yet operators are, or will shortly be, legally required to place only compliant machinery on the market.

**The evidence.** From **20 January 2027** the EU Machinery Regulation **(EU) 2023/1230** applies, replacing the Machinery Directive 2006/42/EC. Annex I, Part A, items 5 and 6 name *"safety components with fully or partially self-evolving behaviour using machine learning approaches ensuring safety functions"* — and the machinery embedding them — as **high-risk categories requiring third-party Notified Body conformity assessment**. Self-certification is removed for exactly this class. Pre-trained models count: a frozen model is still "self-evolving" on the regulation's capability-based reading because it was trained on data and could be retrained. The regulation demands a declared scope of autonomy, protection of safety functions against corruption, an "intended evolution" statement, logging of software updates, and a change record that can trigger **full re-assessment** on a "substantial modification". Meanwhile the machinery guide sections on AI-based safety functions and autonomous mobile machinery are still in draft, and the research literature is unambiguous that formal verification of neural networks does not scale to realistic sizes. ([EUR-Lex summary](https://regulations.ai/regulations/RAI-EU-NA-MACHREG-2023), [AI-compliance analysis](https://inkog.io/labs/eu-machinery-regulation-ai-agents), [compliance guide](https://digital.nemko.com/regulations/eu-machinery-regulation), [industry analysis](https://maintworld.com/asset-management/machinery-regulation-2027-the-clock-is-ticking-for-safety-compliance-and-competitive-edge/))

**Why it is hard.** The requirement is not "make the AI better" — it is "produce evidence that a reviewer accepts". Evidence-generation for a learned component is unsolved because: the specification is often informal ("behave safely near humans"); the input space is unbounded; exhaustive testing is impossible; formal methods do not scale; and the behaviour can change with an update, invalidating prior evidence.

**T1 Real** ✓ (deployers already face this; the deadline is fixed) · **T2 Unsolved** ✓ (no accepted method; guidance still in draft) · **T3 Consequential** ✓✓ (legal deadline, market access, liability — "substantial modification" can make the *user* the legal manufacturer) · **T4 Tractable** ✓ **(see §4)** · **T5 Laptop-free** ✓✓ (this is a methodology and documentation problem)

---

**D2 — Runtime assurance rests on preconditions nobody checks**

**The dilemma.** The dominant architecture for admitting learned controllers into safety-relevant systems is *runtime assurance*: pair an untrusted learned policy with a verified fallback and a monitor that switches when the state approaches the boundary of a safe region. Its safety guarantee is stated as if the monitor's estimation error is zero or benignly stochastic. In real deployments — with sensor faults, latency, or an adversary corrupting measurements — that assumption is simply not verified.

**The evidence.** A 2026 result states and proves the missing condition, calling it the *assurance precondition*: the guarantee survives an adversary controlling *q* measurement channels **if and only if** the plant is *2q*-sparse observable with respect to the safety-relevant output. Crucially, when the precondition is violated the guarantee **fails discontinuously rather than degrading** — and the condition is a comparison between the plant *and the noise the deployment actually has*, not a property of the plant alone. Separately, 2026 work on neuromorphic safety monitors measures detection latency and false-negative rates empirically and derives a probabilistic bound on closed-loop failure risk — an *auditable methodology* — which is precisely what practice lacks. ([Assurance precondition paper](https://arxiv.org/abs/2610.00285), [monitor characterisation](https://www.sciencedirect.com/science/article/pii/S0925231226006120))

**Why it is hard.** It converts a systems-architecture question into a question about what the monitor can *actually observe* under degradation — which most designs never measure.

**T1 Real** ✓ (anyone deploying Simplex-style architecture) · **T2 Unsolved** ✓ (2026 is the first year this is being formalised) · **T3 Consequential** ✓ · **T4 Tractable** ✓ (small-scale measurement studies are genuinely useful here) · **T5 Laptop-free** ✓ (derivation and protocol design)

---

**D3 — Formal verification of neural components does not scale**

**The dilemma.** Sound and complete certification of ReLU networks does not scale to large networks; incomplete methods produce bounds that loosen with depth. Closed-loop safety proofs for DNN-controlled systems exist but rely on approximation or abstraction and must be redone for every new system.

**The evidence.** Systematic literature reviews report scalability as the common limitation of formal verification approaches, with most methods restricted to specific network structures and sizes. Reachability-based methods become intractable over long task horizons because over-approximation error compounds across control periods. ([Verification SLR](https://www.sciencedirect.com/science/article/pii/S0950584920300471), [neural certificates](https://link.springer.com/chapter/10.1007/978-3-032-05435-7_4))

**Why it is hard.** The trade-off is fundamental: expressiveness of the system versus tractability of the proof. No known technique breaks it.

**T1 Real** ~ (real for certification, less so for daily engineering) · **T2 Unsolved** ✓ · **T3 Consequential** ✓ · **T4 Tractable** ✗✗ (an active math research area; not solo-workable) · **T5 Laptop-free** ✗

*Verdict: cite it as the reason D1 exists; do not attack it.*

---

**D4 — Out-of-distribution detection is not safety monitoring**

**The dilemma.** The common safety pattern is "detect OOD input → abstain or fall back". But an OOD input does not necessarily imply a safety violation, and an in-distribution input does not guarantee safety. Designs that alarm only on OOD are **systematically over-conservative exactly where a correct monitor would stay precise**, and benchmarks built on OOD-only metrics mis-rank methods.

**The evidence.** 2026 work argues safety should be monitored *directly* — by predicting whether future trajectories satisfy a temporal-logic safety specification — using adaptive conformal prediction to obtain a long-run probabilistic guarantee that survives arbitrary distribution shift, with incremental learning to recover precision. Their guarantee is stated honestly as a **long-run average**, so per-step or per-episode bounds need another layer. ([OOD safety monitoring](https://arxiv.org/html/2504.13478))

**Why it is hard.** It forces the question "what specification, exactly?" — a temporal-logic specification must be written, and writing it is the hard part.

**T1 Real** ✓ · **T2 Unsolved** ✓ (the guarantee is long-run, and open review questions remain) · **T3 Consequential** ✓ · **T4 Tractable** ✓ (a monitor can be implemented and evaluated on a small robot) · **T5 Laptop-free** ✓ (specification and protocol)

---

**D5 — Accountability is undefined for learned behaviour**

**The dilemma.** When an autonomous system causes harm, liability is unresolved across manufacturer, deployer, AI training provider and operator. Under the new EU framework, "substantial modification" — including digital changes not foreseen by the original manufacturer — can transfer the role of legal manufacturer to the *user*, with civil and criminal liability attached.

**Assessment.** Consequential (T3 ✓✓) but primarily legal and organisational (T4 ✗). It belongs in our **motivation**, not our scope.

---

### Class B — Capability: "it works in the demo and not in the deployment"

---

**D6 — The sim-to-real gap**

**The dilemma.** Policies trained in simulation degrade on hardware, and the degradation is not uniform — it is severe in specific places and mild in others.

**The evidence.** The gap has **four distinct causes** — visual domain gap, physics approximation error, sensor-noise mismatch, and long-tail scenario absence — each requiring a different mitigation; domain randomisation addresses the first and partly the third, and cannot fix contact physics or enumerate unknown unknowns. One 2026 commercial benchmark reports a **~35% composite performance cliff** with a worst case around **50× on grasp adaptiveness** — but note this is a vendor blog, not peer-reviewed, and should be cited as a claim rather than a consensus figure. A better-supported and more useful finding: simulation now **transfers well for locomotion, navigation and much of perception, and poorly for contact**. And there is a methodological trap — if the simulator gets contact wrong it also gets the *score* wrong, and the benchmark returns a number either way, so nothing looks broken. ([Annual Review on the reality gap](https://www.annualreviews.org/content/journals/10.1146/annurev-control-031924-100130), [sim2real status](https://objectways.com/blog/why-the-sim2real-gap-is-still-open-in-2026/), [vendor benchmark](https://embodiedglobal.com/en/article/sim-to-real-quantified-benchmark-2026))

**Why it is hard.** It is not one problem but four, and the hardest one (long tail) cannot be solved by more simulation.

**T1 Real** ✓✓ · **T2 Unsolved** ✓ (partially — "a problem of engineering detail, not algorithmic breakthrough") · **T3 Consequential** ✓✓ · **T4 Tractable** ~ (measurable on a small robot for **navigation**, which is where simulation transfers well — so our contribution would be a *quantified gap report*, not a solution) · **T5 Laptop-free** ~ (protocol and harness design, yes)

*Verdict: strong as a **secondary** dilemma — the measurement is doable and useful; the solution is not.*

---

**D7 — Long-horizon reliability**

**The dilemma.** Compound error means per-step reliability degrades catastrophically over a task chain: ~95% per step gives roughly 60% success over 10 steps. Systems lose coherence over long horizons; one 2026 analysis notes no demonstrated system reliably executes chains of 10+ distinct subtasks in genuinely unstructured environments without human intervention, teleoperation or demonstration cherry-picking.

**Why it is hard.** It is partly a *memory and state-estimation* problem (policies without memory repeat failed actions indefinitely; adding memory increases compute and can degrade performance through distribution shift), and partly an **evaluation** problem — impressive demos are usually short.

**T1 Real** ✓✓ · **T2 Unsolved** ✓✓ · **T3 Consequential** ✓✓ · **T4 Tractable** ✗ for manipulation, **✓ for navigation** (a 90-minute autonomous mission with per-event logging *is* a long-horizon test and is within reach) · **T5 Laptop-free** ✓

---

**D8 — The 99% → 99.9% cliff**

**The dilemma.** The last 0.9% is not more of the same work. A warehouse robot at 95% accuracy fails roughly 50 times per day — constant intervention that defeats the economics of automation — while production environments need ~99.9%. Going from 80% to 99.9% "requires fundamentally different approaches… and takes longer than most timelines assume".

**Why it is hard.** Rare events matter more than common ones in safety-critical domains, and they are by construction sparse in the data.

**T1 Real** ✓✓ · **T2 Unsolved** ✓ (structural) · **T3 Consequential** ✓✓ · **T4 Tractable** ✗ as a solution; **✓ as a measurement** (reporting a degradation curve and failure taxonomy rather than an average)

---

**D9 — Generalization to the open world and the long tail**

**The dilemma.** Models trained for one environment do not transfer to another; rare events are too sparse for ordinary imitation learning and too diverse for fixed scenario suites. Vision-language-action models inherit semantic generality but fail on tasks outside the training set — "if asked to perform a task that falls outside its training set, it's highly likely to fail" — and performance drops sharply on even minor disturbances such as lighting changes.

**T1 Real** ✓✓ · **T2 Unsolved** ✓✓ · **T3 Consequential** ✓✓ · **T4 Tractable** ✗ (needs data and scale beyond a single project)

---

**D10 — Data scarcity**

**The dilemma.** Robot data cannot be scraped or purchased from a broker; it must be generated task by task and environment by environment. Teleoperation is expensive and does not scale as a deployment model. Industry robotic-data spend is projected to exceed **$3 billion** over two years. Simulated data helps but generated video "looking right" does not mean it "behaves right" — policies trained on world-model-generated trajectories scored far below the same policy trained on real demonstrations.

**T1 Real** ✓✓ · **T2 Unsolved** ✓✓ · **T3 Consequential** ✓✓ · **T4 Tractable** ✗

---

**D11 — Continual learning without forgetting**

**The dilemma.** Long-term autonomous operation requires adaptation, but networks suffer catastrophic forgetting; "embodiment drift" — actuators degrading through wear — requires constant retraining, making deployment costly.

**T1 Real** ✓ · **T2 Unsolved** ✓ · **T3 Consequential** ✓✓ (it is also a *certification* problem: retraining invalidates evidence — links to D1) · **T4 Tractable** ~

---

### Class C — Physical and embedded: "it does not fit on the robot"

---

**D12 — The onboard compute and latency envelope** ⭐

**The dilemma.** Advanced autonomy needs more computation than a mobile robot can carry, power, and cool. Large models need 50–100 ms per inference on edge hardware — enabling only 10–20 Hz control — while physical control tasks need 20–100 Hz. There is a "fundamental mismatch between what the AI can think and how fast the robot needs to move."

**Why it is hard.** It is a genuine multi-objective constraint: accuracy, latency, energy, thermals and cost trade against each other on a fixed power budget. And the *degradation* behaviour is the dangerous part: when compute load is constrained, models truncate reasoning, skip verification passes, or lower sensory fidelity — directly removing safety-relevant checks. Resource pressure does not degrade performance gracefully; it removes safety features.

**T1 Real** ✓✓ · **T2 Unsolved** ✓ (structural; every generation of models re-opens it) · **T3 Consequential** ✓✓ · **T4 Tractable** ✓✓ (**instrumented measurement is exactly what a solo project can do well**) · **T5 Laptop-free** ✓ (budget derivation and allocation policy)

---

**D13 — Energy autonomy and graceful failure**

**The dilemma.** Foundation models are energy-intensive; sustained perception and reasoning deplete onboard power, making frequent recharging impractical. And "the other unsolved challenge is graceful failure — at scale, the goal isn't just a system that performs; it's a system that knows exactly what to do when it can't."

**T1 Real** ✓✓ · **T2 Unsolved** ✓ · **T3 Consequential** ✓✓ · **T4 Tractable** ✓ as an engineering discipline (budget, measure, degrade)

---

**D14 — The tactile and contact-sensing gap**

**The dilemma.** Vision cannot detect slip, measure insertion force, or sense contact geometry. "A robot can't just see; it has to feel pressure and tension to handle objects reliably." Tactile data is genuinely absent from most stacks.

**Assessment.** Real and important **for manipulation** — and **irrelevant to a mobile navigation platform**. Listed for completeness and to draw the boundary.

**T4 for us: N/A** (out of scope, explicitly).

---

### Class D — Interaction and security

---

**D15 — Human intent, social navigation, legibility**

**The dilemma.** Robots that share space with people must behave in ways humans can understand, anticipate and trust — not merely avoid collision. Human behaviour is ambiguous and not instrumented; social norms are contextual.

**T1 Real** ✓✓ · **T2 Unsolved** ✓ (partially) · **T3 Consequential** ✓✓ · **T4 Tractable** ✓✓ (socially-aware navigation on a small robot in simulation with scripted human agents is a legitimate, well-bounded study) · **T5 Laptop-free** ✓

---

**D16 — Adversarial robustness and cybersecurity of safety functions**

**The dilemma.** Physical and sensor-level attacks (adversarial patches, spoofed sensors, corrupted model files) can produce hazardous behaviour. Robot-specific threat models differ from standard computer-vision benchmarks: an attacker must sustain a perturbation across frames, which creates temporal signatures a detector can exploit — but defences that work in static settings **fail under continuous robot perception**. The new EU framework makes this a *safety* requirement, not an IT concern: safety functions must be protected against corruption, and "reasonably foreseeable malicious influence" is now a risk category.

**T1 Real** ✓ · **T2 Unsolved** ✓ · **T3 Consequential** ✓✓ (regulatory) · **T4 Tractable** ✓ (a bounded threat model plus measured defences is feasible) · **T5 Laptop-free** ✓

---

## 3. The scoring matrix

Scores: ✓✓ strong, ✓ adequate, ✗ fails, ~ partial. **Bold** = the six candidates that pass T4.

| ID | Dilemma | T1 Real | T2 Unsolved | T3 Conseq. | T4 Tractable | T5 Laptop-free | Verdict |
| --- | --- | :-: | :-: | :-: | :-: | :-: | --- |
| **D1** | **Certification gap for learning-enabled safety** | ✓✓ | ✓✓ | ✓✓ | ✓ | ✓✓ | **Primary candidate** |
| **D2** | **Runtime-assurance preconditions unverified** | ✓ | ✓✓ | ✓ | ✓ | ✓✓ | **Primary candidate** |
| **D4** | **OOD detection ≠ safety monitoring** | ✓ | ✓✓ | ✓ | ✓ | ✓✓ | **Supporting** |
| **D12** | **Onboard compute/latency envelope** | ✓✓ | ✓ | ✓✓ | ✓✓ | ✓ | **Supporting** |
| **D15** | **Human intent / social navigation** | ✓✓ | ✓ | ✓✓ | ✓✓ | ✓ | Supporting |
| **D6** | **Sim-to-real** | ✓✓ | ✓ | ✓✓ | ~ | ~ | Supporting (measurement only) |
| D16 | Adversarial / cyber-physical attacks | ✓ | ✓ | ✓✓ | ✓ | ✓ | Optional |
| D8 | 99% → 99.9% cliff | ✓✓ | ✓ | ✓✓ | ✗ | ✓ | Out of reach as a solution |
| D7 | Long-horizon reliability | ✓✓ | ✓✓ | ✓✓ | ✗/✓ | ✓ | Navigation narrow case only |
| D9 | Open-world generalization | ✓✓ | ✓✓ | ✓✓ | ✗ | ✗ | Out of scope |
| D10 | Data scarcity | ✓✓ | ✓✓ | ✓✓ | ✗ | ✗ | Out of scope |
| D13 | Energy autonomy | ✓✓ | ✓ | ✓✓ | ✓ | ✓ | Secondary constraint |
| D11 | Continual learning w/o forgetting | ✓ | ✓ | ✓✓ | ~ | ✗ | Out of scope |
| D3 | Formal verification scaling | ~ | ✓ | ✓ | ✗✗ | ✗ | Cite, don't attack |
| D5 | Accountability / liability | ✓ | ✓ | ✓✓ | ✗ | ✓ | Motivation only |
| D14 | Tactile / contact gap | ✓✓ | ✓✓ | ✓✓ | N/A | N/A | Excluded by scope |

## 4. The recommendation: attack the evidence gap

### 4.1 The single dilemma

> **An organisation building an autonomous machine with a learning-enabled component must produce evidence that a third-party reviewer accepts — and no accepted method for producing that evidence exists.**

This is D1, with D2 and D4 as the technical mechanisms that make it hard and that we can actually study:

- **D1** says: *you must prove it, and there is no accepted way to prove it, and from 20 January 2027 the law says so.*
- **D2** says: *the runtime-assurance architecture everyone relies on assumes a precondition nobody measures.*
- **D4** says: *the monitoring most people build checks the wrong thing.*

### 4.2 Why this dilemma and not the others

| Reason | Detail |
| --- | --- |
| **It is the only candidate with a legal deadline** | 20 Jan 2027 for the Machinery Regulation; embedded high-risk AI obligations follow on 2 Aug 2028. |
| **It is the one nobody has solved** | The manipulation and data dilemmas are attacked by well-funded labs. The assurance problem is attacked by almost nobody with hands-on experience, because it sits between robotics, control theory, and regulatory practice |
| **It passes T4** | A solo project cannot certify anything — but it **can** build and measure a reference implementation of the evidence pipeline and publish where it fails. That is a real contribution at this scale |
| **It is laptop-free for the first phase** | This entire dilemma is a methodology, specification and measurement-protocol problem. Zero hardware required to make real progress |
| **It matches our existing work** | The architecture in doc 06 — RL above MPC, CBF-QP safety filter, supervisor, degradation ladder — *is* a runtime-assurance architecture. We are not starting from nothing; we are changing the question we ask of it |
| **It matches our demonstrated strength** | The graduation project's limitations and evidence-boundary discipline is precisely the skill this problem demands. Most graduates cannot do it; we have already done it once |
| **It is what the market is about to need** | Assurance and safety engineering for AI-enabled machinery is a discipline being created right now by the regulation |

### 4.3 The contribution claim, stated honestly

**What we will claim:**

> A reference architecture and a measurement methodology for generating the *evidence artifacts* that a conformity assessment for a learning-enabled safety component will require, demonstrated end-to-end on a laboratory mobile robot, with published measurements of where the methodology succeeds and where it fails.

**What we will not claim:**

- That we certify anything, or that the system is compliant or certified.
- That the safety filter's guarantees hold outside their stated assumptions.
- That the monitors detect what they claim to detect, unless measured.
- That the methodology would satisfy a Notified Body — only that it produces the artifacts the assessment is expected to ask for, based on the regulation's text as currently written and the guidance as currently drafted.

### 4.4 What the evidence pipeline must produce (the deliverable list)

This is the concrete answer to "we must prove it". Six artifacts, each testable and each generated by the system rather than written by hand:

| # | Artifact | What it answers | How it is generated |
| --- | --- | --- | --- |
| **E1** | **Capability declaration** | What can this system do, inside what envelope, and what is it forbidden to do? | A machine-readable declaration (envelope limits, permitted behaviours, prohibited actions) that is *loaded and enforced* by the runtime — plus a check that no code path can exceed it |
| **E2** | **Bounded-autonomy monitor** | Is the learned component operating inside the declared envelope right now? | Runtime check against E1, with an out-of-envelope state that bounds authority and logs the event |
| **E3** | **Measured monitor characterisation** | What does the safety monitor actually detect, with what latency and what false-negative rate? | Injection tests producing detection-latency distributions and FNR/FPR — *measured, not asserted* (answers D2's core criticism) |
| **E4** | **Traceability register** | Which claim rests on which requirement, which test, and which run? | Generated from requirement IDs → test IDs → run metadata; a claim without a run is a build failure |
| **E5** | **Change-and-revalidation record** | What changed, and does prior evidence survive it? | Every model swap, policy update, parameter or code change emits a record classifying whether it invalidates prior evidence — directly implementing "substantial modification" and "intended evolution" |
| **E6** | **Safety case** | Why should a reviewer accept this? | Assembled largely from E1–E5, with residual risks and explicit limitations stated |

**E5 is the piece almost nobody builds** and the one the regulation makes unavoidable: a system whose safety evidence cannot survive a model update cannot be maintained. Automating the "does this change invalidate my evidence?" decision is a genuinely novel, tractable contribution.

### 4.5 What this change means downstream

Adopting this dilemma is not cosmetic — it redirects the specification:

| Document | Change required |
| --- | --- |
| `00-mission-and-scope` | Mission is reframed from "an autonomous robot" to "a demonstrably-assured autonomous robot"; the frontier-problem list is replaced by the D1/D2/D4 statement |
| `01-requirements` | A new requirement class is needed: **evidence requirements** (E1–E6), each with a verification method |
| `02-architecture` | Two components are missing: a **capability-declaration/enforcement** module and a **change-revalidation** service |
| `03-simulation-and-docker` | Becomes more central, not less: reproducibility is now *evidence*, and a dirty working tree invalidating a claim is a traceability requirement, not a hygiene preference |
| `04-build-roadmap` | A milestone is missing: **E-pipeline** must exist before the learning layer is deployed on hardware, not after |
| `08-safety-and-verification` | Promoted from supporting document to the project's spine |
| Theory `06` | Elevated: the CBF shield is no longer just a safety mechanism, it is the *object under assessment* |

## 5. What we explicitly do not attack

Naming these is as important as naming the target. A reader trusts a project that states its boundaries.

- **Manipulation, dexterity, tactile sensing (D14)** — different body, different physics, and out of reach at this scale.
- **Open-world generalization and data scarcity (D9, D10)** — requires data and compute beyond a single project.
- **Formal verification scaling (D3)** — an active mathematics research area; we cite it as the *reason* D1 exists.
- **The 99% → 99.9% climb (D8) as a solution** — we measure and report a degradation curve; we do not claim to close it.
- **Humanoid locomotion, VLA scaling, world models** — all depend on resources we do not have, and none of them is where a solo contribution is credible.

## 6. The one-sentence positioning

> We are not building a robot that navigates well. We are building **a robot that can produce evidence that it navigates safely, and a methodology for what that evidence must be** — and we are doing it in the exact form the incoming EU rules will demand, on a system small enough that every claim can be measured.

That sentence is the project's north star and the reason a reviewer should care.

## 7. Sources and quality notes

| Claim | Source | Quality |
| --- | --- | --- |
| Machinery Regulation 2023/1230 applies 20 Jan 2027; Annex I Part A items 5–6 = ML safety components; Notified Body required | [Regulation summary](https://regulations.ai/regulations/RAI-EU-NA-MACHREG-2023), [compliance guide](https://digital.nemko.com/regulations/eu-machinery-regulation) | **Primary/regulatory.** The regulation text is authoritative; verify against EUR-Lex before quoting in the safety case |
| Declared autonomy, corruption protection, intended evolution, change records, re-assessment triggers | [AI-compliance analysis](https://inkog.io/labs/eu-machinery-regulation-ai-agents), [industry analysis](https://maintworld.com/asset-management/machinery-regulation-2027-the-clock-is-ticking-for-safety-compliance-and-competitive-edge/) | **Secondary, industry analysis.** Consistent across sources; the code-level "requirements" are interpretation, not regulation text — treat as expected assessment content, not law |
| AI Act timeline (2 Dec 2027 stand-alone high-risk; 2 Aug 2028 embedded high-risk) | [compliance guide](https://genow.ai/en/blog/eu-ai-act-manufacturing-guide/) | **Secondary.** Verify dates against official sources |
| Runtime-assurance precondition (2q-sparse observability), discontinuous failure | [arXiv 2610.00285](https://arxiv.org/abs/2610.00285) | **Preprint.** Not peer-reviewed; treat as a strong claim requiring separate verification |
| Measured monitor characterisation → probabilistic bound on failure risk | [Neurocomputing article](https://www.sciencedirect.com/science/article/pii/S0925231226006120) | **Peer-reviewed journal** |
| OOD detection ≠ safety monitoring; ACP long-run guarantee | [arXiv 2504.13478](https://arxiv.org/html/2504.13478) | **Preprint under open review** (review raised a conditional objection about self-inclusive calibration updates — read the referee report before relying on the bound) |
| Formal verification scalability limits | [SLR](https://www.sciencedirect.com/science/article/pii/S0950584920300471), [certificates](https://link.springer.com/chapter/10.1007/978-3-032-05435-7_4) | **Peer-reviewed** |
| Sim-to-real four causes; transfers well for locomotion/navigation, poorly for contact | [Annual Review](https://www.annualreviews.org/content/journals/10.1146/annurev-control-031924-100130), [status article](https://objectways.com/blog/why-the-sim2real-gap-is-still-open-in-2026/) | **Peer-reviewed** (Annual Review) + industry commentary |
| ~35% composite sim-to-real cliff, ~50× worst case | [vendor benchmark blog](https://embodiedglobal.com/en/article/sim-to-real-quantified-benchmark-2026) | **Vendor marketing.** Cite as a claim, never as consensus |
| Compound-error arithmetic (95% → ~60% over 10 steps) | [2026 predictions](https://dtsbourg.me/en/articles/predictions-embodied-ai) | **Practitioner forecast.** The arithmetic is checkable; the "unsolved in 2026" claim is a prediction |
| VLA fails outside training set; autonomous planning in new environments largely unsolved | [MIT Technology Review](https://www.technologyreview.com/2026/10/08/1145923/ai-breakthroughs-in-robotics-wont-change-your-life-any-time-soon/) | **Journalism from domain experts** (quotes Agility Robotics' co-founder; dated today) |
| Compute/latency mismatch (50–100 ms inference vs 20–100 Hz need); resource pressure removes safety features | [State of embodied intelligence](https://voxos.ai/blog/embodied-intelligence-robotics-2026/), [MDPI review](https://www.mdpi.com/2218-6581/15/3/55) | Industry analysis + **peer-reviewed review** (the safety-feature-removal finding is from the review) |
| Data costs >$3B; 99% → 99.9% is structural | [Bessemer predictions](https://www.bvp.com/atlas/bessemer-predicts-robotics-and-physical-ai) | **Investor analysis.** Useful framing; not independent evidence |
| Graceful failure as the unsolved at-scale problem | [Robotics Tomorrow interview](https://www.roboticstomorrow.com/article/2026/05/from-demo-to-deployment-where-robotics-actually-fails/26439) | **Practitioner interview** (deployment experience) |
| Robot-specific adversarial threat models differ from static CV | [arXiv digest](https://www.visionforrobotics.com/robotics-digest/digest_2026-03-30.html) | **Paper digest** — secondary; trace to the primary paper before citing |

**Discipline note.** Three of the most quoted numbers in this document come from vendor or investor blogs (the 35% cliff, the $3B data figure, the 50× grasp number). They are included because they frame the industry's perception, and they are **explicitly flagged** because a project whose credibility rests on evidence discipline cannot quietly promote a marketing figure to a fact. When these appear in our own writing, they carry the same label.
