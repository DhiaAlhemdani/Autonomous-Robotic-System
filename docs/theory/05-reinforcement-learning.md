# Theory 05 — Reinforcement Learning for Robot Decision-Making

**Purpose:** specify where learning belongs in this system, formulate the problem correctly, choose the algorithms and training methodology, and define what can and cannot be claimed about a learned policy. This is the component that makes the system "futuristic" — and the one most likely to be implemented badly without this document.

---

## 1. The decisive question: should this be RL at all?

RL is the right tool when the optimal behaviour is difficult to specify analytically but easy to *evaluate*. It is the wrong tool when a well-understood controller exists and the problem is not actually sequential.

| Candidate task | Best tool | Reasoning |
| --- | --- | --- |
| Following a path, avoiding obstacles | **NMPC** (doc 04) | The objective and constraints are explicit; a model exists; MPC is more efficient and inspectable |
| Motor current regulation | **PI / linear MPC** (doc 02) | The plant is nearly linear and the constraints are simple |
| Sensor calibration, object detection | **Supervised deep learning** (doc 07) | There are labels; this is not a sequential decision problem |
| **Fast reflexive collision avoidance with overlapping constraints** | **RL — with a safety shield** | Hard to encode analytically; needs a fast, reactive policy |
| **Behaviour selection / mission sequencing** | **RL (semi-MDP over skills)** | A long-horizon, credit-assignment problem with discrete choices |
| **Speed and clearance policy under uncertainty** | **RL with constrained objective** | The trade-off depends on perception uncertainty, which is hard to model analytically |
| **Recovery from being stuck** | **RL (or a scripted policy set as baseline)** | Genuinely hard to hand-design; benchmark against scripted recovery |
| **Gait / aggressive dynamics** (not this platform) | **RL** | Where model-free methods have genuinely outperformed MPC |
| **Whole navigation end-to-end from pixels** | **RL — research tier only** | Sample cost, sim-to-real gap and non-verifiability make it unsuitable as the primary path |

**Decision:** RL enters at **(a) the mission/behaviour layer** (1–5 Hz, discrete or low-dimensional actions over skills) and **(b) a reflexive safety-adjacent layer** (a fast residual or speed policy), always behind the safety filter of doc 06. It does **not** replace the NMPC trajectory tracker or the drive loops. This is a defensible allocation, and being able to justify it in interview is worth more than having RL somewhere in the stack.

## 2. Problem formulation

### 2.1 MDP

```
M = (S, A, P, R, γ)
S : state          — what the policy observes
A : action         — what the policy decides
P : transition     — environment (Gazebo dynamics + perception, effectively)
R : reward         — the specification of desired behaviour, and the hardest part
γ : discount       — 0.99 typical for long-horizon missions
```

### 2.2 POMDP reality check

The robot never observes the true state: it observes an estimate with error, delay and occlusion. Formally this is a **POMDP**, and treating it as an MDP is an approximation whose consequences must be managed:

- Provide a **history/context window** or recurrent memory (LSTM/GRU or a frame stack) so the policy can in principle infer unobserved state.
- **Model the uncertainty in the observation** rather than hiding it: include covariance, sensor health flags, and staleness indicators in the observation vector so the policy can learn to be conservative when uncertain.
- Recognise that a policy trained on perfect simulation state will usually fail when fed estimated state. **Training with an estimator in the loop (domain-randomised noise, dropout, latency) is mandatory**, not an optimisation.

### 2.3 The specific formulation for this project

**Layer A — mission/behaviour policy (primary)**

| Element | Choice | Justification |
| --- | --- | --- |
| Action space | Discrete over skills: `{advance, explore, inspect, re-plan, back-off, return-to-base}` + 1 continuous speed scalar | A hybrid action space keeps credit assignment tractable; continuous 6-DoF actions over a horizon of minutes do not train reliably |
| Observation | Local costmap patch (e.g. 40×40), goal vector in body frame, velocity, localisation covariance, battery, perception-health vector, time since last event | Everything the policy needs, nothing it cannot get on hardware |
| Reward | `r = progress − collision_penalty − discomfort − effort − time` with **constraint violations as hard penalties** and a small alive bonus | Shaped to the mission; the alive bonus prevents suicidal policies but must be small or it encourages loitering |
| Termination | Collision, leaving the operational envelope, goal reached, timeout | Timeouts must not be rewarded |
| Horizon | `γ = 0.99`, episodes ≈ 60–120 s | Long enough for mission structure, short enough to train |

**Layer B — reflexive speed/residual policy (advanced)**

| Element | Choice | Justification |
| --- | --- | --- |
| Action | A **residual or a single speed-scaling scalar** applied to the NMPC output, bounded so the shield can always intervene | Keeps the learned component's authority small and analysable |
| Observation | Range measurements, NMPC command, predicted stopping distance, human detection confidence | Directly tied to the safety-relevant decision |
| Reward | `−collision − discomfort + progress`, with explicit penalty for near-misses | Teaches anticipatory caution |

The residual formulation has a practical advantage that is often overlooked: a policy that outputs a *correction* to a competent baseline starts from competent behaviour, needs far fewer samples, and cannot catastrophically fail — the worst case is a worse version of the baseline, which the shield then bounds.

## 3. Algorithms

| Algorithm | Type | Use here | Notes |
| --- | --- | --- | --- |
| **PPO** | On-policy policy gradient | **Baseline for Layer A** | Reliable, well understood, robust to hyperparameters; sample-hungry |
| **SAC** | Off-policy, max-entropy | **Baseline for Layer B** (continuous residual)** | Sample-efficient, good exploration, excellent for a low-dimensional continuous action |
| **TD3** | Off-policy deterministic | Comparison to SAC | Sometimes better in deterministic domains; include a deterministic-axis baseline |
| **Recurrent PPO** | On-policy + memory | Layer A if the observation is genuinely partially observed | Complexity cost: harder to debug and to warm-start |
| **Dreamer / model-based (e.g. DreamerV3)** | Learned latent world model | **Advanced tier** | Orders-of-magnitude better sample efficiency — decisive when the simulator is slow. Risk: world-model exploitation (the policy finds the model's flaws instead of solving the task) |
| **Offline RL** (CQL/IQL) | From a fixed dataset | **Research tier** | Attractive because it can use logged runs without an interactive simulator; performance is capped by the dataset |

**Decision:** SAC for the continuous residual layer and PPO for the discrete mission layer, with a model-based method (Dreamer-style) as the advanced-tier experiment. Every algorithm must be run with **≥ 5 seeds** and reported as mean ± spread — single-seed RL results are anecdotal and should be treated as such.

## 4. Curriculum, environments and reward design

### 4.1 Curriculum (essential for this problem)

```
Stage 0  Empty arena, single fixed goal            — learn to move, no obstacles
Stage 1  Static obstacles, sparse                  — learn avoidance
Stage 2  Static obstacles, cluttered, tight gaps    — learn clearance discipline
Stage 3  Moving obstacles, scripted trajectories     — learn prediction
Stage 4  Humans (simulated agents with plausible behaviour)  — learn social margins
Stage 5  Domain-randomised: friction, delay, noise, sensor dropout, localisation error
Stage 6  Real hardware (or high-fidelity twin)       — sim-to-real validation
```

Advance a stage only when a **held-out evaluation** passes its criteria. Training on Stage 5 from scratch fails; training on Stage 1 and then transferring to Stage 5 does not. This is one of the most consistently replicated findings in the field.

### 4.2 Domain randomisation (the sim-to-real workhorse)

Randomise during training: robot mass and inertia (±20%), wheel radius (±5%), friction coefficients, actuator delay (0–50 ms), control rate jitter, sensor noise (range bias σ, dropouts, beam angle error), perception latency, obstacle placement, lighting, and the localisation error distribution.

**Principle:** randomise over the range the hardware will actually see — measured, not invented. Over-randomisation produces a timid, conservative policy that performs poorly everywhere; under-randomisation produces a brittle one.

### 4.3 Reward design discipline

1. **Never mix units carelessly.** A penalty that dominates due to scale (metres vs. radians vs. seconds) silently redefines the task.
2. **Inspect the reward, not the score.** Log per-term contributions per episode. A policy that scores well by farming one term is a reward bug, not a success.
3. **Penalise what you actually care about** — discomfort (jerk, lateral acceleration) and near-misses, not only collisions. A policy that grazes humans but never touches them is not safe.
4. **Reward hacking is expected**, so test the *behaviour*: run scenario suites and inspect trajectories. Numerical returns alone are not evidence.

## 5. Constrained RL

The safety requirements (SF-06, PR-09) are **constraints**, not objectives. Standard RL has no mechanism to respect them, which is why the shield (doc 06) exists — but incorporating constraints during training reduces how often the shield must intervene, and a shield that fires constantly means the policy is not doing the job.

| Method | Idea | Practical note |
| --- | --- | --- |
| **Lagrangian / RCPO** | Introduce a Lagrange multiplier on the constraint cost, adapt it during training | Simple, effective; the multiplier oscillates and needs damping. Report the final constraint violation rate |
| **CPO / PCPO** | Projected policy updates that respect a trust region in constraint space | Cleaner theory, trickier implementation |
| **Safety-layer-at-training** | Train **with** the shield active so the policy learns the shielded dynamics | **Recommended** — the deployed system has a shield, so train the composition, not the policy alone |
| **Shielding as exploration bias** | The shield prevents unsafe actions during exploration | Reduces catastrophic resets and speeds learning |

**Recommended combination:** Lagrangian penalties during training **plus** an active shield, evaluated on the *composed* system's constraint-violation rate — never on the raw policy's.

## 6. Sim-to-real: what actually transfers

| Practice | Effect |
| --- | --- |
| Train with estimator/latency/perception noise in the loop | Largest single factor; policies otherwise assume perfect state |
| Action-rate reduction + filtering | Real actuators cannot follow high-frequency policy output; a low-pass filter on the action both stabilises hardware and smooths training |
| Observation normalisation with fixed statistics | Running normalisation shifts under distribution change and destroys transfer — freeze the statistics after training |
| Randomised physics + delay + dropout | Robustness to unmodelled effects, at some cost in peak simulated performance |
| Residual learning over a classical controller | Guarantees a sensible baseline at initialisation |
| Simulator parameter identification | **Measure** friction, mass, delay from hardware and set the simulator to the measured values instead of randomising them |
| Validation on hardware with the shield active, in a safe enclosure | The only evidence that matters for transfer |

## 7. Assumptions, limits and honest claims

1. **No safety guarantee.** A neural policy is a black box. It cannot be certified by testing alone. Every claim about safety in this project comes from the shield, the supervisor and the independent stop path — never from the policy.
2. **Distribution shift.** The policy is valid in the distribution it trained on; a novel environment (different floor, different lighting, unknown obstacle type) is out of distribution. Mitigation: detection of out-of-distribution conditions (perception health, anomaly detection) → fall back to the conservative controller. This is a system requirement, not a policy property.
3. **Sample cost.** Millions of simulated steps; a 10-minute hardware trial is worth thousands of simulated seconds and is not a substitute for training in simulation.
4. **Evaluation must be statistical.** ≥ 5 seeds, ≥ 100 evaluation episodes per configuration, with the spread reported. A policy that beats the baseline in 3 of 5 seeds has not beaten it.
5. **The learning result that is worth reporting is a comparison**, not a demonstration: policy + shield vs. NMPC-only vs. scripted baseline, on identical scenarios, with the shield intervention rate reported as the headline diagnostic.

## 8. Verification

| Claim | Test | Metric |
| --- | --- | --- |
| Policy learns the task | Evaluation runs on held-out seeds and scenarios | Success rate, return, episode length |
| Policy respects constraints | Composed-system evaluation with the shield active | Violation rate, shield intervention frequency |
| Policy beats the baseline | Identical scenario suite: NMPC-only, scripted, RL+shield | Success rate, time-to-goal, path efficiency, comfort metrics |
| Transfer | Hardware trials in a safe enclosure, Stage 6 curriculum | Success rate vs. simulation; degradation quantified |
| Robustness | Domain-shifted evaluation (unseen lighting, floor, obstacle shapes) | Performance drop; OOD detection triggers correctly |
| Reproducibility (SR-06) | Re-run training from the recorded seed and config | Comparable return distribution; image digest matches |

## 9. Open questions

1. Is the mission layer better solved by RL or by a hierarchical task planner with a learned tie-breaker? → resolved by a baseline comparison; the scripted version is built first regardless.
2. How much authority should the residual policy have — a speed scale, a lateral offset, or a full `(v, ω)` correction? → resolved by starting with a scalar speed scale and increasing only if the measured benefit justifies the loss of analysability.
3. Is simulation throughput sufficient for model-based RL (Dreamer-class), or is the environment too slow to train a world model in reasonable time? → resolved by a simulator throughput benchmark (steps/second) in the container.
