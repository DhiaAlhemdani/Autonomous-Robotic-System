# 05 — Risk Register

Reviewed at every milestone gate. Each risk has an owner action and a trigger that would force a decision.

## Technical risks

| ID | Risk | Impact | Likelihood | Mitigation / decision trigger |
| --- | --- | --- | --- | --- |
| R-01 | **NMPC cannot meet 50 Hz on the target compute** | High — the central control claim fails | Medium–High | Profile the solver **before** tuning the controller; limit `N`; use RTI with bounded iterations and warm starts; accept a lower rate (20 Hz) and re-verify tracking. **Trigger:** p99 solve > 45 ms after optimisation ⇒ reduce horizon/rate or move to a coarser model |
| R-02 | **Sim-to-real gap makes the learned policy useless on hardware** | High — the learning component's value is unproven | High (it is the norm) | Train with estimator, delay and perception noise in the loop; domain randomisation over **measured** ranges; residual formulation over a competent baseline; the shield means a bad policy degrades performance, not safety. **Trigger:** hardware success rate < 60% of simulated ⇒ fall back to reporting the simulation result with the gap quantified, which is itself a legitimate finding |
| R-03 | **Excessive scope** — eight theory documents and seven milestones is more than a single-person project | High — nothing finishes | **High** | The Core/Advanced/Stretch split in doc 00 §5 is binding. **Trigger:** any milestone exceeding its planned effort by 2× without passing its gate ⇒ freeze scope, finish and document what exists, publish the honest partial result |
| R-04 | **Safety filter conservatism causes deadlock and poor mission time** | Medium — the system is safe but unusable | Medium | Tune `γ` and margins from measured stopping data; explicit deadlock detection with a scripted escape (FR-06); report the time cost as a result. **Trigger:** deadlock rate > 5% of runs ⇒ revisit margins and add the escape behaviour |
| R-05 | **Container overhead breaks the control rate** | Medium | Medium | Safety and drive loops stay on the MCU (outside containers); containers host soft-real-time work only; host `PREEMPT_RT` if needed; **measure jitter**, do not assume. **Trigger:** control-period jitter > 20% of the period ⇒ re-partition the deployment |
| R-06 | **Sensor selection is wrong** (e.g. a 2D LiDAR that cannot see a glass wall or a stair edge) | Medium–High — safety geometry has blind spots | Medium | Two independent modalities; **unknown treated as occupied**; near-field ultrasonic confirmation; document the blind spots explicitly and set speed limits accordingly |
| R-07 | **Estimator divergence in feature-poor spaces** | Medium | Medium | Consistency monitoring (NIS/NEES); multi-hypothesis localisation (AMCL-style) once a map exists; VIO cross-check; degradation ladder on covariance growth |
| R-08 | **Reward hacking / policy exploits the simulator** | Medium — a result that looks good and is meaningless | High (expected) | Per-term reward logging; behaviour inspection, not score inspection; held-out scenario suites; an independent baseline. **Trigger:** a policy whose trajectory is bizarre despite a good return ⇒ suspect the reward, not the policy |
| R-09 | **Model parameter drift** (hot motors, worn tyres, battery sag) silently degrades control | Medium | Medium | Online parameter monitoring; re-identification at intervals; robust (tube) MPC if drift exceeds the margin |
| R-10 | **Learning results not reproducible across seeds** | Medium — weakens every claim | High | ≥ 5 seeds, fixed seeds recorded, seed-averaged reporting with spread; container digests pinned |
| R-11 | **Safety case is not credible** (claims exceed evidence) | High — it undermines the project's differentiator | Medium | Every claim attributed to a layer with evidence (doc 08 §3); explicit "does not claim certification" statements; external review at M6 |
| R-12 | **Hardware damage during testing** (overcurrent, thermal, mechanical) | Medium | Medium | Current limits in firmware, not software; fusing; thermal monitoring; bench testing before floor testing; spare parts budgeted |

## Resource and schedule risks

| ID | Risk | Impact | Mitigation |
| --- | --- | --- | --- |
| R-13 | **Development machine is unavailable** for the initial phase | Blocks implementation | The theoretical phase is deliberately laptop-independent (this document set). Do not schedule implementation before hardware access is confirmed |
| R-14 | **GPU unavailable or too slow for training** | Blocks the learning tier | Train in simulation with reduced fidelity; use smaller models; rent GPU time for the final runs; the Core tier needs no GPU for control |
| R-15 | **Budget exceeds the tier** | Delays M5–M7 | Buy in the T1 → T2 → T3 order; simulation covers everything except hardware validation |
| R-16 | **Time-boxing fails** — theory expands indefinitely | Nothing ships | Each theory document is "good enough" when its design decisions are made and its verification plan exists. **Trigger:** no milestone progress for 3 weeks ⇒ stop writing, start the next milestone |

## Risk-to-requirement linkage

Risks are only meaningful if they change the design. Current linkages: R-01 → PR-05 and doc 04 §3.3 · R-02 → doc 05 §6 · R-04 → FR-06 and doc 06 §3.4 · R-05 → doc 03 §3 and SF-01 · R-06 → FR-04 and doc 07 §7 · R-07 → PR-03 · R-11 → doc 08 §3.
