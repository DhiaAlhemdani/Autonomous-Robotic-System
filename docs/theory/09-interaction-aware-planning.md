# Theory 09 — Planning Among Moving Agents: Two Approaches

**Purpose:** document two candidate approaches to planning when the robot shares space with people and movable objects, define when each applies, and state what must be measured before either is adopted. Serves scenario S2 (`docs/00` §2) and layer L3 (`docs/02` §9.2, §9.3). Requirements: SF-01, SF-05, PR-05, FR-05.

**Status:** theory and specification only. Nothing here has been implemented or measured. Both approaches are developed when the laptop is available. No performance is claimed for either.

---

## 1. The question

The robot plans a trajectory `x(·)` through an environment in which `M` other agents (people, movable objects) move. Their future positions are unknown. Two questions follow:

1. **How does the planner use uncertain predictions of those agents?**
2. **Does the robot's plan affect how the agents move?** If it does, prediction and planning are coupled, and a separate prediction step is inconsistent.

Approach A answers question 1 and assumes the agents are unaffected by the plan. Approach B answers both.

---

## 2. Shared notation

| Symbol | Meaning |
| --- | --- |
| `x_k`, `u_k` | Robot state and command at step `k`; horizon `N` steps at `Δt` |
| `o^i_k` | Position of agent `i` (person or object) at step `k`; `i = 1..M` |
| `ô^i_{k}` | Predicted position of agent `i`, with covariance `Σ^i_k` |
| `d_min` | Minimum separation: a margin that depends on the agent class (person larger than object) |
| `J` | Robot cost: tracking, effort, comfort (jerk, lateral acceleration), progress |

Common constraint form: `‖p(x_k) − o^i_k‖ ≥ d_min^i` for all `k`, `i`, where `p(·)` extracts position.

---

## 3. Approach A — Prediction-conditioned NMPC (Core)

### 3.1 Formulation

Predict each agent's trajectory first, independently of the robot. Then solve an NMPC whose obstacle constraints use the predicted positions and their uncertainty:

```
min_{u_0..u_{N-1}}  Σ_k J(x_k, u_k)
s.t.  x_{k+1} = f(x_k, u_k)                                   (robot model)
      u_k ∈ U,  x_k ∈ X                                        (limits)
      P( ‖p(x_k) − o^i_k‖ < d_min^i ) ≤ δ   for all k, i        (chance constraint)
```

The chance constraint is approximated with a Gaussian margin: the constraint is tightened by `z_δ · √(λ_max(Σ^i_k ⊕ Σ_robot))`, where `z_δ` is the Gaussian quantile for the allowed risk `δ`. Here `⊕` denotes the combined covariance of the agent's prediction and the robot's own localisation error.

### 3.2 Prediction models

- **Constant velocity (baseline, must always run):** `ô^i_k = o^i_0 + v^i_0 k Δt`, with covariance growing with `k`. It is cheap and transparent, and it is the benchmark any learned model must beat.
- **Learned marginal predictor (Core target):** a small network that predicts each agent's future distribution from its own recent track and the map, without modelling the robot. Output: a mixture or Gaussian per step, with calibrated uncertainty. Calibration is checked on held-out data before use.

### 3.3 Assumptions and limits

- Agents are unaffected by the robot. This fails when the robot approaches a person, who then steps aside or stops. The planner then over-reacts or freezes. This is the **freezing robot** failure, and it is the main reason Approach B exists.
- Prediction uncertainty is calibrated only on the distribution of the training data. Out-of-distribution agents (a wheelchair, a dropped box) have unknown uncertainty and fall back to worst-case margins (`docs/02` §9.2).
- Chance constraints assume the Gaussian approximation is adequate. Heavy-tailed prediction errors break this.

### 3.4 Design decisions

- Predictions are inputs to L3 as constraints only. They never produce commands.
- If predicted uncertainty for an agent exceeds a threshold, that agent is treated as worst-case, with the largest `d_min` and the reachable set instead of a point prediction.
- Prediction runs at 15–30 Hz with the perception rate, so its age is checked against the staleness limit (`docs/02` §9.1).

---

## 4. Approach B — Interaction-aware planning (Advanced)

### 4.1 Formulation

The robot and each agent are treated as players. Each chooses a trajectory to minimise its own cost, given the others' choices. The planner solves for a joint trajectory that is consistent with how the agents respond to it:

```
min_{x, o^1..o^M}   J_robot(x, o)
s.t.  each o^i minimises J_i(o^i ; x)  (or a best-response surrogate)
      dynamics and constraints for all agents
      ‖p(x_k) − p(o^i_k)‖ ≥ d_min^i
```

In practice this is solved approximately. Options:

- **Iterated best response:** alternate between solving the robot's plan given predicted agent responses, and updating those responses given the robot's plan. Converges to a consistent plan under convexity assumptions that real scenes violate, so the result is a candidate, not a guarantee.
- **Learned response model:** a network `ô^i = g_θ(o^i_history, x_{0:N})` that predicts how agent `i` responds to a candidate robot plan. The planner evaluates candidate plans through it. This is the realistic route to real-time use.

### 4.2 Why it matters

It addresses the freezing failure of Approach A. It also allows the robot to make small, legible moves that encourage a person to pass on one side. Those behaviours are not reachable by treating people as fixed obstacles with margins.

### 4.3 Assumptions and limits

- **The response model is the weak point.** It must be learned from data that include the robot's influence, and that data is expensive to collect safely. Until it exists, Approach B reduces to Approach A with a different prediction, and must not be claimed as interaction-aware.
- Agents may not be rational players. People may ignore the robot, react unpredictably, or be distracted. The game model assumes they optimise a cost that is not observable.
- The joint problem is non-convex and its solution time is unbounded in the worst case. It cannot run at 20–50 Hz without an approximation, and the approximation must be measured.
- Any influence on people is also a safety surface: a robot that "encourages" a person to move may create the collision it was trying to avoid. Such behaviour must stay inside the hard constraints and be checked by L2 (`docs/02` §5).

### 4.4 Design decisions

- Approach B is never the primary path. It runs as a candidate planner that L2 can veto, and its plans are subject to the same hard constraints as Approach A.
- It is adopted only if the measurements in §6 show it beats Approach A in a stated metric without increasing safety interventions.

---

## 5. Staged plan

| Stage | Content | Depends on | Scope tier |
| --- | --- | --- | --- |
| 0 | Constant-velocity prediction plus NMPC with margin constraints; metrics; scenario S2 | Working NMPC (doc 04) and simulation | Core |
| 1 | Learned marginal predictor with calibrated uncertainty; replaces constant velocity if it wins on held-out data | Public pedestrian datasets, training compute | Core |
| 2 | Approach B with a learned response model, evaluated offline and in simulation | Stage 1 data, plus data on robot-influenced motion | Advanced |
| 3 | Hardware trials of the best stage, with the safety case from doc 08 | Hardware, safety review | Core validation |

Stage 0 is the minimum that makes S2 meaningful. Stage 2 is not started until Stage 1 is measured.

---

## 6. Evaluation protocol

Both approaches run on identical scenarios from S2 with the same random seeds. Reported metrics, each with its conditions:

- **Minimum separation** to people and to movable objects, as the distribution over runs.
- **Safety interventions:** count of L2 overrides per run, and whether any override was needed.
- **Freezing:** time the robot spends with speed below a threshold while an agent is within a set radius, and whether it resolves.
- **Efficiency:** path length and time to goal, relative to the constant-velocity baseline.
- **Comfort:** peak lateral acceleration and jerk.
- **Solve time:** p50 and p99 on the target hardware.
- **Prediction calibration:** coverage of the predicted uncertainty on held-out data, compared with the nominal level.

A method is adopted only if it beats the baseline on a pre-stated primary metric without increasing interventions. The primary metric is fixed before the runs.

---

## 7. Open questions

1. Which public pedestrian datasets are licensed for the intended use, and do they include robot-influenced motion? (Most do not.)
2. What is the acceptable collision-risk level `δ` for people, and who sets it? It is a safety decision, not a tuning parameter.
3. Can the response model be validated against real logged interactions before hardware, or only in simulation?
4. Is the freezing failure frequent enough in S2 to justify Approach B's cost? This is answered by Stage 0 measurements.
5. How do movable objects differ from people in this model? An object does not respond to the robot, so it may belong in Approach A permanently.
