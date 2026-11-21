# Theory 06 — Integrating Learning with MPC, and Making It Safe

**Purpose:** the architectural and mathematical core of the project. How a neural policy is permitted into the control loop of a physical machine, and what guarantees can honestly be claimed. Requirements: SF-01, SF-05, SF-06, FR-05, PR-08, PR-09.

---

## 1. The question this document answers

> A learned policy has no correctness proof. How can it be allowed to command a robot that operates near people?

The answer used throughout this project is: **the policy is never the last line of defence.** It proposes; an independent, non-learned layer disposes. This document specifies four integration topologies, selects one, and defines the safety filter that makes the selection defensible.

## 2. Four integration topologies

### Topology 1 — Learned policy alone
```
obs → [policy] → u → robot
```
- Optimal flexibility; fails silently and unpredictably; **unacceptable** as the primary path for this system.
- Kept only as a research baseline for the purely simulated comparison in doc 05 §8.

### Topology 2 — Hierarchical: RL above MPC
```
goal → [RL: behaviour + waypoint] → [NMPC: realises it] → u → robot   (+ shield)
```
- The policy makes low-rate decisions (which behaviour, which waypoint, how fast) and the MPC realises them under explicit constraints.
- **Every failure mode is bounded:** a bad policy produces a bad waypoint, and the MPC still refuses to collide.
- Sample-efficient (low-dimensional actions), inspectable (the waypoint is logged and can be replayed), and testable layer by layer.

### Topology 3 — Residual: MPC plus a learned correction
```
goal → [NMPC] → u_nom → ⊕ → u → robot
              [RL: Δu] ↗      (bounded, filtered, shielded)
```
- The policy improves a competent baseline, typically in regimes where the model is weakest (unmodelled friction, slip, tight gaps).
- Starts from competent behaviour; the policy can only degrade a bounded amount.
- Risk: the correction can fight the MPC's constraints unless the residual is bounded and the shield re-checks the composed command.

### Topology 4 — Learning inside the MPC (learned dynamics or learned terminal cost)
```
goal → [NMPC with f_θ (learned model) or P_θ (learned cost-to-go)] → u → robot
```
- The most elegant: learning improves the *model*, keeping all constraints and guarantees in the same framework.
- Requires guarantees about the learned model's error (or robustness against it), otherwise the "guarantees" of the MPC no longer hold.
- This is a genuine research direction and is the **stretch tier** of this project, not the baseline.

### Selection and rationale

| Criterion | T1 | T2 | T3 | T4 |
| --- | --- | --- | --- | --- |
| Bounded failure modes | ✗ | **✓** | ✓ (± bound on residual) | ✓ only with robust MPC |
| Sample efficiency | ✗ | **✓** | ✓ | — |
| Inspectability / debuggability | ✗ | **✓** | ~ | ~ |
| Where learning adds most value | — | task structure | residual physics | model accuracy |
| Implementation risk | low | **low** | medium | high |

**Decision: Topology 2 as the system baseline, with Topology 3 as the advanced-tier experiment for the same mission scenarios.** Topology 4 is documented as future work with its prerequisites stated. The comparison *T2 vs. T3 vs. NMPC-only* — on identical scenarios, with the shield intervention rate reported — is one of the headline results of the project.

### 2.1 Industry convergence (consistent with this choice; not evidence that it performs)

This selection was derived here from failure-mode reasoning. A similar pattern appears in Waymo's deployed driver, the largest public record among the systems we reviewed (doc 08 §2). Waymo, after 200+ million fully autonomous miles, rejected pure end-to-end architectures for a related reason to the one given in §2 — *"pure end-to-end neural architectures… run the risk of black box failures"* — and deployed **an independent onboard validation layer that monitors every trajectory proposed by the driver model, checking it against hard physics-based constraints and traffic laws, with hard veto authority.** Waymo describes that layer as AI-based, so it is learned where ours is analytic: the pattern matches, the implementation does not. A secondary analysis of Atlas describes a similar split, with learned policies for locomotion and dexterity and a classical whole-body controller as the "safety-critical backbone". That phrase is the analyst's characterisation, not a Boston Dynamics statement.

Two conclusions follow. First, **the pattern is not new** — propose-with-learning, dispose-with-an-independent-verifier appears in several systems we reviewed (doc 08 §8). That is consistent with this choice. It supports the reasoning; it is not evidence that the design performs. Second, and more usefully: some of Waymo's most serious public failures involve a hazard that was **detected** while no declared rule required a stop (the 2026 flood case, per NHTSA's filing and journalism's analysis). That points at the declaration-and-enforcement machinery rather than the shield. Not all of Waymo's failures fit this pattern (emergency-scene recognition is a different kind of failure), so this is a partial explanation. The declaration gap is the subject of the evidence pipeline in [`../07-open-dilemmas.md`](../07-open-dilemmas.md) §4.4. See [`../08-state-of-the-art-analysis.md`](../08-state-of-the-art-analysis.md) §2.4 and §8 for the full analysis.

## 3. Safety filter via control barrier functions

### 3.1 The idea

A control barrier function `h(x)` defines a safe set `C = {x : h(x) ≥ 0}` (e.g. `h(x) = d(x) − d_min`, distance to the nearest obstacle minus the minimum allowed). A controller is safe with respect to that set if, on the boundary of the set, it cannot act in a direction that would leave the set. Differentiating along the dynamics produces the condition:

```
ḣ(x, u) = L_f h(x) + L_g h(x) u  ≥  −α(h(x))          (α a class-𝒦 function, e.g. α(h) = γ h)
```

Enforcing this as a constraint defines the set of admissible inputs at every instant.

### 3.2 The safety filter (CBF-QP)

At every control step, take the nominal command `u_nom` (from the MPC or the policy) and project it onto the admissible set by solving a small QP:

```
minimise   ‖u − u_nom‖²
    u
subject to
    L_f h_i(x) + L_g h_i(x) u  ≥  −γ_i h_i(x)      for each safety constraint i
    u ∈ U                                          input limits
```

Properties that make this attractive:

- **Minimal intervention:** the filter changes the nominal command only as much as the constraint requires. It is a *filter*, not a controller, so it does not fight the MPC for performance.
- **Fast:** a small QP (a few constraints) solves in microseconds — order 100 µs on an embedded CPU, orders of magnitude below the NMPC. It can therefore run at 100–200 Hz, faster than the MPC that feeds it.
- **Composable:** each safety requirement (obstacle clearance, human distance, speed envelope, workspace boundary) becomes one constraint `h_i`.
- **Compatible with any upstream:** it accepts commands from MPC, RL, a human teleoperator, or a script, and makes them safe in the same way. This is what decouples the safety argument from the policy's correctness.

### 3.3 Barrier function candidates for this platform

| Safety requirement | Candidate `h(x)` | Notes |
| --- | --- | --- |
| Obstacle clearance (FR-04) | `h = d_obs(x) − d_min` used with an SDF from doc 04 | Needs a differentiable distance; the SDF provides it |
| Human distance (PR-09) | `h = d_human(x) − (d_min + v t_reaction)` | Velocity-dependent margin — humans are treated with a larger envelope than static objects, and the envelope must be measured, not assumed |
| Speed envelope | `h = v_max(clearance) − v` | Encodes "you may not go faster than you can stop within the sensed distance" |
| Sensing-envelope limit | `h = d_sensed − d_min` | Critical: if the robot cannot see far enough to guarantee stopping, it must slow down regardless of how clear the path looks |
| Workspace/geofence | `h = boundary − x` (linear) | Hard outer limit; useful as a last-resort guard |

### 3.4 Practical failure modes of CBF filters (and the mitigations)

| Failure mode | Cause | Mitigation used here |
| --- | --- | --- |
| **Infeasibility** | Conflicting constraints, or the robot is already inside the unsafe set | Slack on the constraint with a large penalty, then the fallback chain: brake to stop; treat the interior case as a recovery manoeuvre (`h` is defined with a tolerance band so it can still be escaped) |
| **Deadlock / no progress** | The barrier prevents all motion (e.g. head-on with a person who does not move) | Explicit progress term outside the filter, plus a scripted escape behaviour triggered by a timeout; deadlock detection and recovery are required by FR-06 |
| **Conservatism** | Small `γ` or large `d_min` makes the robot timid and slow | Tune `γ`, `d_min` from measured stopping data; report the cost in mission time — this trade-off is a *result to be reported*, not a nuisance |
| **Effect of the discretisation** | Guarantees are continuous-time; the implementation is discrete | Use the discrete-time CBF condition or include the zero-order-hold effect in the derivation; verify empirically with the maximum step size actually used |
| **Model dependence** | The barrier condition depends on `f`, `g` — which are only approximately known | Tighten margins by the measured model error; use robust/adaptive CBF variants if error is large |
| **Detection latency** | A barrier on a human is only as good as the perception feeding it | Bound the margin by the perception latency (`v·t_detect`), which is measured (PR-06) — and never rely on flawless detection |

### 3.5 Discrete implementation of a continuous guarantee

The theory is continuous; the robot is not. The honest engineering treatment:

1. Derive the condition in continuous time.
2. Implement the **discrete-time** version (condition applied at the sample instant with the zero-order-held input), which is a different and stricter condition.
3. Verify empirically: run adversarial scenarios (a person walking directly at the robot, an obstacle appearing at minimum detection range) and log the **minimum realised distance**. The claim is then: "the minimum distance over N adversarial runs was X m; the barrier margin is Y m; measured detection latency is Z ms."
4. State explicitly that this is empirical validation of a designed margin — **not a formal proof**, because model error, detection latency and a finite sample rate all sit outside the certificate.

## 4. The complete safety architecture

```
┌───────────────────────────────────────────────────────────────────────────────┐
│  L5 RL policy  ──► behaviour / waypoint        [learned, unverified]          │
│                          │                                                    │
│  L4 global planner ──────┼──► path                                           │
│                          ▼                                                    │
│  L3 NMPC  ──────────────► u_nom   [model-based, constraints, no guarantee]    │
│                          │                                                    │
│  L2 SAFETY FILTER (CBF-QP, 100–200 Hz) ──► u_safe   [non-learned, fast]       │
│      ├── obstacle + human margins (velocity-dependent)                        │
│      ├── speed envelope vs. sensed distance                                   │
│      └── input limits, feasibility check                                      │
│                          │                                                    │
│  L2 SUPERVISOR (50 Hz) ──┤ watchdogs, health, degradation ladder, state       │
│                          ▼                                                    │
│  L2 MCU GATE (hardware) ─► torque enable, heartbeat, E-stop   [non-software]  │
│                          ▼                                                    │
│  L1 FOC (10–20 kHz, MCU) ─► PWM                                               │
└───────────────────────────────────────────────────────────────────────────────┘
```

**Layered defence, in the order it acts:**
1. **Policy** — proposes behaviour. Can be wrong freely.
2. **MPC** — realises the proposal under explicit constraints. Cannot violate input or obstacle constraints *in its model*.
3. **CBF-QP filter** — enforces the safety set on the *actual* command, whatever produced it, at a faster rate than the MPC.
4. **Supervisor** — watches health, timing and consistency; declares the state; escalates the degradation ladder.
5. **MCU gate** — hardware heartbeat and torque-enable; the only layer that survives a complete failure of the high-level computer.
6. **Physical E-stop** — removes torque independently of all software.

**Claim boundary:** layers 1–3 are *engineered* safety with empirical validation. Layers 4–5 are the *tested* stop path, with measured reaction times. Layer 6 is the ultimate independent safeguard. Safety claims in the project's documentation are attributed to specific layers, never to "the system" generically — and never to the learned policy.

## 5. What can and cannot be claimed (write this into every report)

| Claim | Status |
| --- | --- |
| "The shield guarantees the commanded input satisfies the barrier condition, given the model and the measured state" | **Defensible**, with the assumption set stated |
| "Minimum realised distance to humans over N adversarial runs was X m (p1: Y m)" | **Defensible** — empirical, with N stated |
| "Measured worst-case stop reaction time is Z ms" | **Defensible** — instrumented |
| "The policy learned to navigate safely" | **Not defensible** — replace with composed-system metrics and shield intervention rate |
| "The system is safe" | **Not defensible without a formal safety case and standards compliance for the intended environment** — and the project explicitly does not claim regulatory certification (doc 00 §5) |
| "The system is certified to ISO 3691-4" | **Never claim** — the project follows the methodology; a laboratory prototype is not a certified industrial truck |

## 6. Verification plan for this layer

| Claim | Test | Metric |
| --- | --- | --- |
| Shield minimal intervention | Log `‖u_safe − u_nom‖` across nominal scenarios | Fraction of steps with intervention > threshold; mean deviation |
| Constraint enforcement | Full scenario suite + adversarial runs | Realised minimum distance distribution; violation count (target: 0) |
| Feasibility handling | Force infeasible configurations (boxed-in, head-on human) | Correct fallback triggered; no oscillation; deadlock resolved within the bound (FR-06) |
| Timing | Instrumented filter | p99/max solve time; must be well below its period |
| Adversarial robustness | A scripted adversary that attempts to exploit the filter (e.g. approach from an occluded direction) | Minimum distance maintained; failures documented rather than hidden |
| Policy interference | Compare shield intervention rate for RL vs. NMPC upstream | Reported side by side — a high rate for the policy means the policy is not doing the job |

## 7. Open questions

1. Is a single CBF-QP sufficient at 100–200 Hz on the chosen compute with all constraints active, or must constraints be prioritised (human > obstacle > speed)? → resolved by benchmarking and by a priority-based hierarchical QP if needed.
2. Does the discrete-time barrier condition require a smaller `d_min` than the continuous version to remain feasible in tight spaces, and does that hurt mission time? → resolved by the tight-corridor scenario.
3. Should the human-margin term use a **predicted** human trajectory rather than a static margin? → advanced tier; requires doc 07's tracking, and must be validated against the failure mode of *mis*predicting a human, which is worse than treating them as static.
