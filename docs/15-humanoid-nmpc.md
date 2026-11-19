# 15 — Humanoid NMPC (Layer L3): Design Proposal

**Status: design proposal, not frozen, not implemented.** Scope: the L3 navigation and command-shaping layer of the T800 humanoid, decided in `docs/02` §11 to be NMPC. Interfaces follow `docs/14` §2.4 and §3.1. Every number in this document is an **ASSUMPTION** until measured in simulation. No result in this document is a property of the physical robot.

Sources: `docs/13` (Stage 0 NMPC, planar double integrator), `docs/02` §6 (fallback chain), `docs/14` §§2–4 (locomotion interface, safety states), and the T800 submodule at `8bfd5b4`.

---

## 1. What the NMPC is and is not

| Is | Is not |
| --- | --- |
| A receding-horizon planner over a **base-level** model of the humanoid | A whole-body or footstep planner. The upstream walking policy takes only a body-frame velocity command (`docs/14` §2.2) |
| The local tracker of the global path from L4, with obstacle and person constraints | A balance controller. Balance belongs to L1 locomotion and the joint servo |
| The producer of a bounded `(vx, vy, ω)` reference at 10–20 Hz | A safety layer. L2 holds veto power and sits between L3 and L1 (`docs/02` §1) |

The main consequence: **the NMPC plans the base; it does not control the feet.** Any claim about stepping, falling, or contact has to come from L1, and is tested there (`docs/14` §5, §4.5).

---

## 2. Interface

| Signal | Direction | Rate | Content |
| --- | --- | --- | --- |
| Global path | L4 → L3 | on change, ≥ 0.5 Hz | Waypoints in the world frame (`docs/02` §1) |
| Base state | L0 → L3 | 100 Hz | Pose and velocity, body frame, with covariance (`docs/14` §3.3). Simulator base pose is **not** used |
| Safety field | L2 → L3 | 50–200 Hz | Distance and gradient to the nearest constraint region (`docs/02` §5) |
| Obstacle and person tracks | L0 → L3 | 10–30 Hz | Positions and velocities with covariance |
| **Velocity reference** | **L3 → L2 → L1** | **10–20 Hz** | Body-frame `(vx, vy, ω)`, a validity flag, and a solver status (`docs/14` §3.1) |

L2 filters the reference before it reaches L1. L3 never writes a joint target.

---

## 3. Prediction model

**State** `x = (p_x, p_y, ψ, v_x, v_y, ω)`, world-frame position and heading, body-frame velocity.

**Input** `u = (v_x*, v_y*, ω*)`, the commanded body-frame velocity sent to L1.

**Lag model (ASSUMPTION):** the walking policy does not track `u` instantly. Use a first-order lag per axis, `v̇ = (u − v) / τ`, with τ and a saturation per axis identified from simulation step responses (experiment N1 below). The lag is the only dynamics in the model. Joint dynamics, gait phase, and foot contact are outside the model.

**Why this model:** `docs/13` used a planar double integrator with acceleration input. That model was the right baseline for Stage 0, but for the humanoid the input is a velocity command to an inner loop, not an acceleration of the base. Using acceleration input would plan motion the policy cannot produce.

**Validity limit:** the lag model is accurate only inside the policy's training command range, which is not documented upstream (`docs/14` §2.4). Outside that range, the predicted base motion is unknown. The NMPC must not command outside the range that N1 has measured.

---

## 4. Cost

Stage-by-stage cost over horizon N:

- **Path tracking:** distance to the path segment, weight `w_path` (ASSUMPTION).
- **Goal:** terminal cost on distance to the goal, weight `w_goal` (ASSUMPTION).
- **Effort:** `‖u‖²`, weight `w_u` (ASSUMPTION).
- **Smoothness:** `‖Δu‖²` (rate of command change), weight `w_du` (ASSUMPTION). The starting values are the Stage 0 smoothness weights (`docs/13` §13, `w_jerk` 2, `w_lat` 2). They were chosen for a different model and are **not carried over as validated values**.

**Rule from the project:** the cost may be tuned. The hard constraints in §5 may not be relaxed to improve the cost (`docs/13` §13 applies here too).

---

## 5. Constraints

**Hard (never relaxed by cost tuning):**

| Constraint | Source | Value |
| --- | --- | --- |
| Command limits on `u` | `docs/14` §3.1 | TBD from N1 and N3 |
| Command rate limits on `Δu` | No upstream or `docs/14` source yet | ASSUMPTION; set from N1 and N5 |
| Separation from persons | `docs/02` §5 safety field, Gaussian margin as in `docs/13` §3 | Minimum separation 0.8 m is a Stage 0 placeholder, not a humanoid value |
| Separation from objects | as above | 0.5 m, placeholder |
| Heading-rate limit | `docs/14` §2.4 | TBD from N1 |

**Chance constraints:** the Gaussian margin `z_δ·σ` of `docs/13` is kept for persons, with δ = 0.05 as a placeholder. It assumes the predicted person positions are approximately Gaussian, which has not been checked for humanoid-scale tracking.

---

## 6. Solver and timing

- **Target rate:** 10 Hz to start, within the 10–20 Hz band of `docs/14` §2.4.
- **Budget (ASSUMPTION):** a solve must finish within half the period. At 10 Hz that is 50 ms on the target computer. The target computer is not chosen yet (`docs/02` §2), so this number is not a hardware measurement.
- **Solver choice: not decided.** `docs/13` §9 found that SLSQP failed on all 489 infeasible steps, most with a line-search breakdown on non-convex circular constraints. **SLSQP is not a candidate for the humanoid NMPC.** A candidate is a QP-based real-time iteration with analytic gradients (`docs/13` §12 showed the gradient route is opt-in). The choice will be made by measurement in N4, not by assertion.
- **Warm start:** shifted previous solution, with the infeasible-step fraction reported next to timing (as in `docs/13`).
- **Solve times** are reported as sandbox measurements. They are not target-hardware numbers.

---

## 7. Feasibility and fallback

The order follows `docs/02` §6 and `docs/14` §4:

1. **Re-solve with slack** on the soft constraints only (never on the hard limits in §5).
2. **Conservative reactive stop:** ramp `u` to zero along the limits in §5, which hands over to L2 and the S1 state in `docs/14` §4.2. Stopping by ramping the command is the only stop from walking. Torque is not removed here.
3. **Protective stop:** S2 (hold stance). S3 and S4 follow `docs/14` §4 if the stance fails.

A step that falls back is logged with its reason. In `docs/13` §10, re-solving from other starting guesses recovered 16% of infeasible steps, so fallback counts depend on the solver setup. Each fallback step is counted in the reported results, not hidden.

---

## 8. Evaluation

Simulation only. Fixed seeds, commit, and MuJoCo version recorded (`docs/14` §5).

| Test | Procedure | Metrics | Gate |
| --- | --- | --- | --- |
| **N1** Lag identification | Step commands on each axis in the simulated walking policy | τ, saturation, valid command range | Measured; sets the ranges in §3 and §5 |
| **N2** Path tracking | Scenarios with a path and no persons | Cross-track error (mean, p95), goal reached | Reported |
| **N3** Obstacles and persons | Seeded scenarios with moving persons (ground-truth metrics) | Violations (true distance below limit), freezing (as defined in `docs/13` §3), fallback fraction | Reported; violations must be zero for a gate to be considered |
| **N4** Solver timing | Same scenarios, solver candidates | Solve time (sandbox), infeasible fraction, fallback fraction | Reported with the infeasible fraction alongside timing |
| **N5** Interface check | NMPC reference through the E3 walking test of `docs/14` §5 | Command tracking error vs. the lag model in §3 | Reported; checks §3 is a fair model |

Metrics come from ground truth in simulation only. The NMPC never receives the simulator base pose or true person positions.

---

## 9. Open items and non-claims

| # | Item | Owner / step |
| --- | --- | --- |
| N-O1 | Lag model parameters and command ranges (§3, §5) | N1 |
| N-O2 | Target computer and its solve budget (§6) | `docs/02` §2 |
| N-O3 | Solver choice (§6) | N4 |
| N-O4 | Person-separation values for a humanoid (§5) | project owner, from a safety analysis |
| N-O5 | Gaussian chance-constraint assumption on humanoid tracks (§5) | N3 |
| N-O6 | Whether the fallback stop in §7 is acceptable for the safe-state decision | `docs/14` §4.7 |

**Non-claims.** This document does not claim that the NMPC tracks any path, avoids any person, or stops safely on the T800, in simulation or on hardware. It does not claim the solver meets any timing budget. The Stage 0 results (`docs/13`) are for a planar double integrator and do not transfer to this model.
