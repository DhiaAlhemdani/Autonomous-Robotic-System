# 13 — Stage 0 Report: Constant-Velocity NMPC in Simulation

**Status:** first measured run, in simulation only, on the development sandbox. **Nothing here is a property of the physical robot.** The solve times are for one machine and one solver, and they are not the NMPC rate, which is still undetermined (`docs/02` §9.2).

**Source:** `theory/09` §5 (Stage 0), scenario S2 (`docs/00` §2). Code: `planning/stage0/`. Tests: `planning/tests/`. Runner: `planning/stage0/run.py`.

---

## 1. What was built

| Part | What it does | What it does not do |
| --- | --- | --- |
| Robot model | Planar double integrator, acceleration commands, speed and acceleration limits | Humanoid dynamics, gait, balance, joint limits |
| Predictor | Constant-velocity prediction with a sigma that grows linearly (`theory/09` §3.2 baseline) | Learned prediction (Stage 1) |
| NMPC | Receding-horizon SLSQP, separation constraints with Gaussian margin `z_δ·σ` (`theory/09` §3.1) | Real-time implementation, analytic gradients, a production QP solver |
| Fallback | Brake against the velocity if the solve is infeasible | Any L2 safety filter or supervisor (`docs/02` §9.4) |
| Scenario S2 | Three persons with random-walk velocity, two static objects, seeded | Persons who react to the robot (the freezing case, `theory/09` §3.3) |

## 2. Assumptions (all placeholders, see `planning/stage0/config.py`)

- Planning step 0.1 s, horizon 20 steps (2 s).
- Speed limit 1.0 m/s and acceleration limit 1.0 m/s². Neither is specified for the T800 (`docs/11` §7).
- Minimum separation 0.8 m for persons and 0.5 m for objects.
- Chance-constraint risk δ = 0.05, giving `z_δ` ≈ 1.645 (one-sided Gaussian).
- Agent observation noise 0.10 m; prediction sigma grows at 0.20 m/s.
- Person velocity random walk, acceleration noise 0.3 m/s² per √s. The predictor does not model it.

## 3. Method

- 50 seeds (0–49). Each seed fixes the start, goal, person and object placement, and all noise. Episodes are reproducible from the seed (tested).
- Metrics are computed from ground truth, not from observations. A violation is a step where the true distance to a person is below 0.8 m, or to an object below 0.5 m.
- Freezing is a step where the robot's speed is below 0.05 m/s, it is more than 0.5 m from the goal, and a person is within 2 m.
- Solve times are wall-clock per call, in Python, on the sandbox CPU.

## 4. Results (sandbox, 50 seeds, `runs/stage0_sandbox_50seeds.json`, not committed)

| Metric | Value |
| --- | --- |
| Goal reached | 98% of episodes (49/50) |
| Episodes with a separation violation | 4% (2/50) |
| Episodes with a freezing event | 2% (1/50) |
| Infeasible steps (brake fallback applied) | 9.5% of all steps (489/5148) |
| Minimum true distance to a person, p5 / p50 | 1.21 m / 1.94 m |
| Solve time, p50 / p99 / max | 122 ms / 1.12 s / 2.20 s |

The run took 14 minutes on the sandbox.

## 5. What these results do and do not show

- **Solve time fails the design rate.** `docs/02` §1 puts the NMPC at 20–50 Hz, which is a budget of 20–50 ms. The p50 here (122 ms) is 2.4 to 6 times that budget, and the p99 (1.12 s) is 20 to 55 times it. The baseline solver (SLSQP with finite-difference gradients) is the likely cause, but this has not been tested. The result is a finding about this solver only.
- **The infeasible fraction is high, and it is a solver failure.** About one step in ten fell back to braking. Every one of the 489 infeasible steps was a solver failure (§9). None was a constraint violation, so the 1e-4 tolerance is not the cause. Whether the underlying problems are feasible is not yet known.
- **Violations are rare but occur.** Two episodes went below the margin. The likely causes are the prediction error from the random-walk persons and the braking steps, but this has not been separated.
- **Freezing is almost absent here** (one episode). This is expected: the persons do not react to the robot, so the freezing failure mode in `theory/09` §3.3 cannot appear in this scenario. The Stage 0 result for freezing is therefore not evidence against Approach B.
- **The robot model has no mismatch.** The robot tracks its planned acceleration exactly. Real tracking error would make the margins smaller in practice, and this is not modelled.
- **Three persons, two objects, one scenario family.** The sample is too small to estimate tails. The p5 and the violation rate have wide uncertainty.

## 6. Acceptance for Stage 0 (`theory/09` §5, §6)

| Item | Status |
| --- | --- |
| Constant-velocity NMPC with margin constraints | Built; tested in simulation |
| Metrics defined and computed from ground truth | Done |
| Scenario S2 with seeds | Done (scripted; persons do not react) |
| Solve time measured | Done for the sandbox only |
| NMPC rate set on target hardware | **Not done**. The rate is undetermined. |
| Freezing measured on a reacting-person scenario | **Not done** (needs Approach B's comparison) |

## 7. Next steps (in order)

1. ~~Log the solver status and message for each infeasible step.~~ Done (§9).
2. Test whether the infeasible steps are real infeasibility or solver failure: re-solve them with a fresh start and with analytic gradients (diagnosis only; the controller stays unchanged until the cause is known).
3. Run the same runner on the laptop with `docker compose --profile sim-only run --rm sim`, and record its solve times as the first laptop measurement.
4. Only then choose a faster solver (analytic gradients or a QP-based SQP). Its rate is set by the measured p50 and p99 on target hardware, not assumed.

## 8. Limits

- Not a physical measurement. Not a property of the T800.
- Solve times are for one solver on one machine.
- The sample is 50 seeds of one scripted scenario.
- The Gaussian margin is an approximation (`theory/09` §3.3).

## 9. Diagnosis of infeasible steps (step 1)

Same 50 seeds, rerun with the solver status logged per step. The controller is unchanged, and the summary metrics reproduce §4 exactly. File: `runs/stage0_diag_50seeds.json`, not committed.

| Reason | Steps | Share of infeasible steps |
| --- | --- | --- |
| Solver failed (`SolveResult.reason = solver_failed`) | 489 | 100% |
| Constraint violated on a solver "success" | 0 | 0% |

Solver messages:

| SciPy message | Steps |
| --- | --- |
| Positive directional derivative for linesearch | 462 |
| Inequality constraints incompatible | 15 |
| Iteration limit reached | 12 |

**What this shows.** The main failure is the SLSQP line search breaking down, which points at the solver setup: finite-difference gradients on constraints that are non-convex (circular exclusion zones) and warm-started from the previous solution. This is a hypothesis; it has not been tested. The 15 "constraints incompatible" steps may be real infeasibility of the linearised subproblem.

**What this does not show.** It does not show that the problems are infeasible, that the braking is wrong, or that the solver is wrong for the production setting. It does not explain the two episodes with separation violations; those were not linked to specific steps.

## 10. Step 2: re-solving the infeasible steps

Method (`planning/stage0/diagnose.py`): the baseline controller runs unchanged on seeds 0–19. At each step where it fell back to braking, the same problem is re-solved with a fresh solver from four starting guesses: zeros, constant acceleration toward the goal, constant braking, and a random sequence. A step counts as a solver failure if any start returns a feasible solution. This is a sample of starts, not a feasibility proof.

Results (`runs/stage0_step2.json`, not committed):

| Outcome | Steps | Share |
| --- | --- | --- |
| Feasible solution found from another start (solver failure) | 21 | 16% |
| No feasible solution from any of the four starts | 113 | 84% |

Starts that first found a feasible solution: zeros 14, toward goal 4, brake 3.

The line search failed ("Positive directional derivative for linesearch") in roughly 110 of 113 re-solves from **every** start. The failure does not depend on the starting point.

**What this shows.** Some of the braking is avoidable by a better start (16% of infeasible steps). The rest is not explained by the start, and the result is consistent with either genuine infeasibility or a solver that breaks down on these problems. The "best constraint value" over the starts was negative in every case (median −0.47 m², closest −0.024 m²), so none of the four starts produced a feasible point.

**What this does not show.** It does not show that the 113 are infeasible. It does not show the controller is wrong for the production setting.

**Next test (step 2b).** Separate the two cases directly: relax the separation constraints with a slack variable and minimise the slack. If the minimum slack is clearly above zero, the problem is infeasible given the robot model and the margins. If it is zero, the solver is failing on a feasible problem. This is also the slack-relaxed step that `docs/02` §4 specifies for infeasible NMPC problems, so it tests the design's own fallback.

