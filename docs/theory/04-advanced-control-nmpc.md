# Theory 04 — Nonlinear Model Predictive Control

**Purpose:** the central advanced-control component of the system. Trajectory tracking and obstacle-constrained local planning at 20–50 Hz with hard input constraints. Requirements: FR-04, FR-05, PR-02, PR-05.

---

## 1. Why MPC for this platform

Three properties of the problem make MPC the right tool, and each corresponds to something a classical tracker cannot do:

1. **Hard input constraints** on `(v, ω, a_v, a_ω)` and on traction — MPC enforces them by construction. A pure-pursuit or LQR tracker with saturation violates them exactly when it matters most (hard braking).
2. **Obstacle avoidance is a state constraint**, not a tracking objective. Encoding "do not enter this region" is natural in MPC and awkward in a tracker.
3. **Predictive stopping** matters more than path accuracy. If a human appears 1.5 m ahead, the correct action depends on the *predicted* stopping distance over the horizon, not on the current cross-track error.

## 2. The optimal control problem

At each sampling instant, with the current estimate `x̂_k`, solve over the input sequence `U = {u_0, …, u_{N−1}}`:

```
minimise  J = Σ_{k=0}^{N−1} [ ‖x_k − x_k^ref‖²_Q  +  ‖u_k‖²_R  +  ‖u_k − u_{k−1}‖²_{R_d} ]
              + ‖x_N − x_N^ref‖²_P  +  Σ_k ρ · s_k

subject to
    x_0   = x̂_k                                    initial condition (the estimate)
    x_{k+1} = f_d(x_k, u_k)                        discrete plant model (doc 01)
    u_k ∈ U                     = { (v,ω,a) : |v| ≤ v_max, |ω| ≤ ω_max, |a| ≤ a_max }
    x_k ∈ X                     = { kinematic limits, comfortable-acceleration envelope }
    g_obs(x_k) ≥ d_safe − s_k                      obstacle distance constraint, softened by slack s_k ≥ 0
    κ(g) ≤ v_max(g)                                speed limited by measured clearance
```

| Term | Role |
| --- | --- |
| `Q` | Pose/velocity tracking weight. Large `Q` on position ⇒ aggressive tracking |
| `R` | Control effort. Penalises velocity magnitude and large accelerations; the main knob for smoothness |
| `R_d` | Input-rate penalty. Suppresses chatter and protects the drivetrain — in practice the most effective smoothness lever |
| `P` | Terminal weight. With a terminal set it provides stability guarantees; without one it is a tuning aid (see §6) |
| `ρ · s_k` | Slack penalty. **Obstacle constraints must be soft**: the robot may be boxed in with no feasible escape, and an infeasible QP at 30 Hz on a moving robot is worse than a violated soft constraint. A large `ρ` keeps violations small while preserving feasibility |

### 2.1 The obstacle constraint

Two representations, with an explicit trade-off:

| Representation | Constraint | Property |
| --- | --- | --- |
| **Signed distance field (SDF)** from the occupancy grid | `SDF(x_k) ≥ d_safe` | Smooth and informative — the gradient gives the escape direction; requires a precomputed distance field (cheap to maintain, fast to query) |
| **Discrete occupancy + inflation** | `cell(x_k) ∉ inflated_obstacle_set` | Simple and exact for the discrete map; non-smooth, and the gradient is undefined at cell boundaries |

**Decision:** use an **SDF with a safety margin that varies with speed** — `d_safe(v) = d_0 + v²/(2 a_brake) + v·t_react`. The velocity-dependent margin encodes the stopping requirement directly in the constraint, so the controller cannot choose a speed from which it cannot stop. This single design choice is what makes the MPC's obstacle handling defensible in a safety argument, and it is implemented as a function, not scattered as magic numbers.

### 2.2 Tracking parameterisation

Following a geometric path with `x_ref` sampled by time causes the robot to slow at corners and to rush on straights if the path is sampled poorly. The robust approach is to add the **path parameter `θ_p` as a decision variable**:

```
x_k^ref = path(θ_{p,k}),      θ_{p,k+1} = θ_{p,k} + v_k Δt / (1 + curvature term)
```

with a small penalty encouraging forward progress. This is "MPC on a spatial path with progress optimisation" and it removes a whole class of tuning problems. Cost: extra decision variables and a slightly larger QP. Worth it.

## 3. Discretisation and solving

### 3.1 Direct methods

| Approach | Formulation | Suitability |
| --- | --- | --- |
| **Single shooting** | Eliminate states; optimise inputs only | Fewest variables; sensitive to instability, poor for long horizons |
| **Multiple shooting** | States are decision variables + defect constraints | **Recommended** — better conditioned, warm-startable, tolerant of a nonlinear model |
| **Collocation** | Implicit integration within the NLP | Best accuracy per node; larger NLP; favourable when the dynamics are stiff |

### 3.2 Algorithms

| Method | Rate | Notes |
| --- | --- | --- |
| **SQP with real-time iteration (RTI)** | Highest — one QP per sample | The standard choice for fast NMPC; the RTI variant solves a linearised subproblem per step and is what makes 50 Hz feasible |
| **Interior point** (e.g. IPOPT) | Slower, more robust | Excellent for offline validation of the RTI solution and for trajectory design |
| **ADMM / OSQP** on the QP subproblem | Fast, structure-exploiting | Good when the QP is large and sparse |
| **Gradient/GA** | Offline only | Not for real-time |

**Recommended toolchain:** model the OCP in **CasADi** (symbolic, automatic differentiation, interfaces to IPOPT and QP solvers) and generate real-time C code with **acados**, which exploits the structure of the multiple-shooting problem and typically delivers solve times one to two orders of magnitude below a generic NLP solver. Prototype offline in Python (CasADi + IPOPT), verify the solution, then export generated C for the deployment path.

### 3.3 Solve-time budget (PR-05)

| Horizon | `N` | `Δt` | Warm-started RTI target | Hard deadline |
| --- | --- | --- | --- | --- |
| Short | 20 | 50 ms | ≤ 10 ms | 50 ms |
| Nominal | 30 | 50 ms | ≤ 20 ms | 50 ms |
| Long | 40 | 50 ms | ≤ 35 ms | 50 ms |

**Rules that make the budget achievable:**
- **Always warm-start** from the shifted previous solution. Cold starts are for offline analysis only.
- **Bound the iterations** (e.g. ≤ 2 RTI iterations) and accept a suboptimal solution. Suboptimal but on time beats optimal and late — the previous solution is already a good answer.
- **Exploit structure**: multiple shooting + RTI turns the problem into structured QPs that specialised (condensing) solvers handle in microseconds.
- **Measure p99 and max, not the mean.** A mean solve time of 8 ms with a 300 ms outlier is a crash, not a fast controller.
- **Never run the solver in the control-loop thread without isolation.** A dedicated thread/process with a lock-free hand-off (last solution wins) keeps the safety path alive if the solver stalls.

## 4. Stability and feasibility — what can and cannot be claimed

**What theory gives (with conditions that must actually be met):**
- With a **terminal cost `P` chosen as the value function of a stabilising local controller**, plus a terminal constraint set invariant under that controller, recursive feasibility and asymptotic stability of the origin can be established.
- Quasi-infinite horizon NMPC is a practical route to a computable `P` (linearise about the reference, solve the Lyapunov equation).

**What is true in practice on this system:**
- Obstacles are non-convex, softened constraints void the guarantee, the reference trajectory changes, and solver truncation means the *returned* solution is suboptimal. **No formal stability guarantee holds for the deployed controller.** This must be stated plainly.
- The defensible claim is: *bounded tracking error in the operating envelope, verified empirically over a scenario suite, with a defined fallback whenever feasibility is lost.* That is an engineering claim with evidence behind it, which is worth more than a theoretical guarantee whose assumptions are violated.

**Recursive feasibility in practice** is protected by a three-level mechanism (matching the degradation ladder in doc 02):
1. Slack on obstacle constraints (guarantees the problem is at least feasible if inputs are).
2. If infeasible anyway: relaxed solve with reduced speed envelope.
3. If still infeasible or late: switch to the verified reactive fallback with the stopping envelope, then protective stop.

## 5. Robustness: the real world is not the model

| Disturbance | Mitigation |
| --- | --- |
| Model error (radius, friction, mass) | **Constraint tightening** — reserve a margin in the constraints equal to the measured model error envelope |
| State estimation error | Tighten `d_safe` by the localisation uncertainty (use the covariance, not a constant) |
| Actuator delay | Include a delay in the model, or shift the applied input by the measured delay |
| Exogenous push / floor irregularity | Disturbance observer + feed-forward; or **tube MPC** (a robust variant with a fixed cross-section around the nominal trajectory) |
| Perception error (obstacle position) | Inflate obstacles by the measured perception error; treat unknown regions as occupied (conservative inflation) |

**Tube MPC** is the principled version of "add margin": compute an invariant tube around the nominal trajectory and constrain the tube rather than the nominal state. It converts unbounded model error into a bounded, provable margin. It is the *advanced tier* item here — the core tier uses constant tightening with the margin derived from measured error distributions.

## 6. Alternatives, and why they are baselines rather than the answer

| Method | Strengths | Why it is not the primary |
| --- | --- | --- |
| **Pure pursuit / Stanley** | Tiny compute, reliable, well understood | No constraint handling, no prediction, degrades near obstacle fields |
| **LQR / LQT** | Optimal, cheap, linear | Linear model is valid in a narrow envelope; no constraints |
| **Feedback linearisation** | Globally linearises the nominal model | Model-dependent, no constraint handling, sensitive to unmodelled dynamics |
| **CBF-QP alone** | Provably safe, very fast | Needs a nominal controller to shape performance; can stall (no progress) — hence used as a *filter* (doc 06), not the planner |
| **MDP / learned policy** | Handles perception-driven complexity | No guarantees, sample-hungry (doc 05) |

**Every one of these is implemented as a baseline.** The comparison against them, on identical scenarios with identical metrics, is how the NMPC's value is demonstrated — or shown not to exist for a given scenario.

## 7. Assumptions and limits

1. **The model is adequate.** Validated by the open-loop prediction test in doc 01 §7. If prediction error over one horizon is large, no amount of solver tuning will help.
2. **The state estimate is trustworthy** to a known covariance. Enforced by doc 03 §1.
3. **Obstacles are static over the prediction horizon**, except for humans handled by a separate safety layer (doc 06) and a conservative velocity-proportional margin. Dynamic-obstacle prediction is a stretch goal (doc 07).
4. **The solve completes within its deadline.** Monitored, not assumed; violation triggers the fallback chain.
5. **The horizon is long enough.** At 0.5 m/s with a 1.5 s horizon, the robot sees 0.75 m ahead — less than its own stopping distance at 1 m/s. This is a genuine limitation at high speed and the reason speed must be limited by the sensing envelope: **do not exceed the speed at which you can stop within the horizon of reliable perception.** This is a system-level constraint, not a controller parameter.

## 8. Verification

| Claim | Test | Metric (report with conditions) |
| --- | --- | --- |
| Tracking performance (PR-02) | Reference paths (straight, circle, S-curve, tight corridor) at fixed speeds, ≥ 5 seeds | Cross-track RMSE, heading RMSE, max deviation |
| Constraint satisfaction | Logged minimum distance to obstacles across all scenarios | Min distance, 1st percentile, count of violations |
| Real-time behaviour (PR-05) | Instrumented solver over every run | p50/p99/max solve time, iteration count, infeasibility count |
| Value of the horizon | Sweep `N` and `Δt` | Performance vs. compute; identify diminishing returns explicitly |
| Robustness | Inject model error, delay, estimation noise; re-run suite | Degradation in RMSE; margin adequacy |
| Fallback chain | Force solver timeout and infeasibility | Correct fallback triggered; stop distance ≤ verified envelope |
| Comparison vs. baselines | Same scenarios, pure pursuit + LQR + CBF-QP-only | Side-by-side metric table — the core evidence artifact for this layer |

## 9. Open questions

1. Is the SDF representation fast enough to query at every node every solve, or is a coarse grid plus local refinement required? → resolved by a solver profiling benchmark.
2. Can an embedded solver meet 50 Hz with `N = 30` on the chosen compute? If not, the horizon or the rate must be reduced — decided by measurement, before the controller is tuned.
3. Does `θ_p` (path-progress) optimisation make the problem non-convex enough to hurt solve times in tight corridors? → resolved by the corridor scenario benchmark.
