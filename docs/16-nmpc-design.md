# 16 — NMPC for a planar mobile base: design declaration

**Status: design declaration, written before any code. Simulation only. Not validated on hardware.**

## 1. Scope

- A nonlinear model predictive controller (NMPC) for a planar base with unicycle kinematics, written from scratch in `planning/nmpc/`.
- Techniques: direct multiple shooting, an RK4-discretised model, a nonlinear programme solved per control step with IPOPT (through CasADi), a receding horizon, warm starts from the previous solution, and circular obstacle constraints.
- Not in scope: humanoid whole-body control, walking, estimation, perception, and any hardware interface. No claim about any robot.

## 2. Model (ASSUMPTION values in `planning/nmpc/config.py`)

- State `x = [px, py, th]`, input `u = [v, w]` (forward speed, yaw rate).
- Dynamics `px' = v cos th`, `py' = v sin th`, `th' = w`, discretised with RK4 at step `dt`.
- Input bounds: `|v| <= V_MAX`, `|w| <= W_MAX`.

## 3. Problem

Horizon `N` steps. Decision variables: states `x_0..x_N` and inputs `u_0..u_{N-1}`. Minimise

`sum_k ( |p_k - p_goal|^2 * Q_P + |u_k|^2 * R_U + |u_k - u_{k-1}|^2 * R_DU ) + terminal term`

subject to the dynamics (multiple-shooting equality constraints), the input bounds, and for each obstacle `j` and each stage `k`: `|p_k - c_j|^2 >= (r_j + MARGIN)^2`. The obstacle constraints are soft through a slack variable with a large penalty, so that the problem stays feasible; slack usage is reported.

## 4. Solver and receding horizon

- Solve at each control step; apply the first input; shift the previous solution for the warm start.
- Solver status (success, max iterations, infeasible) is reported on every step. Solve times are sandbox measurements, not hardware figures.

## 5. Tests (declared before implementation)

1. Model: the RK4 step matches the analytic unicycle step for a constant input to within 1e-6.
2. Free space: from rest, the controller reaches a goal 3 m away within 15 s, final position error < 0.05 m.
3. Obstacle: with one obstacle on the straight path, the minimum clearance over a run is >= r + MARGIN - 0.02 m, with slack usage zero.
4. Input bounds: applied inputs respect `V_MAX` and `W_MAX` exactly.
5. Determinism: the same seed gives the same trajectory.

## 6. Seeded sweep (declared)

A seeded sweep over 20 random scenes (goal and obstacle positions drawn from a fixed seed; tuning seeds 0–9, held-out seeds 10–19). Report: goal reached (fraction), minimum clearance, slack usage, solver failures, and solve time p50/p99 (sandbox). The sweep reports results; it does not gate any claim beyond simulation.

## 7. What this does not claim

- No real-time guarantee on the target hardware; solve times are sandbox measurements.
- No recursive feasibility guarantee; infeasible solves are reported, not hidden.
- No robustness claim against disturbances, sensor noise, or model mismatch.
