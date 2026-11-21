# 17 — Legged NMPC (T800, reduced model): design declaration

**Status: design declaration, written before any code. Simulation only. Not validated on hardware. Not a walking controller for the T800.**

## 1. Scope

- A nonlinear model predictive controller (NMPC) for the centre of mass (CoM) of a two-foot, point-contact legged robot, written from scratch in `planning/legged_nmpc/`.
- The robot is the T800 in `vendor/t800/`. Only two numbers are taken from the model: total mass and nominal CoM height (measured with MuJoCo, see §2). Everything else is a declared ASSUMPTION.
- Techniques (general, standard in the field): direct multiple shooting, a receding horizon, warm starts from the previous solution, and a nonlinear programme solved per control step with IPOPT through CasADi. Contact forces per foot are decision variables. Contact constraints: friction pyramid, unilateral normal force, and a zero-moment-point (ZMP) constraint that keeps the centre of pressure inside the support region of the feet in contact.
- Not in scope: whole-body control, joint-level torques, footstep planning (the footstep plan is an input), state estimation, perception, the walking policy, hardware interfaces, and any claim about the T800's real dynamics. This controller is not the locomotion layer described in `docs/14`.
- No code or design is copied from any other project. The reference repository `wei-hsuan-cheng/legged_robot_mpc_controller` has no licence and is not used.

## 2. Model (ASSUMPTION values in `planning/legged_nmpc/config.py`)

- State `x = [px, vx, py, vy, pz, vz]` (CoM position and velocity, world frame).
- Inputs `u = [f_1, f_2]`, the ground reaction force of each of the two feet, `f_i = [fx_i, fy_i, fz_i]`, applied at the foot centre `r_i`.
- Dynamics: `CoM'' = (f_1 + f_2) / M + g`, with `g = [0, 0, -9.81]`. Discretised exactly under a constant force over one step `dt` (the update is linear in the forces).
- `M = 84.917 kg`, the total mass of `vendor/t800/.../serial_t800.xml` (MuJoCo sum of body masses).
- Nominal CoM height `Z_NOM = 0.967 m`, from the model's CoM at its default pose (`subtree_com[0]`, z).
- Foot centres at the default pose (from `LINK_FOOT_L` and `LINK_FOOT_R` positions): `(-0.025, ±0.12)`. Foot support box half-sizes `(0.10, 0.05) m`: ASSUMPTION, the model's foot geometry was not measured.
- Friction coefficient `MU = 0.6`: ASSUMPTION.
- Normal force bounds `0 <= fz_i <= FZ_MAX`: `FZ_MAX = 2 * M * g` ASSUMPTION.
- A swing foot has zero force; the stance schedule is an input to the problem.

## 3. Constraints

- Friction pyramid per stance foot: `|fx_i| <= MU * fz_i`, `|fy_i| <= MU * fz_i`, `0 <= fz_i <= FZ_MAX`.
- ZMP inside the support region. Let `F_z = fz_1 + fz_2`, `a_x = (fx_1 + fx_2)/M`, `a_y = (fy_1 + fy_2)/M`, and `a_z + g = F_z / M`. The ZMP is `zmp_x = px - pz * a_x / (a_z + g)`, similarly for y. Requirement: `zmp_x` in the support box `[xmin, xmax]` of the active stance (both feet's boxes united in x and y when both are in contact), written in product form to stay polynomial. Requirement `F_z > 0`.
- Terminal state: no hard terminal constraint. The terminal cost penalises the tracking error.
- Hard constraints are not softened. If a solve is infeasible, the solver status is reported. No slack variables.

## 4. Problem

Horizon `N` steps. Decision variables: states `x_0..x_N` and inputs `u_0..u_{N-1}`. Minimise

`sum_k ( |v_k - v_ref_k|^2 * Q_V + |p_k - p_ref_k|^2 * Q_P + |u_k|^2 * R_F ) + terminal term`

subject to: multiple-shooting dynamics equalities, friction pyramid, force bounds, ZMP-in-support (per stage, with the stance and footstep plan as parameters).

The reference `p_ref`, `v_ref` is a constant-velocity command from a given speed command, with the current CoM as the starting point.

## 5. Solver and receding horizon

- Solve at each control step. Apply the first input. Shift the previous solution for the warm start.
- Solver status is reported on every step (success, maximum iterations, infeasible).
- Solve times are sandbox measurements, not target-hardware figures.

## 6. Tests (declared before implementation)

1. Model: the discrete update matches the analytic double-integrator solution for a constant force to within 1e-9.
2. Standing: both feet in contact, zero velocity command, starting 0.02 m from the nominal CoM position. After 3 s the CoM position error is below 1 mm. All solves succeed. Maximum ZMP-in-support violation is below 1e-6 m.
3. Walking command: speed command 0.3 m/s along x, with alternating single- and double-support phases from a fixed gait schedule and footstep plan, for 10 s. The mean velocity error over the last 5 s is below 0.02 m/s. All solves succeed. The ZMP stays in the active support region (violation below 1e-6 m), and friction and force bounds hold at every stage.
4. Determinism: the same seed gives the same trajectory.
5. Input bounds: applied forces satisfy the friction pyramid and the force bounds exactly.

## 7. Seeded sweep (declared, reported, no gate)

- Scenes drawn from fixed seeds: speed command in [0.1, 0.4] m/s, initial CoM offset within 0.02 m, a push of 0 to 60 N for 0.2 s applied during the run.
- Tuning seeds 0–9 are used for any choice of `config.py`. Held-out seeds 10–19 are evaluated once, with no tuning on them.
- Report: solver failures, the ZMP violation maximum, the mean velocity error, the maximum CoM deviation from its reference, and solve time p50/p99 (sandbox). Also report the fraction of scenes that stay inside a 0.1 m deviation from the reference, labelled as a reporting measure, not a pass criterion.
- The sweep reports results; it does not gate any claim beyond the reduced model in simulation.

## 8. What this does not claim

- No claim that the T800 walks, balances, or survives any push. The plant is the same reduced model, so the sweep tests the controller and the model together.
- No real-time guarantee on the target hardware; solve times are sandbox measurements.
- No recursive feasibility guarantee; infeasible solves are reported, not hidden.
- No robustness claim against model mismatch, joint limits, actuator limits, contact-timing errors, sensor noise, or the T800's real dynamics.
- No footstep planning, no swing-leg control, no yaw dynamics.
- No licence-dependent code. Nothing from `legged_robot_mpc_controller` is copied.

## 9. Results

_To be filled in after implementation, with the commit and the exact outputs._
