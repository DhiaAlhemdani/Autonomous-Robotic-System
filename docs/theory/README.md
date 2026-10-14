# Theory Series

The derivations, models and assumptions behind every controller in this system. Written before implementation, and kept as the reference that the code is checked against.

| # | Document | Covers | Feeds layer |
| --- | --- | --- | --- |
| 01 | [Kinematics and Dynamics](01-kinematics-and-dynamics.md) | Frames, differential-drive kinematics, nonholonomic constraints, Lagrangian dynamics, discretisation, parameter identification | L3, L4 |
| 02 | [Electric Drive: PMSM and Field-Oriented Control](02-electric-drive-pmsm-foc.md) | dq model, torque production, FOC cascade, MTPA, current-loop bandwidth, NMPC on the current loop, inverter constraints | L1 |
| 03 | [State Estimation](03-state-estimation.md) | EKF/UKF, wheel–IMU fusion, slip detection, LiDAR SLAM, VIO, observability and failure modes | L0, L3 |
| 04 | [Advanced Control: Nonlinear MPC](04-advanced-control-nmpc.md) | OCP formulation, discretisation, solvers, real-time strategies, stability, robust and tube MPC, tracking parameterisation | L3 |
| 05 | [Reinforcement Learning](05-reinforcement-learning.md) | MDP/POMDP, policy-gradient methods, SAC/PPO, model-based RL, constrained RL, sim-to-real | L5 |
| 06 | [Integrating RL with MPC and Safety](06-rl-mpc-integration-and-safety.md) | Four integration topologies, control barrier functions, safety filters, feasibility, discrete-time guarantees | L2, L5 |
| 07 | [Perception and Deep Learning](07-perception-dl.md) | Detection/segmentation/depth, traversability, learned costmaps, uncertainty, latency budget, edge deployment | L0 |
| 08 | [Safety, Verification and Validation](08-safety-and-verification.md) | Hazard analysis, relevant standards, safety architecture, V&V plan, metrics, scenario generation | all |
| 09 | [Planning Among Moving Agents](09-interaction-aware-planning.md) | Prediction-conditioned NMPC (Core) and interaction-aware planning (Advanced), staged plan, evaluation protocol | L3 |

## How to use this series

Each document follows the same structure:

1. **Purpose** — which requirement and which layer it serves.
2. **Model / derivation** — the equations, with the physical meaning of each term.
3. **Assumptions and limits** — explicitly listed, because every model is wrong somewhere and the interesting engineering is knowing where.
4. **Design decisions** — what the model implies for the implementation (rates, tolerances, structure).
5. **Verification** — how the claims in the document will be tested once implementation begins.
6. **Open questions** — what is unresolved and what decision will close it.

## Working convention

- Symbols are defined in each document; no symbol is used before definition.
- Every non-obvious equation has a reference in [`../06-reading-list.md`](../06-reading-list.md).
- Numerical values quoted as *targets* are design targets, not measured results. Anything measured is labelled **measured** with its conditions.
- When a derivation and an experiment disagree, both are recorded. The disagreement is usually the most informative result in the project.
