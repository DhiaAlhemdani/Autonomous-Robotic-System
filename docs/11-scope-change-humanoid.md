# 11 — Scope Change: Humanoid Platform and Wider Operating Domain

**Status:** confirmed by the project owner on 2026-10-14, as recorded here. This is a change record under the rule in `docs/02` §8. It supersedes the platform and scope statements in docs 00–02 **once those documents are revised**. Docs 00–02 are not edited by this record. Their revision waits for the dilemma decision in `docs/07` §4, as previously agreed.

---

## 1. What changed

| Item | Previous (docs 00–02) | Confirmed now |
| --- | --- | --- |
| Platform | Differential-drive mobile robot, approx. 0.5 × 0.4 m, 20 kg | **Humanoid robot** (morphology, mass and size to be specified) |
| Locomotion | Wheeled; wheel torque and `(v, ω)` references | **Legged, bipedal** (locomotion is part of the system) |
| Operating domain | Unmapped, GPS-denied indoor facility; people as hard safety constraint | **Everything static and movable**: people, animals, static objects, movable objects |
| Out of scope (`docs/00` §5) | Manipulation, legged locomotion, outdoor/GPS operation, long-range perception beyond sensor envelope, certification claims | Legged locomotion is **no longer** out of scope. The other items are **still listed as open** until confirmed (see §5) |

## 2. Consequences that must be addressed before docs 00–02 are revised

**2.1 Platform and actuation (`docs/00` §4, `docs/02` §1–§3, `docs/theory/01`, `docs/theory/02`)**
- The wheel-based kinematic model (theory 01) and the wheel-torque references (`docs/02` L3 output) no longer apply. A humanoid needs whole-body or gait-level control.
- The PMSM/FOC drive design (theory 02) describes wheel drives. Humanoid joint actuation is a different design problem.
- The MCU, wheel odometry and encoder assumptions in `docs/02` §2–§3 must be re-derived for joint-level sensing.

**2.2 Locomotion is a new layer**
- Legged locomotion stability (balance, footstep or gait control, fall handling) is a separate control problem. It is not covered by the existing layers.
- A practical option is to adopt an existing locomotion controller and build navigation, perception and planning above it. The choice and its evaluation must be recorded before any work depends on it.

**2.3 Requirements (`docs/01`)**
- Speed, mass, stopping, ramp and power requirements (PR-01, PR-10, BC-01, BC-04, BC-06 and others) are written for the wheeled robot. All must be re-derived for the humanoid.
- Safety requirements (SF-01 onwards) are about people and remain relevant. Their stop chains and fall behaviour change.

**2.4 Scope and feasibility (`docs/00` §5)**
- The scope discipline and the "Core / Advanced / Stretch" tiers were set for a wheeled platform. The Core tier must be re-evaluated for a humanoid before any milestone is committed.
- Adding legged locomotion and open-ended object classes together enlarges the project beyond what the earlier feasibility argument covered. This is the main risk of this change (see §4).

**2.5 Simulation, budget and roadmap (`docs/03`, `docs/04`)**
- The Gazebo model, Docker images and scenario suite (`docs/03`) must include a humanoid model.
- The budget (`docs/04`, T2 + T3 at $2,200–$4,500) and the build roadmap were built around a wheeled robot. Both need recalculating.

**2.6 Comparators and evidence (`docs/08`, `docs/09`, `docs/10`)**
- The comparisons in docs 09 and 10 use wheeled and quadruped systems. Humanoid comparators already exist there (Atlas, Digit, Optimus, Figure, Unitree G1, Walker S2), but the "Ours (as specified)" column in doc 09 and the gaps in doc 10 are written for the wheeled design, and must be rewritten.

**2.7 Theory (`docs/theory/`)**
- Theory 01 (kinematics), 02 (electric drive), 03 (state estimation: wheel–IMU fusion), 07 (perception) and 08 (safety) each assume the wheeled platform. Each needs a review. Theory 09 (planning among moving agents) is platform-independent except for its motion model.

## 3. Operating domain: what "everything static and movable" must mean

As stated, "everything" has no finite specification, and a system that must handle everything cannot be tested against a finite set of cases. This record therefore proposes a bounded form for the next revision. **This is a proposal, not a confirmed decision.**

- **Class-specific behaviour** for a fixed set of classes: person, animal, movable object (for example a box or a trolley), static object.
- **Geometric handling** for any other obstacle: occupancy from the sensors, with the unknown region treated as occupied until observed.
- **Animals** are the least predictable class and have the least public data. They are proposed for the Advanced tier, with the Core tier restricted to people and static or movable objects.

## 4. Feasibility risk

Three additions are made at once: a new platform, legged locomotion, and an open operating domain. Each is a substantial project. Their combination is very likely beyond one person's first pass, which is the reason the original scope was narrow. Before any milestone is committed, the scope needs a stated primary metric and a fixed Core subset. Otherwise, the portfolio value described in `docs/00` §5 (depth in two or three milestones with measured results) is at risk.

## 5. Open decisions

1. Which humanoid platform, and whether a locomotion controller is adopted or built.
2. Whether outdoor and GPS-denied outdoor operation remain out of scope.
3. The bounded set of object classes in §3, and whether animals are Core or Advanced.
4. Whether manipulation remains out of scope (a humanoid may be expected to interact with objects).
5. The Core subset that will be committed to, with its primary metric.
6. Revision of docs 00–02 and the affected theory documents, to be confirmed as a separate step.

## 6. Datasets relevant to the new platform

Public datasets were searched (see the conversation record for details). No humanoid social-navigation dataset was found. The closest wheeled and quadruped options are JRDB, SiT, SCAND and PeRoI. Animal and general-obstacle datasets were not searched. Licences were not checked for any of these. They must be verified before use.
