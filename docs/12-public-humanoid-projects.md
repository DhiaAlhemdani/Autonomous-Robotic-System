# 12 — Public Humanoid Projects to Build On

**Purpose:** a short map of public humanoid projects that the scope in `docs/11` could start from, so the project extends existing work rather than rebuilding it.

**Date of snapshot:** 2026-10-14. Star counts, update dates and maturity signals come from GitHub topic pages and project pages at that date.

**How to read this file**

- **"Advanced" is not a verified property here.** Nothing below has been run in this project. Each entry records what its public materials claim, and the quality of that evidence.
- **Quality flags:** *Primary* = the project's own repository or paper. *Secondary* = an aggregator, blog or summary. *Unverified* = claim not checked against the code.
- **Licences** are noted only where a source gave them. Each must be checked in the repository before reuse. Non-commercial licences restrict some uses.

---

## 1. Locomotion and reinforcement-learning frameworks

| Project | What it provides | Maturity signal | Licence | Relevance to `docs/11` | Quality |
| --- | --- | --- | --- | --- | --- |
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | RL training (Isaac Gym), MuJoCo sim-to-sim checks and deployment code for Unitree Go2, H1, H1_2 and G1 | Vendor-maintained; last commit July 2025 in our earlier check | Not checked | High: the most direct starting point for a G1-class humanoid locomotion layer | Primary (vendor code); performance claims not checked |
| [humanoid-gym](https://github.com/roboterax/humanoid-gym) | RL for humanoid locomotion with zero-shot sim-to-real transfer, from the paper linked in the repository | Last updated January 2025 | Not checked | High for locomotion; older than other entries | Primary |
| [ASAP](https://github.com/LeCAR-Lab/ASAP) | Aligning simulation and real-world physics for agile humanoid skills (RSS 2025) | About 2.1k stars; updated January 2026 | Not checked | Medium: agile skills, not navigation | Primary (paper and code) |
| [TienKung-Lab](https://github.com/Open-X-Humanoid/TienKung-Lab) | Isaac Lab workflow for legged and humanoid robots, with sim-to-sim tooling | Updated July 2026 | Not checked | Medium to high: a current IsaacLab workflow | Primary |
| [HoloMotion](https://github.com/HorizonRobotics/HoloMotion) | Foundation model for whole-body humanoid control and motion tracking | Updated September 2026; about 720 stars | Not checked | Medium: a tracking base, not navigation | Primary; claims not checked |
| [BFM-Zero](https://github.com/LeCAR-Lab/BFM-Zero) | Promptable behavioural foundation model for humanoid control, unsupervised RL | Updated July 2026 | Not checked | Medium: research-level | Primary |

## 2. Whole-body control and model-based control

| Project | What it provides | Maturity signal | Licence | Relevance | Quality |
| --- | --- | --- | --- | --- | --- |
| [wb_humanoid_mpc](https://github.com/manumerous/wb_humanoid_mpc) | Whole-body nonlinear MPC for real-time humanoid loco-manipulation planning and control, in C++ | About 380 stars; updated October 2026 | Not checked | High for the MPC layer in `docs/04` and theory 04 | Primary |
| [IHMC open robotics software](https://github.com/ihmcrobotics/ihmc-open-robotics-software) | Legged locomotion algorithms and a momentum-based controller core with optimisation, used on several humanoids | Long-running project, updated October 2026 | Not checked | High as a reference architecture; Java and a steep learning curve | Primary |
| [X-WBC](https://github.com/LogosRoboticsGroup/x-wbc) | Cross-embodiment foundation model for whole-body control (CoRL 2026) | Updated September 2026 | Not checked | Medium: research, from a paper | Primary |

## 3. Navigation and humanoid integration

These are the most relevant entries for `docs/11`, because they connect navigation to a humanoid.

| Project | What it provides | Maturity signal | Licence | Relevance | Quality |
| --- | --- | --- | --- | --- | --- |
| **rl_hnav** (ULE Robotics Group), described in [this DeepWiki summary](https://deepwiki.com/uleroboticsgroup/rl_hnav) | Unitree G1 with ROS 2 Nav2 and SLAM driving an RL locomotion policy, with a bridge that filters `/cmd_vel` and applies safety timeouts. Simulation in MuJoCo and Gazebo, with real-robot deployment | Summary last checked March 2026. The repository itself was not opened | Not checked | **Very high**: the closest existing design to our navigation-over-locomotion split | Secondary (DeepWiki summary); verify against the repository |
| [g1pilot](https://github.com/hucebot/g1pilot) (hucebot) | ROS 2 package for the Unitree G1. Keeps Unitree's built-in locomotion controller and adds upper-body joint and Cartesian control, odometry and a planner | Docker-first build on ROS 2 Humble; November 2025 | Not checked | High for the ROS 2 interface and the navigation integration | Primary (README) |
| [Dropbear](https://github.com/Hyperspawn/dropbear-wbc) | Whole-body control for an open 3D-printed humanoid, with motion tracking and walking in Isaac Lab | Updated October 2026 | Not checked | Medium: a low-cost open humanoid | Primary (README) |

## 4. Open hardware platforms

| Platform | Notes | Licence and openness | Relevance | Quality |
| --- | --- | --- | --- | --- |
| [Booster T1](https://www.chipsilicon.com/humanoid/booster-t1-humanoid-robot) | 118 cm, about 30 kg, 23 DoF base, ROS 2-compatible SDK, Booster GYM RL framework, Jetson AGX Orin plus a motion-control board | Described as open-source with an SDK on GitHub | High: a full-size humanoid with a documented SDK | Secondary (product aggregator); verify specifications on the vendor page |
| [Asimov v1](https://github.com/menloresearch/asimov-v0) (Menlo Research) | 25 active and 2 passive DoF per its README; aluminium frame with nylon parts; CAD, electronics and MuJoCo models in one repository | Hardware CERN-OHL-S-2.0; software GPL-2.0 (per an Awesome list) | Medium: open hardware; control API still on the roadmap per the listing | Primary (README); roadmap not verified |
| Open Duck Mini (apirrone), [build log and runtime notes](https://github.com/seanmayer/open-duck-mini-build) | About 42 cm, BOM under $400, Pi Zero 2W, MuJoCo sim-to-real | Apache-2.0 (per an Awesome list) | Low for our scale, high for learning the sim-to-real pipeline cheaply | Secondary; verify on the upstream repository |
| Berkeley Humanoid (UC Berkeley) | Low-cost humanoid with quasi-direct-drive actuators and a simple Python and C++ framework (per an aggregator summary) | Not checked | Medium | Secondary; repository not located or checked |

## 5. Foundation models (no usable code)

| Project | Status | Quality |
| --- | --- | --- |
| [WholeBodyVLA](https://github.com/OpenDriveLab/WholebodyVLA) (ICLR 2026) | The repository says there is currently no concrete timeline for open-sourcing the codebase. It is a resource list | Primary; not usable as code |

---

## 6. Assessment against the scope in `docs/11`

- **Locomotion** can be taken from existing work. unitree_rl_gym, humanoid-gym, ASAP and TienKung-Lab all provide RL locomotion with sim-to-real claims, and none has been checked on our platform.
- **Navigation over a legged base** is the least covered area. rl_hnav is the closest existing design found, combining Nav2 and SLAM with an RL locomotion policy on a humanoid. It should be examined in its repository before any design is fixed.
- **Model-based whole-body control** has a usable open implementation in wb_humanoid_mpc, and a reference architecture in IHMC.
- **Perception of people and animals** is not covered by any of these projects. It has to be built, or taken from separate perception work (see the datasets in `docs/11` §6).
- **Hardware choice** decides which software can be used directly. A G1-class platform has the most community support in this list. Booster T1 has a documented SDK. Asimov and Open Duck Mini are open hardware but less mature for this scope.

## 7. Recommended next steps

1. Examine rl_hnav in its repository, not the summary, and check its licence and any reported hardware results.
2. Check the licences of the entries above before reuse. Several datasets and platforms carry non-commercial terms.
3. Decide the platform (`docs/11` §5, question 1). The software options depend on it.
4. Record the chosen base in a change record under `docs/02` §8, with the reuse terms.

## 8. Limits

- Search results and summaries are secondary. Each repository must be read before its claims are relied on.
- Star counts and update dates are a snapshot, and they measure attention more than quality.
- "Most advanced" was not measured. The entries are selected for relevance to `docs/11`, not ranked.
