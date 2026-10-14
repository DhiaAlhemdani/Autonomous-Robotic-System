# 03 — Simulation Environment and Container Architecture

Containerization is a **first-class requirement** in this project (SR-01 … SR-03), not a convenience. The reason is specific: this project combines deep-learning training, GPU simulation, real-time control and CI testing. Without containers, that combination produces the classic failure mode — *a result you cannot reproduce six weeks later because the CUDA, driver, ROS and library versions drifted*. Containers make the environment part of the deliverable.

## 1. Image architecture

Multi-stage, layered by change frequency: the slowest-changing layers sit at the bottom so rebuilds are cheap.

```
┌───────────────────────────────────────────────────────────────────────────────┐
│ docker/                                                                       │
│   base/            Ubuntu 24.04 + ROS 2 Jazzy + build tools + non-root user   │
│   sim/             base + Gazebo (gz-sim LTS) + ros_gz bridge + robot model    │
│   control/         base + CasADi + acados + NMPC node + solver benchmarks      │
│   ml-train/        CUDA runtime + PyTorch + Gymnasium + SB3/CleanRL + TensorBoard│
│   ml-deploy/       minimal runtime + ONNX Runtime/TensorRT + policy node       │
│   dev/             base + sim + control + ml-deploy + VS Code server + debug   │
│   ci/              headless: xvfb + scenario runner + report generator         │
└───────────────────────────────────────────────────────────────────────────────┘
```

| Image | Base | GPU | Approx. size target | Notes |
| --- | --- | --- | --- | --- |
| `base` | Ubuntu 24.04 | no | ≤ 1.2 GB | ROS 2 Jazzy LTS; single non-root user; no GUI |
| `sim` | `base` | optional (rendering) | ≤ 3 GB | Gazebo headless-capable; EGL for offscreen rendering |
| `control` | `base` | no | ≤ 1.5 GB | Compiles the NMPC; solver timing measured inside the container |
| `ml-train` | CUDA | **yes** | 8–12 GB | Pinned torch + CUDA pair; never `latest` |
| `ml-deploy` | `base` | optional | ≤ 2 GB | Deliberately excludes the training toolchain |
| `ci` | `sim` + test deps | no | ≤ 3.5 GB | Runs headless scenario suite, emits report |

**Rules**
1. Base images pinned by **digest**, not tag (tags move; digests do not).
2. Python dependencies pinned to exact versions and hashes (lockfile committed; `pip install` from lockfile only).
3. ROS 2 distribution pinned to an **LTS** release; upgrades are deliberate events with a change record.
4. Never `latest` anywhere, including CUDA and PyTorch pairs.
5. Every image builds reproducibly from a clean checkout; the build is a CI job, and its digest is recorded in every experiment's metadata.

## 2. Runtime topologies (Docker Compose)

| Compose profile | Services | Use |
| --- | --- | --- |
| `sim-only` | `sim` (+ `control` optional) | Desktop development of navigation and control |
| `full-sim` | `sim`, `perception`, `control`, `policy`, `logger` | End-to-end mission runs in simulation |
| `train` | `ml-train` (+ `sim` headless as environment server) | RL / DL training with N seeds |
| `eval` | `ci` runner + headless `sim` | Batch scenario suite, report generation |
| `deploy` | `ml-deploy`, `control`, `logger` | On-robot deployment (composition only; no sim) |

**DDS is the hard part.** ROS 2 nodes in separate containers must discover each other. Two supported options, both documented and tested:
- **`network_mode: host`** on Linux — simplest and lowest latency, at the cost of namespace isolation. Default for simulation.
- **Per-container DDS configuration** with a shared discovery domain and explicit peer lists (or a Zenoh/DDS bridge) — required when host networking is not acceptable, e.g. on some managed or Windows/WSL2 setups.

Pick one and document it; mixing them produces intermittent, hours-long debugging sessions.

## 3. GPU, GUI and real-time caveats

**GPU access**
- Requires the NVIDIA Container Toolkit on the host; containers request it with `--gpus all` (or the Compose `deploy.resources.reservations.devices` block).
- On a Windows laptop: **WSL2 + Docker Desktop with GPU passthrough** works for training and perception, but is the slowest path and adds jitter. If the development machine is the robot's computer, native Ubuntu is strongly preferred.
- The CUDA/driver compatibility pair is the single most fragile dependency in this stack. Pin it, test it in CI with a smoke job (`torch.cuda.is_available()` plus a small inference with a timing assertion), and never upgrade it casually.

**GUI (RViz, Gazebo, plotting)**
- Mount the X11 socket (or use XWayland) and forward `DISPLAY`; on WSL2 use the built-in WSLg.
- Alternative that is more robust for CI and remote work: run rendering headless with EGL and stream images as ROS topics; keep an interactive profile for humans only.

**Real-time (this is where containers can hurt)**
- Containers share the host kernel and add scheduling jitter. **Do not assume** that a control loop inside a container meets a hard deadline.
- If a containerized loop must be hard real-time: run a `PREEMPT_RT` kernel on the host, pin CPUs (`cpuset`), raise `rtprio`/`memlock` limits, and **measure the jitter** — do not assert it.
- **Architectural rule:** the motor current loop and all safety functions run on the MCU, outside containers, on bare metal. Containers host perception, planning, control and learning — soft real-time with measured margins. This is a deliberate safety decision (SF-01), not a limitation.

## 4. Data volumes and artifact handling

| Mount | Contents | Committed to git? |
| --- | --- | --- |
| `./src`, `./config`, `./scenarios` | Source, configuration, scenario definitions | **Yes** |
| `./models` | Trained weights | **No** — external store + checksums (SR-07) |
| `./data` | Datasets, maps, recorded bags | **No** |
| `./runs` | Per-run output: bags, event logs, metadata JSON, reports | **No** (small curated reports may be committed) |
| `./artifacts` | Generated plots, benchmark tables | Selected only |

Dataset and model versioning follows the same discipline used in the graduation project: the repository holds **instructions and checksums**, not binaries. Every run's metadata names the exact dataset and weight versions it used.

## 5. CI pipeline

Every push runs, on the `ci` image:

1. Image build + smoke test (ROS 2 environment, CUDA availability if GPU runners are used).
2. Unit tests (kinematics, controller math, protocol parsing, safety filter logic) — fast, no simulation.
3. Headless scenario suite: deterministic, seeded scenarios executed in Gazebo with `xvfb`; the runner asserts requirements FR-04, FR-06, FR-10, SF-02, SF-05, SF-06 and PR-01/PR-05 at a coarse level.
4. Solver benchmark: NMPC solve-time distribution (p50/p99/max) collected and compared against the previous run; a regression beyond a threshold fails the build.
5. Report generation: JSON + Markdown table published as a build artifact, with the image digest and git commit embedded.

**Performance-regression gating** is the point of this pipeline: control and learning work degrades silently, and without an automated comparison every tuning improvement is anecdotal.

## 6. Deployment path

```
dev container ──► pinned image ──► deploy image ──► robot computer
     │                  │                 │
     └── same code ─────┘                 └── same digest, no training deps
```

- The deploy image contains only runtime dependencies (no CUDA toolkit, no compiler, no training libraries), which reduces attack surface and image size.
- Configuration is injected at run time (files or environment), never baked into the image.
- Containers run as a **non-root user**, with a read-only root filesystem where possible and only the artifact directories writable.
- No secrets, tokens or keys in images or layers.
- On the robot: `restart: unless-stopped` for the perception/control services, plus the **MCU-side watchdog** as the actual safety guarantee — container restart policies are convenience, not safety.

## 7. Reproducibility contract

A result is reproducible when this bundle exists and matches:

```
run_metadata.json
  image_digest            (e.g. sha256:… of the ml-train image)
  git_commit              (with dirty flag; dirty=true results are not reportable)
  config_hash             (SHA-256 of the resolved config file)
  scenario_id + seed
  hardware                (CPU/GPU model, RAM)
  timestamp, duration
  result                  (metrics, pass/fail per requirement)
  notes                   (what changed since the previous comparable run)
```

The tooling **rejects** runs with a dirty working tree or a missing metadata record. This is the mechanism that prevents the common failure of reporting a number that cannot be traced back to the code that produced it.
