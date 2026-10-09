# Theory 07 — Perception and Deep Learning

**Purpose:** the sensing and learning stack that feeds estimation, planning and safety. Requirements: FR-04, FR-05, FR-09, PR-06, SF-06, and the S3 degraded-sensing scenario.

---

## 1. Sensing suite and why each sensor exists

| Sensor | Provides | Rate | Why it is there | Known failure modes |
| --- | --- | --- | --- | --- |
| **2D LiDAR** (360° or wide-FOV) | Geometric range profile | 10–20 Hz | Localisation backbone; geometric obstacle detection; independent of lighting | Glass, mirrors, black/absorbing surfaces, featureless geometry, motion distortion at speed |
| **RGB-D camera** | Colour + dense depth | 15–30 Hz | Semantics (people, objects), traversability, 3D obstacles LiDAR misses (e.g. overhangs) | Lighting dependence, depth noise beyond ~3–5 m, reflective and transparent surfaces, no depth in darkness |
| **IMU** (6/9-axis) | Angular rate, acceleration, orientation | 200–1000 Hz | Fast attitude and rotation; bridges the gap between LiDAR updates; detects impacts | Bias drift, vibration noise, magnetic disturbance (for magnetometer) |
| **Wheel encoders** | Wheel rotation | 1–10 kHz | Velocity, odometry, slip detection | Slip, quantisation, wheel-radius error |
| **Ultrasonic / IR ring** | Short-range presence | 20–50 Hz | Cheap, independent near-field detection for the safety layer | Narrow cone, crosstalk, low update rate, poor at soft or angled surfaces |

**Redundancy rationale:** three modalities cover different physical failure modes — LiDAR fails on transparent and reflective surfaces, the camera fails in darkness, ultrasonic is short-range but lighting- and material-independent (to a point). The safety layer requires **two independent modalities to agree that the path is clear** before a high-speed command is permitted; disagreement triggers the degraded state.

## 2. Perception tasks and their formulations

| Task | Formulation | Output | Where it is used |
| --- | --- | --- | --- |
| **Obstacle detection (geometric)** | Range filtering / clustering on the scan | Set of obstacle points or polygons | Costmap, SDF, safety constraints |
| **Free-space / traversability** | Semantic segmentation → traversable mask, projected to the ground plane | Traversability grid | Costmap enrichment (distinguishes a rug from a stair edge) |
| **Object detection** | Supervised detection (e.g. YOLO-family) | Boxes + classes + confidence | People detection, target objects for inspection (FR-08) |
| **Person detection & tracking** | Detection + association over time (e.g. a simple Kalman tracker per track) | Tracks with velocity | Velocity-dependent safety margin (doc 06 §3.3) |
| **Depth completion / fusion** | Sensor fusion of sparse LiDAR + dense RGB-D | Dense, denoised depth | 3D obstacle geometry, overhang detection |
| **Anomaly / OOD detection** | Autoencoder reconstruction error or energy-based score | Scalar anomaly score | Detect out-of-distribution conditions → degradation ladder (doc 03 §5, doc 05 §7) |

**Learning choice per task:** supervised learning where labels exist (detection, segmentation), classical geometric methods where they are exact and cheaper (range clustering, scan matching), and learning *only* where geometry is insufficient (semantics, classification, anomaly detection). This division is deliberate: the LiDAR projection pipeline is deterministic, testable and fast, and adding a network there would reduce reliability for no benefit.

## 3. Architectures and why

| Purpose | Candidate | Rationale |
| --- | --- | --- |
| 2D/3D detection | YOLO-family (v8/v11) or RT-DETR | Real-time on edge hardware; the author has prior, demonstrated experience with this family (the graduation project) and can therefore spend effort on integration rather than on re-learning the basics |
| Semantic segmentation | Lightweight encoder–decoder (e.g. a MobileNet/ResNet-18 U-Net variant) | Edge-compatible; the task needs speed and reasonable accuracy, not the last percentage of mIoU |
| Depth fusion | Learned fusion network or classical filtering | Start classical (temporal filtering + geometric consistency); move to learned only if measured accuracy requires it |
| Anomaly detection | Autoencoder or feature-space distance | Cheap, no labels, directly useful for the degradation trigger |

**Deployment constraint drives architecture.** The model must run inside the latency budget (PR-06) on the edge computer, alongside the NMPC and the policy. A perception model that consumes the whole GPU starves the controller's solver. **The GPU budget is shared and must be allocated explicitly** — this is a system-level design decision that shapes the model choice, not an afterthought.

## 4. The latency budget (PR-06)

```
Frame captured (t0)
  + driver/transport                       ~5–15 ms
  + preprocess (resize, normalise, tensor)  ~2–5 ms
  + inference (measured, p99)              ~15–50 ms   ← the dominant term, model-dependent
  + postprocess (NMS, projection)           ~2–10 ms
  + fusion into costmap                     ~5–15 ms
  + consumed by planner/MPC                 ~0–50 ms (one control period)
= total perception-to-action latency        target p99 ≤ 150 ms
```

**Consequences that are easy to get wrong:**
1. **Latency is a safety input, not a performance metric.** The safety margin must include `v · t_total`; a faster controller with slower perception is a *less* safe robot.
2. **Measure each term** with timestamps. Quoting only inference time hides the transport and fusion costs, which are often comparable.
3. **The action must be taken against the future state.** Either extrapolate the estimate forward by the measured latency during planning, or subtract it from the available margin. Ignoring it produces a systematic error that grows with speed.
4. **Backpressure, not dropping.** If perception cannot keep up, the *speed* must drop, not the perception rate — a dropped frame is an unobserved region in the world.

## 5. Training data and the bootstrapping problem

A new robot has no data. The pipeline:

| Stage | Source | Purpose |
| --- | --- | --- |
| 1. Simulation, synthetically labelled | Gazebo with automatic ground-truth labels | Detector warm start; unlimited labelled data; zero labelling cost |
| 2. Domain adaptation | Randomised appearance (lighting, textures, occlusion), style transfer | Close the appearance gap between render and reality |
| 3. Real, auto-labelled | Run the simulated-trained detector on real data, correct the errors | The highest-value real data: only the mistakes need human attention |
| 4. Real, hand-labelled | Curated, diverse, hard examples | Final accuracy on the real distribution |
| 5. Continuously logged | Every deployment run | Hard-case mining; regression detection |

**Dataset discipline carried over from the author's graduation project (and to be maintained here):** images, labels and weights live in external versioned storage with checksums; the repository holds only instructions, splits and evaluation scripts. Every reported metric names the exact dataset version and evaluation protocol.

## 6. Evaluation: metrics that mean something

| Task | Primary metrics | Diagnostic metrics | Traps to avoid |
| --- | --- | --- | --- |
| Detection | mAP@0.50, mAP@0.50:0.95, per-class precision/recall | Confusion matrix, PR curves, false-positive rate **per hour of operation** | A high mAP on a small or non-diverse split (the lesson from the graduation project); class imbalance hiding a failed class |
| Segmentation | IoU / mIoU per class | Boundary accuracy, error maps | Ignoring the cost asymmetry (a missed obstacle ≫ a false obstacle) |
| Tracking | MOTA/IDF1, track stability | ID switches per minute | Optimising tracking when detection is the bottleneck |
| Latency | p50/**p99**/max per stage | GPU utilisation, memory, thermal throttling | Quoting mean latency; ignoring thermal throttling over a long mission |

**Cost asymmetry rule:** for obstacle and person detection, false negatives and false positives are not equally costly. The operating point (confidence threshold) is therefore chosen **against the safety requirement** (a missed person is unacceptable; a false obstacle costs time), and the resulting threshold is reported with the trade-off curve that justified it.

## 7. Robustness and graceful degradation (FR-09, S3)

| Condition | Expected failure | Mitigation |
| --- | --- | --- |
| Darkness | Camera blind | LiDAR-only mode; active illumination; low-light retraining; speed limit |
| Dust / smoke | LiDAR backscatter and false returns | Temporal filtering, intensity filtering, camera-assisted rejection; reduce speed; conservative inflation |
| Direct sunlight / glare | Depth sensor saturation | LiDAR-primary mode; invalidate saturated regions as *unknown*, never as *free* |
| Transparent/reflective surfaces | Missing returns | **Treat unknown as occupied** in the safety geometry; ultrasonic confirmation near-field |
| Motion blur at speed | Degraded detection | Speed-dependent confidence thresholds; report the speed at which detection degrades |
| Adversarial user (e.g. a person approaching rapidly from occlusion) | Late detection | Enlarged near-field margin, second modality, bounded speed |

**Rule: unknown is not free.** Any region the sensors cannot confirm as free is treated as occupied by the safety layer. This single conservative rule prevents the most dangerous class of failure — driving confidently into something that was never seen.

## 8. Assumptions and limits

1. Labelled data is available for the task; otherwise only geometric perception and simulation-trained models are usable.
2. Sensor calibration (intrinsics and extrinsics) is accurate; an uncalibrated camera silently biases every projected obstacle position.
3. Lighting and environment are within the trained distribution; OOD detection is a mitigation, not a solution.
4. The GPU budget is finite and shared with control — perception model size is a **system** decision.
5. No perception system is perfect; the safety architecture is designed to tolerate missed detections via margins, redundancy and speed limits (doc 06).

## 9. Verification

| Claim | Test | Metric |
| --- | --- | --- |
| Detection accuracy | Held-out, diverse, real test set | mAP per class, per-class recall at the chosen operating point |
| Detection robustness | Dark, glare, dust, blur conditions | Recall drop vs. nominal; failure modes documented |
| Latency (PR-06) | Instrumented pipeline, sustained run | p50/p99/max per stage; thermal-throttled performance |
| Degradation (FR-09) | Fault injection: unplug/occlude each sensor | Correct state transition; no silent use of a stale sensor |
| Safety-relevant perception | Adversarial scenarios (occlusion, rapid approach, reflective obstacle) | Minimum realised distance; detection latency distribution |
| OOD detection | Unseen environments and object classes | Detection rate of OOD conditions; false triggers |

## 10. Open questions

1. Is a dedicated accelerator (e.g. an edge TPU or a second GPU) required to meet PR-06 while the NMPC runs, or is time-slicing with explicit budgets sufficient? → resolved by profiling.
2. Is depth completion genuinely needed for the core tier, or does LiDAR geometry plus RGB-D framing suffice? → resolved by the S3 evaluation.
3. For person tracking, is the benefit of a predicted trajectory worth the risk of a mispredicted one? → decided by adversarial testing (doc 06 §3.3).
