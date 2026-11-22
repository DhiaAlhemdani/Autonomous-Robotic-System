# Theory 03 — State Estimation and Localisation

**Purpose:** produce the state estimate `x̂` that the NMPC, the planner and the safety layer all consume. An error here propagates into every layer above it, and it is the most common cause of "my controller works in simulation and fails on hardware." Requirements: FR-02, PR-03.

---

## 1. What must be estimated, and how accurately

| Quantity | Rate needed | Accuracy target | Source |
| --- | --- | --- | --- |
| Pose `(x, y, θ)` in map | 50–200 Hz (interpolated from 10–20 Hz corrections) | ≤ 0.10 m, ≤ 3° (1σ) | LiDAR SLAM + fused odometry |
| Body velocity `(v, ω)` | 100 Hz | ≤ 5% | Wheel encoders + IMU |
| Wheel slip | 20 Hz | Detection, not magnitude | Encoder vs. external reference |
| IMU biases (gyro, accel) | Continuously | Bias stability | EKF state |
| Localisation covariance | 100 Hz | Must be **reported**, not assumed | EKF covariance / scan-match residual |

**The covariance matters as much as the mean.** The safety layer and the degradation ladder (doc 02 §6) both branch on *confidence*, not position. An estimator that reports a precise wrong pose is more dangerous than one that reports an uncertain correct one.

## 2. Wheel odometry

From doc 01, integrating wheel rates gives a pose estimate. Using mid-point (second-order) integration reduces the error from a first-order Euler scheme at negligible cost:

```
Δs = (r/2)(Δφ_R + Δφ_L)                          distance travelled
Δθ = (r/L)(Δφ_R − Δφ_L)                          heading change
x⁺ = x + Δs cos(θ + Δθ/2)
y⁺ = y + Δs sin(θ + Δθ/2)
θ⁺ = θ + Δθ
```

**Error growth is the defining property.** Sources: wheel-radius uncertainty (systematic — scales with distance), track-width uncertainty (systematic — scales with total turning), encoder quantisation, and slip (non-systematic, unbounded). Expect ~1–3% of distance for radius error alone; over 100 m, that is metres of drift. This is why odometry is only ever a *prediction* source and requires absolute correction.

## 3. IMU fusion

The IMU supplies gyro rates (fast, but drifting due to bias) and accelerations (noisy, and contaminated by tilt). Fusing with wheel odometry addresses their complementary weaknesses:

```
State:     x = (x, y, θ, v, ω, b_g, b_a)ᵀ
Prediction: x_k = f(x_{k-1}, u_{k-1})      from the kinematic model, at IMU rate
Update:     z = (v_odom, ω_odom) + (a_x, a_y, ω_z)  with appropriate H
```

**Gyro bias estimation is mandatory**, not optional: an unestimated bias of 0.5°/s integrates to 30° of heading error per minute — fatal for navigation.

**Observability note:** with wheel odometry and IMU only, the position is not observable — nothing in the measurement set anchors the robot to the world. Position requires an exteroceptive sensor (LiDAR or camera). State this explicitly, because it is the reason the next section exists.

## 4. EKF formulation (the workhorse)

```
Predict:
    x̂_{k|k−1} = f(x̂_{k−1|k−1}, u_{k−1})
    P_{k|k−1} = F_k P_{k−1|k−1} F_kᵀ + Q_k          F_k = ∂f/∂x |_{x̂_{k−1|k−1}}

Update:
    ỹ_k = z_k − h(x̂_{k|k−1})                        innovation
    S_k = H_k P_{k|k−1} H_kᵀ + R_k                   innovation covariance
    K_k = P_{k|k−1} H_kᵀ S_k⁻¹                        Kalman gain
    x̂_{k|k} = x̂_{k|k−1} + K_k ỹ_k
    P_{k|k} = (I − K_k H_k) P_{k|k−1}
```

**Engineering discipline that separates a working EKF from a plausible one:**

1. **Consistency testing.** Normalised innovation squared (NIS) must lie within its χ² bounds. If it does not, either `Q`/`R` are wrong or there is an unmodelled effect (slip, delay, bias). Publish the NIS statistic and monitor it — it is the estimator's own health signal.
2. **`Q` and `R` from measurement, not taste.** `R` from sensor noise characterisation; `Q` from the residual of the prediction against ground truth. Both are reported with the method used to obtain them.
3. **Angle wrapping.** Heading must be wrapped correctly in both the innovation and the Jacobian, or the filter develops a discontinuity at ±π.
4. **Time-aligned measurements.** A delayed measurement must be applied at its correct timestamp, or the filter is silently inconsistent. This is the most common cause of an EKF that "works but drifts."
5. **UKF when nonlinearity dominates.** If the heading innovation is large (> ~20°), the linearisation error in the EKF becomes significant and the UKF's sigma-point propagation is preferable. Choose based on a measured comparison, not preference.

## 5. Exteroceptive localisation — the components

| Method | Provides | Rate | Strengths | Failure modes |
| --- | --- | --- | --- | --- |
| **2D LiDAR scan matching** (ICP / correlative matching) | Pose correction in a local frame | 10–20 Hz | Accurate, robust indoors, inexpensive sensor | Featureless corridors (long walls, ambiguous geometry), glass, mirrors; moving obstacles in the scan |
| **SLAM** (graph-based: poses + landmarks, loop closure) | Map **and** pose | 10–20 Hz | Removes drift on revisit; enables unknown-environment operation (FR-02) | Loop closure can be catastrophically wrong if a false positive is accepted — needs robust back-end (switchable constraints, robust kernels) |
| **AMCL / localisation in a known map** (particle filter) | Pose only | 10–20 Hz | Very robust to ambiguity because it retains multi-hypothesis belief | Requires a prior map — not the primary mode for this project, but the fallback once a map exists |
| **Visual-inertial odometry (VIO)** | Pose + velocity | 20–30 Hz | Works where LiDAR degenerates (open spaces, texture-rich); light | Scale drift with monocular; fails in darkness/textureless surfaces; sensitive to time sync |
| **UWB / external beacon** | Absolute position | 10–50 Hz | Anchors the map-free case | **Excluded by FR-01** (no external infrastructure) — noted only to close the option |

**Design decision:** LiDAR-based SLAM is the backbone (geometry is reliable indoors and independent of lighting), with VIO as an independent modality used for cross-checking and for degraded-LiDAR scenarios (S3). Two independent pose sources allow a **consistency monitor**: if the LiDAR and visual estimates disagree beyond a threshold, the estimator's covariance is inflated and the degradation ladder is triggered. That monitor is a safety-relevant feature, not a performance optimisation.

## 6. Slip detection (needed for the degradation ladder)

```
Slip indicator:   κ = | v_odom − v_external | / max(v_odom, ε)
```

with `v_external` from scan-matching displacement or IMU-integrated velocity. A sustained `κ` above threshold (measured, not guessed) triggers: odometry `R` inflated, speed limit reduced, and the event logged. **Detection is required for the core tier; compensation is not** — as stated in doc 01's assumptions.

## 7. Time, delay and the estimation–control interface

| Issue | Consequence | Handling |
| --- | --- | --- |
| Sensor delay | Controller acts on a stale pose → oscillation, instability at high gain | Timestamp at acquisition; compensate or reduce gain per measured delay |
| Multiple clocks | Bags and event logs cannot be aligned; latency cannot be measured | One monotonic clock source; MCU sync (doc 02 §4) |
| TF tree inconsistency | Silent, catastrophic frame errors | Single authority per transform; TF monitored; frame conventions fixed and tested |
| Interpolation | Controller runs faster than corrections (50 Hz vs 20 Hz) | Integrate odometry forward with the model between corrections; never zero-order-hold a pose |

## 8. Assumptions and limits

1. The environment is **static enough** for scan matching over one scan interval; dynamic objects are filtered by the perception layer, not by the matcher.
2. Geometric structure exists (AS-02). In a featureless space, LiDAR-only SLAM degrades and the system must fall back to VIO or reduce autonomy.
3. Sensor extrinsics (LiDAR/camera/IMU relative poses) are calibrated. **Uncalibrated extrinsics are a systematic error indistinguishable from poor control.** Calibration is a prerequisite task, not an afterthought.
4. No GNSS, no beacons, no markers (FR-01).
5. The map is metrically consistent; a bad loop closure is treated as a fault, not a nuisance.

## 9. Verification

| Claim | Test | Metric |
| --- | --- | --- |
| Localisation accuracy (PR-03) | Ground-truth trajectory in simulation; in hardware, closed-loop reference path measured externally | Position/heading error, mean and 95th percentile |
| Estimator consistency | NIS/NEES tests over the full run | Fraction inside χ² bounds; report the value |
| Drift rate | Return-to-start runs without loop closure | Position error per 10 m travelled |
| Robustness | S3 fault injection: occlusion, darkness, moving obstacles, slip | Degradation ladder triggers correctly; no silent bad estimates |
| Latency | Timestamped end-to-end sensor-to-estimate measurement | p50/p99 |
| Extrinsic calibration | Reprojection/registration residual after calibration | Residual, with the method documented |

## 10. Open questions

1. Is a single 2D LiDAR sufficient for robust localisation in the target environment, or is a 3D LiDAR / depth camera required? → resolved by S1 testing in a feature-poor corridor.
2. Is VIO worth the complexity at the core tier, or should it be deferred to the degraded-sensing milestone? → resolved by S3 requirements.
3. What is the acceptable worst-case localisation failure mode — and does the safety layer depend on it? → decided by design: the safety layer must depend on *measured distance*, never on *map-relative position*.
