# Theory 01 — Kinematics and Dynamics of the Platform

**Purpose:** supply the plant model `f(x, u)` that the NMPC (doc 04), the estimator (doc 03) and the simulation all depend on. Requirements served: FR-03, FR-04, PR-01, PR-02.

---

## 1. Notation and frames

| Symbol | Meaning |
| --- | --- |
| `W` | World frame — fixed, origin at the start pose. All maps and goals are expressed here |
| `B` | Body frame — attached to the chassis, origin at the midpoint of the drive axle, x forward, y left, z up |
| `q = (x, y, θ)ᵀ` | Planar pose of `B` in `W`; `θ` is heading (yaw), measured counter-clockwise from the world x-axis |
| `v`, `ω` | Body-frame longitudinal velocity [m/s] and yaw rate [rad/s] |
| `r`, `L` | Wheel radius [m], wheel separation (track width) [m] |
| `ω_R`, `ω_L` | Right and left wheel angular velocities [rad/s] |
| `m`, `I_z` | Total mass [kg], yaw moment of inertia [kg·m²] |
| `τ_R`, `τ_L` | Drive torque at right/left wheel [N·m] |

A pose is a rigid transform `ᵂT_B ∈ SE(2)`; composition and inversion follow the standard rules. The robot's forward direction is the **+x axis of `B`** by convention — stated here because a flipped convention is the classic source of sign errors in differential-drive code.

## 2. Kinematics

### 2.1 The differential-drive model

Pure rolling without slipping gives the mapping between wheel rates and body velocity:

```
v   = (r/2) (ω_R + ω_L)
ω   = (r/L) (ω_R − ω_L)
```

and, inverted, the wheel commands required for a desired body velocity — this is what the low-level controller actually computes:

```
ω_R = (2v + ω L) / (2r)
ω_L = (2v − ω L) / (2r)
```

The pose evolves as the **unicycle model**:

```
ẋ = v cos θ
ẏ = v sin θ
θ̇ = ω
```

### 2.2 The nonholonomic constraint, and why it matters

Eliminating `v` between the three equations above yields a constraint on velocities that contains no derivative of any input:

```
ẋ sin θ − ẏ cos θ = 0
```

Written in Pfaffian form `A(q) q̇ = 0` with `A(q) = [sin θ, −cos θ, 0]`. This is a **non-integrable** (nonholonomic) constraint: the robot cannot move instantaneously sideways, yet *can* reach any pose in the plane by manoeuvring. Three consequences that shape the whole project:

1. **The system cannot be stabilised to an arbitrary pose by a smooth time-invariant state feedback law** — a classical result. Practical controllers therefore use either a time-varying law, a discontinuous law, or a receding-horizon formulation (our NMPC), which sidesteps the issue by optimising over a finite horizon.
2. **The reachable set is not a linear subspace** — so linear control design must be done along a trajectory (LTV), not about a fixed point.
3. **Path following ≠ trajectory tracking.** Following a geometric path requires a progress variable, which is why doc 04 introduces a path parameter as a decision variable.

### 2.3 Acceleration-level model (the control-relevant form)

Since the NMPC plans over `(v, ω)` and their rates, apply a dynamic extension: treat body accelerations as the virtual inputs and let the low-level loop realise them.

```
ṗ = [ ẋ, ẏ, θ̇, v̇, ω̇ ]ᵀ
ẋ = v cos θ
ẏ = v sin θ
θ̇ = ω
v̇ = a_v        (virtual longitudinal acceleration input)
ω̇ = a_ω        (virtual angular acceleration input)
```

with the actuator limits expressed as constraints:

```
|v| ≤ v_max ,   |ω| ≤ ω_max ,   |a_v| ≤ a_v,max ,   |a_ω| ≤ a_ω,max
v̇² / a_v,max² + ω̇² / a_ω,max² ≤ 1        (coupled actuator/traction limit)
```

The elliptical constraint is the honest form when both wheels share one power budget and traction limit; the box form is a conservative simplification used for the first implementation. Documenting which is used, and why, is required — the two produce different braking behaviour near the limit.

## 3. Dynamics

### 3.1 Lagrangian formulation with multipliers

For a vehicle with generalised coordinates `q ∈ ℝⁿ` and nonholonomic constraints `A(q) q̇ = 0`:

```
M(q) q̈ + V(q, q̇) + Aᵀ(q) λ = B(q) τ
```

where `M` is the inertia matrix, `V` collects Coriolis/centrifugal and friction terms, `λ` are Lagrange multipliers (constraint forces), and `B` maps the wheel torques `τ = (τ_R, τ_L)ᵀ` into generalised forces.

### 3.2 Reduced-order model in body coordinates

Because the constraint is non-integrable, the dynamics are projected onto the null space of `A` — the two-dimensional space spanned by `(cos θ, sin θ, 0)` and `(0, 0, 1)`. Using `v = (v, ω)ᵀ` as the velocity coordinates, the standard reduced form is:

```
[ v̇ ]   [ m   0  ]⁻¹ ( [ F   ] − [ c_v (v, ω) ] )
[ ω̇ ] = [ 0   I_z ]   ( [ M_z ]   [ c_ω (v, ω) ] )
```

with, for a robot whose centre of mass lies on the drive axis (so longitudinal and rotational dynamics decouple):

```
m v̇   = F        with  F   = (τ_R + τ_L)/r − (b_v v + F_friction)
I_z ω̇ = M_z      with  M_z = L (τ_R − τ_L)/(2r) − b_ω ω
```

and viscous friction coefficients `b_v`, `b_ω` **identified from data** (section 5), not taken from datasheets. When the centre of mass is offset from the axle by a distance `d`, a coupling term `m d ω²` appears in the longitudinal equation — for a first prototype, either place the battery to null `d` or model it; do not silently ignore it.

### 3.3 Actuator dynamics — do not omit these

Two lags dominate the achievable tracking bandwidth and, in practice, matter more than the friction terms:

```
Motor electrical/current-loop lag:   τ_loop  ≈ 1/(2π f_c)   with f_c the current-loop bandwidth (doc 02)
Communication + filter lag:          τ_comm  ≈ frame period + filter group delay
Command → torque (first-order):      τ̇_m = (τ_cmd − τ_m)/τ_act
```

With `f_c ≈ 1 kHz` the electrical lag is ~0.16 ms — negligible. The **communication and scheduling lag is the real problem**: a serial frame at 100 Hz plus a 50 Hz control period contributes ~30–40 ms of pure delay, which at 0.5 m/s is 15–20 mm of position error before any control error occurs. This is why doc 02's interface specifies timestamps and why the latency budget (PR-06) is measured rather than assumed.

## 4. Discretisation

The NMPC needs a discrete-time model `x_{k+1} = f_d(x_k, u_k)`. For the acceleration-level model the exact solution on a zero-order-held input is available in closed form and is **preferred over RK4** because it is exact and cheap:

```
With a_v, a_ω constant over the interval Δt and starting from (x, y, θ, v, ω):

θ⁺  = θ + ω Δt + ½ a_ω Δt²
v⁺  = v + a_v Δt
ω⁺  = ω + a_ω Δt
x⁺  = x + v Δt cos θ + ½ a_v Δt² cos θ − ½ (v ω Δt² + ⅓ a_v ω Δt³ + ½ v a_ω Δt² + ⅛ a_ω a_v Δt³) sin θ  …
```

Because the closed form gets unwieldy and error-prone at this level of coupling, the engineering decision is:

- **Use RK4 with 2–5 substeps per control interval** for the controller model (simple, adequately accurate at `Δt = 20–50 ms`, and easy to verify against the closed form in tests).
- **Verify the integrator**: compare RK4 against the exact solution for constant `(a_v, a_ω)` on a grid of states and inputs; report maximum pose error. An integrator whose error is unquantified silently biases every performance number downstream.

## 5. Parameter identification plan

Every parameter below is obtained from a defined experiment, with the method and the residual recorded.

| Parameter | Method | Notes |
| --- | --- | --- |
| `r`, `L`, `m`, `I_z` | Direct measurement + CAD, then refined | `L` must be measured at the *contact patch*, not the hub face |
| `b_v`, `b_ω` | Coast-down test: accelerate to a set speed, remove torque, fit exponential decay | Repeat at several speeds; a single linear fit is valid only in the tested range |
| `τ_act` (actuator lag) | Step torque command with wheels free; fit first-order response | Report the residual, not just the fit |
| Latency (sensor → command) | Timestamp injection / event marker | The single most under-measured parameter in student robots |
| Traction limit | Ramp torque until slip, per surface (tile, concrete, mat) | Required before setting `a_v,max`, `a_ω,max` |
| Wheel slip coefficient | Compare odometry against an external reference (tape measure, LiDAR) | Basis for slip detection in doc 03 |

## 6. Assumptions and limits

1. **Rigid body, planar motion.** No suspension dynamics, no body roll/pitch modelling. Valid at ≤ 1 m/s on smooth floors; invalidated by thresholds and ramps, which are handled as disturbances rather than modelled.
2. **No-slip rolling.** Violated on wet or dusty floors and during hard braking. Doc 03 adds slip *detection*; slip *compensation* is out of scope for the core tier.
3. **Decoupled longitudinal/yaw dynamics.** Requires the centre of mass near the drive axle.
4. **Lumped friction.** Real friction has Coulomb, viscous and Stribeck components; a linearised viscous term is adequate for control at low speed but will mispredict stopping behaviour. This is documented rather than hidden.
5. **Commanded actuator lag is first-order.** Real drivers have current limits, dead-time and quantisation.

## 7. Verification (once implementation begins)

| Claim | Test |
| --- | --- |
| Kinematics correct | Command a known `(ω_R, ω_L)`; compare integrated pose with an external measurement over 3 m. Error budget: ≤ 2% of distance |
| Integrator accuracy | RK4 vs. exact solution sweep; report max pose error over the operating envelope |
| Parameter identification | Predicted vs. measured velocity decay for a hold-out run; report R² and max error |
| Latency | Timestamped step response; report p50/p99 of measured total delay |
| Model fidelity for control | Open-loop prediction of a recorded trajectory for 1 s from each of N states; report terminal position error distribution. **This is the number that predicts NMPC quality** |

## 8. Open questions

1. Is the elliptical actuator/traction constraint necessary, or does the box form suffice for the target speed envelope? → resolved by testing tracking quality near the limit.
2. Does the centre-of-mass offset `d` remain small at all payload configurations, or must the coupled model be retained? → resolved by a static balance measurement.
3. Is a slip estimator needed for the *core* tier, or is it acceptable to bound operation to surfaces where slip is negligible? → resolved at the traction-limit test.
