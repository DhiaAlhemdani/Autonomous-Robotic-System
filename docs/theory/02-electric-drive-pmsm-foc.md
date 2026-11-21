# Theory 02 — PMSM Drive and Field-Oriented Control

**Purpose:** model and control the lowest layer of the stack — the permanent-magnet synchronous motor (PMSM/BLDC) and its inverter. Serves PR-11, PR-12, SF-03, and the drive-level control claim of the project. This document is the direct continuation of the author's previous work on PMSM control and is where the **NMPC-on-the-current-loop** component lives.

---

## 1. Why this layer is in scope

Most mobile-robot projects treat the motor as an idealised torque source behind a black box. That abstraction hides exactly the effects that matter at the limit: current-loop bandwidth limits how fast torque can change, the DC-bus voltage limits the maximum speed under load, and inverter dead-time distorts the applied voltage at low current. A robot that must stop reliably in front of a human, and accelerate hard out of a corridor, is limited by these effects. Controlling them explicitly is the engineering differentiator of this project.

## 2. PMSM model

### 2.1 Three-phase model

Stator voltages `v_a, v_b, v_c`, currents `i_a, i_b, i_c`, and the flux linkage from the rotor magnets `ψ_m` (constant for surface-mounted magnets). The three phases are magnetically coupled and the inductance matrix is position-dependent — which is why no sane controller works in this frame.

### 2.2 The dq frame — the Clarke–Park transformation

Transform the three phases to a two-axis frame rotating with the rotor electrical angle `θ_e` (Clarke then Park). The inductances become **constant** and the time-varying coupling becomes a linear rotation:

```
v_d = R_s i_d + L_d (di_d/dt) − ω_e L_q i_q
v_q = R_s i_q + L_q (di_q/dt) + ω_e L_d i_d + ω_e ψ_m
```

| Symbol | Meaning |
| --- | --- |
| `R_s` | Stator resistance per phase |
| `L_d`, `L_q` | d- and q-axis inductances (`L_d = L_q` for surface-mounted magnets; `L_q > L_d` for interior magnets) |
| `ψ_m` | Rotor permanent-magnet flux linkage |
| `ω_e = p ω_m` | Electrical angular frequency; `p` = pole pairs, `ω_m` = mechanical speed |
| `i_d`, `i_q` | Current components. `i_q` produces torque; `i_d` is aligned with the rotor flux |

The cross-coupling terms `−ω_e L_q i_q` and `+ω_e L_d i_d + ω_e ψ_m` are the **back-EMF and coupling terms**. They are what make the system nonlinear and they grow with speed — so a controller tuned to work at standstill will lose performance at high speed unless these are compensated or the controller is nonlinear.

### 2.3 Torque production

```
T_e = (3/2) p [ ψ_m i_q + (L_d − L_q) i_d i_q ]
```

- For **surface-mounted PMSM** (`L_d = L_q`): `T_e = (3/2) p ψ_m i_q` — torque is proportional to `i_q` alone, and the optimal strategy is simply `i_d* = 0`.
- For **interior PMSM** (`L_q > L_d`): the second term is **reluctance torque** and can be exploited by deliberately making `i_d < 0`. This is the basis of MTPA below.

### 2.4 Mechanical dynamics

```
J (dω_m/dt) = T_e − T_L − B ω_m
```

with `J` rotor + reflected load inertia, `T_L` load torque (for a wheel on the ground, this includes rolling resistance and any traction load), `B` viscous friction.

### 2.5 Inverter constraints (do not omit)

The inverter can only apply voltages inside the **voltage hexagon** defined by the DC-bus voltage `V_dc`:

```
|v_d² + v_q²|^½ ≤ V_dc/√3        (linear-modulation limit, circle inscribed in the hexagon)
```

and currents are bounded by the drive and thermal limits:

```
i_d² + i_q² ≤ I_max²
```

These are **hard actuator constraints**, which is precisely the reason a constrained optimal controller is attractive here: the two limits bound the achievable torque–speed region, and violating them in the controller model while the hardware enforces them in reality produces windup and instability. The voltage circle shrinks relative to the back-EMF as speed rises, which is why the achievable torque falls with speed — the constant-power region.

## 3. Field-oriented control (FOC)

### 3.1 Structure

```
ω* ──►[speed PI]──► i_q* ──►[q-axis current PI]──► v_q* ──┐
                     │                                     ├──►[inverse Park]──►[SVPWM]──► inverter ──► PMSM
i_d* = 0 (or MTPA) ──►[d-axis current PI]──► v_d* ────────┘                ▲
                                                                            │
                          θ_e ◄── [encoder / observer] ◄──── rotor position ─┘
                          i_a,i_b (or i_c) ◄── ADC ◄── phase currents
```

### 3.2 The cascade and its bandwidth separation rule

| Loop | Typical bandwidth | Typical rate |
| --- | --- | --- |
| Current (inner, d and q) | 1–2 kHz | 10–20 kHz (synchronised to PWM) |
| Speed (middle) | 100–200 Hz | 1–10 kHz |
| Position (outer) | 10–50 Hz | 100–1000 Hz |

**Rule:** each loop's bandwidth must be ≤ 1/5–1/10 of the loop inside it. This is what makes the cascade analysable as independent single-input systems, and violating it is the most common cause of "the controller oscillates and I don't know why."

Design the current loop **first** (it is the fastest and defines the inner dynamics), then the speed loop around the *closed* current loop, then position.

### 3.3 Current-loop design

For a surface-mounted PMSM with the coupling terms compensated by feed-forward, the d and q axes decouple into two identical first-order plants:

```
G_i(s) = 1 / (L s + R_s)
```

with a processing delay (PWM update + ADC + computation) modelled as `e^{-sT_d}`, `T_d ≈ 1.5 T_s` where `T_s` is the sampling period. Design a PI controller for the desired crossover frequency `ω_c`:

```
K_p = ω_c L ,     K_i = ω_c R_s
```

**Practical discipline:**
- Measure the actual crossing frequency and phase margin (step response, or frequency sweep with an injected signal) — do not assume the design target was achieved.
- Current sampling must be **synchronised to the PWM carrier** and sampled away from switching transients (centre-aligned sampling).
- Add **dead-time compensation** or accept a low-current distortion that shows up as torque ripple at low speed — measure it before dismissing it.
- Clamp the integrator to the voltage limit (**anti-windup**). This is mandatory: without it, the controller commands impossible voltages during transients and the recovery is visible as an overshoot.

### 3.4 MTPA for interior PMSM

To minimise copper loss for a required torque, solve for the `(i_d, i_q)` pair that maximises torque per ampere. The classic closed-form result for `L_q > L_d` is:

```
i_d* = ψ_m / (2 (L_q − L_d)) − sqrt( ψ_m² / (4 (L_q − L_d)²) + i_q*² )
```

For surface-mounted machines this collapses to `i_d* = 0`. **Note honestly:** most hobby BLDC gimbals and hub motors used in student robots are surface-mounted, so MTPA yields nothing. The correct engineering decision is to measure or obtain `L_d` vs `L_q`; if they are equal within measurement error, use `i_d* = 0` and say so, rather than implementing MTPA to appear sophisticated.

### 3.5 Field weakening (stretch)

Above base speed, the back-EMF exceeds the available voltage and `i_d* = 0` cannot deliver the commanded current. Injecting negative `i_d` reduces the flux linkage and extends the speed range:

```
Voltage constraint:   (R_s i_d − ω_e L_q i_q)² + (R_s i_q + ω_e L_d i_d + ω_e ψ_m)² ≤ (V_dc/√3)²
Current constraint:   i_d² + i_q² ≤ I_max²
```

The optimal operating point is the intersection of the torque hyperbola with these constraints. This is a constrained optimisation on a two-dimensional state — which is exactly an **NMPC current-loop problem**, and the bridge to section 4.

## 4. NMPC on the current loop (the advanced-control component)

### 4.1 Problem statement

State `x = (i_d, i_q)ᵀ`, input `u = (v_d, v_q)ᵀ`, disturbance/parameter `ω_e`:

```
minimise  Σ_{k=0}^{N-1} [ ‖x_k − x_k*‖²_Q + ‖u_k‖²_R + ‖Δu_k‖²_{R_d} ] + ‖x_N − x_N*‖²_P
subject to
    x_{k+1} = f_d(x_k, u_k, ω_e(k))            discretised PMSM model (RK4 or exact ZOH on the linear-in-i form)
    x_k*    = (i_d*, i_q*)                      from the speed loop, with i_d* from MTPA/field-weakening
    v_d,k² + v_q,k² ≤ (V_dc/√3)²                inverter voltage limit
    i_d,k² + i_q,k² ≤ I_max²                    current/thermal limit
```

Distinct advantages over the PI cascade, in principle:

1. **Constraints are handled explicitly** — no windup, no ad-hoc saturation, and the controller knows *why* it is limiting.
2. **Cross-coupling is handled by the model**, not by a feed-forward term tuned at one operating point.
3. **Optimal trade-off** between tracking, control effort and current ripple, expressed as weights rather than as three independently tuned PI loops.

### 4.2 Why the PI cascade still wins at the core tier (and when NMPC is justified)

| Factor | PI cascade | NMPC current loop |
| --- | --- | --- |
| Rate | 10–20 kHz, trivially | 10–20 kHz is **hard**: solve time must fit in ≲ 50 µs |
| Compute | A cheap MCU | A powerful MCU/DSP, or FPGA-assisted |
| Implementation effort | Days | Weeks, plus a solver budget that must be *proved*, not hoped |
| Tuning | Three loops | Weights + horizon, but the constraints come free |
| Benefit at low speed, low current | Minimal | Minimal |

**Honest engineering position:** for the core tier, implement a **well-instrumented PI cascade** and characterise it (bandwidth, phase margin, step response, torque ripple). Implement NMPC on the current loop as the **advanced tier**, on a slower inner-loop-adjacent problem (e.g. 1–2 kHz), where the solve-time budget is defensible. A measured, honest comparison between the two — including where NMPC does *not* help — is a far stronger result than an NMPC that runs at a rate that defeats its own purpose.

### 4.3 A cheaper, defensible middle path

If the goal is to demonstrate advanced drive control without an NMPC at 20 kHz:

- **Explicit MPC on the linearised current loop at 1–5 kHz** (the plant is linear in `i` for fixed `ω_e`; the nonlinearity enters as a scheduled parameter). Linear time-varying MPC with a QP solver is realistic on a modern MCU.
- **NMPC at the outer (speed/torque-vector) layer** at 100–500 Hz, where the coupled voltage/current constraints and the field-weakening decision live. This is where the constrained optimisation genuinely pays off.

This decomposition — NMPC where the constraints bind, PI where the rate binds — is defensible engineering, and stating it explicitly distinguishes the design from "we used NMPC because NMPC is modern."

## 5. Assumptions and limits

1. **Magnetic linearity.** No saturation modelled; at high current the real torque-per-ampere falls. If the operating envelope approaches saturation, add a saturation model identified from a torque-stand test, or limit `I_max` conservatively.
2. **Constant parameters.** `R_s` rises ~40% from cold to hot (copper); `ψ_m` falls with magnet temperature. Thermal drift is a real cause of torque mismatch. Mitigation: measure `R_s` intermittently, or accept and bound the error.
3. **Ideal inverter.** Dead-time, switch drop and DC-bus ripple are not in the model. Dead-time produces a measurable low-current distortion — verify whether it matters at the operating point before compensating.
4. **Position known.** Sensored control assumed (encoder/hall). Sensorless control needs an observer and a start-up strategy and is explicitly out of scope for the core tier.
5. **Velocity is the outer loop's concern.** This document stops at the torque/current interface; the wheel and body dynamics are in doc 01.

## 6. Design decisions implied

| Decision | Value | Reason |
| --- | --- | --- |
| PWM / current-loop rate | ≥ 10 kHz, synchronised sampling | Bandwidth separation and clean current reconstruction |
| Current-loop bandwidth | 1–2 kHz | Speed loop at 100–200 Hz then satisfies the 1/10 rule |
| `i_d*` strategy | `= 0` unless `L_q ≠ L_d` is measured | Avoid implementing MTPA for a machine that gains nothing from it |
| Anti-windup | Clamping to the voltage circle | Prevents overshoot after saturation transients |
| Constraints enforced | Voltage circle + current circle | Both are hardware limits; the model must know them |
| Safety independence | Torque-enable line gated by the MCU safety function | SF-03: a software-independent stop path |

## 7. Verification

| Claim | Test | Metric |
| --- | --- | --- |
| Current loop bandwidth | Step response + frequency sweep, current reference injected | −3 dB bandwidth, phase margin (report both, measured) |
| Torque linearity | Torque vs. `i_q` at constant speed using a dynamometer or a known load | Linearity error, saturation onset |
| Parameter accuracy | `R_s` by DC injection, `L_d`/`L_q` by AC injection at two rotor positions, `ψ_m` from open-circuit back-EMF | Percentage error vs. datasheet and vs. cold/hot |
| Constraint enforcement | Command an unreachable torque; verify the controller saturates on the voltage circle without overshoot | No overshoot, no windup (measured on the release transient) |
| NMPC vs PI (advanced tier) | Identical reference profile, identical hardware | Tracking RMSE, current ripple, solve time p50/p99 — plus an honest statement of where NMPC did not help |

## 8. Open questions

1. Which motor class will the platform actually use — surface-mounted hub motors (simple, `i_d*=0`) or interior-magnet servos (MTPA/field-weakening available)? → resolved by part selection.
2. Is the MCU fast enough for a 1–5 kHz linear MPC current loop *with measured* worst-case solve time, or does the design stay with a PI cascade at the core tier? → resolved by a solver-timing benchmark before committing.
3. Is a torque sensor or dynamometer available for verification, or must torque be inferred from current and a calibrated model? → resolved by lab access assessment.
