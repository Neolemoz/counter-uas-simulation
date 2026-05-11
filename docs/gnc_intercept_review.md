# GNC Technical Review: ROS2 + Gazebo Counter-UAS Intercept Simulation

**Audience:** Senior robotics / GNC engineer
**Scope:** Architecture, guidance, control, feasibility, estimation, Monte Carlo vs Gazebo alignment, failure modes, roadmap
**Codebase context:** `guidance_lib.py`, `interception_logic_node.py`, `simulation/core/dynamics.py`, tracking/fusion pipeline

---

## Context (as stated by operator)

- Intercept math: `solve_intercept_time`, `compute_intercept` (constant-speed CV intercept vs CV target).
- Guidance: naive, predictive, PN (partial).
- Real-time loop ~10–20 Hz.
- Monte Carlo evaluation: P(hit), miss distance.
- Gazebo: turn-rate + acceleration limits.

**Observed aggregate metrics (mixed log population — caveat):** success ~30%, median miss ~4 m, mean miss very large, P95 miss ~km-scale outliers.

**Stated problem:** Solver appears correct; failures arise from **control + constraints** (turn rate, accel, delay, velocity estimation error, replanning instability).

**Goal:** >80% success in simulation; robust under noise, delay, dynamics; consistent MC vs Gazebo.

---

## 1. Architecture review

### What is fundamentally wrong with “solve intercept → command velocity”?

The solver (`solve_intercept_time` / `compute_intercept` in `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py`) solves a **kinematic boundary-value problem**: smallest \(t>0\) such that \(\|r_0 + v_T t\| = s_i t\) with interceptor **instantly** flying at speed \(s_i\) along the LOS to the predicted collision point. That is **not** the same plant as:

- **Turn-rate-limited** heading evolution (`update_velocity` in `simulation/core/dynamics.py`).
- **Acceleration-limited** velocity changes (rollout / node `_accel_limit_velocity` patterns).
- **Cycle-to-cycle** replanning under noisy \(v_T\) and delay-compensated target state.

The codebase already acknowledges **speed / geometry inconsistency** after saturation via `align_speed_after_saturation` (recompute \(s \approx \|p_{hit}-p_I\|/t_{go}\) clamped to \(v_{max}\)). That fixes one class of bias but does **not** certify **heading reachability** within \(t_{go}\).

### Where the architecture breaks under real dynamics

| Failure mechanism | Why |
|-------------------|-----|
| **Heading lag** | Desired `u` rotates faster than \(\omega_{max}\) allows → effective closing geometry differs from solver assumption. |
| **Replanning “moving aimpoint”** | Each cycle recomputes \(p_{hit}(t_{go})\); with noisy velocity, \(t_{go}\) roots jump → **whip** on aimpoint. `filter_t_go` exists explicitly to damp this. |
| **Delay mismatch** | `compensate_target_for_delay` extrapolates CV; wrong \(\tau\) → solver optimizes wrong ghost target. |
| **PN as lateral accel then same \(\omega\) cap** | `update_velocity_from_pn_lateral` folds PN into the same turn-rate limiter as naive/predictive — high PN demand can saturate and desynchronize from predictive geometry. |

**Bottom line:** You have **open-loop-in-time geometry** wrapped around **saturated closed-loop kinematics**. The gap is the dominant source of large misses and instability.

---

## 2. Guidance design

### Combining predictive intercept, pursuit, and PN

Recommended layering:

1. **Mid-range:** predictive intercept (or ZEM/ZEV-style equivalent) as **primary closing geometry** — encodes interceptability against CV target well.
2. **Terminal:** transition to **pure pursuit on LOS** or **PN on LOS rate** to reduce sensitivity to \(t_{go}\) noise and reduce aimpoint whip.

### Should PN be primary?

Generally **no** for the full envelope if tracks are noisy and PN gains are aggressive; PN is best as a **terminal correction** or blended term. (Launch comments in this repo already reflect turning PN off for some fast-target / lag cases.)

### Anti-oscillation / zig-zag from replanning

- Continue / strengthen **`filter_t_go`** (already documented as reducing jerk when BVP root jumps).
- Add **rate limits on commanded heading or unit vector `u`**, or **hold** `u` when predicted change exceeds \(\omega_{max}\Delta t\).
- Consider **event-triggered replan** (recompute intercept only when LOS rate or range band crosses thresholds), not necessarily every ROS tick.

**Pseudo-code (conceptual)**

```text
each tick:
  target := compensate_delay(track, tau_eff)
  t_raw := solve_intercept_time(..., s_plan)
  t_go := filter_t_go(prev, t_raw, alpha)

  if range < R_terminal:
      u_cmd := blend_PN_pure_pursuit(LOS, N, v_closing)
  else:
      (_, p_hit, u_pred) := compute_intercept(..., s_plan)
      u_cmd := saturate_turn_rate(current_heading, u_pred, omega_max, dt)

  v_cmd := align_speed_after_saturation(u_cmd, p_i, p_hit, t_go, v_max)
```

---

## 3. Controller design (dynamics-aware)

**Two-timescale structure:**

**A. Reference generation:** predictive intercept gives desired **direction + scheduled speed** (`align_speed_after_saturation` is a coherent speed schedule tied to \(t_{go}\)).

**B. Tracking / saturation:** enforce \(\omega_{max}\), \(a_{max}\) as **hard invariants** on the integrated velocity state (not only post-hoc clipping of cmd_vel).

**Speed scheduling vs \(t_{go}\):** far: near \(v_{max}\); near: reduce speed using a braking law (e.g., cap \(v \propto \sqrt{a_{max} r}\)) to reduce terminal overshoot.

**Overshoot prevention:** switch to **velocity-to-zero along LOS** or braking field inside a terminal radius rather than chasing a moving \(p_{hit}\) at full speed.

---

## 4. Feasibility gating

### Must-check before hard engage

Existing geometry gates (`is_intercept_feasible`, `minimum_intercept_closing_speed`, `_tti_feasible` in `interception_logic_node.py`) cover **CV speed feasibility in a time window**. That is necessary but **not sufficient** for saturated dynamics.

**Add / prioritize:**

1. **Closing velocity** \(v_c = -\hat r \cdot v_{rel}\): reject or reassign if below threshold (pattern exists in PN helper with `min_vc`).
2. **Time margin:** compare \(t_{go}\) to a **dynamics-required** time from short rollout or conservative bound.
3. **Reachable intercept:** run `_monte_carlo_kinematic_hit_rollout` with **same** `max_turn_rate_rad_s` and `max_accel_m_s2` as Gazebo / cmd path (`simulate_intercept_once` already supports `use_kinematic_rollout`).

**Early rejection:** yes — but reject on **plant-consistent** criteria, not geometry-only, otherwise oracle/TTI can look “correct” while the vehicle cannot execute.

---

## 5. Tracking / state estimation

Finite-difference velocity + smoothing tends to **lag** and amplify noise → bad \(t_{go}\) and phantom intercepts.

**Minimum viable upgrades (ROI order):**

1. **α–β filter** on measured position (CV model) — simple, large latency/noise win.
2. **α–β–γ** if maneuvers are material in the engagement dome.
3. **EKF** when multi-sensor fusion + irregular timestamps matter.

**Latency:** keep `compensate_target_for_delay`, but tune \(\tau\) to **effective transport delay**; for higher fidelity, augment filter / use retrodiction rather than aggressive open-loop extrapolation.

---

## 6. Monte Carlo vs Gazebo mismatch — how to fix properly

The repo contains multiple “success” notions:

- Fast feasibility: `is_intercept_feasible` (optimistic).
- Kinematic closed-loop rollout: `_monte_carlo_kinematic_hit_rollout` (closer to constrained motion).
- `estimate_hit_probability_light` explicitly documents **constant-velocity interceptor over \(t_{go}\)** — not the full plant.

**Fix:**

- Make reported **P(hit)** use **rollout with limits matched to Gazebo** (same parameters wired from config).
- Longer term: **single shared engagement dynamics module** used by heatmap exporter, MC harness, and (where possible) the live node’s predict step.

---

## 7. Failure mode analysis (mean ≫ median, huge P95)

**Interpretation:** most runs end “near miss” (median ~ few m), a **tail** of runs never enters normal terminal engagement (timeouts, infeasible dynamics, bad tracks), producing **km-scale** miss metrics when parsed.

**Suggested taxonomy**

| Class | Description |
|-------|-------------|
| F1 | Timeout / aborted sim — no `[HIT]`, large residual range |
| F2 | Geometry feasible, dynamics infeasible |
| F3 | Track/delay induced \(t_{go}\) / aimpoint instability |
| F4 | Multi-agent assignment switching |
| F5 | Metric contamination (aggregating unrelated logs / experiments) |

**Systematic debug logs (structured, periodic):** range, \(v_c\), \(t_{go,raw}\), \(t_{go,filt}\), commanded vs realized \(\omega\), commanded vs realized \(a\), feasibility bits, rollout pass/fail, “large jump” events in `u` or \(t_{go}\).

---

## 8. Priority roadmap

**Must-have**

1. **Clean evaluation cohorts** — MC aggregates must not mix unrelated logs (otherwise mean/P95 lie).
2. **Align MC success with plant** — enable kinematic rollout + matched limits for P(hit) and gates.
3. **Dynamics-aware feasibility** — rollout gate before commit.
4. **Stronger guidance damping** — `filter_t_go` + limits on `u` / aimpoint updates + terminal blending.

**High ROI next**

5. α–β / EKF velocity
6. Terminal braking / speed schedule near capture
7. Calibrated delay model

**Nice-to-have**

8. Short-horizon MPC for final squeezing under constraints
9. Adaptive PN gain schedule

---

## One-line executive summary

The CV intercept solver is **geometrically coherent**, but the system’s reliability bottleneck is the **missing reachable-set / dynamics-consistent layer** and **insufficient separation of optimistic MC vs Gazebo truth** — fix gates, estimation, and MC–plant parity before chasing higher intercept solver complexity.
