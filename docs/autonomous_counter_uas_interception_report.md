# Autonomous Counter-UAS Interception in a ROS 2 / Gazebo Kinematic Testbed

**Technical report (repository-grounded)**  
*Branch of analysis: implementation under `src/`, offline modules under `simulation/`, evaluation under `scripts`.*

| | |
|---|---|
| **Repository** | counter-uas-simulation |
| **Document type** | Systems engineering / autonomous defense simulation |
| **Scope** | Architecture, algorithms, simulation flow, validation, limitations |

---

## Table of contents

1. [Abstract](#abstract)
2. [System motivation](#1-system-motivation)
3. [System architecture](#2-system-architecture)
4. [Guidance and interception](#3-guidance-and-interception)
5. [Simulation framework](#4-simulation-framework)
6. [Sensor and tracking stack](#5-sensor-and-tracking-stack)
7. [Monte Carlo evaluation](#6-monte-carlo-evaluation)
8. [Engineering challenges](#7-engineering-challenges)
9. [Qualitative assessment (codebase inspection)](#8-qualitative-assessment-codebase-inspection)
10. [Limitations](#9-limitations)
11. [Future work](#10-future-work)
12. [Conclusion](#11-conclusion)
13. [Key code references](#12-key-code-references)

---

## Abstract

This report documents a **software integration testbed** for notional counter-uncrewed aerial system (C-UAS) intercept scenarios. The implemented stack combines a **constant-velocity (CV) point-mass intercept** root finder with a **heuristic outer loop**: line-of-sight pursuit with optional lead blending, **predictive** guidance from the CV intercept point, and a **PN-derived lateral steering** vector blended in the live node (`use_pn_refinement`, `pn_blend_gain`)—all subject to **commanded** turn-rate and acceleration limits and optional autopilot lag in `interceptor_controller_node`. Threat and defender motion are **pose-teleported** in Gazebo (`gz service set_pose`), not simulated as rigid bodies with aerodynamics. Peripheral ROS nodes emit **position-only** radar/camera proxies with Gaussian noise and Bernoulli detection, **scalar-weighted fusion** of points, and a **CV Kalman filter** with nearest-neighbor gating. Feasibility and Monte Carlo utilities evaluate **conditional** outcomes under explicitly named noise parameters; they are **not** validated against flight data and must not be read as \(P_{\mathrm{kill}}\).

**Scope disclaimer:** What follows is a **repository description**, not a peer-reviewed research contribution. Claims of “contributions” in the sense of novel guidance theory, stability certificates, or statistically calibrated effectiveness **are not made**. The artefact value lies in **traceable software**, **documented idealizations**, and **scripted regression-style evaluation** (`scripts/monte_carlo.py`, `scripts/analyze_run.py`, `scripts/validate_heatmap_vs_gazebo.py`).

---

## 1. System motivation

Closing on a maneuvering target with bounded lateral acceleration is a **coupled estimation–guidance–control** problem. The **idealized** subproblem treated by the CV intercept solver assumes the defender can move in \(\mathbb{R}^3\) with speed \(\|\mathbf{v}_I\| = s_i\) along the instantaneous collision ray. Real actuators enforce \(\|\dot{\mathbf{v}}_I\| \leq a_{\max}\) and bounded turn rate \(\dot{\sigma}_{\max}\); the **realized** velocity **lags** the LOS command, so the closed loop is **not** the same dynamical system as the quadratic used for planning.

**Measurement delay** \(\tau\) compounds bias: if the solver ingests \(\mathbf{p}_T(t-\tau)\) while applying CV extrapolation \(\mathbf{p}_T(t) \approx \mathbf{p}_T(t-\tau) + \mathbf{v}_T \tau\), correctness requires \(\mathbf{v}_T\) **coherent with the same measurement epoch** and **constant over \(\tau\)**. The implementation exposes `measurement_delay_s` as a **scalar** extrapolation in `interception_logic_node.py`; it does **not** replace a **time-stamped, clock-aligned** fusion/track pipeline with known filter delays.

**Terminal geometry:** as range \(\|\mathbf{r}\| \to 0\), LOS rate magnitude generically grows for non-degenerate lateral motion, amplifying any noise on \(\mathbf{v}_T\) in \(\mathbf{p}_{\mathrm{hit}} = \mathbf{p}_T + \mathbf{v}_T t\).

This repository therefore separates **(A)** a closed-form CV intercept used for bearing/speed intent from **(B)** `_accel_limit_velocity` and optional autopilot dynamics—**sequential** heuristics, not a single optimal controller or differential-game solution.

---

## 2. System architecture

### 2.1 Major subsystems

| Subsystem | Location | Role |
|-----------|----------|------|
| World / threat motion | `gazebo_target_sim/target_controller_node.py` | Hostile trajectory (orbit, approach, LOS-to-origin); ground truth topics; Gazebo `set_pose`. |
| Defender plant | `gazebo_target_sim/interceptor_controller_node.py` | `cmd_velocity` integration; speed clamp; optional EMA / first-order autopilot; command-delay FIFO; impact hide. |
| Engagement / guidance | `gazebo_target_sim/interception_logic_node.py` | Selection, CV solve, pursuit/predict hysteresis, PN-like blend, dome/layer policy, hit logic, metrics logging. |
| Pure-Python math | `gazebo_target_sim/guidance_lib.py` | Intercept time, delay compensation, speed alignment, \(t_{\mathrm{go}}\) filter helpers. |
| Offline intercept study | `simulation/core/intercept.py`, `simulation/guidance/*.py`, `simulation/realtime_sim.py` | Feasibility helpers; naive / predictive / PN; **distinct** dynamics from live ROS limiter. |
| Radar / camera | `radar_sim/radar_sim_node.py`, `camera_sim/camera_sim_node.py` | Range, FOV, Gaussian noise, \(P_d\), optional publish delay. |
| Fusion | `fusion/fusion_node.py` | Weighted mean; disagreement policy; paired-input gate. |
| Tracking | `tracking/tracking_node.py` | CV Kalman; dual-gate association; candidate confirmation. |
| Threat labeling | `threat_assessment/threat_assessment_node.py` | Distance-based coarse label (not a classifier). |
| Launch | `counter_uas/launch/bringup.launch.py` | Orchestrates Gazebo stack and perception nodes. |

### 2.2 Execution flow

1. **Truth:** Threat pose updated at fixed rate; `/drone/position` (or per-target topics in multi-target modes).
2. **Perception (optional):** detections → fusion → `/tracks` or `/tracks/state`.
3. **`interception_logic_node`:** reads `intercept_measurement_source` (`ground_truth`, `fused`, `tracks`, `tracks_state` per parameters in `InterceptionLogicNode.__init__`); maintains committed interceptor; publishes per-id **`geometry_msgs/Vector3`** `cmd_velocity`.
4. **`interceptor_controller_node`:** applies dynamics layer; integrates; calls **`set_pose`** each tick.
5. **Termination:** geometric hit (with segment–segment closest approach and safety guards); `ImpactEvent` (`gazebo_target_sim_interfaces/msg/ImpactEvent.msg`); stop signals; optional Gazebo pause.

### 2.3 Structural integrity note

Intercept geometry is implemented in **multiple places** (`simulation/core/intercept.py`, `guidance_lib.py`, and internally in `interception_logic_node.py`). Tolerances and bugfixes can **diverge**—a verification and maintenance liability.

---

## 3. Guidance and interception

### 3.1 Predictive interception (CV lead)

Given \(\mathbf{r}_0 = \mathbf{p}_T - \mathbf{p}_I\) and constant \(\mathbf{v}_T\), the smallest \(t > 0\) satisfying

\[
\|\mathbf{r}_0 + \mathbf{v}_T t\| = s_i t
\]

is computed in `solve_intercept_time` (`simulation/core/intercept.py`) and `guidance_lib.solve_intercept_time`. The predicted intercept is \(\mathbf{p}_{\mathrm{hit}} = \mathbf{p}_T + \mathbf{v}_T t\); the collision direction \(\hat{\mathbf{u}}\) points from \(\mathbf{p}_I\) toward \(\mathbf{p}_{\mathrm{hit}}\).

**Why:** minimal closed form for non-maneuvering threats; supplies \(t_{\mathrm{go}}\) for speed scheduling and logging.

**Caveat:** assumes defender can realize \(s_i\) along that ray; **`align_speed_to_solver`** and `align_speed_after_saturation` (`guidance_lib.py`) partially align commanded speed with the solver; **turn and accel limits** still violate the ideal model every cycle.

### 3.2 Pursuit behavior

Pure geometric pursuit uses \(\hat{\mathbf{u}}_{\mathrm{pp}} = (\mathbf{p}_T - \mathbf{p}_I) / \|\mathbf{p}_T - \mathbf{p}_I\|\) (`compute_naive_direction` in `simulation/guidance/naive.py`). The live node **does not** implement this as an isolated law: it mixes in lead via **`pursuit_lead_blend`** against a velocity estimate, applies **`target_velocity_smooth_alpha`** on \(\mathbf{v}_T\), and may **re-aim** at a point on the mid-dome shell (`aim_strike_on_mid_shell`) for scenario policy. Consequently the label “pursuit” in logs refers to a **mode state**, not a textbook pure-pursuit trajectory.

### 3.3 Proportional navigation refinement

**Literature vs implementation:** Classical true proportional navigation commands acceleration **perpendicular** to the LOS in the engagement plane; 3D extensions vary by definition of the LOS triad. In `simulation/guidance/pn.py`, `compute_pn_acceleration` forms

\[
\boldsymbol\omega = \frac{\mathbf{r} \times \mathbf{v}_r}{\|\mathbf{r}\|^2}, \quad
V_c = -\hat{\mathbf{r}}^{\top} \mathbf{v}_r, \quad
\mathbf{a}_{\mathrm{PN}} = N \, V_c \, (\boldsymbol\omega \times \hat{\mathbf{r}}),
\]

with \(\mathbf{r} = \mathbf{p}_T - \mathbf{p}_I\), \(\mathbf{v}_r = \mathbf{v}_T - \mathbf{v}_I\), then **saturates** \(\|\mathbf{a}_{\mathrm{PN}}\| \leq 40\,\mathrm{m/s^2}\) (hard-coded cap). The live stack maps a related quantity through `_pn_steering_vector` and blends it with predictive LOS using `pn_blend_gain` and `pn_navigation_constant`, then passes the result through **velocity** limiters—not a continuous-time PN plant. **Terminology:** “PN” here means **PN-inspired lateral steering in software**, not a certified PN loop with documented capture region under these discrete, saturated dynamics.

### 3.4 Blending and mode hysteresis

Predictive vs pursuit mode is latched in `_update_guidance_mode` (`interception_logic_node.py`): valid/invalid intercept-solve streaks (`predict_enter_frames`, `predict_exit_frames`) drive a discrete state machine; the returned **blend factor** ramps during transitions so commanded direction does not jump when feasibility flickers. Separate parameters (`guidance_terminal_range_m`, `guidance_terminal_pursuit_blend`, `pn_blend_terminal_range_m`) reduce lead/PN authority near intercept. **Verification gap:** no Lyapunov or capture-region analysis is supplied; stability is **engineering judgment** plus log tuning.

### 3.5 Feasibility validation

**CV intercept existence:** Squaring \(\|\mathbf{r}_0 + \mathbf{v}_T t\| = s_i t\) yields a scalar quadratic \(a t^2 + b t + c = 0\) with \(a = \|\mathbf{v}_T\|^2 - s_i^2\), \(b = 2 \mathbf{r}_0^{\top}\mathbf{v}_T\), \(c = \|\mathbf{r}_0\|^2\). Implementations select positive roots that satisfy the **unsquared** norm equality within numerical tolerance (`solve_intercept_time` in `simulation/core/intercept.py` / `guidance_lib.py`).

**TTI to a defensive sphere:** `compute_tti_target` solves \(\|\mathbf{p}_T + \mathbf{v}_T t - \mathbf{c}\|^2 = R^2\) for the smallest \(t>0\) with an **approaching** radial component (\(\mathbf{r}_0^{\top}\mathbf{v}_T < 0\) in the implementation).

**Windowed feasibility:** `minimum_intercept_closing_speed` / `is_intercept_feasible` ask whether some \(s \leq v_{I,\max}\) admits an intercept with \(t \in [t_{\min}, t_{\max}]\) under the **same** CV, constant-speed idealization.

**Sensing gates:** `evaluate_intercept_feasibility_realistic` concatenates disk detection, fixed tracking delay, and delay-budget sums—**combinatorial logic on scalars**, not a stochastic timed automaton.

These checks certify **consistency of the toy model**, not operational engagement success.

### 3.6 Motion constraints

`_accel_limit_velocity` applies (1) a **heading rate limit**: rotate previous velocity direction toward the desired direction by at most \(\dot{\psi}_{\max} \Delta t\), then (2) clamp \(\|\mathbf{v}_{\mathrm{cmd},k} - \mathbf{v}_{\mathrm{cmd},k-1}\| \leq a_{\max} \Delta t\). The outer `interceptor_controller_node` may further low-pass or delay commands (`autopilot.tau_s`, `autopilot.cmd_delay_s`).

**Tradeoff (precise language):** additional lag **reduces high-frequency command energy** (smoother Gazebo pose updates) but **increases effective time delay** in the guidance loop, **tightening** gain–bandwidth limits for any feedback path that depends on fast LOS correction—**not** “phase margin” in the sense of a linearized open-loop Bode plot (no such analysis is presented).

---

## 4. Simulation framework

### 4.1 Loop and time step

Default declared rates differ by node: `interception_logic_node` uses `rate_hz` (commonly 20 Hz); `interceptor_controller_node` integrates at `rate_hz` (commonly 10 Hz). **Multi-rate** execution implies the defender sees a **zero-order hold** on `cmd_velocity` between guidance updates and that **worst-case asynchronous delay** is on the order of one guidance period unless message timestamps drive compensation (they **do not** in the default `Vector3` contract).

### 4.2 Propagation

**Defender (after smoothing/delay in plant):** with applied velocity \(\mathbf{v}_k\) at step \(k\),

\[
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \, \Delta t .
\]

Poses are written to the simulator via **`set_pose`**, so **no contact dynamics, drag, or IMU** appear in the baseline loop.

**Threat:** piecewise rules in `target_controller_node.py` (orbit, then LOS attack toward the asset with configurable closing speed and dive gain). This is **scripted kinematics**, not an adversarial policy.

**Fidelity statement:** The testbed is a **pose integration + policy** simulator, **not** a 6-DoF or autopilot-in-the-loop engagement model.

### 4.3 Hit evaluation

Let \(\mathbf{p}_T^{k}\), \(\mathbf{p}_I^{k}\) be discrete samples. The code forms segment–segment closest distance \(d_{\mathrm{seg}}\) between \([\mathbf{p}_T^{k-1}, \mathbf{p}_T^{k}]\) and \([\mathbf{p}_I^{k-1}, \mathbf{p}_I^{k}]\), then uses \(\min(\|\mathbf{p}_T^{k} - \mathbf{p}_I^{k}\|, d_{\mathrm{seg}})\) against a threshold. This mitigates **tunneling** when \(\Delta t \|\mathbf{v}_T - \mathbf{v}_I\|\) is large relative to the kill radius. **Additional gates:** minimum interceptor altitude and path length (`hit_min_interceptor_z_m`, `hit_min_interceptor_travel_m`) before declaring `[HIT]`.

**Validation gap:** Kill radius is **not** tied to fuze or warhead lethal radius; correlation to soft `ImpactEvent` vs hard contact is **optional** (`hit_contact_corroborator_node.py`).

### 4.4 Success metrics

Run outcome is **defined by log parsing**: `parse_run_to_result` in `scripts/analyze_run.py` applies regexes to `[HIT]`, `[METRICS]`, and related tags. **Limitations:** log-drop or non-standard output breaks metrics; no formal **measurement uncertainty** on reported `min_miss`; success is **binary** at the threshold, not a graded lethality model.

---

## 5. Sensor and tracking stack

### 5.1 Radar and camera

Both publish noisy **`geometry_msgs/Point`**: range/beam or FOV gates, Gaussian position noise, Bernoulli \(P_d\), optional timer-based delay (`delay_mean_s`, `delay_jitter_s`).

**Not modeled:** waveform SNR, clutter maps, resolution cells, track-before-detect.

### 5.2 Fusion

Weighted average when sensors agree; on disagreement **one** output per policy (`prefer_radar`, `prefer_camera`, `drop`). **`fusion.require_paired_inputs`** avoids duplicate publishes from stale/fresh pairs (`fusion_node.py`).

### 5.3 Tracking

`tracking_node.py`: 6-state CV Kalman; dual association gates; candidate confirmation; **timestamp-aware** velocity seeding from history (documented in module docstring).

**Not implemented:** JPDA/MHT, IMM, dense clutter field.

### 5.4 Additional noise path

`noisy_measurement_node.py`: Gaussian noise + dropout on arbitrary `Point` topics.

---

## 6. Monte Carlo evaluation

### 6.1 Full-stack harness

`scripts/monte_carlo.py`: `run` mode drives `run_capture.py` with per-trial seeds; `aggregate` mode scans logs; emits JSON/CSV summaries (`_summarise`). **Statistical hygiene:** the script reports sample success rate \(\hat{p} = n_{\mathrm{ok}}/N\) and simple moments (mean/median/percentiles of miss distance). It does **not** by default output **confidence intervals** (e.g. Wilson or Agresti–Coull for binomial \(p\)), **Monte Carlo standard error** for rare events, or **multi-seed replication** reports. **Interpretation:** for small \(N\), or when \(\hat{p}\) is near 0 or 1, tabulated rates are **noisy point estimates**.

### 6.2 In-node probability / heatmap

`simulate_intercept_once`, `_monte_carlo_kinematic_hit_rollout`, `estimate_hit_probability`, `estimate_hit_probability_light` in `interception_logic_node.py`. **`estimate_hit_probability_light`** explicitly states it is **not** calibrated operational \(P_{\mathrm{kill}}\).

Rollout can mirror turn-then-accel ordering of `_accel_limit_velocity`; optional instant-heading legacy path when limits are zero.

**Estimator–reality gap:** light estimators may assume constant closing along the solver ray or post-limit `cmd_velocity`; the full stack adds **second plant filtering/delay** and **`set_pose` latency** not modeled in simple draws. Heatmap threat motion parameters can **differ** from `target_controller_node` unless scenarios are **explicitly coupled**; `scripts/validate_heatmap_vs_gazebo.py` is **spot-check validation**, not calibration over the full envelope.

### 6.3 Limitations (summary)

- Per-cell MC in heatmaps uses finite \(n\); **no reported standard error** in the exported artefacts by default.
- **i.i.d.** trial assumptions are **not** enforced; matrix campaigns need disciplined **cohort** metadata (`--cohort`, `.meta.json`).
- MC answers **conditional on this codebase and recorded parameters**; **external validity is unclaimed**.

---

## 7. Engineering challenges

| Challenge | Mechanism in code | Residual risk |
|-----------|-------------------|---------------|
| Solver–plant mismatch | `align_speed_to_solver`, two-pass speed in intercept solve | Saturation still biases geometry |
| Delay | `measurement_delay_s` CV extrapolation | Scalar knob ≠ stamped pipeline |
| Terminal jitter | `t_go_filter_alpha`, intercept point EMA | Heuristic, no optimality claim |
| Mode chatter | Hysteresis frames | Tuning-dependent |
| Soft hit | Geometry + optional `hit_contact_corroborator_node.py` | Physics contact not mandatory |

---

## 8. Qualitative assessment (codebase inspection)

This section records **static** observations from the implementation—not an experimental campaign. **No** flight-test comparisons, **no** Monte Carlo sample sizes, and **no** statistical significance claims appear here.

**Properties that support internal R&D use**

- Runnable ROS 2 graph from perception stubs through guidance to Gazebo pose updates.
- Structured stdout (`[HIT]`, `[METRICS]`) ingestible by `scripts/analyze_run.py` for regression-style batching.
- Explicit motion limiters and optional delay extrapolation hooks—**documented** idealizations rather than silent assumptions.
- Hit logic uses segment–segment distance to reduce one form of discrete-time artefact.

**Structural liabilities**

- `interception_logic_node.py` concentrates policy, guidance, visualization, and estimators (high change–risk surface).
- Intercept geometry duplicated across `simulation/core/intercept.py`, `guidance_lib.py`, and the live node (consistency hazard).
- Sensing and fusion are **position-toy** models; any “stack-level” demo remains far from RF/optical physics.

---

## 9. Limitations

1. **No credible operational \(P_{\mathrm{kill}}\)**—only model-conditioned estimates.
2. **No aerodynamics / wind / propulsion**—scalar caps are not energy models.
3. **Thin message contracts** on some paths (`Point`/`Vector3` without stamp/covariance on GT), though `tracks_state` improves the estimation interface.
4. **Scripted threat**; limited evasion.
5. **Classification / ROE** uses placeholder confidence parameters, not a detector–classifier stack.

---

## 10. Future work

- **Estimation:** stamped track messages; IMM; clutter and false-track stress tests.
- **Validation:** rollouts including `interceptor_controller_node` tau/delay; scenario YAML locking heatmap assumptions to `target_controller_node`.
- **Assignment:** formalize multi-target bipartite / stability criteria already sketched in parameters.
- **Sensing tiers:** detection lists, SNR-dependent \(P_d\), angular measurements.
- **Multi-agent:** deconfliction and resource constraints.
- **Hardware:** replace `set_pose` with autopilot-in-the-loop while preserving guidance outputs.

---

## 11. Conclusion

The artifact is best classified as a **software–integration demonstrator** with **named** guidance and estimation blocks: CV intercept roots provide a transparent planning aid; outer-loop heuristics approximate pursuit, lead, and PN-style correction under saturation; **success is defined in-simulation** via geometry and log parsing. Readers must **not** infer validated lethality, RF performance, or autopilot-coupled stability from this repository alone.

Raising the document to **publication-grade verification** would require, at minimum: (i) a **single** audited intercept kernel shared by offline and online code; (ii) **time-stamped** measurement interfaces and documented delay budgets; (iii) Monte Carlo outputs with **interval estimates** and **scenario versioning**; (iv) where claims approach operations, **hardware-in-the-loop** or high-fidelity vehicle evidence cited **outside** this Markdown file.

---

## 12. Key code references

| Artifact | Path |
|----------|------|
| Live guidance / policy / hit | `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py` |
| Defender plant | `src/gazebo_target_sim/gazebo_target_sim/interceptor_controller_node.py` |
| Threat motion | `src/gazebo_target_sim/gazebo_target_sim/target_controller_node.py` |
| Intercept geometry (NumPy) | `simulation/core/intercept.py` |
| Intercept helpers (stdlib) | `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py` |
| Offline PN / predictive / naive | `simulation/guidance/pn.py`, `predictive.py`, `naive.py` |
| Offline comparison loop | `simulation/realtime_sim.py` |
| Fusion | `src/fusion/fusion/fusion_node.py` |
| Tracking | `src/tracking/tracking/tracking_node.py` |
| Radar / camera | `src/radar_sim/radar_sim/radar_sim_node.py`, `src/camera_sim/camera_sim/camera_sim_node.py` |
| MC harness | `scripts/monte_carlo.py` |
| Log metrics parser | `scripts/analyze_run.py` |
| Heatmap vs Gazebo | `scripts/validate_heatmap_vs_gazebo.py` |
| Impact corroboration | `src/gazebo_target_sim/gazebo_target_sim/hit_contact_corroborator_node.py` |
| Credibility memo (partially historical) | `docs/CREDIBILITY_REALISM_TECHNICAL_REVIEW.md` |

---

*End of document.*
