# Technical review memo (credibility & realism — not demo quality)

**Subject:** Counter-UAS simulation stack — realism assessment and architecture  
**Lens:** Defense / systems engineering source selection or IRAD gate (what would survive a skeptical technical review)

---

## 1. What is “fake realism” (looks real, isn’t)

| Area | What the stack does | Why it’s fake for fielded C-UAS |
|------|---------------------|----------------------------------|
| **Sensing** | `Point` detections with Gaussian noise, range/FOV gates, Bernoulli \(P_d\) | No waveform, no SNR, no multipath/clutter model, no resolution cell, no track-before-detect, no registration between sensors. “Radar” and “camera” are **2D/3D point toys** with noise knobs. |
| **Fusion** | Weighted average of latest radar + camera `Point`s; on disagreement, **two separate publishes** | Not a **track-level** or **state-level** fusion; no cross-sensor association uncertainty, no time sync, no retained hypothesis set. Reads as **placeholder**, not fusion. |
| **Tracking** | Single CV Kalman + greedy NN, fixed gates | No maneuver index, no interacting multiple model, no clutter model density, no consistent JPDA/MHT. Fine for **pedagogy**; not **threat tracking under clutter**. |
| **Threat / classification** | Placeholder confidence threshold | Not a classifier, not a kill chain criterion tied to ROE. |
| **Interception** | Closed-form CV intercept + blended pursuit/PN-like steering + feasibility bisection | Geometry for **ideal point mass** at constant \(v_T\); PN is **not** implemented as acceleration normal to LOS from LOS rate. **Dynamics mismatch** (instant heading/speed vs limits) is **unmodeled** in the solve. |
| **Effector / interceptor** | Kinematic integration + `cmd_velocity`; Gazebo pose via service | No aerodynamics, no autopilot inner loop, no latency/jitter on effector, no seeker, no fuze, no drag/wind. **Weapon is a point that slides along velocity.** |
| **Engagement policy** | Dome shells, strike band, MC “P(hit)” from simplified draws | Policy is **scenario dressing** unless tied to ROE, weapon envelopes, and **validated** \(P_{\mathrm{kill}}\) models. MC layer risks **false precision** if presented as operational probability. |
| **Latency** | Optional gate timer | Not a **transport + processing + fusion** latency budget with **compensated** state at a common time. |

**Bottom line:** The stack is a **coherent software integration demo** with **named** defense subsystems; many of those names **do not denote** the engineering artifacts a contractor would mean by them.

---

## 2. What is scientifically correct (defensible)

| Element | Why it’s sound |
|---------|----------------|
| **CV intercept quadratic** | For constant \(\mathbf v_T\) and **kinematic** interceptor at constant speed along the collision LOS from a fixed \(\mathbf p_I\), \(\|\mathbf r_0 + \mathbf v_T t\| = s t\) is **standard**; root selection and geometric tolerance checks are legitimate numerics. |
| **Feasibility / min closing speed (bisection)** | Asking whether an intercept exists in \([t_{\min}, t_{\max}]\) under a speed cap is a **well-posed** constrained feasibility question for that same idealized model. |
| **Segment–segment closest approach for HIT** | Using closest distance between motion segments between ticks is a **correct** way to reduce tunneling artifacts for **discrete** simulation time (collision detection literature). |
| **Basic Kalman predict/update** | CV model + \(Q,R\) + gating is **valid** as a **simple** estimator; math is fine **for the model class**. |
| **Explicit dome / range policy as code** | Layered engagement geometry is a **reasonable** way to encode **notional** ROE in sim — as long as you **label** it scenario policy, not validated doctrine. |

These pieces are **internally consistent** within their **stated** assumptions. The problem is those assumptions are **not** disclosed as limiting when you sell “counter-UAS.”

---

## 3. What I would reject in a technical review

**Would reject (or flag as non-compliant) if presented as production-relevant:**

1. **Fusion** that emits **inconsistent** outputs (e.g. two unrelated points per “fusion event”) — fails basic **single coherent track** requirement.
2. **`geometry_msgs/Point` as the sole track interface** — no stamp, no velocity/covariance in the message contract used for engagement (even if velocity exists internally). **Not auditable** for safety-of-flight reasoning.
3. **Guidance law** that mixes predictive LOS, blended modes, and “PN” **without** a single **closed-loop stability** or **performance** argument under delay and saturation — **not acceptable** for a weapon channel without analysis or test evidence.
4. **Claiming proportional navigation** without defining **lateral acceleration** from **LOS rate** in a consistent frame — reviewers will call this **mislabeled**.
5. **Monte Carlo / heatmap P(hit)** unless tied to **documented** error models, **validated** against higher-fidelity truth, and **not** conflated with operational kill probability.
6. **Interceptor model** as perfect velocity tracking with optional smoothing — **no** separation of **guidance / autopilot / actuator**; cannot support **bandwidth / delay** claims.
7. **Threat assessment** as a scalar placeholder — cannot support **target identification** or **collateral** arguments.

**Process rejection:** Any submission that does not separate **assumptions**, **validated** vs **parametric** submodels, and **test results** (Monte Carlo with defined scenarios) would not pass a serious **V&V** line of questioning.

---

## 4. Proposed corrected architecture (credibility-first)

**Goal:** Each block has a **clear I/O contract**, **time**, **uncertainty**, and **fidelity tier** you can defend.

```
[Truth / higher-fidelity sim] (optional parallel branch for V&V only)
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ SENSOR SIMULATION (tiered)                                 │
│  L0: stamped detections + noise (current)                │
│  L1: + latency distribution, dropout, rate limits         │
│  L2: + simple SNR/clutter draw (even crude) + sensor IDs   │
└───────────────────────────────────────────────────────────┘
        │  DetectionsStamped{sensor_id, t, z, R, optional feat}
        ▼
┌───────────────────────────────────────────────────────────┐
│ TRACKING / ASSOCIATION                                     │
│  Output: TrackListStamped — per track: id, x, P, mode     │
│  (CV baseline; IMM/JPDA as upgrade path — same output type) │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ FUSION (track-level)                                       │
│  Single time-aligned state per threat OR explicit MHT out │
│  Never “two unrelated points” as one fusion tick          │
└───────────────────────────────────────────────────────────┘
        │  ThreatState{track_id, t_ref, x, P, classification?}
        ▼
┌───────────────────────────────────────────────────────────┐
│ THREAT / ROE (explicitly labeled “scenario policy”)        │
│  Outputs: engage_yes/no, weapon, constraints              │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ GUIDANCE (separate library, unit-tested)                   │
│  Inputs: ThreatState at t_ref, OwnshipState, limits       │
│  Options:                                                   │
│    - CV intercept + **reachable velocity** each step       │
│    - or short-horizon MPC / iLQR under a_max              │
│  PN only as **defined** a_perp = N * Vc * lambda_dot       │
└───────────────────────────────────────────────────────────┘
        │  Command: a_cmd or v_cmd + limits, stamped
        ▼
┌───────────────────────────────────────────────────────────┐
│ EFFECTOR / AUTOPILOT MODEL                                 │
│  L0: saturated integrator (current)                        │
│  L1: + delay, bandwidth, rate limits                       │
│  L2: simplified 6-DOF or industry surrogate model          │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ METRICS & V&V                                              │
│  Miss distance CDF, sensitivity to τ and σ, ablation logs  │
│  Truth branch comparison (when available)                  │
└───────────────────────────────────────────────────────────┘
```

**Principles a contractor would expect**

1. **One threat state message** (or explicit multi-hypothesis list) — not ambiguous `Point` streams.
2. **Time** everywhere: `header.stamp` + `t_ref` for what the state means.
3. **Fusion produces one coherent output per cycle** (or labeled alternatives).
4. **Guidance** does not live in a 6k-line node with RViz; it’s a **library** with tests.
5. **Fidelity tiers (L0/L1/L2)** so you can say what is validated vs illustrative.
6. **No false labels**: “PN” and “P_kill” mean specific, reviewable definitions.

---

## Closing (reviewer voice)

The current system is **useful for integration learning and geometry prototyping**; it is **not** credible as a representation of a fielded kill chain until the **measurement–track–guidance contract** and **dynamics alignment** are fixed and **V&V metrics** are standard. Optimizing for a **working demo** without tightening those contracts **increases** reputational risk in front of a defense audience.

---

*Exported for sharing with other tools / reviewers. Path in repo: `docs/CREDIBILITY_REALISM_TECHNICAL_REVIEW.md`*
