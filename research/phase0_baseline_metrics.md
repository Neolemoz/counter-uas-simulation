## Phase 0 — Baseline metrics & success criteria (log-derived)

Goal: define **consistent metrics** and **pass/fail** rules for the whole research pipeline, using **existing stdout logs** only (no guidance changes).

### Unit of evidence
- **One run** = one stdout/stderr log capture from a single scenario + parameter set.
- The parser should treat one log file as one run.

### Metrics (what they mean, and how to read them)

#### **Hit event** — `[HIT]`
Emitted when the active hit check triggers (distance falls below the configured threshold, and any gating is satisfied).

**Log shape (single-target)**:
- `"[HIT] <selected_id> layer=<layer?>  min_miss=<mm>  hit_threshold = <thr> m"`

**Log shape (multi-target)**:
- `"[HIT] <target_label> by <interceptor_id> layer=<layer?>  min_miss=<mm>  hit_threshold = <thr> m"`

**Fields to parse**:
- **hit**: `true` if any `[HIT]` line exists in the run.
- **layer_at_hit**: from the `layer=<...>` token if present; otherwise `unknown`.
- **hit_threshold_m**: parse the numeric after `hit_threshold =`.
- **min_miss_m_at_hit_line**: parse numeric after `min_miss=` if present (string ends with `m`).

Source: `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py` prints `[HIT] ...` when `dist_to_target < hit_threshold_m`.

#### **Minimum miss distance** — `[min_miss]`
This is the **minimum 3D range** \(|P_T-P_I|\) observed during “active guidance / hit-check” ticks in a run.

**Log shape (periodic)**:
- `"[min_miss] = <value> m"` or `"[min_miss] = n/a"`
- `"hit_threshold = <thr> m"`

**Fields to parse**:
- **min_miss_m**: smallest numeric value ever printed on `[min_miss] = ... m` lines for the run (if any).
- **hit_threshold_m**: numeric from `hit_threshold = ... m` (should be constant within a run; keep the last seen value).

Notes:
- If `[min_miss] = n/a` for the whole run, treat `min_miss_m` as missing.

Source: `InterceptionLogicNode._print_miss_distance_lines()` prints these lines on a timer cadence.

#### **Dome layer** — `[LAYER]`
Layer is derived from the threat distance to dome center, mapped to:
- `outside`, `detect`, `select`, `engage` (or `engage` when dome is disabled).

**Log shape**:
- `"[LAYER] <layer>"`

**Fields to parse**:
- **layer_last**: last layer seen in the run.
- **layer_transitions** (optional): ordered list of layer change events.

Source: `InterceptionLogicNode._on_control()` prints `[LAYER]` only when layer changes.

#### **Intercept mode + debug dump** — `[Intercept Debug]`
When `intercept_debug:=true`, the node prints a multi-line block:

**Log shape**:
- `"[Intercept Debug]"`
- `mode=<predict|pursuit|naive|hold>`
- `distance=<m>`
- `t_hit=<s|n/a>`
- `target_pos=(x,y,z)`
- `interceptor_pos=(x,y,z)`
- `cmd_vel=(vx,vy,vz)`
- `alignment=<dot(unit(LOS), unit(cmd_vel))|n/a>`

**Fields to parse (optional for Phase 1)**:
- **mode_series**: list of `mode=` values over time.
- **distance_series_m**: list of `distance=` values over time.
- **t_hit_series_s**: list of `t_hit=` values (ignore `n/a`).

Source: `InterceptionLogicNode._maybe_detail_log()` prints this block.

---

### Pass/Fail definition (across runs)

#### Per-run labels
- **hit_run**: `true` iff a `[HIT]` line exists in that run.
- **min_miss_m**: from `[min_miss]` lines (if missing, treat as `inf` for conservative scoring).
- **hit_threshold_m**: from `hit_threshold =` lines.
- **success_by_distance** (fallback label): `true` iff `min_miss_m <= hit_threshold_m`.

Canonical rule for success uses events first:
- **success_run = hit_run**.

Fallback rule (only if `[HIT]` is absent but `[min_miss]` exists):
- **success_run = success_by_distance**.

#### Aggregate success criteria (baseline)
For a batch of \(N\) runs under the same scenario + parameter set:
- **hit_rate** \(= \frac{\#success\_run}{N}\)
- **min_miss distribution**: summarize with median + p90 of `min_miss_m` (treat missing as `inf`).

**Pass** (baseline gate) when BOTH:
- **hit_rate ≥ 0.80** (default baseline threshold; tighten later once runner is stable)
- **median(min_miss_m - hit_threshold_m) ≤ 0.00 m**

**Fail** otherwise.

Rationale:
- `hit_rate` captures reliability under stochasticity / timing.
- `min_miss - hit_threshold` captures “how close” even when no `[HIT]` is emitted (or when runs end early).

---

### Exact output format we will parse later (Phase 1/5)
Phase 1/5 should be able to summarize each run into a single record using only existing tags.

#### Parse targets (required)
- **`[HIT]`**: determines `hit_run` and provides `layer_at_hit` (if present).
- **`[min_miss]` + `hit_threshold =`**: determines `min_miss_m` and `hit_threshold_m`.
- **`[LAYER]`**: provides `layer_last` and (optionally) transitions.

#### Derived per-run summary record (JSON-like schema)
This is the exact field set we will generate in Phase 1/5 (from parsing the log):

```text
run_id: <string>                  # filename stem or timestamp
hit: <true|false>
hit_threshold_m: <float|nil>
min_miss_m: <float|nil>           # best-known min miss in the run
layer_at_hit: <string|unknown>    # from [HIT] if present
layer_last: <string|unknown>      # from last [LAYER] if any
```

#### Notes for parser implementation
- Prefer `[HIT]` as ground truth for hit; use `min_miss <= hit_threshold` only as fallback.
- Treat missing numeric values as `nil` during parsing; only coerce to `inf` at aggregation time.
- `hit_threshold_m` should be constant within a run; if it changes, keep the last seen and flag the run for review.

