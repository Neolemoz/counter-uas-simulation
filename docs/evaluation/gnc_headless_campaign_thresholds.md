# GNC headless campaign — pass/fail thresholds and archived aggregates

Formal **gates** below are **campaign parameters** chosen for this dome / single-target sim. Adjust numerically when tactics, timeouts, threat priors, or plant limits change.

## Section 7 — Formal success criteria (rollout gate B vs baseline A)

| Criterion | Pass rule | Notes |
|-----------|-----------|--------|
| P95 miss (finite samples) | P95<sub>B</sub> ≤ **0.90 × P95<sub>A</sub>** OR P95<sub>A</sub> − P95<sub>B</sub> ≥ **2.0 m** | Pick one dominance rule before unblinding; second is sanity check |
| P95 deterioration | Fail if P95<sub>B</sub> > **1.07 × P95<sub>A</sub>** (7% regress) OR > **10% relative** whichever is stricter vs agreed baseline | Tune 5–10% band with stakeholders |
| Success rate | **Non-inferiority**: SR<sub>B</sub> ≥ SR<sub>A</sub> − **2.0 percentage points** | Gate may trade hits vs false engagement; tighten if doctrine demands |
| Success lift (alternate) | If pursuing active lift: SR<sub>B</sub> − SR<sub>A</sub> ≥ **3.0** points **and** timeouts stable | Requires N ≥ 40 paired seeds |
| F2+F5 trajectory | Hist(F2)+Hist(F5) ↓ versus A **unless** justified by F4/F1 redistribution | Inspect with `summarize_failure_classes.py` |
| Timeout / F1 | ΔF1 vs A ≤ **+5 absolute points** | Coreutils timeout / `capture_rc` 124 / `=== TIMEOUT ===` |

Guidance damping (group C vs A): prioritize **same P95 rules** plus **non-inferiority** on SR; instrument `delta_t_go_raw` via `eng_metrics_period_s` in a metrics sub-batch.

## Archived aggregates (fixture copies)

Committed snapshots (**`runs/` is gitignored**) mirror the Monte Carlo aggregates under:

- [`scripts/evaluation/fixtures/gnc_primary_baseline.summary.json`](../../scripts/evaluation/fixtures/gnc_primary_baseline.summary.json)
- [`scripts/evaluation/fixtures/gnc_primary_rollout.summary.json`](../../scripts/evaluation/fixtures/gnc_primary_rollout.summary.json)

Failure histograms from the paired CSV cohorts:

- [`scripts/evaluation/fixtures/gnc_primary_baseline.failure_hist.json`](../../scripts/evaluation/fixtures/gnc_primary_baseline.failure_hist.json)
- [`scripts/evaluation/fixtures/gnc_primary_rollout.failure_hist.json`](../../scripts/evaluation/fixtures/gnc_primary_rollout.failure_hist.json)

Raw logs remain under **`runs/logs/`** locally (`*.meta.json` carries `cohort`).

### Recorded paired campaign (same seeds 9201–9240)

| Arm | Meta cohort tag | Seed base | Runs | SR | Miss P95 |
|-----|-----------------|----------|-----|-----|----------|
| A baseline | `gnc_primary_A_baseline` | 9201 | 40 | 70.0% | ~30.64 m |
| B rollout gate | `gnc_primary_B_rollout` | 9201 | 40 | 75.0% | ~19.39 m |

Against the **≤0.90 × P95<sub>A</sub>** rule (example): 0.90 × 30.64 ≈ 27.58 m margin — rollout P95 ~19.4 m (**pass**).

Run `python3 scripts/monte_carlo.py compare --inputs fixtures/gnc_primary_baseline.summary.json fixtures/gnc_primary_rollout.summary.json` when executed from **`scripts/evaluation/`** … or pass absolute repo paths:

```bash
python3 scripts/monte_carlo.py compare \
  --inputs scripts/evaluation/fixtures/gnc_primary_baseline.summary.json \
            scripts/evaluation/fixtures/gnc_primary_rollout.summary.json
```

## Reproduce

```bash
export LIBGL_ALWAYS_SOFTWARE=1
# Baseline arm
python3 scripts/monte_carlo.py run --n 40 --seed-base 9201 --scenario single --timeout-s 90 \
  --label gnc_primary_baseline --cohort gnc_primary_A_baseline \
  --launch-args 'use_gazebo_gui:=false'
# Gate arm — same cohort seeds
python3 scripts/monte_carlo.py run --n 40 --seed-base 9201 --scenario single --timeout-s 90 \
  --label gnc_primary_rollout --cohort gnc_primary_B_rollout \
  --launch-args 'use_gazebo_gui:=false eng_rollout_feasibility_gate:=true'
```

Aggregate by cohort exclusively:

```bash
python3 scripts/monte_carlo.py aggregate --logs-dir runs/logs --pattern '*.log' \
  --meta-cohort gnc_primary_A_baseline --label my_baseline --out-dir runs/mc
python3 scripts/monte_carlo.py aggregate --logs-dir runs/logs --pattern '*.log' \
  --meta-cohort gnc_primary_B_rollout --label my_rollout --out-dir runs/mc
python3 scripts/evaluation/summarize_failure_classes.py runs/mc/my_baseline.csv
```

## Guidance tightening (Steps 1–3, gate remains ON)

All arms below assume a **common spine** — at minimum `use_gazebo_gui:=false eng_rollout_feasibility_gate:=true`. Compared cohorts share **matched** `noise_seed`, `timeout-s`, geometry; only the listed knobs differ.

**Step 1 — terminal blend** (`gnc_guidance_step1_<rev>` cohort example):

```text
guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35
```

**Step 2 — align-speed coherence** (stack Step 2 on Step 1 if desired, or isolate with gate-only spine):

```text
t_go_filter_max_step_s:=0.8 align_speed_use_smooth_hit_range:=true
```

**Step 3 — slew + PN in terminal**:

```text
guidance_u_max_step_rad:=0.45 use_pn_refinement:=true pn_blend_gain:=0.12 pn_blend_terminal_range_m:=1500
```

Tune numerically per scenario; **`pn_blend_terminal_range_m`** scales PN toward zero when `dist` exceeds this value (LOS-rate PN only ramps in when close).

Smoke helper (runs two short batches if `SKIP_GAZEBO` is unset). Numeric defaults above are **starting points only** — re-tune from logs before interpreting pass/fail; the script checks wiring + matched seeds.

[`scripts/evaluation/run_gnc_guidance_stack_smoke.sh`](../../scripts/evaluation/run_gnc_guidance_stack_smoke.sh)
