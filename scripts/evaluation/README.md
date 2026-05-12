# Evaluation & Monte Carlo utilities

This folder supports the roadmap for **spatial DOE**, **heatmap surrogate validation**, **selection-vs-oracle auditing**, and **optional rosbag** capture.

## Metric definitions

See [`metrics_definitions.yaml`](metrics_definitions.yaml) for field semantics (`success`, `hit`, oracle selection).

## Scripts (repo root vs this folder)

| Tool | Purpose |
|------|---------|
| [`scripts/run_capture.py`](../run_capture.py) | One timed Gazebo launch + log artifact |
| [`scripts/summarize_run.py`](../summarize_run.py) | `[HIT]` / layer / threshold row |
| [`scripts/analyze_run.py`](../analyze_run.py) | Long-form metrics (`parse_run_to_result`) |
| [`scripts/monte_carlo.py`](../monte_carlo.py) | Batch noise MC + aggregates |
| [`scripts/run_scenario_matrix.py`](../run_scenario_matrix.py) | Threat-parameter grid → CSV rows |
| [`scripts/validate_heatmap_vs_gazebo.py`](../validate_heatmap_vs_gazebo.py) | Sparse surrogate vs Gazebo |
| `selection_audit.py` | oracle vs selected from logs |
| `evaluation_row.py` | One aggregated JSON row for matrices |
| `classify_run.py` | F1–F5 failure bucket from log + optional `capture_rc` |
| [`summarize_failure_classes.py`](summarize_failure_classes.py) | F1–F5 histogram over a `monte_carlo` per-run CSV (`log_path`) |
| [`pair_mc_seed_outcomes.py`](pair_mc_seed_outcomes.py) | Join two MC CSVs on `noise_seed` from `.meta.json` notes; bucket G1–G4 paired outcomes |
| [`run_scenario_generalization_a_vs_d.sh`](run_scenario_generalization_a_vs_d.sh) | Run matched-seed scenario generalization MC for gate-only A vs frozen D |
| [`summarize_scenario_generalization.py`](summarize_scenario_generalization.py) | Reduce per-cell A/D MC JSONs into per-cell and worst-cell CSV reports |
| [`run_gnc_tuning_playbook.sh`](run_gnc_tuning_playbook.sh) | [Phase 0–4 guidance tuning MC driver](gnc_guidance_tuning_playbook_ops.md) |
| [`fixtures/sample_heatmap_for_validate.csv`](fixtures/sample_heatmap_for_validate.csv) | Tiny heatmap for `--dry-run` / CI |

### Launch knobs (evaluation)

From `ros2 launch gazebo_target_sim gazebo_target.launch.py`:

- `evaluation_enable_intercept_heatmap_prob:=true` — enables probabilistic intercept heatmap MC export (`runs/intercept_heatmap_export` under current working directory by default unless `evaluation_intercept_heatmap_export_dir` is set).
- `interceptor_ic_layout:=default|spread|east_bias|custom:...` — repositions interceptor spawns (`custom` expects nine comma-separated numbers `x0,y0,x1,y1,x2,y2` at ground-z).
- **Headless A/B knobs (merged into `interception_logic_node` parameters):** `eng_metrics_period_s`, `eng_rollout_feasibility_gate`, `eng_rollout_gate_horizon_s`, `guidance_u_max_step_rad`, `guidance_terminal_range_m`, `guidance_terminal_pursuit_blend`, **`t_go_filter_max_step_s`**, **`align_speed_use_smooth_hit_range`**, **`pn_blend_terminal_range_m`**, **`use_pn_refinement`**, **`pn_blend_gain`**. Pass via `scripts/run_capture.py --launch-args '...'` (same declarations exist on `gazebo_target_multi.launch.py`). Campaign thresholds and archived paired-run JSON live in [`docs/evaluation/gnc_headless_campaign_thresholds.md`](../../docs/evaluation/gnc_headless_campaign_thresholds.md).

### Guidance tightening (post roll-out gate)

After `eng_rollout_feasibility_gate:=true`, optional **stacked** tweaks (independent A/B cohorts):

| Step | Intent | Example `launch-args` fragment (km-scale single target) |
|------|--------|------------------------------------------------------------|
| 1 | Terminal predict→pursuit | `guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35` |
| 2 | t_go / range coherence | `t_go_filter_max_step_s:=0.8 align_speed_use_smooth_hit_range:=true` |
| 3 | Unit-vector slew + PN only near target | `guidance_u_max_step_rad:=0.45 use_pn_refinement:=true pn_blend_gain:=0.12 pn_blend_terminal_range_m:=1500` |

Use **one new `--cohort`** per step (e.g. `gnc_step2_coherence_gitXXXX`). Keep matched `--seed-base` / `n` across compared arms. See [`docs/evaluation/gnc_headless_campaign_thresholds.md`](../../docs/evaluation/gnc_headless_campaign_thresholds.md) § Guidance tightening.
Tip: unset `evaluation_enable_intercept_heatmap_prob` (default **false**) for everyday interactive runs — heatmaps are CPU heavy.

### Heatmap P(hit) MC vs Gazebo dynamics

- The live `interception_logic_node` uses `estimate_hit_probability(..., use_kinematic_rollout=True)` with `rollout_max_turn_rate_rad_s` and `rollout_max_accel_m_s2` wired from the same parameters as the executed `cmd_vel` limits (`max_turn_rate_rad_s`, `max_acceleration_m_s2`).
- **Offline** [`scripts/render_intercept_heatmap_prob_offline.py`](../render_intercept_heatmap_prob_offline.py) defaults **`--rollout-max-turn-rate-rad-s 2.5`** and **`--rollout-max-accel-m-s2 30.0`** to match [`gazebo_target.launch.py`](../../src/gazebo_target_sim/launch/gazebo_target.launch.py) `DeclareLaunchArgument` defaults. Override those flags if your launch uses different limits.
- For credibility, keep offline rollout limits aligned with the Gazebo run you are comparing to. The legacy “light” hit model (`estimate_hit_probability_light`) is for fast debug only, not calibrated plant match.
- `validate_heatmap_vs_gazebo.py` is a sparse surrogate-agreement spot check, not an operational P(kill) claim. It samples up to three high (`P>0.8`), three mid (`0.4<=P<=0.8`), and three low (`P<0.4`) cells with deterministic `--seed` shuffling, padding from probability quantiles when a tier is underfilled.
- Validation verdicts are fixed credibility bands for model agreement: `GOOD` requires mean absolute error <= 0.15 and max error <= 0.25; `PARTIAL` allows mean <= 0.30 and max <= 0.45; otherwise the result is `POOR`.
- Replay spot checks pin `target_start_x_m`, `target_start_y_m`, `target_start_z_m`, and headless mode. Keep the heatmap generation assumptions, launch args, and rollout limits aligned before interpreting agreement.

### Monte Carlo aggregate cohorts

`python3 scripts/monte_carlo.py aggregate --meta-cohort <tag>` includes only runs whose paired `.meta.json` has `"cohort": "<tag>"` (set via `run_capture --cohort` or `RUN_COHORT` + `run_scenario_matrix --cohort`). Use `--notes-substring` to filter on free-text notes or log header.

### Statistical validation contracts

- Treat `scripts/run_capture.py` `.meta.json` files as the replay manifest: they preserve `run_id`, `created_utc`, workspace, full launch command, timeout, git commit/dirty state, optional `cohort`, and free-text notes.
- For matched Monte Carlo comparisons, keep `--seed-base`, `--n`, launch arguments, and scenario geometry fixed across arms. `scripts/monte_carlo.py run` records `noise_seed_mc`, the backwards-compatible `seed` alias, and optional `geometry_id` in per-run rows.
- Use `--cohort` plus `aggregate --meta-cohort` to avoid mixed cohorts. Use `--notes-substring` only as a secondary filter because notes are free text.
- `parse_run_to_result` is the stable parser-visible contract for aggregate statistics: `success`, `miss_distance_m`, `intercept_time_s`, `time_margin_s`, `layer_at_hit`, and `notes`.
- Heatmap-vs-Gazebo validation samples cells deterministically from the heatmap CSV using `--seed`; replay launch args pin `target_start_x_m`, `target_start_y_m`, `target_start_z_m`, and default to headless `use_gazebo_gui:=false`.

### Reporting standards

- Report Monte Carlo outputs as descriptive simulation statistics unless a script explicitly computes uncertainty intervals. Current summaries include run count, success count/rate, miss-distance quantiles, empirical CDF samples, and intercept-time quantiles; they are not confidence intervals.
- Use `compare` tables as side-by-side descriptive comparisons. Do not describe one arm as statistically superior unless matched seed/geometry/cohort/launch assumptions are documented and a separate significance or uncertainty analysis is performed.
- Use heatmap `GOOD` / `PARTIAL` / `POOR` only for offline-vs-Gazebo surrogate agreement under aligned assumptions. Do not translate those labels into operational readiness, field P(kill), or doctrine language.
- When presenting parser-derived metrics, keep the metric source visible: `miss_distance_m` may come from `[min_miss]` or the minimum parsed distance series, and `intercept_time_s` may come from the last `t_go` or `t_hit` sample.

### Engagement metrics (`[ENG_METRIC]`)

When `eng_metrics_period_s` > 0 on `interception_logic_node`, the node prints occasional single-line snapshots (`range_m`, `v_closing`, `t_go_raw` / `t_go_filt`, `delta_t_go_raw`, `feasible_geom`, `rollout_gate_ok`, commanded speed). Default **0** leaves behavior unchanged.

### Rosbag (optional operator flow)

[`record_rosbag.sh`](record_rosbag.sh) records a small default topic bundle while you run launches in another terminal.

## One-command tiered runs

From repo root, [`scripts/ci_eval.sh`](../ci_eval.sh) runs the planned stack: **tier0** (pytest + heatmap dry-run), **tier1** (single `run_capture`), **tier2** / **tier2-smoke** (matrix CSV), **tier3-aggregate** / **tier3-run** (Monte Carlo). See `scripts/ci_eval.sh --help`.

## Typical flows

Single-row evaluation JSON:

```bash
python3 scripts/evaluation/evaluation_row.py runs/logs/YOUR_RUN.log \\
  --meta runs/logs/YOUR_RUN.meta.json
```

Threat CSV matrix (columns: `scenario_id,target_start_x_m,...`):

```bash
python3 scripts/run_scenario_matrix.py \\
  --matrix-csv scripts/evaluation/fixtures/threat_spawn_matrix_example.csv \\
  --out-csv runs/evaluation/matrix_out.csv \\
  --scenario single --timeout-s 90
```

Dry-run sparse validation picker (no Gazebo):

```bash
python3 scripts/validate_heatmap_vs_gazebo.py \\
  --heatmap-csv scripts/evaluation/fixtures/sample_heatmap_for_validate.csv \\
  --dry-run --seed 1
```
