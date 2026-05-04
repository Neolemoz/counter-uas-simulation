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
| [`fixtures/sample_heatmap_for_validate.csv`](fixtures/sample_heatmap_for_validate.csv) | Tiny heatmap for `--dry-run` / CI |

### Launch knobs (evaluation)

From `ros2 launch gazebo_target_sim gazebo_target.launch.py`:

- `evaluation_enable_intercept_heatmap_prob:=true` — enables probabilistic intercept heatmap MC export (`runs/intercept_heatmap_export` under current working directory by default unless `evaluation_intercept_heatmap_export_dir` is set).
- `interceptor_ic_layout:=default|spread|east_bias|custom:...` — repositions interceptor spawns (`custom` expects nine comma-separated numbers `x0,y0,x1,y1,x2,y2` at ground-z).
- **Headless A/B knobs (merged into `interception_logic_node` parameters):** `eng_metrics_period_s`, `eng_rollout_feasibility_gate`, `eng_rollout_gate_horizon_s`, `guidance_u_max_step_rad`, `guidance_terminal_range_m`, `guidance_terminal_pursuit_blend`. Pass via `scripts/run_capture.py --launch-args '...'` (same declarations exist on `gazebo_target_multi.launch.py`). Campaign thresholds and archived paired-run JSON live in [`docs/evaluation/gnc_headless_campaign_thresholds.md`](../../docs/evaluation/gnc_headless_campaign_thresholds.md).

Tip: unset `evaluation_enable_intercept_heatmap_prob` (default **false**) for everyday interactive runs — heatmaps are CPU heavy.

### Heatmap P(hit) MC vs Gazebo dynamics

- The live `interception_logic_node` uses `estimate_hit_probability(..., use_kinematic_rollout=True)` with `rollout_max_turn_rate_rad_s` and `rollout_max_accel_m_s2` wired from the same parameters as the executed `cmd_vel` limits (`max_turn_rate_rad_s`, `max_acceleration_m_s2`).
- **Offline** [`scripts/render_intercept_heatmap_prob_offline.py`](../render_intercept_heatmap_prob_offline.py) defaults **`--rollout-max-turn-rate-rad-s 2.5`** and **`--rollout-max-accel-m-s2 30.0`** to match [`gazebo_target.launch.py`](../../src/gazebo_target_sim/launch/gazebo_target.launch.py) `DeclareLaunchArgument` defaults. Override those flags if your launch uses different limits.
- For credibility, keep offline rollout limits aligned with the Gazebo run you are comparing to. The legacy “light” hit model (`estimate_hit_probability_light`) is for fast debug only, not calibrated plant match.

### Monte Carlo aggregate cohorts

`python3 scripts/monte_carlo.py aggregate --meta-cohort <tag>` includes only runs whose paired `.meta.json` has `"cohort": "<tag>"` (set via `run_capture --cohort` or `RUN_COHORT` + `run_scenario_matrix --cohort`). Use `--notes-substring` to filter on free-text notes or log header.

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
