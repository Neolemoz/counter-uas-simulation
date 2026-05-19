# Evaluation & Monte Carlo utilities

This folder supports the roadmap for **spatial DOE**, **heatmap surrogate validation**, **selection-vs-oracle auditing**, and **optional rosbag** capture.

## Metric definitions

See [`metrics_definitions.yaml`](metrics_definitions.yaml) for field semantics (`success`, `hit`, oracle selection).

For the current freeze registry, layer map, and meta-governance review guidance, see [`docs/evaluation/meta_governance_maturity_review_r1.md`](../../docs/evaluation/meta_governance_maturity_review_r1.md).
For the Phase 2 replay narrative UX scope and freeze boundaries, see [`docs/evaluation/replay_narrative_ux_phase2_plan.md`](../../docs/evaluation/replay_narrative_ux_phase2_plan.md).
For the Phase 2 replay narrative UX planning context, see [`docs/evaluation/replay_narrative_ux_freeze_audit.md`](../../docs/evaluation/replay_narrative_ux_freeze_audit.md) (superseded for implementation sign-off).
For the Replay Narrative Tooling R1 post-implementation freeze audit (`56c7185`), see [`docs/evaluation/replay_narrative_tooling_r1_freeze_audit.md`](../../docs/evaluation/replay_narrative_tooling_r1_freeze_audit.md).
For the Phase 3 replay narrative validation scope, reviewer matrix/rubric, and freeze audit, see [`docs/evaluation/replay_narrative_validation_phase3_plan.md`](../../docs/evaluation/replay_narrative_validation_phase3_plan.md), [`docs/evaluation/replay_narrative_validation_phase3_review.md`](../../docs/evaluation/replay_narrative_validation_phase3_review.md), and [`docs/evaluation/replay_narrative_validation_phase3_freeze_audit.md`](../../docs/evaluation/replay_narrative_validation_phase3_freeze_audit.md).

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
| [`run_realism_sweep.py`](run_realism_sweep.py) | Small matched-seed runtime-realism sweep over additive delay/stale/dropout/timing overlays |
| [`classify_realism_failure.py`](classify_realism_failure.py) | Additive realism-stressor taxonomy (R0–R5) layered on top of existing F1–F5 buckets |
| [`run_ambiguity_sweep.py`](run_ambiguity_sweep.py) | Small matched-seed sensing-ambiguity sweep over additive ghost/fragmentation overlays |
| [`classify_ambiguity_failure.py`](classify_ambiguity_failure.py) | Additive ambiguity taxonomy (A0–A5) layered on top of existing F1–F5 and R0–R5 buckets |
| [`classify_selection_oracle_divergence.py`](classify_selection_oracle_divergence.py) | Additive selection/oracle divergence taxonomy (D0–D5), evidence-only and non-authoritative |
| [`replay_observability.py`](replay_observability.py) | Additive replay evidence bundles, divergence traces, lifecycle timelines, replay narratives, matched-seed reports, topology/timing indexes, governance linting, and static reviewer reports |
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
- Runtime-realism overlays remain additive and default-off. Use `run_realism_sweep.py` or explicit launch args with matched seeds/cohorts to keep comparisons interpretable.
- Sensing-ambiguity overlays remain additive and default-off. Use `run_ambiguity_sweep.py` or explicit launch args with matched seeds/cohorts to keep ghost/fragmentation comparisons interpretable.
- Wave 4 bringup observability is additive and default-off. Observer-aware evaluation rows may include passive continuity, persistence, and churn evidence, but historical logs without observer markers still parse with zero-valued additive fields.
- Wave 7 selection/oracle divergence fields are additive and evidence-only: mismatch block indices/counts localize replay-side oracle disagreement without changing authority or parser contracts.
- Wave 7 is now frozen stable: selection/oracle divergence is replay-classifiable, late-stage, and geometry-sensitive, while fragmented timing remains locally useful but not robustly transferable.
- Wave 4 is now frozen stable: the passive bringup observer and selection-visibility proxy are evidence-only, additive, and non-authoritative.
- Wave 5 is now frozen stable: confirmed-track reachability and recovery exist in the bringup topology, but the timing envelope is narrow and phase-sensitive.
- Matched seeds preserve comparability, not bitwise identity: Gazebo / ROS scheduling can still introduce bounded rerun variance in miss distance or intercept time even when the qualitative outcome and overlay counts remain aligned.

### Wave 5 timing-envelope freeze

Wave 5 established a bounded recovery envelope under the real bringup topology.

Validated findings:

- recovery exists once confirmed-track reachability is present
- recovery is cadence-sensitive
- lifecycle stability is phase-sensitive
- aggressive silence does not monotonically improve recovery
- `cycle=7 gap=4 phase=1` is an instability pocket in the tested region
- `cycle=7 gap=4 phase=3` is the strongest bounded fragmented operating point in the tested region

Safe bounded experimental region:

- keep the validated reachability geometry:
  - `target_start_x_m:=-1500.0`
  - `target_start_y_m:=0.0`
  - `target_start_z_m:=300.0`
- use matched seeds when comparing cadence / phase variants
- treat the observer-enabled reachability baseline as the safest reference arm
- if fragmentation timing is required, stay near the `cycle=7 / gap=4` family and avoid `phase=1`

Phase-sensitive caution:

- identical cadence shape with different `fragmentation_stagger_phase_ticks` can materially change success, churn, and deletion
- keep phase comparisons explicit in profile ids / cohort names
- do not infer monotonic behavior from stronger silence windows alone

### Wave 3 lifecycle-activation freeze

Wave 3 threshold-sensitive lifecycle activation is frozen with a **stable** validation verdict.

Implemented Wave 3 scope:

- bringup-topology sweep support
- cadence-aligned silence bursts
- silence/resume oscillation refinement
- Wave 3 threshold-sensitive profile matrix in `fixtures/ambiguity_sweep_profiles_wave3.csv`
- matched-seed lifecycle activation sweeps
- regression and governance validation

Parser and replay boundaries remain unchanged:

- parser-visible summaries from `parse_run_to_result` remain stable
- added silence/timing annotations are explanatory only
- added lifecycle metrics are additive derivations only
- downstream tools are not required to consume the new annotations

Current empirical limit:

- bringup topology was exercised successfully
- fragmentation pressure increased
- lifecycle counters still remained dormant
- tactical / selection visibility may itself be under-driven in the current bringup evaluation topology
- sustained threshold crossing remains unresolved

Key architectural insight:

- propagation refinement alone was insufficient
- lifecycle activation may now be limited by evaluation/topology visibility itself
- additional realism breadth is not yet justified
- current priority remains topology-aware lifecycle observability refinement through existing paths

Next narrow realism frontier:

- topology-aware lifecycle observability refinement through the existing `bringup.launch.py -> /fused_detections -> tracking_node -> /tracks/state` path
- keep it default-off, replay-safe, parser-safe, and additive-only

Warning:

- do not interpret dormant lifecycle counters or `nan` tactical summary fields as proof of tracker robustness

### Wave 2 lifecycle-propagation freeze

Wave 2 lifecycle-propagation refinement is frozen with a **stable** validation verdict.

Implemented Wave 2 scope:

- sensor-path propagation toggle
- phase-aligned fragmentation refinement
- near-threshold ghost persistence
- Wave 2 lifecycle-pressure profile matrix in `fixtures/ambiguity_sweep_profiles_wave2.csv`
- matched-seed lifecycle activation sweeps
- regression and governance validation

Parser and replay boundaries remain unchanged:

- parser-visible summaries from `parse_run_to_result` remain stable
- added propagation/timing annotations are explanatory only
- added lifecycle metrics are additive derivations only
- downstream tools are not required to consume the new annotations

Current empirical limit:

- propagation refinement succeeded partially
- downstream tactical/runtime metrics moved
- lifecycle counters still remained dormant
- current blocker is sustained threshold crossing
- ambiguity now reaches sensing/fusion/tracking behavior more strongly than Wave 1

Key architectural insight:

- propagation quality mattered more than realism breadth
- additional realism families are not yet justified
- current priority remains lifecycle-threshold activation through existing paths

Next roadmap frontier:

- additive threshold-sensitive lifecycle activation refinement through the existing `/fused_detections -> tracking_node -> /tracks/state` flow
- keep it default-off, seed-controlled, parser-safe, and free of tracker/fusion redesign

Warning:

- do not interpret dormant lifecycle counters as proof of tracker robustness

### Wave 1 ambiguity freeze

Wave 1 tracks-state-centered ambiguity realism is frozen with a **stable** validation verdict.

Implemented Wave 1 scope:

- near-threshold ghost placement
- staggered fragmentation windows
- evaluation-only lifecycle thrash derivation in `realism_metrics.py`
- Wave 1 ambiguity profile matrix in `fixtures/ambiguity_sweep_profiles.csv`
- replay-safe additive ambiguity annotations

Parser and replay boundaries remain unchanged:

- parser-visible summaries from `parse_run_to_result` remain stable
- added ambiguity annotations are explanatory only
- added lifecycle metrics are additive derivations only
- downstream tools are not required to consume the new annotations

Current empirical limit:

- Wave 1 increases ambiguity event pressure and runtime degradation
- Wave 1 does not yet produce meaningful activation of tracker lifecycle counters
- do not interpret dormant lifecycle counters as proof of tracker robustness

Next roadmap frontier:

- additive lifecycle-propagation refinement through the existing `/fused_detections -> tracking_node -> /tracks/state` flow
- keep it default-off, seed-controlled, parser-safe, and free of tracker/fusion redesign

### Tactical observability boundary

Tactical observability logs (`[TACTICAL_*]`) are **additive observability evidence only**. Stable parser-visible contracts remain the existing parser surfaces used by `parse_run_to_result`, `selection_audit.py`, and `classify_run.py`.

For tooling, treat topic semantics such as `/interceptor/selected_id` and per-interceptor `assigned_target` as the authoritative tactical truth. Treat `[TACTICAL_*]` logs as descriptive or explanatory context only. Any future parser dependence on `[TACTICAL_*]` requires separate governance review.

### Reporting standards

- Report Monte Carlo outputs as descriptive simulation statistics unless a script explicitly computes uncertainty intervals. Current summaries include run count, success count/rate, miss-distance quantiles, empirical CDF samples, and intercept-time quantiles; they are not confidence intervals.
- Use `compare` tables as side-by-side descriptive comparisons. Do not describe one arm as statistically superior unless matched seed/geometry/cohort/launch assumptions are documented and a separate significance or uncertainty analysis is performed.
- Use heatmap `GOOD` / `PARTIAL` / `POOR` only for offline-vs-Gazebo surrogate agreement under aligned assumptions. Do not translate those labels into operational readiness, field P(kill), or doctrine language.
- When presenting parser-derived metrics, keep the metric source visible: `miss_distance_m` may come from `[min_miss]` or the minimum parsed distance series, and `intercept_time_s` may come from the last `t_go` or `t_hit` sample.
- Replay observability reports from `replay_observability.py` are derived evaluation artifacts only. They preserve raw lineage and distinguish raw runtime evidence, canonical parser-visible summaries, additive evaluation artifacts, and explanatory visualization layers.

### Replay observability reviewer artifacts

`replay_observability.py` packages existing evidence without changing runtime behavior, parser-visible contracts, lifecycle semantics, divergence taxonomy labels, or authority surfaces.

```bash
python3 scripts/evaluation/replay_observability.py single-run-report \
  runs/logs/YOUR_RUN.log \
  --meta runs/logs/YOUR_RUN.meta.json \
  --out-json runs/evaluation/YOUR_RUN.replay_observability.json

python3 scripts/evaluation/replay_observability.py narrative \
  --single-run-json runs/evaluation/YOUR_RUN.replay_observability.json \
  --out-json runs/evaluation/YOUR_RUN.replay_narrative.json

python3 scripts/evaluation/replay_observability.py paired-comparison \
  runs/mc/baseline.csv runs/mc/candidate.csv \
  --out-json runs/evaluation/baseline_vs_candidate.replay_observability.json

python3 scripts/evaluation/replay_observability.py topology-index \
  scripts/evaluation/fixtures/ambiguity_sweep_profiles_wave6_transferability.csv \
  --out-json runs/evaluation/wave6_topology_timing_index.json

python3 scripts/evaluation/replay_observability.py governance-lint \
  runs/evaluation/YOUR_RUN.replay_observability.json

python3 scripts/evaluation/replay_observability.py static-report \
  runs/evaluation/YOUR_RUN.replay_narrative.json \
  --out-markdown runs/evaluation/YOUR_RUN.replay_narrative.md \
  --out-html runs/evaluation/YOUR_RUN.replay_narrative.html

python3 scripts/evaluation/replay_observability.py dashboard \
  --single-run-json runs/evaluation/YOUR_RUN.replay_observability.json \
  --paired-comparison-json runs/evaluation/baseline_vs_candidate.replay_observability.json \
  --topology-index-json runs/evaluation/wave6_topology_timing_index.json \
  --governance-lint-json runs/evaluation/YOUR_RUN.governance_lint.json \
  --out-html runs/evaluation/replay_reviewer_dashboard.html \
  --out-markdown runs/evaluation/replay_reviewer_dashboard.md
```

These reports are review conveniences: they do not create a unified authoritative replay state and must not be used as operational readiness, hardware readiness, tactical authority, or lifecycle robustness claims.

Replay narrative reports (`artifact_type: replay_narrative_report`, `narrative_schema_version: replay_narrative_v1`) group existing lifecycle, selection, divergence, ambiguity, outcome, and provenance-warning evidence into deterministic static sequence summaries. Narrative event order is reviewer-facing sequence context only; it is not causal proof, a parser contract, tactical authority, lifecycle truth, governance approval, or readiness evidence.

Reviewer interpretation guidance:

- Keep raw runtime evidence, canonical parser-visible summaries, derived evaluation artifacts, and explanatory visualization layers separate when reading dashboard sections.
- Treat provenance fields (`log_path`, `meta_path`, seed source, cohort, git state, launch args) as lineage for review, not certification of validity or comparability.
- Treat fragmented-gap adjacency and divergence timing as non-causal localization evidence unless a separate governed analysis establishes causality.
- Treat matched-seed buckets as descriptive comparability aids, not statistical superiority, ranking, or general robustness claims.
- Treat topology/profile labels as lineage-linked shorthand, not runtime topology semantics or certified operating regions.
- Treat lifecycle/churn counters as explanatory overlays over raw log lines, not tracker lifecycle truth or robustness proof.

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
