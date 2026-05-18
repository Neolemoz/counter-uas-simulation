# Layer C Statistical Validation

Layer C makes guidance evaluation statistically defensible, reproducible, and resistant to misleading conclusions. It validates the kinematic Counter-UAS simulation **within assumptions**; it does not make aerodynamic, operational `P_kill`, field-ready, or real-world effectiveness claims.

## Evidence Tiers

- Smoke: `N=1..3`. Wiring checks only. Use these to verify launch, parser, and artifact paths.
- Pilot: `N=10..39`. Useful for variance discovery and gross regressions. Do not promote tuning closure from pilot results.
- Validation: `N>=40` per primary arm, trajectory family, or heatmap validation cell when using the result as research-grade validation guidance.

Point estimates alone are insufficient. Reports must include confidence intervals, paired-seed comparison where applicable, and raw replay artifacts.

## Cohort Methodology

Comparable cohorts must hold constant every non-treatment variable: scenario, trajectory family, timeout, seed list, geometry set, launch envelope, parser version, and metric definitions. Baseline/candidate comparisons should use matched seeds and identical geometry IDs.

Each validation study should archive:

- `campaign_manifest.json`
- `run_index.csv`
- raw `*.log` files and paired `.meta.json`
- per-arm MC `*.csv` and `*.json`
- interval-aware aggregate JSON
- paired comparison CSV/JSON for A/B studies
- failure histogram JSON
- MC-vs-Gazebo agreement CSV/JSON when heatmap claims are made
- `report.md` with cautious conclusion language

## Statistical Outputs

`scripts/monte_carlo.py` now emits Wilson 95% confidence intervals for success rate and deterministic bootstrap 95% intervals for miss-distance/intercept-time P50/P95. It also preserves seed, geometry, cohort, metadata path, git state, launch args, and notes in per-run CSVs.

For standalone interval reports:

```bash
python3 scripts/evaluation/statistical_validation.py aggregate \
  --input runs/mc/layer_c_primary_A_n40_s9201.csv \
  --summary runs/mc/layer_c_primary_A_n40_s9201.json \
  --out-json runs/evaluation/layer_c_primary_A_stats.json
```

For paired-seed comparison:

```bash
python3 scripts/evaluation/statistical_validation.py paired \
  --baseline runs/mc/layer_c_primary_A_n40_s9201.csv \
  --candidate runs/mc/layer_c_primary_D_n40_s9201.csv \
  --out-json runs/evaluation/layer_c_A_vs_D_paired_stats.json \
  --out-csv runs/evaluation/layer_c_A_vs_D_paired_rows.csv
```

The paired report includes matched seed count, missing seed diagnostics, paired success delta, exact McNemar/binomial evidence for discordant pairs, paired bootstrap deltas for P50/P95 miss distance and P95 intercept time, and failure transitions.

## Failure Taxonomy

The stable F1-F5 labels remain:

- `F1_timeout`
- `F2_geom_not_dyn`
- `F3_track_instability`
- `F4_assignment`
- `F5_unknown`

`F5_unknown` is a quality signal, not a harmless bucket. Promotion should be blocked or downgraded if unknown failures are high enough to mask interpretation.

```bash
python3 scripts/evaluation/summarize_failure_classes.py \
  runs/mc/layer_c_primary_D_n40_s9201.csv \
  --out-json runs/evaluation/layer_c_primary_D_failure_hist.json
```

The JSON includes class-proportion Wilson intervals and evidence fields such as timeout, `[ENG_METRIC]` availability, `delta_t_go` jumps, feasible geometry hints, assignment switch count, and parser warnings.

## MC vs Gazebo Agreement

Treat heatmap probability as a surrogate prediction and Gazebo outcomes as repeated Bernoulli observations. Agreement requires per-cell binomial uncertainty and aggregate calibration metrics.

```bash
python3 scripts/validate_heatmap_vs_gazebo.py \
  --heatmap-csv runs/intercept_heatmap_export/intercept_heatmap_prob_latest.csv \
  --runs-per-cell 40 \
  --seed-base 5001 \
  --timeout-s 90 \
  --out-csv runs/evaluation/layer_c_heatmap_vs_gazebo.csv \
  --out-json runs/evaluation/layer_c_heatmap_vs_gazebo.json
```

The report includes per-cell Wilson confidence intervals for `p_gazebo`, signed and absolute calibration error, Brier score, tier summaries, and replay seeds/log paths.

## Promotion Gates

A closure report can use research-grade validation language only when:

- `N>=40` validation guidance is satisfied for the claimed cohort or the remaining uncertainty is explicitly accepted.
- Matched-seed completeness is checked and missing seeds are justified.
- Success-rate conclusions use Wilson confidence intervals.
- Miss-distance and intercept-time conclusions use bootstrap P50/P95 uncertainty.
- Paired-seed comparison supports the claimed direction when comparing arms.
- Failure-class histograms and transitions do not hide timeout, assignment, or unknown-failure regressions.
- MC-vs-Gazebo agreement is quantified before heatmap/surrogate claims are made.

Use cautious wording: “validated within assumptions,” “confidence interval,” “paired-seed comparison,” and “research-grade validation.” Avoid optimistic language from small-N runs.

## Anti-Patterns

- Do not treat smoke runs as evidence of guidance quality.
- Do not compare unmatched seed bases, mixed cohorts, or different launch envelopes as A/B evidence.
- Do not claim improvement from point estimates without uncertainty.
- Do not average away worst-cell or worst-family behavior.
- Do not tune on the validation cohort and then report it as independent validation.
- Do not claim MC-vs-Gazebo agreement from sparse cells without binomial intervals.
