# Layer C: Statistical Validation

## 1. Layer Purpose

Layer C defines how this repository makes statistical claims about the kinematic simulation. It owns Monte Carlo methodology, matched-seed comparisons, parser/schema stability, heatmap validation, replay artifacts, confidence intervals, and reporting discipline.

Layer C validates model-conditioned simulation behavior. It does not support operational probability-of-kill, field readiness, real-world combat claims, ML benchmarking, classified threat modeling, or military-grade verification claims.

## 2. Current Evaluation State

- `run_capture.py` launches a scenario and captures a log plus `.meta.json` sidecar.
- `analyze_run.py` parses logs into success, miss distance, intercept time, time margin, layer, and notes.
- `monte_carlo.py` drives live trials, aggregates results, compares cohorts, and writes CSV/JSON outputs.
- `scripts/evaluation/statistical_validation.py` and `stats_helpers.py` provide Wilson intervals, bootstrap intervals, paired deltas, and manifest-style checks.
- `render_intercept_heatmap_prob_offline.py` exports offline probability heatmaps.
- `validate_heatmap_vs_gazebo.py` samples heatmap cells and compares live Gazebo outcomes with replayable seeds/logs.
- Failure taxonomy scripts classify timeouts, geometry/dynamics mismatch, tracking instability, assignment issues, and unknown outcomes.
- Current credibility limits include mixed cohort risk, parser drift, incomplete schema versioning, heatmap/live mismatch, and stale launch/default metadata.

## 3. Evaluation Ownership Boundaries

- Monte Carlo: `scripts/monte_carlo.py` and scenario driver scripts.
- Parser/schema: `scripts/run_capture.py`, `scripts/analyze_run.py`, `scripts/monte_carlo.py`, and `scripts/evaluation/metrics_definitions.yaml`.
- Statistical aggregation: `scripts/evaluation/stats_helpers.py` and `scripts/evaluation/statistical_validation.py`.
- Failure taxonomy: `scripts/evaluation/classify_run.py`, `summarize_failure_classes.py`, and related tests.
- Heatmap validation: `render_intercept_heatmap_prob_offline.py`, `validate_heatmap_vs_gazebo.py`, and `test_heatmap_agreement.py`.
- Replay/capture: raw logs, `.meta.json` sidecars, run IDs, cohort tags, launch args, seed fields, git state.
- Report generation: CSV/JSON/statistical reports and future generated summaries.
- Launch/evaluation metadata: launch args, scenario, timeout, cohort, seed, geometry ID, git commit/dirty, notes, and log paths.

## 4. Planned Validation Workstreams

- Matched-seed cohort validation:
  - Expected files/modules: `monte_carlo.py`, `statistical_validation.py`, `pair_mc_seed_outcomes.py`.
  - Risks: unmatched seeds, duplicate seeds, geometry mismatch, mixed launch envelopes.
  - Validation: MC and statistical validation tests, paired fixture tests.
  - Non-goals: guidance tuning or launch-default changes.
- Parser/schema stabilization:
  - Expected files/modules: `run_capture.py`, `analyze_run.py`, `monte_carlo.py`, metrics definitions.
  - Risks: tag drift, missing fields, stale sidecars, whitespace-split launch args.
  - Validation: parser fixtures, MC tests, manifest checks.
  - Non-goals: wholesale log format redesign.
- Failure taxonomy stabilization:
  - Expected files/modules: `classify_run.py`, `summarize_failure_classes.py`, taxonomy tests.
  - Risks: `F5_unknown` hiding regressions, success rows misclassified, missing evidence fields.
  - Validation: synthetic log fixtures and histogram checks.
  - Non-goals: ML root-cause classification.
- Heatmap-vs-live agreement:
  - Expected files/modules: heatmap render, heatmap validation, agreement tests.
  - Risks: offline/live rollout mismatch, sparse-cell overclaiming, stale CSV schema.
  - Validation: dry-run selector tests, agreement JSON/CSV tests, matched Gazebo campaigns.
  - Non-goals: calibrated real-world lethality.
- Confidence interval/reporting discipline:
  - Expected files/modules: `stats_helpers.py`, statistical reports, failure summaries.
  - Risks: point estimates without intervals, pilot-sized results promoted as validation.
  - Validation: Wilson/bootstrap tests and report-shape fixtures.
  - Non-goals: full Bayesian uncertainty quantification.
- Replay/reproducibility:
  - Expected files/modules: logs, `.meta.json`, MC CSV/JSON, future manifests.
  - Risks: dirty git ambiguity, lost launch envelope, unreplayable artifacts.
  - Validation: metadata fixture tests and manifest checks.
  - Non-goals: full container/environment capture in this layer.
- Evaluation metadata/versioning:
  - Expected files/modules: metrics definitions, MC outputs, sidecars, reports.
  - Risks: schema drift without version stamps.
  - Validation: schema/version tests once implemented.
  - Non-goals: breaking existing readers without migration.

## 5. Explicit Non-Goals

- No operational probability-of-kill claims.
- No real-world combat validation.
- No ML benchmarking.
- No classified threat modeling.
- No full uncertainty quantification.
- No military-grade verification claims.
- No launch-default or guidance redesign.

## 6. Validation Philosophy

Credible-enough validation means claims are scoped to the exact code, launch args, seeds, scenarios, and artifacts used. Smoke runs validate wiring. Pilot cohorts explore sensitivity. Larger matched cohorts with intervals support stronger repository-internal claims.

Deterministic code should be tested deterministically. Stochastic claims require seeds, cohorts, intervals, and raw artifacts. Offline/live disagreement should be treated as evidence to investigate, not averaged away. Parser/schema drift should fail tests before it corrupts reports.

## 7. Layer C Contracts

- Parser-visible tags include `[HIT]`, `[min_miss]`, `[METRICS]`, `[ENG_METRIC]`, `[FEASIBILITY]`, and `[LAYER]` where emitted.
- Core result fields include `success`, `miss_distance_m`, `intercept_time_s`, `time_margin_s`, `layer_at_hit`, and `notes`.
- Replay metadata includes `run_id`, `created_utc`, `git_commit`, `git_dirty`, `cohort`, `scenario`, `timeout_s`, `cmd`, `launch_args_raw`, and parsed launch args.
- MC outputs should preserve seed, geometry, cohort, meta path, log path, and launch envelope fields.
- Heatmap artifacts must keep CSV/JSON/SVG schemas compatible with validators and reports.
- Matched-seed comparisons require identical seed sets and controlled non-treatment parameters.

## 8. AI-Agent Guidance

- Read `ARCHITECTURE_MAP.md` and evaluation docs before touching scripts.
- Preserve parser-visible tags unless the task explicitly migrates parsers and tests.
- Do not compare unmatched cohorts as if they were paired experiments.
- Do not present dry-run or smoke output as validation evidence.
- Keep raw logs and sidecars as replay truth.
- Run parser, MC, statistics, failure taxonomy, and heatmap tests for evaluation changes.

## 9. Evaluation Hotspots

- Parser drift from stdout tag changes.
- Launch-default mismatch between live runs and offline assumptions.
- Mixed cohorts with different source selectors or plant settings.
- Stale metadata or missing git/launch context.
- Unmatched seeds in A/B comparisons.
- Point vs `tracks_state` confusion.
- Heatmap/live mismatch from plant, target, delay, or noise assumptions.
- Hidden tuning changes during evaluation campaigns.

## 10. Unresolved Validation Ambiguities

- Acceptance thresholds for heatmap/live agreement are not fully formalized.
- Confidence interval requirements by report type need stronger conventions.
- Replay/versioning gaps remain around schema versions and environment capture.
- Geometry-aware pairing is less mature than seed-aware pairing.
- Failure taxonomy may need more evidence fields before `F5_unknown` becomes rare.

## 11. Recommended First Layer C Implementation Candidate

Start with parser/schema stabilization: document and test the log tags, CSV fields, sidecar metadata, and schema/version expectations before adding new statistical reports or changing Monte Carlo behavior.
