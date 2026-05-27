# RT-F5 — Freeze Audit (PLAN-RT-F5)

**Phase:** PLAN-RT-F5 — advanced runtime experiments  
**Status:** frozen (docs only)

**Plan:** [rt_f5_advanced_runtime_experiments_plan.md](../platform/rt_f5_advanced_runtime_experiments_plan.md), [rt_f5_experiment_matrix_plan.md](../platform/rt_f5_experiment_matrix_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Experiment model contract | [rt_experiment_model_v1.md](rt_experiment_model_v1.md) |
| 2 | Experiment metrics contract | [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) |
| 3 | Experiment workflow contract | [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md) |
| 4 | Advanced UI contract | [rt_experiment_advanced_ui_v1.md](rt_experiment_advanced_ui_v1.md) |
| 5 | Reference fixtures | [fixtures/rt_experiments/f5_spec_examples/](../../fixtures/rt_experiments/f5_spec_examples/) |
| 6 | Architecture review | [rt_f5_architecture_review_r1.md](rt_f5_architecture_review_r1.md) |
| 7 | Governance review | [rt_f5_governance_review_r1.md](rt_f5_governance_review_r1.md) |
| 8 | Experiment review | [rt_f5_experiment_review_r1.md](rt_f5_experiment_review_r1.md) |
| 9 | PLAT roadmap | [rt_roadmap_plat_rt_f5_v1.md](rt_roadmap_plat_rt_f5_v1.md) |
| 10 | Next frontiers update | [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md) |
| 11 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, or `src/counter_uas/` for this wave.

---

## Experiment architecture summary

**Classes:** `terrain_comparison`, `sensor_range_comparison`, `tactical_mode_comparison`, `repeatability_sweep`, `parameter_matrix`.

**Pipeline:** `rt_experiment_spec_v1` → compile → `rt_experiment_batch_v1` → `rt_experiment_batch.py` → `rt_experiment_manifest_v1` → F1 analytics report → F5 metrics report → optional F3 annex review → advisory `handoff_eligibility` (SA1 manual import only).

**Metrics layering:** F1 `rt_experiment_analytics_report_v1` unchanged; F5 `rt_experiment_metrics_report_v1` adds terrain/visibility/tactical/repeat/matrix rollups and extended compare badges.

**UI (planned PLAT):** Extended compare (≤4 runs), matrix panel, filter bar, repeatability trend strip, handoff eligibility strip — all read-only, `BANNER_EXPERIMENT_F5`.

---

## Boundary guarantees

- Spec, metrics, and eligibility are **not** operational authority  
- No parser/topic/bridge changes in PLAN wave  
- Browser `capture_session` still forbidden (X1)  
- No SA viewer or auto-import scope  
- PLAN-RT-F5 ≠ registry RT-1..7 realism waves  
- Gazebo sensor-truth coupling deferred to **Runtime fidelity coupling** advisory (next frontiers)

---

## Recommended PLAT-RT-F5 scope (advisory)

See [rt_roadmap_plat_rt_f5_v1.md](rt_roadmap_plat_rt_f5_v1.md):

- **P0:** `experimentSpecCompile.ts`, `metricsDerive.ts`, `f5_spec_examples` tests, `rt_experiment_spec_compile.py`  
- **P1:** `ExperimentExtendedComparePanel`, `ExperimentMatrixPanel`, `ExperimentFilterBar`, `ExperimentHandoffEligibilityStrip`  
- **P2:** `rt_experiment_metrics.py`, `ExperimentRepeatabilityTrendStrip`  

**Not authorized** by this freeze.

---

## Regression evidence

Docs-only wave — cite existing platform tests (no new code in PLAN):

```text
lint_rt_runtime_subcommands
pytest platform/rt-sandbox-bridge/tests/
npm run test --prefix platform/rt-sandbox-ui  # tier0-rt-ui subset as in F1/F4 audits
```

---

## Stop line

PLAN-RT-F5 frozen. Do not start PLAT-RT-F5 without [rt_plat_f5_* implementation plan](../platform/) + governance review + freeze audit.
