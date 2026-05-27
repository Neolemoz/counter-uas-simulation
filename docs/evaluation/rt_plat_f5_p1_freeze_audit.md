# RT-F5 — Freeze Audit (PLAT-RT-F5 P1)

**Phase:** PLAT-RT-F5 P1 — advanced experiment UI  
**Status:** frozen (P1 scope)

**Plan:** [rt_plat_f5_p1_advanced_experiment_ui_implementation_plan.md](../platform/rt_plat_f5_p1_advanced_experiment_ui_implementation_plan.md)

**P0 audit:** [rt_plat_f5_freeze_audit.md](rt_plat_f5_freeze_audit.md)

---

## Scope delivered

| # | Item | Done |
|---|------|------|
| 1 | `ExperimentMatrixPanel` | Yes |
| 2 | `ExperimentExtendedComparePanel` | Yes |
| 3 | `ExperimentFilterBar` | Yes |
| 4 | `ExperimentHandoffEligibilityStrip` | Yes |
| 5 | Workbench spec import + compile preview + CLI hint | Yes |
| 6 | `BANNER_EXPERIMENT_F5` + cognition hub line | Yes |
| 7 | Vitest + governance docs | Yes |

**Not delivered (P2):** `rt_experiment_metrics.py`, `ExperimentRepeatabilityTrendStrip`.

---

## UI surfaces summary

- **Matrix review** — `parameter_matrix`, `terrain_comparison`, `sensor_range_comparison`; coords grid or run-chip fallback; count-only matrix rollup.
- **Extended compare** — up to 4 manifest runs; F5 compare badges; tactical/terrain/visibility rollup text.
- **Filters** — experiment class, tactical mode, terrain preset, visibility context (client-side).
- **Handoff eligibility** — advisory `eligible` / `review_needed` / `blocked` display; gate table; no actions.

---

## Boundary guarantees

- No bridge / parser / SA viewer changes
- Metrics and eligibility remain non-authoritative
- F5 UI consumes P0 `deriveExperimentMetrics` only

---

## Regression evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_spec_compile.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
```

---

## Stop line

PLAT-RT-F5 P1 frozen. **P2** frozen — see [rt_plat_f5_p2_freeze_audit.md](rt_plat_f5_p2_freeze_audit.md). F5 roadmap complete.
