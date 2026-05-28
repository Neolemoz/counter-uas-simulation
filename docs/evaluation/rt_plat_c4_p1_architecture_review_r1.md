# RT-C4 P1 — Architecture Review R1 (PLAT-RT-C4 P1)

**Phase:** PLAT-RT-C4 P1 — experiment section cleanup  
**Plan:** [rt_plat_c4_p1_experiment_section_cleanup_plan.md](../platform/rt_plat_c4_p1_experiment_section_cleanup_plan.md)  
**Baseline:** [rt_plat_c4_p0_architecture_review_r1.md](rt_plat_c4_p0_architecture_review_r1.md)  
**Status:** pass

## Verdict

P1 further reduces `ExperimentWorkbenchPanel` concentration via presentational section extractions. No layer boundary regression.

## Concentration relief

| Surface | After P0 | After P1 |
|---------|----------|----------|
| `ExperimentWorkbenchPanel.tsx` | ~780 LOC | ~671 LOC |
| `ExperimentCompareSection.tsx` | — | A/B selectors + `ExperimentComparePanel` |
| `ExperimentF5MetricsSection.tsx` | — | F5 `PanelShell` + metrics/fidelity/matrix/extended compare/advisory strips |

## Preserved

- Compare normalization and side resolution remain in parent.
- Advisory rollup effect and `deriveAdvisoryForRow` inputs remain in parent.
- v2 hook and shell unchanged.

## Stop line

No P2 `App.tsx` decomposition in this wave.
