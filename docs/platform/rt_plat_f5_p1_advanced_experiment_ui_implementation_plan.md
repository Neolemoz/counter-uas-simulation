# RT-F5 — Advanced Experiment UI (PLAT-RT-F5 P1)

**Phase:** PLAT-RT-F5 P1 — advanced experiment UI  
**Prerequisite:** PLAT-RT-F5 P0 frozen — [rt_plat_f5_freeze_audit.md](../evaluation/rt_plat_f5_freeze_audit.md)  
**Authority:** [rt_experiment_advanced_ui_v1.md](../evaluation/rt_experiment_advanced_ui_v1.md), [rt_experiment_metrics_v1.md](../evaluation/rt_experiment_metrics_v1.md)

## Goal

Wire read-only F5 cognition surfaces on frozen P0 compile/derive modules — matrix review, extended compare, filters, handoff eligibility, and workbench spec import — without bridge, SA viewer, or parser changes.

## Delivered (P1)

| Item | Location |
|------|----------|
| `ExperimentFilterBar` | `platform/rt-sandbox-ui/src/experiment/ExperimentFilterBar.tsx` |
| `ExperimentMatrixPanel` | `platform/rt-sandbox-ui/src/experiment/ExperimentMatrixPanel.tsx` |
| `ExperimentExtendedComparePanel` | `platform/rt-sandbox-ui/src/experiment/ExperimentExtendedComparePanel.tsx` |
| `ExperimentHandoffEligibilityStrip` | `platform/rt-sandbox-ui/src/experiment/ExperimentHandoffEligibilityStrip.tsx` |
| `experimentF5UiHelpers.ts` | filter/matrix/handoff display helpers |
| Workbench wiring | `ExperimentWorkbenchPanel.tsx`, `App.tsx`, `RuntimeCognitionHub.tsx` |
| `BANNER_EXPERIMENT_F5` | `src/governance/banners.ts` |
| Vitest | `experimentF5UiHelpers.test.ts`, panel `*.test.tsx` |

## Forbidden (unchanged)

- Bridge protocol / telemetry changes
- `platform/sa-r0-viewer/` changes
- `rt_experiment_metrics.py` (P2)
- `ExperimentRepeatabilityTrendStrip` (P2)
- Browser `capture_session`; SA auto-import; M3 distributed queue
- Parser/topic/schema changes

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_spec_compile.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
```

## Stop line

PLAT-RT-F5 P1 frozen. Do not start **P2** (`rt_experiment_metrics.py`, repeatability trend strip) without separate wave audit.

## Related

- [rt_plat_f5_p1_freeze_audit.md](../evaluation/rt_plat_f5_p1_freeze_audit.md)
- [rt_plat_f5_p1_governance_review_r1.md](../evaluation/rt_plat_f5_p1_governance_review_r1.md)
- [rt_roadmap_plat_rt_f5_v1.md](../evaluation/rt_roadmap_plat_rt_f5_v1.md)
