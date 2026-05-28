# RT-X2 P0 — Cohort Index + Workbench v2 Shells (PLAT-RT-X2 P0)

**Phase:** PLAT-RT-X2 P0 — cohort index store + read-only workbench v2 zones  
**Prerequisite:** PLAN-RT-X2 frozen — [rt_x2_freeze_audit.md](../evaluation/rt_x2_freeze_audit.md)  
**Authority:** [rt_experiment_cohort_v1.md](../evaluation/rt_experiment_cohort_v1.md), [rt_experiment_workbench_v2_v1.md](../evaluation/rt_experiment_workbench_v2_v1.md), [rt_experiment_unified_review_v1.md](../evaluation/rt_experiment_unified_review_v1.md)

## Goal

Implement `rt_experiment_cohort_index_v1` localStorage store, import guards, and read-only workbench v2 shells (cohort navigator, review lane, report dock, compare stage) — without bridge changes, derive changes, SA viewer changes, or frozen X1 panel behavior changes.

## Delivered (P0 — frozen)

| Item | Location |
|------|----------|
| Cohort schema | `platform/rt-sandbox-ui/src/experiment/cohortSchema.ts` |
| Import guards | `cohortImportGuards.ts` |
| Cohort store | `cohortIndexStore.ts` |
| Workbench v2 UI state | `workbenchV2State.ts` |
| Unified review constants | `experimentUnifiedReview.ts` |
| V2 shell components | `ExperimentWorkbenchV2Shell.tsx`, `ExperimentCohortNavigator.tsx`, `ExperimentReviewLaneShell.tsx`, `ExperimentReportDockShell.tsx`, `ExperimentCompareStageShell.tsx` |
| Panel wiring | `ExperimentWorkbenchPanel.tsx` (read-only props; frozen panels unchanged) |
| Banner | `BANNER_EXPERIMENT_V2` in `governance/banners.ts` |
| Tests | `cohortSchema.test.ts`, `cohortImportGuards.test.ts`, `cohortIndexStore.test.ts` |
| Fixture | `fixtures/rt_experiments/x2_cohort_index_example.json` |
| Reviews | `rt_plat_x2_p0_*_review_r1.md`, `rt_plat_x2_p0_freeze_audit.md` |

## Forbidden (unchanged)

- `platform/rt-sandbox-bridge/` changes  
- `platform/sa-r0-viewer/` changes  
- Bridge HTTP / subcommand registry changes  
- Browser `capture_session`, `rt_sa_import`, subprocess batch  
- Derive algorithm / report schema changes  
- Merged manifest export; cross-manifest `run_id` join  
- Interactive review stepper / report-dock import — **P1**  
- Compare mode wiring / `multi_manifest_diff` table — **P2**  
- `App.tsx` integration — deferred to P2  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-X2 P0 frozen. Do not start **P1 unified review panel** without `rt_plat_x2_p1_*` plan + reviews + freeze.

## Related

- [rt_roadmap_plat_rt_x2_v1.md](../evaluation/rt_roadmap_plat_rt_x2_v1.md)
- [rt_plat_x2_p0_freeze_audit.md](../evaluation/rt_plat_x2_p0_freeze_audit.md)
