# RT-X2 P1 — Unified Review Lane (PLAT-RT-X2 P1)

**Phase:** PLAT-RT-X2 P1 — interactive review lane + report dock + compare wiring  
**Prerequisite:** PLAT-RT-X2 P0 frozen — [rt_plat_x2_p0_freeze_audit.md](../evaluation/rt_plat_x2_p0_freeze_audit.md)  
**Authority:** [rt_experiment_unified_review_v1.md](../evaluation/rt_experiment_unified_review_v1.md), [rt_experiment_workbench_v2_v1.md](../evaluation/rt_experiment_workbench_v2_v1.md)

## Goal

Activate unified review lane: step navigation, run selection, report dock import/preview, compare mode wiring to frozen X1/F3/F5/F5b panels — without bridge, derive, or SA viewer changes.

## Delivered (P1 — frozen)

| Item | Location |
|------|----------|
| State extensions | `workbenchV2State.ts`, `experimentUnifiedReview.ts` |
| Orchestration | `reviewLaneOrchestration.ts` |
| Review packet | `reviewPacketSchema.ts`, `reviewPacketPreview.ts` |
| Panels | `ExperimentUnifiedReviewPanel.tsx`, `ExperimentReportDockPanel.tsx`, `ExperimentCompareStagePanel.tsx` |
| Shell refactor | `ExperimentWorkbenchV2Shell.tsx` |
| Parent wiring | `ExperimentWorkbenchPanel.tsx` |
| Tests | `reviewPacket*.test.ts`, `reviewLaneOrchestration.test.ts`, `workbenchV2State.test.ts`, panel tests |
| Contamination review | `rt_plat_x2_p1_handoff_contamination_review_r1.md` |

## Forbidden (unchanged)

- Bridge / SA viewer / derive changes  
- `multi_manifest_diff` table — **P2**  
- `reviewPacketExport.ts` filesystem write — **P2**  
- Browser capture / auto-import  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-X2 P1 frozen. Do not start **P2** without plan + reviews + freeze.
