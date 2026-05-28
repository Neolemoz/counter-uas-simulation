# RT-X2 P2 — Multi-Manifest Review + Export Polish (PLAT-RT-X2 P2)

**Phase:** PLAT-RT-X2 P2 — compare workflow v2 completion + review packet file export  
**Prerequisite:** PLAT-RT-X2 P1 frozen — [rt_plat_x2_p1_freeze_audit.md](../evaluation/rt_plat_x2_p1_freeze_audit.md)  
**Authority:** [rt_experiment_compare_workflow_v2_v1.md](../evaluation/rt_experiment_compare_workflow_v2_v1.md), [rt_experiment_unified_review_v1.md](../evaluation/rt_experiment_unified_review_v1.md), [rt_roadmap_plat_rt_x2_v1.md](../evaluation/rt_roadmap_plat_rt_x2_v1.md)

## Goal

Complete PLAT-RT-X2: enable `multi_manifest_diff` metadata table, advisory `rt_experiment_review_packet_v1` file export, and thin workbench v2 hook extraction — **PLAT-RT-X2 complete** after freeze.

## Delivered (P2)

| Item | Location |
|------|----------|
| Multi-manifest diff | `multiManifestDiff.ts`, `MultiManifestDiffTable.tsx`, `ManifestSummaryChips.tsx` |
| Review packet export | `reviewPacketExport.ts` |
| Workbench hook | `useExperimentWorkbenchV2.ts` |
| Wiring | `ExperimentWorkbenchV2Shell.tsx`, `ExperimentCompareStagePanel.tsx`, `ExperimentReportDockPanel.tsx`, `experimentUnifiedReview.ts` |
| Tests | `multiManifestDiff.test.ts`, `MultiManifestDiffTable.test.tsx`, `reviewPacketExport.test.ts` |

## Forbidden (unchanged)

- Bridge / SA viewer / derive / Python CLI changes  
- Browser capture mutation / auto-import / federation writes  
- Cross-manifest `run_id` pairing / merged manifest export  
- Distributed multi-bridge / PLAN-RT-F8 implementation  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-X2 complete after P2 freeze. No P3 without new PLAN wave.
