# RT-X3 P2 — Multi-Manifest Review and Packet Polish (PLAT-RT-X3 P2)

**Phase:** PLAT-RT-X3 P2 — compare readability + packet preview ergonomics  
**Prerequisite:** PLAT-RT-X3 P1 frozen — [rt_plat_x3_p1_freeze_audit.md](../evaluation/rt_plat_x3_p1_freeze_audit.md)  
**Authority:** [rt_experiment_compare_workflow_v3_v1.md](../evaluation/rt_experiment_compare_workflow_v3_v1.md) §5, [rt_experiment_review_workflow_v3_v1.md](../evaluation/rt_experiment_review_workflow_v3_v1.md) §4

## Goal

Complete PLAT-RT-X3 with metadata-only multi-manifest drill-down polish and review-packet preview ergonomics — without bridge/runtime, SA, import/export JSON shape changes, or cross-manifest run pairing.

## Delivered (P2)

| Item | Location |
|------|----------|
| Column order + status rollup | `multiManifestDiffColumns.ts` |
| Metadata drill-down table | `MultiManifestMetadataDrillDown.tsx` |
| Compare chip styling | `CompareStatusChip.tsx` |
| Packet grouped summary | `ReviewPacketGroupedSummary.tsx` |
| Section groups (preview) | `packetSectionGroups.ts` |
| Packet tab hook | `useReportDockPacketTab.ts` |
| Dock + shell wiring | `ExperimentReportDockPanel.tsx`, `ExperimentWorkbenchV2Shell.tsx` |

## Preserved

- `exportReviewPacketJson` / copy / download — **no** `sections[]`
- `buildMultiManifestDiff` derivation unchanged
- No `App.tsx`, bridge, or SA changes

## Forbidden

- Export `sections[]` on download/copy
- Cross-manifest run pairing
- F7 queue / SA import paths

## Validation

```bash
cd platform/rt-sandbox-ui && npm test && npm run build
cd ../.. && scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

## Stop line

**PLAT-RT-X3 complete** (P0–P2). No P3 without new scoped PLAT plan.

## Related

- [rt_plat_x3_p2_freeze_audit.md](../evaluation/rt_plat_x3_p2_freeze_audit.md)
