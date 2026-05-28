# RT-F8 P1 — Advisory Triage UX Integration (PLAT-RT-F8 P1)

**Phase:** PLAT-RT-F8 P1 — integrated triage hub with v2 rollups and F8 controls  
**Prerequisite:** PLAT-RT-F8 P0 frozen — [rt_plat_f8_p0_freeze_audit.md](../evaluation/rt_plat_f8_p0_freeze_audit.md)  
**Authority:** [rt_advisory_maintainer_workflow_v2.md](../evaluation/rt_advisory_maintainer_workflow_v2.md), [rt_advisory_aggregation_v2.md](../evaluation/rt_advisory_aggregation_v2.md)

## Goal

Deepen maintainer triage UX by integrating frozen F8 P0 preset/focus/stand-up/template tooling and v2 rollups into `AdvisoryTriageQueuePanel` — read-only, no bridge or SA viewer changes.

## Delivered (P1)

| Item | Location |
|------|----------|
| v2 triage row enrichment | `enrichRowsForTriage` → `enrichAdvisoryRowV2` |
| Triage hub | `AdvisoryTriageQueuePanel.tsx` — rollup bar, preset, focus, passes, template |
| Stand-up pass selector | `AdvisoryStandupPassSelector.tsx`, `STANDUP_PASSES` in `advisoryAggregationV2.ts` |
| Rollup summary bar | `AdvisoryRollupSummaryBar.tsx` |
| Group modes | `cohort_v2`, `handoff_stage` in `advisoryTriageGrouping.ts` |
| Group open memory | `advisoryTriageGroupMemory.ts` |
| Panel wiring | `CaptureHandoffWorkflowPanel.tsx` — deduped external F8 strips |
| Vitest | triage, grouping, standup, aggregation tests |

## Forbidden (unchanged)

- Bridge HTTP / subcommand registry  
- SA viewer  
- Auto-import / browser commit  
- Preset/pass → CLI spawn  
- P2 corpus-preview / dry-run v2 hardening  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
cd platform/rt-sandbox-ui && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F8 P1 frozen. Do not start **P2** without `rt_plat_f8_p2_*` plan + contamination re-audit + freeze.

## Related

- [rt_roadmap_plat_rt_f8_v1.md](../evaluation/rt_roadmap_plat_rt_f8_v1.md)
- [rt_plat_f8_p1_freeze_audit.md](../evaluation/rt_plat_f8_p1_freeze_audit.md)
