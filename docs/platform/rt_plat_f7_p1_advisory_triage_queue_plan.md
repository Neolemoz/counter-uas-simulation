# RT-F7 P1 — Advisory Triage Queue UI (PLAT-RT-F7 P1)

**Phase:** PLAT-RT-F7 P1 — read-only triage queue UI  
**Prerequisite:** PLAT-RT-F7 P0 frozen — [rt_plat_f7_p0_freeze_audit.md](../evaluation/rt_plat_f7_p0_freeze_audit.md)  
**Authority:** [rt_advisory_maintainer_workflow_v1.md](../evaluation/rt_advisory_maintainer_workflow_v1.md), [rt_advisory_aggregation_v1.md](../evaluation/rt_advisory_aggregation_v1.md), [rt_advisory_contamination_gates_v1.md](../evaluation/rt_advisory_contamination_gates_v1.md)

## Goal

Build maintainer triage UX on frozen F7 P0 queue/cohort/blocker derive — `AdvisoryTriageQueuePanel`, grouped blocker views, optional experiment rollup from workbench metrics — without bridge protocol, SA viewer, or write-path changes.

## Delivered (P1)

| Item | Location |
|------|----------|
| `advisoryTriageGrouping.ts` | `platform/rt-sandbox-ui/src/handoff/` |
| `AdvisoryTriageQueuePanel.tsx` | `platform/rt-sandbox-ui/src/handoff/` |
| `AdvisoryGroupedBlockerStrip.tsx` | `platform/rt-sandbox-ui/src/handoff/` |
| `captureIdDisplay.ts` | `platform/rt-sandbox-ui/src/handoff/` |
| `enrichRowsForTriage` | `advisoryAggregate.ts` |
| Handoff panel wiring | `CaptureHandoffWorkflowPanel.tsx` |
| Experiment rollup callback | `ExperimentWorkbenchPanel.tsx`, `App.tsx` |
| Vitest | `advisoryTriageGrouping.test.ts`, `AdvisoryTriageQueuePanel.test.tsx` |
| Reviews + freeze | `docs/evaluation/rt_plat_f7_p1_*` |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- `platform/sa-r0-viewer/` changes  
- Approve / import / capture buttons in browser  
- Auto-import; batch `--commit-all`  
- Parser/topic/schema changes  
- Cross-session merged authority queue  
- P2 stand-up export / dry-run hardening  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_advisory_queue.py src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
cd platform/rt-sandbox-ui && npm run test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F7 P1 frozen. Do not start **P2** bulk workflow hardening without P2 contamination re-audit + `rt_plat_f7_p2_*` governance review.

## Related

- [rt_plat_f7_p1_freeze_audit.md](../evaluation/rt_plat_f7_p1_freeze_audit.md)
- [rt_roadmap_plat_rt_f7_v1.md](../evaluation/rt_roadmap_plat_rt_f7_v1.md)
