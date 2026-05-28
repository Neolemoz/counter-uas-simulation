# RT-F7 P0 — Advisory Queue + Aggregation (PLAT-RT-F7 P0)

**Phase:** PLAT-RT-F7 P0 — queue priority, blocker groups, batch summary  
**Prerequisite:** PLAN-RT-F7 frozen — [rt_f7_freeze_audit.md](../evaluation/rt_f7_freeze_audit.md)  
**Authority:** [rt_advisory_maintainer_workflow_v1.md](../evaluation/rt_advisory_maintainer_workflow_v1.md), [rt_advisory_aggregation_v1.md](../evaluation/rt_advisory_aggregation_v1.md)

## Goal

Implement read-only F7 advisory queue and aggregation atop frozen F6 derive and P2 batch CLIs — without bridge protocol changes, auto-import, or SA viewer changes.

## Delivered (P0)

| Item | Location |
|------|----------|
| Queue / cohort / groups (Python) | `platform/rt-sandbox-bridge/rt_sandbox/advisory_queue.py` |
| Batch summary builder | `batch_advisory.py` — `build_advisory_batch_summary_document` |
| Lineage warnings on derive | `advisory_derive.py` — `lineage_warnings` |
| CLI extensions | `scripts/rt/rt_handoff_batch_advisory.py` — `--sort`, `--group-by`, `--manifest-ref`, `--schema` |
| TS mirror | `advisoryQueue.ts`, `advisoryAggregate.ts` |
| UI strips / chips | `AdvisoryQueueBandChip`, `AdvisoryBlockerGroupChips`, `ReadinessCohortChip`, `AdvisoryBatchMirrorStrip` |
| Panel wiring | `CaptureHandoffWorkflowPanel.tsx` |
| Tests | `test_advisory_queue.py`, extended batch tests, `advisoryQueue.test.ts` |
| Fixtures | `fixtures/rt_handoff/f7_advisory_examples/` |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- SA viewer changes  
- Auto-import / browser commit  
- Full `AdvisoryTriageQueuePanel` — P1  
- P2 stand-up export hardening  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_advisory_queue.py src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
cd platform/rt-sandbox-ui && npm run test && npm run build
```

## Stop line

PLAT-RT-F7 P0 frozen. Do not start **P1 triage queue UI** without `rt_plat_f7_p1_*` plan + reviews + freeze.

## Related

- [rt_roadmap_plat_rt_f7_v1.md](../evaluation/rt_roadmap_plat_rt_f7_v1.md)
- [rt_plat_f7_p0_freeze_audit.md](../evaluation/rt_plat_f7_p0_freeze_audit.md)
