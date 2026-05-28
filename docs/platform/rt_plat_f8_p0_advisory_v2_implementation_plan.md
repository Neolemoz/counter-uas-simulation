# RT-F8 P0 — Advisory Summary v2 + Presets (PLAT-RT-F8 P0)

**Phase:** PLAT-RT-F8 P0 — aggregation v2, filter presets, focus sets, render-only template packs  
**Prerequisite:** PLAN-RT-F8 frozen — [rt_f8_freeze_audit.md](../evaluation/rt_f8_freeze_audit.md)  
**Authority:** [rt_advisory_maintainer_workflow_v2.md](../evaluation/rt_advisory_maintainer_workflow_v2.md), [rt_advisory_aggregation_v2.md](../evaluation/rt_advisory_aggregation_v2.md), [rt_advisory_contamination_gates_v2.md](../evaluation/rt_advisory_contamination_gates_v2.md)

## Goal

Deliver `rt_advisory_batch_summary_v2` with multi-capture/handoff/experiment-handoff rollups, maintainer CLI preset/focus/schema flags, and read-only workstation mirrors — without bridge protocol changes, auto-import, SA viewer changes, or preset→CLI automation.

## Delivered (P0)

| Item | Location |
|------|----------|
| v2 schema, presets, focus, cohort v2 rollups | `platform/rt-sandbox-bridge/rt_sandbox/advisory_queue.py` |
| `build_advisory_batch_summary_v2_document`, `render_template_pack` | `batch_advisory.py` |
| CLI `--schema f8`, `--preset`, `--focus-captures`, `--template-pack`, `--cohort-index-ref` | `scripts/rt/rt_handoff_batch_advisory.py` |
| TS mirror | `advisoryAggregationV2.ts`, `advisoryTypes.ts` |
| UI strips | `AdvisoryPresetSelector`, `AdvisoryFocusChips`, `AdvisoryCohortV2Strip`, `AdvisoryExperimentHandoffStrip`, `AdvisoryStandupPassesStrip`, `AdvisoryTemplatePackPreview`, `ReadinessCohortV2Chip` |
| Panel wiring | `CaptureHandoffWorkflowPanel.tsx` |
| Tests | `test_advisory_queue.py`, `test_rt_handoff_batch_advisory.py`, `advisoryAggregationV2.test.ts`, `AdvisoryPresetSelector.test.tsx` |
| Fixtures | `fixtures/rt_handoff/f8_advisory_examples/` (PLAN) |

## CLI schema naming

| Flag | Schema |
|------|--------|
| `--schema f7` (default report/export summary) | `rt_advisory_batch_summary_v1` |
| `--schema v2` | `rt_advisory_batch_review_v2` (F7 P2 stand-up) |
| `--schema f8` | `rt_advisory_batch_summary_v2` |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- SA viewer changes  
- Auto-import / browser commit / `--commit-all`  
- Preset or template triggering subprocess or corpus write  
- `readiness_score` or operational readiness fields  
- Dedicated triage panel refactor — **P1**  
- Corpus-preview / dry-run-review v2 hardening — **P2**  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_advisory_queue.py \
  src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
cd platform/rt-sandbox-ui && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F8 P0 frozen. Do not start **P1** without `rt_plat_f8_p1_*` plan + reviews + freeze.

## Related

- [rt_roadmap_plat_rt_f8_v1.md](../evaluation/rt_roadmap_plat_rt_f8_v1.md)
- [rt_plat_f8_p0_freeze_audit.md](../evaluation/rt_plat_f8_p0_freeze_audit.md)
