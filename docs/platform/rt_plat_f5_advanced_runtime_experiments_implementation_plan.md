# RT-F5 — Advanced Runtime Experiments (PLAT-RT-F5 P0)

**Phase:** PLAT-RT-F5 P0 — spec compile + metrics derive  
**Prerequisite:** PLAN-RT-F5 frozen — [rt_f5_freeze_audit.md](../evaluation/rt_f5_freeze_audit.md)  
**Authority:** [rt_experiment_model_v1.md](../evaluation/rt_experiment_model_v1.md), [rt_experiment_metrics_v1.md](../evaluation/rt_experiment_metrics_v1.md)

## Goal

Implement P0 foundations: deterministic `rt_experiment_spec_v1` → `rt_experiment_batch_v1` compile and `rt_experiment_metrics_report_v1` derive atop frozen F1 analytics — without bridge, SA viewer, or experiment UI panels.

## Delivered (P0)

| Item | Location |
|------|----------|
| Spec compile (TS) | `platform/rt-sandbox-ui/src/experiment/experimentSpecCompile.ts` |
| Metrics derive (TS) | `platform/rt-sandbox-ui/src/experiment/metricsDerive.ts` |
| Schemas + guards | `experimentSchema.ts`, `templateIds.ts`, `templateGuards.ts` |
| Spec compile CLI | `scripts/rt/rt_experiment_spec_compile.py` |
| Compile goldens | `fixtures/rt_experiments/f5_compile_goldens/` |
| Metrics fixture | `fixtures/rt_experiments/f5_metrics_golden/manifest.json` |
| Batch F5 passthrough | `scripts/rt/rt_experiment_batch.py` (optional manifest fields) |
| Vitest + pytest | `experimentSpecCompile.test.ts`, `metricsDerive.test.ts`, `test_rt_experiment_spec_compile.py` |

## Forbidden (unchanged)

- Bridge protocol / telemetry changes
- `platform/sa-r0-viewer/` changes
- P1 UI panels (matrix, extended compare, handoff strip)
- `rt_experiment_metrics.py` (P2)
- Browser `capture_session`; SA auto-import; M3 distributed queue

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_spec_compile.py \
  src/counter_uas/test/test_rt_experiment_analytics.py \
  src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
```

## Stop line

PLAT-RT-F5 P0 frozen. Do not start **P1 UI** without separate scope confirmation. P2 `rt_experiment_metrics.py` remains advisory.

## Related

- [rt_roadmap_plat_rt_f5_v1.md](../evaluation/rt_roadmap_plat_rt_f5_v1.md)
- [rt_plat_f5_freeze_audit.md](../evaluation/rt_plat_f5_freeze_audit.md)
- [rt_plat_f5_governance_review_r1.md](../evaluation/rt_plat_f5_governance_review_r1.md)
