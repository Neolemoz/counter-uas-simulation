# RT-F1 — Experiment Analytics + Sweep Catalog (PLAT-RT-F1)

**Phase:** PLAT-RT-F1 — derived analytics and sweep catalog UI  
**Prerequisite:** PLAN-RT-F1 frozen; PLAT-RT-X1 frozen  
**Authority:** [rt_experiment_analytics_v1.md](../evaluation/rt_experiment_analytics_v1.md), [rt_experiment_sweep_catalog_v1.md](../evaluation/rt_experiment_sweep_catalog_v1.md), [rt_experiment_analytics_ui_v1.md](../evaluation/rt_experiment_analytics_ui_v1.md)

## Goal

Add deterministic experiment analytics derivation, sweep catalog browser, and maintainer analytics CLI on top of the X1 workbench — without bridge protocol changes, SA viewer changes, or distributed runtime.

## Delivered

| Item | Location |
|------|----------|
| Analytics derive + Zod | `platform/rt-sandbox-ui/src/experiment/analyticsDerive.ts`, `experimentSchema.ts` |
| Sweep compile + catalog | `sweepCompile.ts`, `sweepCatalogData.ts`, `fixtures/rt_experiments/sweep_catalog_v1.json` |
| UI panels | `ExperimentAnalyticsPanel`, `ExperimentTrendStrip`, `SweepCatalogBrowser` |
| Workbench wiring | `ExperimentWorkbenchPanel`, `RuntimeCognitionHub` |
| Maintainer CLI | `scripts/rt/rt_experiment_analytics.py` |
| Governance | `BANNER_ANALYTICS`, extended vitest governance/isolation |

## Forbidden

- `platform/rt-sandbox-bridge/` protocol changes
- `platform/sa-r0-viewer/` changes
- Browser `capture_session`
- Readiness/winner UI; SA auto-import; M3 distributed runtime

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_experiment_analytics.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F1 frozen. Do not start post-F1 expansion (F3 annex timeline UI, M3, SA workflow automation) without explicit new wave audit.

## Related

- [rt_plat_f1_governance_review_r1.md](../evaluation/rt_plat_f1_governance_review_r1.md)
- [rt_plat_f1_workbench_review_r1.md](../evaluation/rt_plat_f1_workbench_review_r1.md)
- [rt_plat_f1_freeze_audit.md](../evaluation/rt_plat_f1_freeze_audit.md)
