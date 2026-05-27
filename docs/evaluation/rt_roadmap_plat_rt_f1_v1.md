# PLAT-RT-F1 — Implementation Roadmap v1

**Phase:** PLAT-RT-F1 — **frozen**  
**Prerequisite:** [rt_f1_freeze_audit.md](rt_f1_freeze_audit.md) (PLAN-RT-F1 docs frozen)  
**Freeze:** [rt_plat_f1_freeze_audit.md](rt_plat_f1_freeze_audit.md)  
**Contracts:** [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md), [rt_experiment_sweep_catalog_v1.md](rt_experiment_sweep_catalog_v1.md), [rt_experiment_analytics_ui_v1.md](rt_experiment_analytics_ui_v1.md)

---

## Delivered (P0 + P1)

| Item | Location | Status |
|------|----------|--------|
| `deriveExperimentAnalytics` | `analyticsDerive.ts` | Done |
| Vitest derive + sweep tests | `analyticsDerive.test.ts`, `sweepCompile.test.ts` | Done |
| `ExperimentAnalyticsPanel` | `ExperimentAnalyticsPanel.tsx` | Done |
| `ExperimentTrendStrip` | `ExperimentTrendStrip.tsx` | Done |
| `SweepCatalogBrowser` | `SweepCatalogBrowser.tsx` | Done |
| `compileSweepGroup` | `sweepCompile.ts` | Done |
| `BANNER_ANALYTICS` | `governance/banners.ts` | Done |
| Workbench wiring | `ExperimentWorkbenchPanel.tsx` | Done |
| Cognition hub line | `RuntimeCognitionHub.tsx` | Done |
| `rt_experiment_analytics.py` | `scripts/rt/` | Done |
| isolation + governance tests | Extended | Done |

---

## Explicit out of scope (unchanged)

- Bridge protocol changes  
- SA viewer changes  
- M3 distributed multi-bridge  
- SA auto-import, readiness/winner UI  

---

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_experiment_analytics.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

---

## Next frontier (advisory)

See [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md):

- **F3** — full annex timeline UI in RT workbench (read-only cognition)  
- **Platform hardening** — lint/docs debt from PLAN-RT-R2  
- Not authorized: M3, SA workflow automation, post-F1 analytics expansion without new audit

---

## Stop line

**PLAT-RT-F1 frozen.** Do not start post-F1 implementation without explicit plan + governance review + freeze audit.
