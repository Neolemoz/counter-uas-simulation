# RT-F5 — Freeze Audit (PLAT-RT-F5 P0)

**Phase:** PLAT-RT-F5 P0 — advanced runtime experiments (implementation)  
**Status:** frozen (P0 scope)

**Plan:** [rt_plat_f5_advanced_runtime_experiments_implementation_plan.md](../platform/rt_plat_f5_advanced_runtime_experiments_implementation_plan.md)

---

## Scope delivered

| # | Item | Done |
|---|------|------|
| 1 | `experimentSpecCompile.ts` | Yes |
| 2 | `metricsDerive.ts` | Yes |
| 3 | F5 Zod schemas + guards | Yes |
| 4 | `rt_experiment_spec_compile.py` | Yes |
| 5 | Compile goldens + metrics fixture | Yes |
| 6 | Batch manifest F5 passthrough | Yes |
| 7 | Vitest + pytest | Yes |
| 8 | Governance + freeze docs | Yes |

**Not delivered (P1/P2):** experiment UI panels, `rt_experiment_metrics.py`, workbench spec import.

---

## Compile architecture summary

`rt_experiment_spec_v1` compiles to `rt_experiment_batch_v1` via `explicit_list`, `cartesian`, or `repeat_expand`. Each run carries `spec_fingerprint` and class metadata. TS and Python compilers produce matching `run_id` ordering for parameter matrix fixtures.

---

## Metrics rollups summary

`deriveExperimentMetrics(manifest, f1Report, { spec })` emits `rt_experiment_metrics_report_v1` with `rollup_extended` blocks: terrain, visibility, tactical, repeatability, matrix; extended compare badges; advisory `handoff_eligibility`.

---

## Boundary guarantees

- No bridge / parser / SA viewer changes
- P0 does not add experiment workbench UI
- Metrics and eligibility are not operational authority

---

## Regression evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_spec_compile.py \
  src/counter_uas/test/test_rt_experiment_analytics.py \
  src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
```

---

## Stop line

PLAT-RT-F5 P0 frozen. **P1** and **P2** frozen — see [rt_plat_f5_p1_freeze_audit.md](rt_plat_f5_p1_freeze_audit.md), [rt_plat_f5_p2_freeze_audit.md](rt_plat_f5_p2_freeze_audit.md). F5 roadmap complete.
