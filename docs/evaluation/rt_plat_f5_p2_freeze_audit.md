# RT-F5 — Freeze Audit (PLAT-RT-F5 P2)

**Phase:** PLAT-RT-F5 P2 — metrics CLI + repeatability trend  
**Status:** frozen (P2 scope)

**Plan:** [rt_plat_f5_p2_metrics_cli_repeatability_plan.md](../platform/rt_plat_f5_p2_metrics_cli_repeatability_plan.md)

**Prior:** [rt_plat_f5_p1_freeze_audit.md](rt_plat_f5_p1_freeze_audit.md), [rt_plat_f5_freeze_audit.md](rt_plat_f5_freeze_audit.md)

---

## Scope delivered

| # | Item | Done |
|---|------|------|
| 1 | `rt_experiment_metrics.py` | Yes |
| 2 | `metrics_report.json` golden (TS/Python parity) | Yes |
| 3 | `ExperimentRepeatabilityTrendStrip` | Yes |
| 4 | Workbench wiring (`repeatability_sweep`) | Yes |
| 5 | Pytest + vitest + governance docs | Yes |

---

## F5 completion (P0 + P1 + P2)

| Wave | Deliverable |
|------|-------------|
| P0 | Spec compile + `metricsDerive.ts` + `rt_experiment_spec_compile.py` |
| P1 | Matrix / extended compare / filters / handoff UI |
| P2 | `rt_experiment_metrics.py` + repeatability trend strip |

---

## Regression evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_metrics.py \
  src/counter_uas/test/test_rt_experiment_analytics.py \
  src/counter_uas/test/test_rt_experiment_spec_compile.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
python3 scripts/rt/rt_experiment_metrics.py \
  --manifest fixtures/rt_experiments/f5_metrics_golden/manifest.json \
  --spec fixtures/rt_experiments/f5_spec_examples/parameter_matrix.json \
  --out /tmp/metrics_report.json
```

---

## Stop line

PLAT-RT-F5 **complete** (P0 + P1 + P2 frozen). **PLAN-RT-F5b frozen** — see [rt_f5b_freeze_audit.md](rt_f5b_freeze_audit.md). Do not start **PLAT-RT-F5b**, **M3**, or **F6** without separate wave audit.
