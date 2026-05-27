# RT-F5 — Metrics CLI + Repeatability Trend (PLAT-RT-F5 P2)

**Phase:** PLAT-RT-F5 P2 — maintainer metrics CLI + repeatability trend UI  
**Prerequisite:** PLAT-RT-F5 P0 + P1 frozen  
**Authority:** [rt_experiment_metrics_v1.md](../evaluation/rt_experiment_metrics_v1.md), [rt_experiment_advanced_ui_v1.md](../evaluation/rt_experiment_advanced_ui_v1.md)

## Goal

Complete the F5 roadmap: Python metrics derive CLI mirroring `metricsDerive.ts`, and repeatability sweep trend cognition in the workbench.

## Delivered (P2)

| Item | Location |
|------|----------|
| `rt_experiment_metrics.py` | `scripts/rt/` |
| `fixtures/rt_experiments/f5_metrics_golden/metrics_report.json` | TS/Python parity golden |
| `ExperimentRepeatabilityTrendStrip` | `platform/rt-sandbox-ui/src/experiment/` |
| `experimentRepeatabilityTrend.ts` | ordering + consecutive pair helpers |
| Workbench wiring + CLI hint | `ExperimentWorkbenchPanel.tsx` |
| Pytest + vitest | `test_rt_experiment_metrics.py`, `*.test.ts(x)` |

## Forbidden (unchanged)

- Bridge / SA viewer / parser changes
- New experiment classes
- M3 distributed queue, F5b fidelity, F6 SA automation

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_metrics.py -q
cd platform/rt-sandbox-ui && npm test && npm run build
```

## Stop line

PLAT-RT-F5 P2 frozen — full F5 roadmap complete. Do not start F5b or M3 without new wave audit.

## Related

- [rt_plat_f5_p2_freeze_audit.md](../evaluation/rt_plat_f5_p2_freeze_audit.md)
- [rt_plat_f5_p2_governance_review_r1.md](../evaluation/rt_plat_f5_p2_governance_review_r1.md)
