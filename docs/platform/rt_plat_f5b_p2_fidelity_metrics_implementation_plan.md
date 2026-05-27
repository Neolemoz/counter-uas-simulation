# RT-F5b — Fidelity Metrics + Compare (PLAT-RT-F5b P2)

**Phase:** PLAT-RT-F5b P2 — experiment fidelity metrics derive + compare strip  
**Prerequisite:** PLAT-RT-F5b P0 + P1 frozen  
**Authority:** [rt_experiment_metrics_v1.md](../evaluation/rt_experiment_metrics_v1.md) §11, [rt_runtime_fidelity_coupling_v1.md](../evaluation/rt_runtime_fidelity_coupling_v1.md)

## Goal

Complete the F5b roadmap: third derived report `rt_experiment_fidelity_metrics_report_v1`, maintainer CLI mirroring TS derive, and read-only experiment fidelity compare strip in the workbench.

## Delivered (P2)

| Item | Location |
|------|----------|
| `fidelityMetricsDerive.ts` | `platform/rt-sandbox-ui/src/experiment/` |
| `rt_experiment_fidelity_metrics.py` | `scripts/rt/` |
| `manifest_fidelity_golden.json` + staging | `fixtures/rt_experiments/f5b_fidelity_examples/` |
| `ExperimentFidelityCompareStrip` | `platform/rt-sandbox-ui/src/experiment/` |
| Workbench wiring + CLI hint | `ExperimentWorkbenchPanel.tsx` |
| Vitest + pytest golden parity | `fidelityMetricsDerive.test.ts`, `test_rt_experiment_fidelity_metrics.py` |

## Forbidden (unchanged)

- Bridge HTTP / subcommand protocol changes
- SA viewer changes
- Parser/topic changes
- Tactical redesign
- Mutating frozen F5 `metricsDerive.ts` / `rt_experiment_metrics_report_v1`

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_fidelity_metrics.py -q
cd platform/rt-sandbox-ui && npm test && npm run build
```

## Stop line

PLAT-RT-F5b P2 frozen — **F5b roadmap complete**. Do not start M3 distributed runtime or F6 SA workflow automation without explicit new wave audit.

## Related

- [rt_plat_f5b_p2_freeze_audit.md](../evaluation/rt_plat_f5b_p2_freeze_audit.md)
- [rt_plat_f5b_p2_governance_review_r1.md](../evaluation/rt_plat_f5b_p2_governance_review_r1.md)
- [rt_roadmap_plat_rt_f5b_v1.md](../evaluation/rt_roadmap_plat_rt_f5b_v1.md)
