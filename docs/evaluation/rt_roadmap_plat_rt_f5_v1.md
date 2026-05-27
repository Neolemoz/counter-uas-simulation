# RT — PLAT-RT-F5 Implementation Roadmap v1

**Phase:** PLAN-RT-F5 frozen → **PLAT-RT-F5** advisory backlog  
**Prerequisite:** [rt_f5_freeze_audit.md](rt_f5_freeze_audit.md) (PLAN-RT-F5 docs frozen)  
**Freeze:** PLAT-RT-F5 P0 — [rt_plat_f5_freeze_audit.md](rt_plat_f5_freeze_audit.md)  
**Contracts:** [rt_experiment_model_v1.md](rt_experiment_model_v1.md), [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md), [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md), [rt_experiment_advanced_ui_v1.md](rt_experiment_advanced_ui_v1.md)

---

## P0 — Spec compile + metrics derive

| Item | Location | Status |
|------|----------|--------|
| `experimentSpecCompile.ts` | `platform/rt-sandbox-ui/src/experiment/` | Done (P0) |
| Vitest golden tests from `f5_spec_examples/` | `platform/rt-sandbox-ui/src/experiment/` | Done (P0) |
| `metricsDerive.ts` | `platform/rt-sandbox-ui/src/experiment/` | Done (P0) |
| `rt_experiment_spec_compile.py` | `scripts/rt/` | Done (P0) |
| Batch F5 manifest passthrough | `scripts/rt/rt_experiment_batch.py` | Done (P0) |
| Workbench: import spec + show batch CLI snippet | `ExperimentWorkbenchPanel.tsx` | Done (P1) |

---

## P1 — Advanced experiment UI

**Freeze:** [rt_plat_f5_p1_freeze_audit.md](rt_plat_f5_p1_freeze_audit.md)

| Item | Location | Status |
|------|----------|--------|
| `ExperimentExtendedComparePanel` | `src/experiment/` | Done (P1) |
| `ExperimentMatrixPanel` | `src/experiment/` | Done (P1) |
| `ExperimentFilterBar` | `src/experiment/` | Done (P1) |
| `ExperimentHandoffEligibilityStrip` | `src/experiment/` | Done (P1) |
| `BANNER_EXPERIMENT_F5` wiring | workbench + cognition hub | Done (P1) |

---

## P2 — Maintainer CLI + trend

**Freeze:** [rt_plat_f5_p2_freeze_audit.md](rt_plat_f5_p2_freeze_audit.md)

| Item | Location | Status |
|------|----------|--------|
| `rt_experiment_metrics.py` | `scripts/rt/` | Done (P2) |
| `ExperimentRepeatabilityTrendStrip` | `src/experiment/` | Done (P2) |
| Pytest golden parity | `test_rt_experiment_metrics.py` | Done (P2) |

---

## Explicit out of scope (PLAT-RT-F5)

- Bridge protocol / telemetry / new subcommands  
- `platform/sa-r0-viewer/` changes  
- Auto-import, federation, browser `capture_session`  
- Distributed batch workers (M3)  
- Tactical controller redesign  
- Gazebo sensor-truth / physics fidelity (deferred fidelity frontier)  
- Parser/topic/schema changes  

---

## Validation (PLAT wave)

```bash
lint_rt_runtime_subcommands
pytest platform/rt-sandbox-bridge/tests/
cd platform/rt-sandbox-ui && npm run test
python scripts/rt/rt_experiment_spec_compile.py --spec fixtures/rt_experiments/f5_spec_examples/parameter_matrix.json --out /tmp/batch.yaml
python3 scripts/rt/rt_experiment_metrics.py --manifest fixtures/rt_experiments/f5_metrics_golden/manifest.json --spec fixtures/rt_experiments/f5_spec_examples/parameter_matrix.json --out /tmp/metrics_report.json
```

---

## Next frontier (advisory)

PLAN-RT-F5b frozen — see [rt_f5b_freeze_audit.md](rt_f5b_freeze_audit.md), [rt_roadmap_plat_rt_f5b_v1.md](rt_roadmap_plat_rt_f5b_v1.md). Recommended implementation: **PLAT-RT-F5b** before F6/M3.

---

## Stop line

PLAN-RT-F5 frozen. PLAT-RT-F5 **P0 + P1 + P2 frozen** — F5 roadmap complete. Do not start F5b or M3 without new wave audit.
