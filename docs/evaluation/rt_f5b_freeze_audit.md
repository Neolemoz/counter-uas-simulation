# RT-F5b — Freeze Audit (PLAN-RT-F5b)

**Phase:** PLAN-RT-F5b — runtime fidelity coupling  
**Status:** frozen (docs only)

**Plan:** [rt_f5b_runtime_fidelity_coupling_plan.md](../platform/rt_f5b_runtime_fidelity_coupling_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Fidelity coupling contract | [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md) |
| 2 | Fidelity cognition contract | [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md) |
| 3 | Experiment metrics update (§11) | [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) |
| 4 | Reference fixtures | [fixtures/rt_experiments/f5b_fidelity_examples/](../../fixtures/rt_experiments/f5b_fidelity_examples/) |
| 5 | Architecture review | [rt_f5b_architecture_review_r1.md](rt_f5b_architecture_review_r1.md) |
| 6 | Governance review | [rt_f5b_governance_review_r1.md](rt_f5b_governance_review_r1.md) |
| 7 | PLAT roadmap | [rt_roadmap_plat_rt_f5b_v1.md](rt_roadmap_plat_rt_f5b_v1.md) |
| 8 | Next frontiers update | [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md) |
| 9 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, or `src/counter_uas/` for this wave.

---

## Fidelity architecture summary

**Three layers:**

1. **Command (`command_authoritative`)** — `EntityRegistry`, bridge intent, `command_pose` in capture. Sole authority for spawn/move and normalized export.
2. **Truth-attested (`truth_attested`, default-off)** — `entity_state` / adapter `fidelity_truth` when `enable_fidelity_coupling=true`. Sim-scoped pose, AGL, LOS/dome/occlusion snapshots. Never overwrites registry.
3. **Explanatory (`explanatory`)** — F4/V2 fictional terrain, heuristic LOS/dome, sync drift mirrors, tactical recommendations.

**Coupling flag:** `enable_fidelity_coupling` default `false`. When off, fidelity metrics return `unavailable` and UI hides truth labels.

**Capture:** Optional `fidelity_pose_block` on normalized capture; optional `rt_fidelity_truth_snapshot_v1` at capture instant.

**Experiments:** Optional third report `rt_experiment_fidelity_metrics_report_v1` — compares truth-attested fields to F5 explanatory `los_cognition_label`; `truth_fingerprint` for repeatability under runtime truth.

**Banners:** All truth and fidelity-metric surfaces carry sim-scoped disclaimers — not SA replay or operational sensor authority.

---

## Boundary guarantees

- Truth-attested data is **not** operational authority  
- No parser/topic/bridge HTTP changes in PLAN wave  
- No SA viewer or auto-import scope  
- PLAN-RT-F5b ≠ registry RT-1..7 realism waves  
- PLAN-RT-F5b ≠ PLAN-RT-F5 experiment architecture  
- F4 fictional terrain remains explanatory unless truth channel present with coupling on  
- Registry command authority preserved per [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md)

---

## Recommended PLAT-RT-F5b scope (advisory)

See [rt_roadmap_plat_rt_f5b_v1.md](rt_roadmap_plat_rt_f5b_v1.md):

| Phase | Scope |
|-------|--------|
| **P0** | `enable_fidelity_coupling`; adapter IPC `fidelity_truth`; capture `fidelity_pose_block`; audit `fidelity_truth_*` events |
| **P1** | Workstation + Cesium truth vs explanatory badges; stale/divergence strips; `BANNER_FIDELITY_TRUTH` |
| **P2** | `fidelityMetricsDerive.ts`, `rt_experiment_fidelity_metrics.py`, manifest `fidelity_context`, experiment fidelity compare strip |

**Not authorized** by this freeze.

**Explicitly out of PLAT-RT-F5b:** M3 distributed, SA viewer, auto-import, new ROS topics without boundary audit, tactical redesign, physics/sensor model redesign.

---

## Regression evidence

Docs-only wave — cite existing platform tests (no new code in PLAN):

```text
lint_rt_runtime_subcommands
pytest platform/rt-sandbox-bridge/tests/
cd platform/rt-sandbox-ui && npm run test  # tier0-rt-ui subset as in F5 audits
```

---

## Stop line

PLAN-RT-F5b frozen. Do not start PLAT-RT-F5b without implementation plan + `rt_plat_f5b_*` governance review + freeze audit.
