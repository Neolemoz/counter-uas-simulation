# RT — PLAT-RT-F5b Implementation Roadmap v1

**Phase:** PLAN-RT-F5b frozen → **PLAT-RT-F5b** advisory backlog  
**Prerequisite:** [rt_f5b_freeze_audit.md](rt_f5b_freeze_audit.md) (PLAN-RT-F5b docs frozen); PLAT-RT-F5 P0/P1/P2, PLAT-RT-G6, PLAT-RT-F4, PLAT-RT-R2e frozen  
**Contracts:** [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md), [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md), [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) §11

---

## P0 — Adapter truth + capture blocks

**Freeze:** [rt_plat_f5b_p0_freeze_audit.md](rt_plat_f5b_p0_freeze_audit.md)

| Item | Location | Status |
|------|----------|--------|
| `enable_fidelity_coupling` session/adapter flag | `governance.py`, `runtime_handle.py` | Done (P0) |
| Optional `fidelity_truth` on `poll_telemetry` IPC | `adapter_worker.py`, `telemetry_bridge.py` | Done (P0) |
| `fidelity_pose_block` in normalization | `capture_fidelity_coupling.py`, `capture_normalize.py` | Done (P0) |
| Audit events `fidelity_truth_*` | `audit_vocabulary.py`, `adapter_poll.py` | Done (P0) |
| Bridge pytest coverage | `test_rt_sandbox_bridge.py` | Done (P0) |

**Constraint:** No [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) HTTP route changes.

---

## P1 — Runtime cognition UI

**Freeze:** [rt_plat_f5b_p1_freeze_audit.md](rt_plat_f5b_p1_freeze_audit.md)

| Item | Location | Status |
|------|----------|--------|
| Dual AGL / truth vs explanatory badges | `terrainCognition.ts`, Cesium panels | Done (P1) |
| `BANNER_FIDELITY_TRUTH` | workstation + cognition hub | Done (P1) |
| Stale / divergence strips | `RuntimeCognitionHub`, diagnostics | Done (P1) |
| Multi-session fidelity scoped per tab | session chrome | Done (P1) |
| Pull passthrough metadata | `telemetry_bridge.py`, `fidelity_coupling.py` | Done (P1) |

---

## P2 — Experiment fidelity metrics

**Freeze:** [rt_plat_f5b_p2_freeze_audit.md](rt_plat_f5b_p2_freeze_audit.md)

| Item | Location | Status |
|------|----------|--------|
| `fidelityMetricsDerive.ts` | `platform/rt-sandbox-ui/src/experiment/` | Done (P2) |
| `rt_experiment_fidelity_metrics.py` | `scripts/rt/` | Done (P2) |
| Manifest `fidelity_context` passthrough | `experimentSchema.ts` | Done (P2) |
| `ExperimentFidelityCompareStrip` | experiment workbench | Done (P2) |
| Vitest / pytest golden parity | `f5b_fidelity_examples/` | Done (P2) |

---

## Explicit out of scope (PLAT-RT-F5b)

- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) HTTP / subcommand changes  
- `platform/sa-r0-viewer/` changes  
- Auto-import, federation, browser `capture_session`  
- Distributed batch workers (M3)  
- Tactical controller redesign  
- Parser/topic/schema changes (including `/tracks/state`)  
- New ROS topics outside [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md) without boundary re-audit  
- Rewriting registry poses from sim or fictional terrain  

---

## Validation (PLAT wave)

```bash
lint_rt_runtime_subcommands
pytest platform/rt-sandbox-bridge/tests/
cd platform/rt-sandbox-ui && npm run test
# P2 advisory:
python3 scripts/rt/rt_experiment_fidelity_metrics.py \
  --manifest fixtures/rt_experiments/f5b_fidelity_examples/manifest_fidelity_golden.json \
  --repo-root . \
  --out /tmp/fidelity_metrics_report.json
```

---

## Next frontier (advisory)

After PLAT-RT-F5b freeze: **F6 richer SA workflow** or **M3 distributed** only with explicit wave audit — see [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md).

---

## Stop line

PLAN-RT-F5b frozen. PLAT-RT-F5b **P0 frozen** — see [rt_plat_f5b_p0_freeze_audit.md](rt_plat_f5b_p0_freeze_audit.md). PLAT-RT-F5b **P1 frozen** — see [rt_plat_f5b_p1_freeze_audit.md](rt_plat_f5b_p1_freeze_audit.md). PLAT-RT-F5b **P2 frozen** — see [rt_plat_f5b_p2_freeze_audit.md](rt_plat_f5b_p2_freeze_audit.md). **F5b roadmap complete.** Do not start M3 or F6 without explicit wave audit.
