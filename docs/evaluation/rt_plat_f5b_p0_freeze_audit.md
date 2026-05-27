# RT-F5b P0 — Freeze Audit (PLAT-RT-F5b P0)

**Phase:** PLAT-RT-F5b P0 — fidelity coupling foundations  
**Status:** frozen

**Plan:** [rt_plat_f5b_p0_fidelity_coupling_implementation_plan.md](../platform/rt_plat_f5b_p0_fidelity_coupling_implementation_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | `enable_fidelity_coupling` (default off) | `governance.py` |
| 2 | Fidelity coupling helpers | `fidelity_coupling.py` |
| 3 | Adapter IPC `fidelity_truth` | `adapter_worker.py`, `runtime_adapter.py` |
| 4 | Telemetry mirror + poll audits | `telemetry_bridge.py`, `adapter_poll.py` |
| 5 | Capture `fidelity_pose_block` | `capture_fidelity_coupling.py`, `capture_normalize.py` |
| 6 | Raw `fidelity_truth.json` staging | `session_capture_handler.py` |
| 7 | Audit events (4 kinds) | `audit_vocabulary.py` |
| 8 | `truth_attested` authority label | `authority_labels.py` |
| 9 | Tests | `test_rt_sandbox_bridge.py` |
| 10 | Governance + registry | Yes |

No changes under `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, or `src/counter_uas/` (except tests).

---

## Fidelity foundation summary

**Coupling flag:** `GovernanceConfig.enable_fidelity_coupling` default `false`. Active only with `enable_gazebo_adapter` and live adapter session.

**Adapter truth:** Optional `fidelity_truth` on `rt_adapter_telemetry_v1` poll — mock path derives `entity_truth`, `sim_agl_m`, `los_truth`, `dome_truth` from sim feedback poses. Stored in `TelemetryMirror`; never written to `EntityRegistry`.

**Capture:** When coupling on at capture, normalized manifest includes optional `fidelity_pose_block` (`rt_fidelity_pose_block_v1`) with command vs truth poses, drift, timestamps, flags. Raw staging may include `fidelity_truth.json`.

**Audits:** `fidelity_truth_update`, `fidelity_truth_stale`, `fidelity_truth_mismatch`, `fidelity_capture_snapshot` — append-only.

**Authority:** Registry / `command_pose` remain command-authoritative per [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md).

---

## Boundary guarantees

- No bridge HTTP protocol changes  
- No parser/topic/schema changes  
- No SA viewer or auto-import  
- PLAT-RT-F5b P0 ≠ registry RT-1..7 realism waves  
- Normalization does not fail on truth/command divergence  

---

## Recommended PLAT-RT-F5b P1 scope (advisory)

See [rt_roadmap_plat_rt_f5b_v1.md](rt_roadmap_plat_rt_f5b_v1.md):

- Workstation + Cesium truth vs explanatory badges  
- `BANNER_FIDELITY_TRUTH`  
- Stale/divergence cognition strips  

**Not authorized** by this freeze.

---

## Regression evidence

```text
lint_rt_runtime_subcommands OK
pytest src/counter_uas/test/test_rt_sandbox_bridge.py -k fidelity — 5 passed
pytest test_registry_unchanged_by_truth_drift — 1 passed
```

---

## Stop line

PLAT-RT-F5b P0 frozen. Do not start P1 without governance review + freeze audit.
