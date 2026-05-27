# RT-F5b — Runtime Fidelity Coupling (PLAT-RT-F5b P0)

**Phase:** PLAT-RT-F5b P0 — fidelity coupling foundations  
**Prerequisite:** PLAN-RT-F5b frozen — [rt_f5b_freeze_audit.md](../evaluation/rt_f5b_freeze_audit.md)  
**Authority:** [rt_runtime_fidelity_coupling_v1.md](../evaluation/rt_runtime_fidelity_coupling_v1.md)

## Goal

Implement runtime-side fidelity coupling foundations: default-off `enable_fidelity_coupling`, adapter IPC `fidelity_truth`, capture `fidelity_pose_block`, and fidelity audit events — without UI, SA viewer, bridge HTTP changes, or parser/topic changes.

## Delivered (P0)

| Item | Location |
|------|----------|
| Coupling flag + helpers | `governance.py`, `fidelity_coupling.py` |
| Adapter IPC `fidelity_truth` | `adapter_worker.py`, `runtime_adapter.py`, `runtime_handle.py` |
| Telemetry mirror storage | `telemetry_bridge.py` |
| Poll fidelity audits | `adapter_poll.py` |
| Capture fidelity block | `capture_fidelity_coupling.py`, `capture_normalize.py` |
| Capture handler + raw artifact | `session_capture_handler.py` (`fidelity_truth.json`) |
| Authority label | `authority_labels.py` (`truth_attested`) |
| Audit vocabulary | `audit_vocabulary.py` |
| Maintainer ctx passthrough | `scripts/rt/rt_capture_normalize.py` |
| Tests | `src/counter_uas/test/test_rt_sandbox_bridge.py` (fidelity + registry) |

## Forbidden (unchanged)

- `platform/rt-sandbox-ui/` workstation / Cesium strips (P1)
- `platform/sa-r0-viewer/` changes
- Bridge HTTP route / subcommand changes
- Parser/topic/schema changes
- Fidelity metrics derive / experiment UI (P2)
- Tactical redesign; M3 distributed

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -k fidelity -q
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py::test_registry_unchanged_by_truth_drift -q
```

## Stop line

PLAT-RT-F5b P0 frozen. Do not start **P1 UI** without separate scope confirmation.

## Related

- [rt_roadmap_plat_rt_f5b_v1.md](../evaluation/rt_roadmap_plat_rt_f5b_v1.md)
- [rt_plat_f5b_p0_freeze_audit.md](../evaluation/rt_plat_f5b_p0_freeze_audit.md)
- [rt_plat_f5b_p0_governance_review_r1.md](../evaluation/rt_plat_f5b_p0_governance_review_r1.md)
