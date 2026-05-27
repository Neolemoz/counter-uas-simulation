# RT-G3 — Transient Pose Sync Freeze Audit (PLAT-RT-G3)

## Scope

- [rt_g3_pose_sync_plan.md](../platform/rt_g3_pose_sync_plan.md)
- [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md)
- `platform/rt-sandbox-bridge/rt_sandbox/pose_sync.py`
- `platform/rt-sandbox-bridge/rt_sandbox/adapter_sync.py`
- Extended `adapter_worker.py`, `runtime_adapter.py`, `session_manager.py`, `world_state.py`, `governance.py`
- `scripts/rt/rt_adapter_inspect.py` (`sync-status`)
- `src/counter_uas/test/test_rt_sandbox_bridge.py`
- [rt_g3_governance_review_r1.md](rt_g3_governance_review_r1.md)
- Additive updates: [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md), [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md), [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md)

Not in scope: SA viewer, G4 telemetry bridge, G5 capture normalization, parser topics, rosbridge, launch file edits.

Prerequisite: PLAT-RT-G2 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-G3.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Default stub path (`enable_gazebo_adapter=false`) | Pass |
| Mock adapter in CI | Pass |
| Command registry authority preserved | Pass |
| Feedback mirror non-authoritative | Pass |
| No federation/orchestration writes | Pass |
| Capture boundary preserved | Pass |
| `entity_pose_mirror` still registry-fed | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `pose_sync.py` mirror + stale detector | Yes |
| 2 | `poll_feedback` / `mock_inject_drift` / `resync_all` IPC | Yes |
| 3 | Entity op sync pipeline + audit events | Yes |
| 4 | `world_summary` sync fields | Yes |
| 5 | G3 bridge error codes | Yes |
| 6 | `adapter_resync` / `adapter_poll_feedback` sub-commands | Yes |
| 7 | `rt_adapter_inspect.py sync-status` | Yes |
| 8 | Governance review R1 | Yes |
| 9 | Freeze audit (this document) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

67 tests passed at freeze time (59 regression + 8 G2 + 9 G3).

## Stop Line

Do not implement PLAT-RT-G4 adapter-fed telemetry fan-in or SA viewer live hooks without a new scoped wave and freeze audit.
