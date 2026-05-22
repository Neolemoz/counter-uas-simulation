# RT-G2 — Gazebo Adapter Freeze Audit (PLAT-RT-G2)

## Scope

- [rt_g2_gazebo_adapter_plan.md](../platform/rt_g2_gazebo_adapter_plan.md)
- `platform/rt-sandbox-bridge/rt_sandbox/runtime_handle.py`
- `platform/rt-sandbox-bridge/rt_sandbox/runtime_adapter.py`
- `platform/rt-sandbox-bridge/rt_sandbox/adapter_worker.py`
- `platform/rt-sandbox-bridge/rt_sandbox/adapter_ipc.py`
- `platform/rt-sandbox-bridge/rt_sandbox/adapter_sync.py`
- `platform/rt-sandbox-bridge/rt_sandbox/ros_allowlist.py`
- Extended `runtime_stub.py`, `session_manager.py`, `governance.py`, `telemetry_subscriptions.py`
- `scripts/rt/rt_adapter_inspect.py`
- `src/counter_uas/test/test_rt_sandbox_bridge.py`
- [rt_g2_governance_review_r1.md](rt_g2_governance_review_r1.md)
- Additive updates: [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md), [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md), [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md), [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md), [rt_roadmap_g2_g5_v1.md](rt_roadmap_g2_g5_v1.md)

Not in scope: SA viewer, launch/world/node source edits, G3 stale-sync, G4 telemetry bridge, G5 capture normalization, auto replay import, rosbridge.

Prerequisite: PLAN-RT-G1 frozen; PLAT-RT-S6 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-G2.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Default stub path (`enable_gazebo_adapter=false`) | Pass |
| Mock adapter in CI | Pass |
| ROS allow-list enforced | Pass |
| No federation/orchestration writes | Pass |
| Capture boundary preserved | Pass |
| No launch file edits | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | RuntimeHandle + factory | Yes |
| 2 | GazeboRuntimeAdapter + worker | Yes |
| 3 | ROS allow-list | Yes |
| 4 | Minimal entity pose sync | Yes |
| 5 | `send_runtime_command` adapter sub-commands | Yes |
| 6 | Audit extensions | Yes |
| 7 | `rt_adapter_inspect.py` | Yes |
| 8 | Governance review R1 | Yes |
| 9 | Freeze audit (this document) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

59 tests passed at freeze time (51 with stub default + 8 G2 adapter).

## Stop Line

Do not implement PLAT-RT-G3 bidirectional stale-sync or sim feedback authority without a new scoped wave and freeze audit.
