# RT-G6 — Gazebo Runtime Fidelity Freeze Audit (PLAT-RT-G6)

## Scope

- [rt_g6_gazebo_runtime_fidelity_plan.md](../platform/rt_g6_gazebo_runtime_fidelity_plan.md)
- [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md)
- [rt_gazebo_visual_fidelity_v1.md](rt_gazebo_visual_fidelity_v1.md)
- [rt_roadmap_g6_v1.md](rt_roadmap_g6_v1.md)
- `src/rt_sandbox_gz/` — flat world, models, gz bridge node, launch
- `platform/rt-sandbox-bridge/rt_sandbox/live_ros_client.py`
- `platform/rt-sandbox-bridge/rt_sandbox/entity_model_map.py`
- Extended `adapter_worker.py`, `pose_sync.py`, `adapter_poll.py`, `governance.py`, `runtime_adapter.py`, `audit_vocabulary.py`
- `platform/rt-sandbox-ui/src/sync/`, Cesium sync cognition
- `scripts/rt/rt_adapter_inspect.py` — `sync-status --live`, `world-health`
- `src/counter_uas/test/test_rt_g6_*.py`
- [rt_g6_governance_review_r1.md](rt_g6_governance_review_r1.md)

Not in scope: SA viewer, replay ingestion, federation, multi-session UI, WebSocket push, parser/topic changes, live-default adapter.

Prerequisite: PLAT-RT-G5, PLAT-RT-R3d, PLAT-RT-T1–T5, PLAT-RT-SA1 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-G6.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Default stub path | Pass |
| Mock adapter CI | Pass |
| EntityRegistry authority | Pass |
| No federation writes | Pass |
| No auto SA import | Pass |
| Dedicated RT Gazebo stack | Pass |
| rclpy worker-only | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `rt_sandbox_gz` ROS package | Yes |
| 2 | `live_ros_client.py` | Yes |
| 3 | `entity_model_map.py` | Yes |
| 4 | Live worker path + mock fallback | Yes |
| 5 | Sync hardening (stale/missing/lag) | Yes |
| 6 | UI sync cognition + pendingReconcile fix | Yes |
| 7 | Audit vocabulary + inspect CLI | Yes |
| 8 | Contracts G6 | Yes |
| 9 | Governance review R1 | Yes |
| 10 | Freeze audit (this document) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py src/counter_uas/test/test_rt_g6_entity_model_map.py src/counter_uas/test/test_rt_g6_pose_sync.py -q
(cd platform/rt-sandbox-ui && npm test && npm run build)
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
```

138 bridge tests + 6 G6 unit tests passed at freeze time; 48 UI vitest tests passed.

## Gazebo runtime sync architecture summary

| Layer | Role |
|-------|------|
| Bridge registry | Command-authoritative poses |
| adapter_worker | IPC + optional live rclpy client |
| `/rt_sandbox/{session}/entity_pose_cmd` | Spawn/move/delete intent |
| `rt_sandbox_gz_bridge_node` | Gazebo spawn/set_pose/remove |
| `/rt_sandbox/{session}/entity_state` | Sim truth mirror |
| `PoseSyncMirror` | Drift/stale/mismatch diagnostics |
| Cesium UI | Explanatory tri-source cognition |

## Fidelity improvements summary

- Dedicated flat RT world decoupled from counter-UAS scenario
- Entity type visual proxies with ground snap
- Live feedback from Gazebo via entity_state (when live mode + ROS available)
- Per-entity drift and apply lag visible in UI
- Command ghost marker on Cesium when drift detected

## Stop Line

Do not implement multi-session UI, deeper SA workflow integration, bridge staging API, or live-default adapter without a new scoped wave and freeze audit.

Next frontiers (plan-only): multi-session planning, deeper SA workflow UX.
