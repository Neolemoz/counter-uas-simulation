# RT Live Control Truth Contract (`rt_live_control_truth_v1`)

**Phase:** PLAT-RT-LIVE-CONTROL-TRUTH1 — Gazebo pause/resume/reset runtime truth  
**Authority:** Additive note atop [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md); [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md)

Documents live-mode **control truth** between `adapter_worker`, Gazebo Sim WorldControl, `rt_sandbox_gz_bridge_node`, and session `clock` mirror. Explanatory only — not SA replay authority or parser contract.

---

## 1. Authority separation

| Layer | Role |
|-------|------|
| Bridge session lifecycle | `pause_session` / `resume` / `reset_session` command authority |
| `adapter_worker` | IPC + Gazebo WorldControl + session ROS topics |
| Gazebo WorldControl | Physics/sim-time pause, resume, reset-all |
| `rt_sandbox_gz_bridge_node` | Kinematic integration gate; obeys session `clock` pause mirror |
| `clock_mirror` telemetry channel | Derived pause + optional sim time — not replay clock |

---

## 2. Gazebo WorldControl (live adapter)

Invoked via `gz_tools.py` helpers from `adapter_worker` when `gz` is on PATH and `mode=live`:

| Helper | WorldControl request | When |
|--------|---------------------|------|
| `gz_world_pause` | `pause: true` | `_pause` IPC |
| `gz_world_resume` | `pause: false` | `_resume` IPC |
| `gz_world_reset_all` | `reset: {all: true}` | `_reset_world` IPC |

Service: `/world/{rt_sandbox_world}/control` (`gz.msgs.WorldControl` → `gz.msgs.Boolean`).

Mock mode: WorldControl calls are skipped; in-memory `MockSimState` only.

---

## 3. Session `clock` topic (pause signal)

**Topic:** `/rt_sandbox/{session_id}/clock` (allow-listed publish from worker)

**Payload (session pause mirror):**

```json
{
  "paused": false,
  "mode": "live"
}
```

**Additive sim-time fields** when bridged Gazebo `/clock` is available on the adapter worker:

| Field | Type | Description |
|-------|------|-------------|
| `sim_time_sec` | int | `rosgraph_msgs/Clock` sec from `ros_gz_bridge` |
| `sim_time_nsec` | int | nanosec component |
| `sim_time_source` | string | `"gazebo_clock_bridge"` |

When `/clock` is absent or not yet received, only `paused` and `mode` are present (unchanged G2 behavior).

`RtSandboxGzBridgeNode` subscribes to this topic and gates timer-based kinematic integration while `paused=true`. Explicit spawn/move/delete commands remain accepted; auto-advance is frozen.

---

## 4. `entity_pose_cmd` — `reset_world` op (additive)

**Schema:** `rt_entity_pose_cmd_v1` (unchanged)

| `op` | Behavior |
|------|----------|
| `apply` | Spawn or move (existing) |
| `delete` | Remove model + bridge entity |
| `reset_world` | Clear bridge runtime maps; publish empty `rt_entity_state_v1` |

Example:

```json
{
  "schema": "rt_entity_pose_cmd_v1",
  "op": "reset_world",
  "entity_id": "_world_",
  "entity_type": "drone",
  "pose": {}
}
```

Live `_reset_world` IPC sequence:

1. Delete known RT entities via `delete` ops (preserved refs)
2. Clear adapter runtime maps
3. Publish `reset_world` to bridge node
4. Clear live feedback cache
5. `gz_world_reset_all()`
6. If session `paused=true`, re-apply `gz_world_pause()` (reset may resume Gazebo physics)
7. Republish session `clock` mirror

Running-session reset does **not** force pause.

---

## 5. Mock parity

| Path | Behavior |
|------|----------|
| `_poll_telemetry` plant integration | Skipped while `paused` |
| `_reset_world` | Clears entities, drift, refs, seq counters, `last_poll_monotonic` |
| `clock_mirror` | `paused` + `mode` only (no sim-time fields) |

---

## 6. Non-goals (TRUTH1)

- No telemetry schema version change (`rt_adapter_telemetry_v1` unchanged)
- No parser/topic contract changes
- No bridge-node authority redesign
- No autonomy / tactical semantics

---

## Related

- [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md)
- [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md) — `clock_mirror` channel
- [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md) — sim vs wall clock
- `src/rt_sandbox_gz/rt_sandbox_gz/gz_tools.py` — WorldControl helpers
