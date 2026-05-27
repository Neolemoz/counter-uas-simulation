# RT Adapter Live Sync Contract (`rt_adapter_live_sync_v1`)

**Phase:** PLAT-RT-G6 — Gazebo runtime visual fidelity & real runtime sync  
**Authority:** [rt_g6_gazebo_runtime_fidelity_plan.md](../platform/rt_g6_gazebo_runtime_fidelity_plan.md); [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md)

Live-mode ROS coupling between **adapter_worker** (rclpy client) and **rt_sandbox_gz_bridge_node** (Gazebo executor). Bridge process never imports `rclpy`.

---

## 1. Authority rules

| Layer | Role |
|-------|------|
| Bridge `EntityRegistry` | Command-authoritative poses |
| `entity_pose_cmd` topic | Bridge→sim intent (transient) |
| `entity_state` topic | Sim→adapter feedback mirror |
| `PoseSyncMirror` | Bridge-side diagnostics — never overwrites registry |

---

## 2. Session topics (unchanged allow-list)

Prefix: `/rt_sandbox/<session_id>/`

| Topic | Direction | Payload |
|-------|-----------|---------|
| `entity_pose_cmd` | Worker → Gz node | `rt_entity_pose_cmd_v1` JSON on `std_msgs/String` |
| `entity_state` | Gz node → Worker | `rt_entity_state_v1` JSON on `std_msgs/String` |
| `clock` | Optional mirror | Unchanged from G2 |

---

## 3. Message schemas

### 3.1 `rt_entity_pose_cmd_v1`

```json
{
  "schema": "rt_entity_pose_cmd_v1",
  "op": "apply",
  "entity_id": "uuid",
  "entity_type": "drone",
  "sim_entity_ref": "sim-abc12345",
  "pose": { "x": 0, "y": 0, "z": 10, "yaw_deg": 0 },
  "bridge_revision": 3
}
```

| `op` | Behavior |
|------|----------|
| `apply` | Spawn if new `sim_entity_ref`; else `set_pose` |
| `delete` | Remove Gazebo model; drop from state publish |

### 3.2 `rt_entity_state_v1`

```json
{
  "schema": "rt_entity_state_v1",
  "timestamp_utc": "2026-05-25T12:00:00+00:00",
  "sync_seq": 3,
  "entities": [
    {
      "entity_id": "uuid",
      "entity_type": "drone",
      "sim_entity_ref": "sim-abc12345",
      "pose": { "x": 0, "y": 0, "z": 10, "yaw_deg": 0 }
    }
  ]
}
```

Worker maps `entity_state` → `rt_adapter_feedback_v1` for `poll_feedback` IPC.

---

## 4. Live launch

Live attach launches `rt_sandbox.launch.py` (not `gazebo_target.launch.py`):

- `gz sim` with `rt_sandbox_flat.sdf`
- `rt_sandbox_gz_bridge_node` with session-scoped topic prefix
- Clock bridge only — no target/interceptor/guidance nodes

Launch args: `session_id`, `ros_domain_id`, `use_gazebo_gui` (default false).

---

## 5. Fallback semantics

| Condition | Behavior |
|-----------|----------|
| `adapter_mode=mock` | In-memory `MockSimState` (G2–G5 path) |
| `adapter_mode=live`, rclpy unavailable | Shadow mock state; audit `gazebo_launch_failed` or `RUNTIME_UNAVAILABLE` |
| ROS topics silent > `adapter_feedback_stale_s` | `sync_health: feedback_lost` (explanatory audit) |

---

## 6. Additive world_summary fields (G6)

| Field | Description |
|-------|-------------|
| `adapter_mode` | `mock` or `live` |
| `last_command_utc` | Last entity command timestamp |
| `last_feedback_utc` | Last feedback poll timestamp |
| `apply_lag_ms` | Milliseconds between last command and feedback |
| `feedback_entities` | Per-entity command/feedback/drift (from G3, now UI-visible) |

---

## Related

- [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md)
- [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md)
- [rt_gazebo_visual_fidelity_v1.md](rt_gazebo_visual_fidelity_v1.md)
