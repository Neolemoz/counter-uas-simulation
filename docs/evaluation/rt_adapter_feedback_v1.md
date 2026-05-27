# RT Adapter Feedback Contract (`rt_adapter_feedback_v1`)

**Phase:** PLAT-RT-G3 — transient pose synchronization  
**Authority:** [rt_g3_pose_sync_plan.md](../platform/rt_g3_pose_sync_plan.md); [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md)

IPC feedback between **GazeboRuntimeAdapter** (bridge) and **adapter_worker** subprocess. Bridge never imports `rclpy`.

---

## 1. Authority rules

| Layer | Role |
|-------|------|
| Bridge `EntityRegistry` | Command-authoritative poses |
| Adapter feedback | Transient runtime truth mirror only |
| `PoseSyncMirror` | Bridge-side feedback storage — never overwrites registry |

---

## 2. IPC operations (additive)

### 2.1 `poll_feedback`

**Request payload:** `{}` or optional test fields:

| Field | Required | Description |
|-------|----------|-------------|
| `mock_inject_drift` | no | Test-only: `{ entity_id, offset: {x,y,z} }` applied to sim pose before read |

**Response result schema:** `rt_adapter_feedback_v1`

```json
{
  "schema": "rt_adapter_feedback_v1",
  "timestamp_utc": "2026-05-22T12:00:00+00:00",
  "sync_seq": 3,
  "entities": [
    {
      "entity_id": "uuid",
      "sim_entity_ref": "sim-abc12345",
      "pose": { "x": 0, "y": 0, "z": 10 }
    }
  ]
}
```

### 2.2 `apply_pose` (extended result)

Additional fields on success:

| Field | Description |
|-------|-------------|
| `bridge_revision` | Echo from request payload when provided |
| `sync_seq` | Monotonic worker sequence |
| `feedback_timestamp_utc` | ISO UTC at apply time |

### 2.3 `resync_all`

**Request payload:**

```json
{
  "entities": [
    { "entity_id": "uuid", "entity_type": "drone", "pose": { "x": 0, "y": 0, "z": 10 } }
  ]
}
```

Re-applies all registry poses to sim state without reading sim into bridge registry.

### 2.4 `mock_inject_drift` (test / maintainer)

**Request payload:**

```json
{
  "entity_id": "uuid",
  "offset": { "x": 5.0, "y": 0, "z": 0 }
}
```

Offsets sim feedback pose only (mock mode).

---

## 3. Sync revision policy

- Bridge `WorldStateStore.revision` increments on each entity command.
- Mirror entries record `sync_revision` at command time.
- `apply_pose` IPC may include `bridge_revision` for worker audit correlation.
- Feedback `sync_seq` is worker-local monotonic counter — updated on each `poll_feedback`; not compared to `world.revision` for failure.
- Per-entity `sync_revision` records `world.revision` at command time; may be less than current `world.revision` if entity unchanged.

Revision divergence policy: [rt_world_revision_hint_policy_v1.md](rt_world_revision_hint_policy_v1.md).

---

## 4. Stale detection (bridge)

| Condition | Bridge error | Audit |
|-----------|--------------|-------|
| ‖command − feedback‖ > `pose_sync_drift_threshold_m` | `SYNC_STALE` | `sync_stale` |
| Unknown entity or ref mismatch | `SYNC_MISMATCH` | `sync_mismatch` |
| IPC poll failure | `ADAPTER_FEEDBACK_LOST` | `adapter_feedback_lost` |
| No poll within `adapter_feedback_stale_s` | `ADAPTER_FEEDBACK_LOST` | `adapter_feedback_lost` |
| OK | — | `sync_update` |

Recovery: `reset_session` clears mirror and sim state.

Clock-age feedback staleness uses shared `time_utils.is_poll_stale()` via `PoseSyncMirror.check_feedback_stale()` — see [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md).

Unified poll ownership: [adapter_poll.py](../../platform/rt-sandbox-bridge/rt_sandbox/adapter_poll.py) `run_adapter_poll_tick()`.

---

## 5. Non-authoritative rule

Feedback and sync audit entries are **explanatory runtime diagnostics** — not SA replay authority, not corpus/federation writes.

---

## Related

- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) — G3 error codes
- [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md) §7
