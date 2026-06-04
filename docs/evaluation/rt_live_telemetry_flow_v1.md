# RT Live Telemetry Flow (`rt_live_telemetry_flow_v1`)

**Phase:** Web ↔ Gazebo Live Runtime — Step 4 telemetry flow audit  
**Authority:** additive read-only; no parser/topic/schema changes

Documents the live path from Gazebo through `entity_pose_mirror` to browser Cesium, including polling intervals, refresh triggers, and timeout behavior.

---

## 1. End-to-end path

```
Gazebo Sim (rt_sandbox_gz)
  → ROS 2 session topics: …/entity_state, …/entity_pose_cmd
       ↓
GazeboRuntimeAdapter (runtime_adapter.py)
  → adapter_worker IPC: poll_telemetry / poll_feedback
       ↓
AdapterWorker (adapter_worker.py)
  live: LiveRosClient subscribes entity_state, aggregates entity_state_payload()
  mock: in-memory MockSimState.entity_state_payload()
       ↓
run_adapter_poll_tick() (adapter_poll.py)
  → poll_and_update_mirror() (telemetry_bridge.py)
       ↓
TelemetryMirror.update_from_poll(bundle)
  → entity_pose_mirror, telemetry_health, last_poll_utc, telemetry_revision
       ↓
apply_adapter_poll_result() → publish_all_telemetry_channels()
  → TelemetrySubscriptionStore ring buffer
       ↓
pull_telemetry() (session_manager.py)
  → _tick_timeouts() → _tick_live_background_poll() [live only]
  → drain subscription events to browser
       ↓
UI pullSlot() → mergeChannelSnapshots()
  → snapshots.entity_pose_mirror
       ↓
useSessionEntityEditing → entities
  → CesiumRuntimeView → syncEntityMarkers + tactical/world overlay layers
```

**Parallel feedback path (G3 pose sync):** `poll_feedback` → `PoseSyncMirror` → `world_summary.sync_health`, `last_poll_utc`. Does not replace `entity_pose_mirror` authority for marker positions under adapter-fed telemetry.

---

## 2. Polling intervals

| Trigger | Rate | Scope |
|---------|------|--------|
| Live background poll | **1 Hz** (`LIVE_ADAPTER_BACKGROUND_POLL_HZ`) | `runtime_profile=live` sessions only; enabled via `_live_poll_hz()` per-session config |
| UI auto-refresh pull | **1 Hz default** (user configurable ≤ `MAX_PULL_HZ`) | Active slot; drains subscription ring |
| Background slot pull | **0.2 Hz** (`BACKGROUND_PULL_HZ`) | Non-active connected sessions |
| Command-driven poll | Event-driven | spawn/move/reset/resync → `run_adapter_poll_tick` on transition |
| Manual maintainer | On demand | `adapter_poll_telemetry` subcommand |

Live background poll runs inside `_tick_timeouts()`, invoked from:

- `handle_command()` (all bridge commands)
- `pull_telemetry()` (Step 3 — ties UI refresh to adapter poll)

Rate limit: `_tick_live_background_poll` skips if `(now - last_live_background_poll_monotonic) < 1/hz`.

---

## 3. Refresh path (browser → Cesium)

1. `useRtSessionWorkspace` auto-refresh interval calls `pullSlot(sessionId)`.
2. `pull_telemetry` drains events; latest per channel wins in `mergeChannelSnapshots`.
3. `App.tsx` passes `snapshots.entity_pose_mirror` to `useSessionEntityEditing`.
4. `entitiesFromSnapshot(entityPoseMirror)` → `mergeTelemetryAndLocalEntities` → `entities`.
5. `CesiumRuntimeView` `useEffect` depends on `entities`, `mirrorSnapshot`, `runtimeTelemetryByEntityId`.
6. `syncEntityMarkers`, `syncTacticalTrajectoryLayer`, terrain/visibility overlays consume `entities` on each update — **no manual refresh required** when auto-refresh is on.

---

## 4. Timeout / stale behavior

| Constant | Default | Effect |
|----------|---------|--------|
| `telemetry_stale_s` | 30 s | `TelemetryMirror.check_stale()` → `telemetry_health: stale` on adapter poll bundle age |
| `adapter_feedback_stale_s` | 30 s | `PoseSyncMirror.check_feedback_stale()` → `sync_health: feedback_lost` |
| Live poll rate cap | 1 Hz | Bridge `last_live_poll_utc` / `session_health.live_background_poll_hz` |
| UI pull stale | ~2× pull interval | `isPullAgeStale()` in adapter status panel |

**IPC loss:** `poll_adapter_telemetry` error → `telemetry_health: feedback_lost`, audit `telemetry_feedback_lost`.

**Pre-mirror fallback:** Before first adapter poll (`telemetry_mirror.last_poll_utc is None`), `resolve_channel_payload` serves stub/registry `entity_pose_mirror` (command authority source).

---

## 5. Channel payload fields (existing schema)

`entity_pose_mirror` (adapter-fed):

- `entities[]`, `telemetry_health`, `telemetry_revision`
- `source: adapter_feedback`, `authority_label: explanatory_telemetry`
- Event envelope: `timestamp_utc` (subscription publish time)

`session_health` (live extensions, Step 3):

- `runtime_profile`, `live_background_poll_hz`, `last_live_poll_utc`

`world_summary`:

- `telemetry_health`, `telemetry_revision`, `sync_health`, `last_poll_utc` (pose-sync feedback)

---

## 6. UI mirror freshness (Step 4)

Browser derives **fresh / stale / unavailable** from existing timestamps and health fields only — see `mirrorFreshness.ts`. No telemetry schema changes.

---

## 7. Maintainer validation

- `scripts/rt/rt_live_smoke.py` — full stack loopback smoke (not UI-invoked)
- `scripts/rt/rt_adapter_inspect.py telemetry-status` — bridge mirror inspection
