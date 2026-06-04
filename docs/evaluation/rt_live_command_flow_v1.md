# RT Live Command Flow (`rt_live_command_flow_v1`)

**Phase:** Web ↔ Gazebo Live Runtime — Step 5 command path audit  
**Authority:** additive read-only; no parser/topic/schema changes

Documents the command path **Web → Bridge → Adapter → Gazebo → Telemetry Mirror → Cesium** for entity and session lifecycle commands.

See also: [rt_live_telemetry_flow_v1.md](rt_live_telemetry_flow_v1.md) (mirror pull path).

---

## 1. Entity commands (spawn / move / delete)

```
Browser (entityCommands.ts)
  sendCommand({ command_type: spawn_entity | move_entity | delete_entity })
       ↓
BridgeSessionManager.handle_command()
  → session_entity_handlers.handle_entity()
       ↓
EntityRegistry (command authority)
  registry.spawn | move | delete → world.bump_revision()
       ↓
adapter_sync.sync_spawn | sync_move | sync_delete()
  → GazeboRuntimeAdapter.apply_pose | delete_entity (IPC)
       ↓
AdapterWorker
  live: LiveRosClient.publish_pose_cmd → ROS …/entity_pose_cmd
        entity_state feedback on …/entity_state
  mock: in-memory MockSimState
       ↓
finish_entity_sync() [spawn/move only]
  → run_adapter_poll_tick(poll_telemetry=True)
  → apply_adapter_poll_result → publish_all_telemetry_channels
       ↓
publish_channels_for_transition()
  → poll_telemetry_bridge [delete; spawn/move skip — already polled]
  → publish entity_pose_mirror, world_summary, session_health, …
       ↓
UI doPull() after command success (useSessionEntityEditing)
  → mergeChannelSnapshots → entity_pose_mirror → entities → Cesium
```

**Aliases:** `spawn_attacker` / `spawn_defender` → `spawn_entity`; `reposition_entity` → `move_entity`.

**Authority:** Bridge `EntityRegistry` is command truth; adapter/Gazebo sync is downstream; `entity_pose_mirror` is explanatory telemetry only.

---

## 2. start_session

```
Browser startSession({ runtimeProfile: live | mock_adapter | stub })
       ↓
BridgeSessionManager.handle_command("start_session")
  → live preflight when runtime_profile=live
  → _config_for_command enables adapter + live poll hz
  → session_lifecycle_handlers.start_session()
       ↓
create_runtime() → RuntimeStub | GazeboRuntimeAdapter
  → runtime.start() [adapter: spawn adapter_worker, attach session]
       ↓
SessionRecord RUNNING, publish_transition("start_session")
  → initial session_health, world_summary, entity_pose_mirror (registry/stub)
       ↓
UI subscribe_telemetry(TELEMETRY_CHANNELS) + auto-refresh pull
```

---

## 3. stop_session / stop_sim

```
Browser stopSession() | stopSim() [live → stop_sim]
       ↓
BridgeSessionManager.handle_command
  stop_sim alias → stop_session(terminate_runtime=True)
       ↓
session_lifecycle_handlers.stop_session()
  → runtime.stop(); terminate when stop_sim
  → SessionState.STOPPED
  → publish_transition("stop_session")
```

Live UI disconnect uses `stop_sim` to terminate Gazebo adapter immediately (Step 3).

---

## 4. Command result visibility (no maintainer CLI)

| Step | Mechanism |
|------|-----------|
| Bridge post-command | `finish_entity_sync` + `publish_channels_for_transition` publish `entity_pose_mirror` |
| UI after OK response | `runEntityCommand` → `await doPull()` |
| Auto-refresh | 1 Hz pull drains subscription ring |
| Cesium | `entities` from mirror → `syncEntityMarkers` + overlay layers |

Successful spawn/move/delete is observable in `entity_pose_mirror` on the next pull without `rt_adapter_inspect.py`.

---

## 5. Command readiness (UI-only, Step 5)

Derived in `liveCommandHealth.ts` from existing session/profile fields:

- **command_ready:** live profile + connected + adapter alive + lifecycle running/paused + editing enabled
- **command_unavailable:** live context but preflight failed, disconnected, adapter down, blocked lifecycle, or editing lock

No bridge schema changes.

---

## 6. Forbidden / unchanged

- No `/tracks/state` or perception topics in command or telemetry allow-lists
- No tactical execution path changes
- No parser contract changes
