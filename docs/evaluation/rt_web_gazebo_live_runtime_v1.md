# Web ↔ Gazebo Live Runtime V1 (`rt_web_gazebo_live_runtime_v1`)

**Freeze ID:** PLAT-RT-LIVE-GZ1  
**Phase:** Web ↔ Gazebo Live Runtime — Steps 1–7 (complete)  
**Status:** frozen  
**Authority:** additive RT sandbox UX + bridge governance only; no parser/topic/perception changes

---

## 1. Architecture summary

Bidirectional live loop between browser workstation and local Gazebo stack via loopback bridge:

```
Web (rt-sandbox-ui)
  ↔ HTTP loopback bridge (rt-sandbox-bridge)
  ↔ GazeboRuntimeAdapter / adapter_worker
  ↔ ROS 2 session topics (entity_pose_cmd / entity_state)
  ↔ Gazebo Sim (rt_sandbox_gz)
```

**Command path (authoritative registry):**  
Web → Bridge → Adapter → Gazebo — see [rt_live_command_flow_v1.md](rt_live_command_flow_v1.md)

**Telemetry path (explanatory mirror):**  
Gazebo → Adapter → Bridge → `entity_pose_mirror` → Cesium — see [rt_live_telemetry_flow_v1.md](rt_live_telemetry_flow_v1.md)

**Validation:** [rt_live_runtime_validation_v1.md](rt_live_runtime_validation_v1.md)  
**Maintainer smoke:** `scripts/rt/rt_live_smoke.py` (not UI-invoked)

---

## 2. Live profile

| Surface | Behavior |
|---------|----------|
| UI selector | **Live Gazebo** (`runtime_profile: live`) |
| Bridge | `SESSION_RUNTIME_PROFILES` includes `live`; `start_session` enables adapter mode `live` |
| Default paths | Stub and mock_adapter unchanged |
| Background poll | 1 Hz (`LIVE_ADAPTER_BACKGROUND_POLL_HZ`) on live sessions |
| Preflight | Required before live `start_session`; fail-closed |

---

## 3. Preflight behavior

- Command: `check_live_runtime_preflight`
- Checks: `ros2` on PATH, `gz` (Gazebo Sim), `rt_sandbox_gz` ROS package
- UI: `useLiveRuntimePreflight` + Adapter Status live preflight panel
- Failure: `RUNTIME_UNAVAILABLE`; start blocked

---

## 4. Stop semantics

| Profile | UI stop / disconnect | Bridge command | Adapter |
|---------|---------------------|----------------|---------|
| stub / mock_adapter | Stop & unsubscribe | `stop_session` | Pause (attached) |
| **live** | Stop & unsubscribe | **`stop_sim`** | **Terminate Gazebo adapter** |

Copy: *"Live stop terminates the Gazebo adapter."* (BridgeConnectionBar, LifecycleControlBar)

Implementation: `sessionStop.ts` → `liveStopCommandType()` → `stopSessionForProfile()`

---

## 5. Telemetry flow (summary)

- Live background poll in `_tick_live_background_poll` + `pull_telemetry` tick
- `session_health` exposes `runtime_profile`, `live_background_poll_hz`, `last_live_poll_utc`
- UI auto-refresh (1 Hz default) drains subscription ring
- **Mirror freshness:** UI-only `fresh` / `stale` / `unavailable` (`mirrorFreshness.ts`)

---

## 6. Command flow (summary)

- Entity commands: `spawn_entity`, `move_entity`, `delete_entity` (+ aliases)
- Post-command: adapter telemetry poll → `entity_pose_mirror` publish
- UI: `doPull()` after successful entity command
- **Command health:** UI-only `command_ready` / `command_unavailable` (`liveCommandHealth.ts`)

---

## 7. Validation checklist

Sign-off checklist (A–E) in [rt_live_runtime_validation_v1.md](rt_live_runtime_validation_v1.md) §6.1:

- A Start session — adapter alive, poll active, command ready
- B Spawn attacker — mirror + Cesium without maintainer CLI
- C Move entity — pose update in mirror + Cesium
- D Delete entity — mirror + Cesium removal
- E Stop session — adapter terminated, clean teardown

Automated regression: live CommandHealth, mirrorFreshness, commandPathVisibility, sessionStop, live preflight, bridge live entity mirror tests.

---

## 8. Limitations

- **Host-dependent:** Full Gazebo stack required for live profile; CI uses fake adapter
- **Maintainer smoke not in UI:** `rt_live_smoke.py` is read-only hint only
- **Local loopback only:** Single bridge, ≤3 sessions (PLAT-RT-M2 cap unchanged)
- **No perception / tracks:** ROS allow-list blocks `/tracks/state`
- **Explanatory mirror:** `entity_pose_mirror` is not command authority
- **Poll rate cap:** Live background poll 1 Hz; UI pull may skip redundant polls within interval

---

## 9. Future work (not authorized by this freeze)

- CI `@pytest.mark.integration` gate for `rt_live_smoke.py`
- Live session recovery UX (adapter crash mid-session)
- Multi-entity live editing polish under rate limits
- Perception or `/tracks/state` bridge — **explicitly out of scope**

---

## 10. Governance audit (Steps 1–7)

| Boundary | Verdict |
|----------|---------|
| No perception integration | **Pass** |
| No `/tracks/state` integration | **Pass** (allow-list + UI channels unchanged) |
| No tactical execution changes | **Pass** |
| No SA import | **Pass** |
| No parser/topic changes | **Pass** |
| No Gazebo world redesign | **Pass** |
| Command authority = bridge registry | **Pass** |
| Additive-only live profile | **Pass** |

**Freeze verdict:** PLAT-RT-LIVE-GZ1 **frozen** — Web ↔ Gazebo Live Runtime V1 complete.

---

## 11. Key artifact surfaces

| Area | Paths |
|------|-------|
| Bridge preflight | `live_preflight.py`, `governance.py` |
| Bridge session | `session_manager.py`, `session_record.py`, `telemetry_bridge.py` |
| UI profile / preflight | `sessionRuntimeProfile.ts`, `RuntimeProfileSelector.tsx`, `useLiveRuntimePreflight.ts` |
| UI live UX | `liveSessionUx.ts`, `sessionStop.ts`, `liveCommandHealth.ts`, `mirrorFreshness.ts` |
| UI status / workflow | `AdapterStatusPanel.tsx`, `SessionWorkflowStrip.tsx`, `RefreshControls.tsx` |
| Tests | `test_rt_live_preflight.py`, bridge live tests, UI live runtime test suites |
| Docs | `rt_live_*_v1.md`, this document |
