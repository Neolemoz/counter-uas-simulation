# RT Live Runtime Validation v1 (`rt_live_runtime_validation_v1`)

**Phase:** Web ↔ Gazebo Live Runtime — Step 6 end-to-end validation  
**Authority:** docs-only validation artifact; no schema/parser/topic changes  
**Prerequisites:** Steps 1–5 complete

Related contracts:

- [rt_live_telemetry_flow_v1.md](rt_live_telemetry_flow_v1.md) — Gazebo → mirror → Cesium
- [rt_live_command_flow_v1.md](rt_live_command_flow_v1.md) — Web → Gazebo commands
- Maintainer CLI: `scripts/rt/rt_live_smoke.py`
- Golden trace reference: [fixtures/rt_sandbox/live_smoke_trace_v1.json](../../fixtures/rt_sandbox/live_smoke_trace_v1.json)

---

## 1. Validation scope

### In scope

| Layer | Path |
|-------|------|
| Command | Web → Bridge → Adapter → Gazebo |
| Telemetry | Gazebo → Adapter → Bridge → `entity_pose_mirror` → Cesium |
| Session | `start_session` (live profile), `stop_sim` / disconnect |
| UX (read-only) | Preflight, command health, mirror freshness, adapter status |

### Out of scope (frozen)

- Perception / `/tracks/state` integration
- Tactical execution or assignment authority changes
- SA import / replay authority
- Parser or topic contract changes
- Gazebo world redesign

---

## 2. Validation methods

| Method | When | Command / surface |
|--------|------|-------------------|
| **Automated unit** | CI / pre-merge | `npm test` live runtime suites; `pytest` bridge live tests |
| **Maintainer smoke** | Full stack on host with ROS 2 + Gazebo | `python3 scripts/rt/rt_live_smoke.py` |
| **Interactive UI** | Reviewer / maintainer manual pass | RT sandbox UI, loopback bridge `:18765` |

UI validation does **not** invoke `rt_live_smoke.py`; smoke hint is read-only in Adapter Status panel.

---

## 3. Live smoke checklist (A–E)

### A. Start session

**Action (UI):** Select **Live Gazebo** → **Start session & subscribe**  
**Action (CLI smoke):** `start_sim` or `start_session` with `runtime_profile: live`

| Check | Expected outcome | Where to observe |
|-------|------------------|------------------|
| Preflight | `ros2`, `gz`, `rt_sandbox_gz` on PATH (live only) | Adapter Status → Live preflight |
| Session lifecycle | `running` | Workflow strip → `lifecycle: running` |
| Live profile | Active | Workflow → **Live runtime connected**; profile badge |
| Adapter alive | `adapter_alive: true`, mode `live` | Adapter Status → launch health **launched** |
| Poll active | `live_background_poll_hz` > 0, `last_live_poll_utc` advances | Adapter Status → live poll rate / last live poll |
| Command path | **Command path: ready** when editing enabled | Workflow strip + Adapter Status |
| Subscribe | `entity_pose_mirror` in channel set | UI diagnostics / initial pull |

**Failure modes:** preflight fail → `RUNTIME_UNAVAILABLE`; adapter attach fail → `adapter_alive: false`, command unavailable  
**Recovery:** Fix host stack (ROS/GZ package); retry connect; check `rt_live_smoke.py` preflight_cleanup

---

### B. Spawn attacker

**Action:** Entity bar → **Spawn attacker** (or Cesium spawn gesture)

| Check | Expected outcome | Where to observe |
|-------|------------------|------------------|
| Bridge response | `ok: true`, `entity_id` returned | UI error strip clear; edit history entry |
| Registry | `world_summary.entity_count` +1 | World summary panel |
| Adapter sync | `adapter_sync` in audit (maintainer) | — |
| Mirror | New entity in `entity_pose_mirror.entities` | Entity Pose Mirror panel; mirror **fresh** |
| Cesium | Marker appears without manual CLI | Globe entity markers |
| Post-command pull | Auto `doPull()` after command | `last_pull_utc` updates |

**Failure modes:** `INVALID_STATE`, `RESOURCE_LIMIT_EXCEEDED`, `SYNC_*` errors, command rate limit  
**Recovery:** Confirm lifecycle `running`/`paused`, editing enabled, command path ready; retry after pull

---

### C. Move entity

**Action:** Select entity → drag on Cesium / SVG grid → release

| Check | Expected outcome | Where to observe |
|-------|------------------|------------------|
| Bridge response | `move_entity` OK | Edit history |
| Mirror | Updated `pose` for entity id | Entity Pose Mirror; `telemetry_revision` may bump |
| Cesium | Marker moves to new position | Globe (after pull / auto-refresh) |
| Mirror freshness | **Mirror: fresh** (recent event timestamp) | Adapter Status freshness badges |

**Failure modes:** stale mirror (>2× poll interval), editing blocked, adapter down  
**Recovery:** Enable auto-refresh; verify live background poll; re-select editing session tab

---

### D. Delete entity

**Action:** Select entity → **Delete selected** (or Delete key)

| Check | Expected outcome | Where to observe |
|-------|------------------|------------------|
| Bridge response | `delete_entity` OK | Edit history |
| Mirror | Entity absent from `entities[]` | Entity Pose Mirror panel |
| Cesium | Marker removed | Globe |
| World summary | `entity_count` decremented | World summary panel |

**Failure modes:** entity still in mirror if pull stale; local mirror overlay until reconcile  
**Recovery:** **Refresh now** or wait for auto-refresh; confirm `pending_reconcile` clears

---

### E. Stop session

**Action (UI):** **Stop & unsubscribe** or lifecycle **Stop session** (live → `stop_sim`)  
**Action (CLI smoke):** `stop_sim` → optional `discard_session`

| Check | Expected outcome | Where to observe |
|-------|------------------|------------------|
| Stop semantics | Live stop **terminates Gazebo adapter** | Copy: *Live stop terminates the Gazebo adapter* |
| Lifecycle | `stopped` | Workflow strip |
| Adapter | `adapter_alive: false` / terminated | Adapter Status after final pull |
| Orphans (maintainer) | No `ros2 launch rt_sandbox_gz` / `gz sim` orphans | `rt_live_smoke.py` post_orphans check |
| Cleanup | `discard_session` evicts slot | Session tab closed |

**Failure modes:** pause-only stop if stub/mock path used on live session by mistake; orphan Gazebo processes  
**Recovery:** Use disconnect/stop_sim for live; run smoke preflight_kill_stale_orphans; `discard_session`

---

## 4. Runtime health review (during validation)

Monitor read-only derived UX (no schema changes):

| Signal | Healthy | Degraded | Unavailable |
|--------|---------|----------|-------------|
| **Command health** | Command path: ready | Command path: unavailable (lifecycle/edit/adapter) | N/A (non-live profile) |
| **Mirror freshness** | Mirror: fresh | Mirror: stale (old event or `telemetry_health: stale`) | Mirror: unavailable / feedback_lost |
| **Live poll** | Live poll: fresh | Live poll: stale / pending | Live poll: n/a (non-live) |
| **Adapter launch** | launch health: launched | not_running | preflight missing |
| **UI pull** | UI pull: current | UI pull: stale | No pull yet |

Review surfaces: **Session Workflow strip**, **Adapter Status panel**, **Entity Pose Mirror panel**, Cesium marker motion.

---

## 5. Failure handling review

### Preflight failure

- **Trigger:** `check_live_runtime_preflight` fails before `start_session` (live profile)
- **UI:** Live preflight chips **missing**; start blocked with `RUNTIME_UNAVAILABLE`
- **Recovery:** Install/s source ROS 2, Gazebo Sim (`gz`), build `rt_sandbox_gz`; re-select Live Gazebo

### Adapter launch failure

- **Trigger:** `runtime.start()` / adapter worker exit
- **UI:** `adapter_alive: false`, command unavailable, launch health **not_running**
- **Recovery:** Inspect bridge audit; run `rt_live_smoke.py`; check orphan cleanup; retry session

### Stale telemetry

- **Trigger:** Mirror event age > threshold or bridge `telemetry_health: stale`
- **UI:** Mirror: stale badge; optional stale tactical overlay flag (display-only)
- **Recovery:** Confirm auto-refresh on; live background poll active; manual **Refresh now**

### Command unavailable

- **Trigger:** Stopped lifecycle, editing lock, inactive tab, or adapter not alive
- **UI:** Command path: unavailable with detail tooltip
- **Recovery:** Resume session; select editing tab; reconnect live session if adapter terminated

---

## 6. Validation artifacts

### 6.1 Checklist (copy for maintainer sign-off)

```
[ ] A  Start session — live profile, adapter alive, poll active, command ready
[ ] B  Spawn attacker — mirror + Cesium marker without maintainer CLI
[ ] C  Move entity — pose update in mirror + Cesium
[ ] D  Delete entity — mirror empty for id + Cesium removal
[ ] E  Stop session — adapter terminated, no Gazebo orphans (maintainer pgrep)
[ ]    Workflow strip shows live connected + command health during A–D
[ ]    Mirror freshness fresh during active polling
[ ]    rt_live_smoke.py ok (optional full-stack host)
```

### 6.2 Review notes (Step 6 automated pass — 2026-06-05)

**Wiring review (no defects found):**

- Command path: `entityCommands.ts` → bridge `handle_entity` → adapter sync → post-command telemetry poll → `entity_pose_mirror` publish; UI `doPull()` on success ([rt_live_command_flow_v1.md](rt_live_command_flow_v1.md)).
- Telemetry path: live background poll on `pull_telemetry` + 1 Hz UI auto-refresh → Cesium `syncEntityMarkers` ([rt_live_telemetry_flow_v1.md](rt_live_telemetry_flow_v1.md)).
- Live stop: UI `stop_sim` for live profile (Step 3).
- Derived UX: `liveCommandHealth.ts`, `mirrorFreshness.ts` — UI-only, no schema changes.

**Automated validation executed:**

| Suite | Result |
|-------|--------|
| `npm test` — liveCommandHealth, commandPathVisibility, mirrorFreshness, SessionWorkflowStrip, sessionStop | 25/25 passed |
| `pytest` — live preflight, live entity mirror spawn/move/delete, pull→mirror, stop_sim terminate | 5/5 passed |

**Not executed in Step 6 (host-dependent):** Full `rt_live_smoke.py` against live ROS/Gazebo stack — requires maintainer environment.

### 6.3 Governance confirmation

| Boundary | Status |
|----------|--------|
| No parser/topic/schema changes in Steps 1–6 | Confirmed |
| No `/tracks/state` or perception bridge | Confirmed (ROS allow-list + UI channel set unchanged) |
| No tactical execution changes | Confirmed (visualization-only overlays unchanged) |
| No SA import / replay authority | Confirmed |
| Command authority remains bridge registry | Confirmed |
| `entity_pose_mirror` remains explanatory telemetry | Confirmed |
| Live profile additive-only (`runtime_profile: live`) | Confirmed |
| Maintainer smoke not invoked from UI | Confirmed |

---

## 7. Maintainer commands (reference)

```bash
# Full-stack smoke (requires ROS 2 + Gazebo + rt_sandbox_gz + loopback bridge)
python3 scripts/rt/rt_live_smoke.py --url http://127.0.0.1:18765/v1/command

# Bridge mirror inspection (optional)
python3 scripts/rt/rt_adapter_inspect.py telemetry-status

# Automated regression (no Gazebo required)
cd platform/rt-sandbox-ui && npm test -- --run src/runtime/liveCommandHealth.test.ts src/telemetry/commandPathVisibility.test.ts
python3 -m pytest src/counter_uas/test/test_rt_live_preflight.py \
  src/counter_uas/test/test_rt_sandbox_bridge.py::test_live_entity_commands_visible_in_pose_mirror -q
```

---

## 8. Step 7 handoff

See recommended Step 7 scope in validation summary (freeze doc, regression gate, optional CI integration mark for smoke).
