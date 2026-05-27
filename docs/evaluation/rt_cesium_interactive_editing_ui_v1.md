# RT Cesium Interactive Editing UI Contract (`rt_cesium_interactive_editing_ui_v1`)

**Phase:** PLAT-RT-T5 — Cesium interactive editing  
**Authority:** [rt_t5_cesium_interactive_editing_plan.md](../platform/rt_t5_cesium_interactive_editing_plan.md); [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md); [rt_authority_model_v1.md](rt_authority_model_v1.md)

Contract for interactive Cesium editing at `platform/rt-sandbox-ui/`. Supersedes PLAT-RT-T3 non-goal “Cesium-based entity editing” for edit gestures only; registry commands remain authoritative.

---

## 1. Transport

| Rule | Value |
|------|-------|
| Bridge host | Loopback only (`127.0.0.1:18765`) |
| Commands | `POST /v1/command` — `spawn_entity`, `move_entity`, `delete_entity` only |
| Telemetry data plane | `GET /v1/telemetry/pull` **only** |
| Max pull rate | **10 Hz** |

No bridge protocol changes in PLAT-RT-T5.

---

## 2. Required governance banners

When session connected (six banners):

| Banner | Text |
|--------|------|
| Primary | `RT SANDBOX — experimental simulation; not operational state` |
| Secondary | `TRANSIENT RUNTIME ONLY — not replay authority` |
| Isolation | `NOT SA REPLAY AUTHORITY` |
| World editing | `WORLD EDITING ACTIVE` |
| Cesium | `CESIUM RUNTIME VIEW` |
| Interactive | `INTERACTIVE EDITING` |

---

## 3. Edit gestures (Cesium)

| Gesture | Command | Notes |
|---------|---------|-------|
| Click empty globe | `spawn_entity` | Uses palette `selectedType`; `canSpawn` + `clampPose` |
| Click marker | Select only | Shared `selectedEntityId` with SVG |
| Drag marker | `move_entity` on release | Camera controller disabled during drag |
| Delete selected | `delete_entity` | Button + keyboard Delete (App) |

Entity types: `radar`, `interceptor`, `drone`, `waypoint_marker`.

---

## 4. Dual edit surfaces

| Surface | Role |
|---------|------|
| SVG grid (T2) | Retained — same registry commands |
| Cesium globe (T5) | Interactive co-editor for same session |

Both surfaces share selection and command pipeline in `App.tsx`. Telemetry mirrors remain explanatory.

---

## 5. Camera helpers

| Control | Scope |
|---------|-------|
| Reset to bounds | Local inspect |
| Focus selected entity | Local inspect |
| Follow selected | Optional toggle; default off; cleared on disconnect |

No bridge commands for camera state.

---

## 6. Editing cognition

Cesium panel must surface:

- Command intent and result (registry authoritative)
- Mirror lag / pending reconcile
- `source`, `authority_label`, `sync_health`, `telemetry_health`, stale indicators from pull payloads

---

## 7. Session safety

- Editing only when `isEditingAllowed(sessionState)` (`running` / `paused`)
- Bounds ±500 m x/y, z 0–200; entity caps mirrored from governance
- Handlers destroyed on viewer teardown / disconnect
- No persistence across sessions

---

## 8. Explicit non-goals

- SA viewer integration
- New bridge commands or channels
- Replay ingestion / federation UI
- Browser→ROS direct control
- Tactical overlays; multi-session UI
- Autonomous or distributed runtime

---

## 9. T3 boundary

PLAT-RT-T3 provided read-only Cesium mirror. PLAT-RT-T5 adds interactive editing via existing entity commands only — pull contract unchanged.

---

## Related

- [rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md)
- [rt_world_editing_ui_v1.md](rt_world_editing_ui_v1.md)
- [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md)
