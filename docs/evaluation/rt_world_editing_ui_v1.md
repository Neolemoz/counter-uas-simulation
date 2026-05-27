# RT World Editing UI Contract (`rt_world_editing_ui_v1`)

**Phase:** PLAT-RT-T2 — drag/drop runtime world editing  
**Authority:** [rt_t2_world_editing_ui_plan.md](../platform/rt_t2_world_editing_ui_plan.md); [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md); [rt_authority_model_v1.md](rt_authority_model_v1.md)

Contract for interactive world editing in `platform/rt-sandbox-ui/`. Registry commands are command-authoritative; telemetry mirrors remain explanatory.

---

## 1. Transport

| Rule | Value |
|------|-------|
| Entity edits | `POST /v1/command` only |
| Telemetry data plane | `GET /v1/telemetry/pull` unchanged (pull-only) |
| Commands | `spawn_entity`, `move_entity`, `delete_entity` |
| `authority_scope` | `rt_sandbox_prototype` |
| `issued_by` | `rt_sandbox_ui` |

---

## 2. Governance banners

Persistent chrome (T1 banners plus editing banner when connected):

| Banner | Text |
|--------|------|
| Primary | `RT SANDBOX — experimental simulation; not operational state` |
| Secondary | `TRANSIENT RUNTIME ONLY — not replay authority` |
| Isolation | `NOT SA REPLAY AUTHORITY` |
| Editing | `WORLD EDITING ACTIVE` (when session connected) |

---

## 3. Entity catalog

| `entity_type` | Max/session | Default z |
|---------------|-------------|-----------|
| `radar` | 8 | 10 |
| `interceptor` | 8 | 10 |
| `drone` | 8 | 10 |
| `waypoint_marker` | 8 | 5 |
| **Total** | **32** | — |

---

## 4. World bounds

Mirror [`governance.py`](../../platform/rt-sandbox-bridge/rt_sandbox/governance.py) `WORLD_BOUNDS`:

| Axis | Min | Max |
|------|-----|-----|
| x | -500 | 500 |
| y | -500 | 500 |
| z | 0 | 200 |

Client must clamp poses before send; server validates with `INVALID_POSE`.

---

## 5. Editing interactions

| Interaction | Command | Preconditions |
|-------------|---------|---------------|
| Palette + click empty cell | `spawn_entity` | Session `running` or `paused`; under caps |
| Drag marker + release | `move_entity` | Entity selected; valid `entity_id` |
| Select + Delete | `delete_entity` | Entity selected |

Post-command: append local edit history → pull telemetry to reconcile mirror.

---

## 6. Editing cognition

Surface on editing panels:

| Field | Meaning |
|-------|---------|
| Command intent | Registry command — command authoritative |
| Mirror lag | Explanatory telemetry may lag until pull |
| `source` / `authority_label` | From mirror payload after pull |
| `sync_health` / `telemetry_health` | Stale/mismatch badges when adapter active |

---

## 7. Session safety

- Edits session-scoped only (require active `session_id`)
- Edit history cleared on disconnect — no persistence
- Disable editing when session not `running` or `paused`
- Debounce commands (respect bridge rate limits)

---

## 8. Explicit non-goals

- Cesium or geospatial map
- SA viewer integration
- Replay ingestion or federation UI
- Autonomous or distributed runtime
- New bridge commands or channels

---

## 9. T1 boundary

PLAT-RT-T1 provides read-only telemetry pull consumer. PLAT-RT-T2 adds entity editing via frozen RT-S3 commands — telemetry pull contract unchanged.
