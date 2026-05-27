# RT Telemetry UI Contract (`rt_telemetry_ui_v1`)

**Phase:** PLAT-RT-T1 — runtime telemetry UI foundations  
**Authority:** [rt_t1_telemetry_ui_plan.md](../platform/rt_t1_telemetry_ui_plan.md); [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md); [rt_authority_model_v1.md](rt_authority_model_v1.md)

Contract for the RT-only browser telemetry surface at `platform/rt-sandbox-ui/`. Explanatory telemetry only — not replay authority.

---

## 1. Transport

| Rule | Value |
|------|-------|
| Bridge host | Loopback only (`127.0.0.1:18765`) |
| Commands | `POST /v1/command` (session lifecycle, subscribe/unsubscribe) |
| Telemetry data plane | `GET /v1/telemetry/pull` **only** — no WebSocket, no SSE |
| Max pull rate | **10 Hz** (`telemetry_update_rate_cap_hz`) |
| Dev proxy | Vite forwards `/v1/*` to bridge (same-origin to UI) |

---

## 2. Required governance banners

Persistent chrome on all UI routes:

| Banner | Text |
|--------|------|
| Primary | `RT SANDBOX — experimental simulation; not operational state` |
| Secondary | `TRANSIENT RUNTIME ONLY — not replay authority` |
| Isolation | `NOT SA REPLAY AUTHORITY` |

Per-event payloads may also carry embedded `governance_banner` from bridge — display in cognition strip when present.

---

## 3. Subscribed channels

All five frozen PLAT-RT-S4 channels:

| Channel | Display panel |
|---------|---------------|
| `lifecycle_state` | Session lifecycle |
| `session_health` | Session health |
| `world_summary` | World summary |
| `entity_pose_mirror` | Entity pose mirror + 2D grid |
| `clock_mirror` | Clock mirror |

---

## 4. Telemetry cognition fields

Every panel must surface cognition metadata from enriched payloads ([rt_authority_model_v1.md](rt_authority_model_v1.md)):

| Field | Meaning |
|-------|---------|
| `source` | Payload `source`: `bridge_session`, `bridge_registry`, `adapter_feedback`, `adapter_telemetry` |
| `authority_label` | `command_authoritative`, `explanatory_sync`, `explanatory_telemetry`, `replay_boundary_scoped` |
| `sync_health` | On `world_summary` when pose sync active: `ok`, `stale`, `mismatch`, `feedback_lost` |
| `telemetry_health` | On adapter-fed channels: `ok`, `stale`, `feedback_lost` |
| Stale indicators | Badge when `telemetry_health === "stale"` or `sync_health !== "ok"` |

**Reviewer copy:**

- **Explanatory telemetry** — transient runtime mirror; not replay authority
- **Command authoritative** — bridge registry/session command truth
- **Explanatory sync** — PoseSyncMirror drift/stale evidence only

---

## 5. Panel requirements

### Session lifecycle

- `state`, `previous_state`, `command_type` from `lifecycle_state` channel

### Session health

- Runtime health fields from `session_health` channel
- Adapter fields when `enable_gazebo_adapter=true`

### World summary

- `entity_count`, `revision`, optional `sync_health`, `telemetry_health`, `world_revision_hint` (labeled explanatory)

### Entity pose mirror

- Entity table: `entity_id`, `entity_type`, `pose`
- 2D ASCII-style grid (read-only; no drag/drop)

### Clock mirror

- `paused`, sim clock fields from adapter or stub

---

## 6. Allowed UI tooling

- Manual refresh button
- Interval pull toggle (default 1 Hz, max 10 Hz)
- UI diagnostics: last pull UTC, drained count, subscription id, bridge URL
- Footer link to maintainer audit: `scripts/rt/rt_adapter_inspect.py telemetry-status`

---

## 7. Explicit non-goals

- Cesium or geospatial map
- Drag/drop entity editing
- SA viewer integration or shared components
- Replay ingestion or federation UI
- Multi-session management
- Tactical/operational dashboard lexicon ([rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) §7)

---

## 8. S4 boundary

PLAT-RT-S4 provides CLI ASCII viz (`scripts/rt/rt_telemetry_viz.py`). PLAT-RT-T1 is the **browser consumer** over the same pull contract — no new bridge channels or semantics.
