# RT Cesium Runtime UI Contract (`rt_cesium_runtime_ui_v1`)

**Phase:** PLAT-RT-T3 — Cesium runtime visualization  
**Authority:** [rt_t3_cesium_runtime_visualization_plan.md](../platform/rt_t3_cesium_runtime_visualization_plan.md); [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md); [rt_authority_model_v1.md](rt_authority_model_v1.md)

Contract for the RT-only Cesium 3D surface at `platform/rt-sandbox-ui/`. Explanatory mirror visualization only — not replay authority.

---

## 1. Transport

| Rule | Value |
|------|-------|
| Bridge host | Loopback only (`127.0.0.1:18765`) |
| Commands | `POST /v1/command` (unchanged — entity ops via T2 SVG) |
| Telemetry data plane | `GET /v1/telemetry/pull` **only** |
| Max pull rate | **10 Hz** |
| Dev proxy | Vite forwards `/v1/*` to bridge |

No bridge protocol changes in PLAT-RT-T3.

---

## 2. Required governance banners

Persistent chrome when session connected:

| Banner | Text |
|--------|------|
| Primary | `RT SANDBOX — experimental simulation; not operational state` |
| Secondary | `TRANSIENT RUNTIME ONLY — not replay authority` |
| Isolation | `NOT SA REPLAY AUTHORITY` |
| World editing | `WORLD EDITING ACTIVE` |
| Cesium | `CESIUM RUNTIME VIEW` |

Panel-local caveat inside Cesium panel: scenario-local fictional georef — not deployed geography.

Per-event `governance_banner` from pull payloads displayed in cognition strips when present.

---

## 3. Cesium view requirements

| Requirement | Notes |
|-------------|-------|
| Globe / scene | Cesium `Viewer` with OSM imagery + ellipsoid terrain (no Ion token) |
| Fictional georef | Fixed RT anchor in `src/cesium/constants.ts`; ENU meters from world bounds |
| Entity markers | Synced from `entity_pose_mirror` pull snapshot only |
| World bounds | Visual overlay for ±500 m x/y, z 0–200 |
| Session overlays | Bounds + labels scoped to active `session_id` |
| Camera controls | Orbit/zoom; reset-to-bounds helper (local only) |
| Viewer lifecycle | `viewer.destroy()` on disconnect or session change |

Cesium does **not** issue registry commands in PLAT-RT-T3; T2 SVG grid remains the edit surface.

---

## 4. Telemetry cognition fields

Cesium panel must surface cognition from enriched payloads ([rt_authority_model_v1.md](rt_authority_model_v1.md)):

| Field | Meaning |
|-------|---------|
| `source` | `bridge_session`, `bridge_registry`, `adapter_feedback`, `adapter_telemetry` |
| `authority_label` | `command_authoritative`, `explanatory_sync`, `explanatory_telemetry`, etc. |
| `sync_health` | On `world_summary` when pose sync active |
| `telemetry_health` | On adapter-fed channels |
| Stale indicators | When health fields not `ok` |

Dual strips: `entity_pose_mirror` and `world_summary` on the Cesium panel.

---

## 5. Camera helpers / visualization toggles

Allowed (local inspect only):

- Reset camera to world bounds center
- Toggle bounds overlay visibility
- Toggle entity label visibility

Forbidden: bridge commands for camera state; persistence of camera pose across sessions.

---

## 6. Session safety

- Rendering scoped to active session only
- Clear Cesium entities and destroy viewer on disconnect
- No `localStorage`, IndexedDB, or export-to-disk from UI
- No SA imports or replay bundle loading

---

## 7. Explicit non-goals

- SA viewer integration or shared Cesium modules
- Replay ingestion or federation UI
- Autonomous or distributed runtime
- New bridge commands or channels
- Cesium-based entity editing (T2 SVG authoritative for edits)
- Tactical overlays; readiness scoring; multi-session UI
- Cesium Ion / operational geography

---

## 8. T1/T2 boundary

PLAT-RT-T1 provides pull telemetry consumer. PLAT-RT-T2 provides SVG entity editing via frozen RT-S3 commands. PLAT-RT-T3 adds read-only 3D mirror visualization — telemetry pull contract and entity command set unchanged.

---

## 9. PLAT-RT-V1 additive (visualization fidelity)

**Phase:** PLAT-RT-V1  
**Authority:** [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md)

| Addition | Rule |
|----------|------|
| Markers | Larger points; labels include short `entity_id`; selection ring |
| Bounds | Vertical edges + top ring + corner labels |
| Camera | Fit entities, tight bounds, session-switch fly (local) |
| Session strip | Active session id + accent in Cesium panel |
| Multi-globe | Still **forbidden** — chrome-only session identity |
