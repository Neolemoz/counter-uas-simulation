# RT V1 Runtime Visualization Contract (`rt_v1_runtime_visualization_v1`)

**Phase:** PLAT-RT-V1 — runtime visualization fidelity  
**Authority:** [rt_v1_runtime_visualization_fidelity_plan.md](../platform/rt_v1_runtime_visualization_fidelity_plan.md); [rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md)

Additive supplement for Cesium readability, camera presets, cognition presentation, and multi-session workstation chrome. Does not replace frozen T3/T5 transport or M2 registry semantics.

---

## 1. Core invariant

Visualization changes are **explanatory only**. Mirrors remain non-authoritative. One Cesium globe renders the **selected session** only.

---

## 2. Cesium visual fidelity

### 2.1 Markers (`rt_capture_handoff_row` N/A — entity markers)

| Requirement | Rule |
|-------------|------|
| Label text | `{glyph} {type} · {shortEntityId}` (8-char suffix) |
| Marker size | Base 12px; selected 16px; optional distance scale clamp |
| Selection | Distinct outline + optional ring entity |
| Health | ok / stale / warn colors; stale may use subtle alpha pulse |
| Command ghost | Labeled `cmd` point when drift ghost active |

### 2.2 Bounds overlay

| Layer | Rule |
|-------|------|
| Ground ring | Dashed polyline at z.min (existing) |
| Vertical edges | Four corners min→max z when `showVerticalBounds` |
| Top ring | Closed polyline at z.max |
| Corner labels | Optional `±500m`, `z 0–200` when bounds visible |
| Width / opacity | Higher contrast than T3 defaults |

### 2.3 Session strip

Cesium panel shows: `Viewing session {shortId}`, accent swatch, editing lock when applicable.

---

## 3. Camera helpers (local only)

| Preset | Behavior |
|--------|----------|
| `bounds` | Fly to bounds center + `DEFAULT_CAMERA_HEIGHT_M` |
| `tightBounds` | Fly to bounds center + ~1200m |
| `entities` | Bounding sphere of mirror entities + padding; fallback bounds |
| `focusEntity` | Existing per-entity fly |
| `follow` | Existing `trackedEntity` |
| Session switch | `flyOnSessionSwitch` after viewer create (0.8s) |

**Forbidden:** bridge commands; `localStorage` camera persistence.

---

## 4. Cognition overlays

| Surface | V1 addition |
|---------|-------------|
| `TelemetryCognitionStrip` | Larger chips; visible stale ring; optional `sessionContextLine` |
| `RuntimeCognitionHub` | Active session header; authority + health on one line per channel |
| `CesiumEditingCognitionStrip` | Stale when world or pose mirror stale |
| `BackgroundDiagnostics` | Compact health badges from diagnostic snapshots; accent dot |

Helper exports: `formatAuthorityChip`, `formatHealthChip`, `sessionContextLine` in `telemetry/cognition.ts`.

---

## 5. Multi-session visual identity (chrome only)

| Mechanism | Rule |
|-----------|------|
| `sessionVisualIdentity.ts` | Stable accent per slot index (amber / sky / violet) |
| `SessionTabBar` | Left accent bar; `title` = full session_id |
| `SessionWorkflowStrip` | `sessions: {n}/3` label |
| Cesium | Accent tint on bounds/markers for active session |

**Forbidden:** background session entities on globe.

---

## 6. Governance chrome

Banner **text** unchanged from T1/T3/T5/SA2. Layout readability improvements allowed (spacing, scroll, aria).

---

## 7. Explicit non-goals

- SA viewer; federation; replay ingestion
- New bridge commands or telemetry channels
- Tactical overlays; ops geography; Cesium Ion
- Autonomous / distributed runtime

---

## Related

- [rt_cesium_interactive_editing_ui_v1.md](rt_cesium_interactive_editing_ui_v1.md)
- [rt_multi_session_workstation_ui_v1.md](rt_multi_session_workstation_ui_v1.md)
