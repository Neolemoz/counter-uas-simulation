# RT Runtime Workstation UI Contract (`rt_runtime_workstation_ui_v1`)

**Phase:** PLAT-RT-T4 — runtime session workspace UX polish  
**Authority:** [rt_t4_runtime_session_workspace_plan.md](../platform/rt_t4_runtime_session_workspace_plan.md); [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md); [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md)

Contract for the RT-only runtime workstation layout at `platform/rt-sandbox-ui/`. UX and workflow visibility only — not replay authority.

---

## 1. Transport

| Rule | Value |
|------|-------|
| Bridge host | Loopback only (`127.0.0.1:18765`) |
| Commands | `POST /v1/command` (unchanged — start/stop/subscribe + T2 entity ops) |
| Telemetry data plane | `GET /v1/telemetry/pull` **only** |
| Max pull rate | **10 Hz** |
| Dev proxy | Vite forwards `/v1/*` to bridge |

No bridge protocol changes in PLAT-RT-T4.

---

## 2. Required governance banners

Same five persistent banners as PLAT-RT-T3 ([rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md) §2). Presentation may be polished (layout, contrast); **text unchanged**.

Capture/handoff panel uses **inline** governance copy — not a sixth global banner.

---

## 3. Workstation zones

| Zone | Contents | When visible |
|------|----------|--------------|
| Governance header | Five banners | Always |
| Page header | Runtime workstation title + transient caveat | Always |
| Session rail | Connect bar, refresh controls, workflow strip | Always (strip reflects connection) |
| World column | Palette, SVG grid, editing cognition, edit history | Connected |
| Viz column | Cesium runtime panel | Connected |
| Mirrors column | Telemetry panels + runtime cognition hub | Connected (full); disconnected shows idle card |
| Pipeline footer | Capture/handoff workflow panel | Always |
| Diagnostics | Collapsible UI diagnostics | Always (collapsed by default when connected) |

---

## 4. Session workflow strip

Must surface (from pull + UI state):

| Signal | Source |
|--------|--------|
| Connection | UI `connected` |
| Lifecycle state | `lifecycle_state` or `session_health` payload `state` |
| Sim paused | `clock_mirror.paused` |
| Editing allowed | Derived from lifecycle + `isEditingAllowed` |
| Pull fault | UI `lastError` when set |

---

## 5. Runtime cognition hub

Consolidated read-only summary for channels:

- `world_summary`
- `session_health`
- `entity_pose_mirror`

Per channel: `source`, `authority_label`, descriptions, `sync_health` / `telemetry_health` badges, stale highlight.

Individual telemetry panels may omit duplicate full cognition strips when hub is visible (panel-specific cognition retained where needed, e.g. Cesium mirror lag).

---

## 6. Capture / handoff panel

| Field | Authority in T4 |
|-------|-----------------|
| Session capture readiness | **Explanatory** — derived from lifecycle ([rt_capture_continuity_v1.md](rt_capture_continuity_v1.md)) |
| Normalized / handoff pipeline | **Workflow map + maintainer CLI names** — not live staging filesystem status |
| Staging truth | Maintainer CLIs only; browser does not read `runs/rt_sandbox/captures/` or `sa_handoff/` |

Pipeline phases (read-only checklist): Capture → Normalize → Review → Approve → Prepare → Import per [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md).

Handoff event names (`handoff_ready`, etc.) may appear as **documentation chips** — not polled from export audit.

**Forbidden in browser:** `capture_session`, `rt_sa_import commit`, SA corpus writes.

---

## 7. Session safety

- Workstation scoped to single active session
- Clear state on disconnect (unchanged T1–T3 behavior)
- No `localStorage`, IndexedDB, or export-to-disk from UI
- No SA imports or replay bundle loading

---

## 8. Explicit non-goals

- SA viewer integration
- New bridge commands or telemetry channels
- Live staging status API or filesystem polling
- Automatic SA import
- Federation UI
- Multi-session workstation
- Tactical/HITL/ops-dashboard semantics

---

## 9. T1–T3 boundary

PLAT-RT-T4 **extends layout only**. Frozen behaviors preserved:

- T1: five telemetry channels, pull consumer, governance banners
- T2: SVG authoritative editing, entity commands
- T3: Cesium read-only mirror, fifth banner when connected

---

## 11. PLAT-RT-V1 additive (visualization fidelity)

**Phase:** PLAT-RT-V1  
**Authority:** [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md)

| Change | Rule |
|--------|------|
| Cognition hub/strips | Stronger authority/health/stale presentation |
| Background diagnostics | Telemetry health badges + session accent |
| Session tabs | Accent bar + full id tooltip |
| Workflow strip | `sessions: n/3` |
| Governance header | Layout readability only — banner text frozen |

---

## 10. PLAT-RT-SA2 additive (handoff mirror)

**Phase:** PLAT-RT-SA2  
**Authority:** [rt_sa2_multi_session_handoff_ui_v1.md](rt_sa2_multi_session_handoff_ui_v1.md)

| Change | Rule |
|--------|------|
| Sixth global banner | `MANUAL HANDOFF ONLY` in `BASE_BANNERS` |
| Capture/handoff panel | Live rows via `list_capture_handoff_status` (read-only) |
| Multi-session overview | Per-slot capture counts and `workflow_phase` badges |
| §8 non-goal override | Staging mirror API allowed; import/corpus writes still forbidden |

---

## Related

- [rt_telemetry_ui_v1.md](rt_telemetry_ui_v1.md)
- [rt_world_editing_ui_v1.md](rt_world_editing_ui_v1.md)
- [rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md)
- [rt_sa_import_bridge_v1.md](rt_sa_import_bridge_v1.md)
- [rt_sa2_multi_session_handoff_ui_v1.md](rt_sa2_multi_session_handoff_ui_v1.md)
