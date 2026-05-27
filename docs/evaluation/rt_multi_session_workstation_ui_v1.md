# RT Multi-Session Workstation UI Contract (`rt_multi_session_workstation_ui_v1`)

**Phase:** PLAN-RT-M1 — multi-session UX planning (docs only)  
**Authority:** [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md); [rt_multi_session_governance_v1.md](rt_multi_session_governance_v1.md)

UX contract for multi-session RT workstation at `platform/rt-sandbox-ui/`. **Extends** PLAT-RT-T4 — does not replace frozen T1–T5 behaviors. **No implementation in PLAN-RT-M1.**

---

## 1. Transport (unchanged)

| Rule | Value |
|------|-------|
| Bridge host | Loopback only (`127.0.0.1:18765`) |
| Commands | `POST /v1/command` + new M2 commands `list_sessions`, `set_editing_session` |
| Telemetry | `GET /v1/telemetry/pull` per session |
| Active pull rate | ≤ 10 Hz |
| Background pull rate | ≤ 1 Hz |

---

## 2. Governance banners

**Existing five banners** from PLAT-RT-T3 — text unchanged.

**Additive sixth banner** (visible when ≥ 2 sessions connected):

`MULTI-SESSION — local prototype; not operational coordination`

Capture/handoff panel uses inline governance copy — not a seventh global banner.

---

## 3. Workstation layout (revised)

```
┌─ Governance header (5 + conditional 6th banner) ──┐
├─ Session rail ─────────────────────────────────────┤
│  [Tab A*] [Tab B] [Tab C] [+ New]  Connect/disconnect per tab │
│  Workflow strip (selected session)                 │
├─ Active workspace (selected tab) ──────────────────┤
│  World column | Viz column | Mirrors column        │
├─ Background diagnostics (collapsed accordion) ───────┤
│  B: running, 4 entities, sync stale                │
├─ Pipeline footer (selected session capture panel) ─┤
└─ Diagnostics (collapsible) ────────────────────────┘
```

| Zone | Contents | When visible |
|------|----------|--------------|
| Governance header | Five + optional sixth banner | Always |
| Session rail tabs | Up to 3 tabs + New button | Always when bridge reachable |
| Session rail strip | Workflow signals for **selected** session | Selected session connected |
| Active workspace | T4 zones bound to selected session | Selected session connected |
| Background diagnostics | Compact row per non-selected connected session | ≥ 2 sessions connected |
| Pipeline footer | Capture/handoff for selected session | Always |
| Diagnostics | Collapsible UI diagnostics | Always |

---

## 4. Session tabs

| Element | Behavior |
|---------|----------|
| Tab label | Short session id suffix + lifecycle badge |
| Active indicator | `*` or highlight on selected tab |
| Editing indicator | Lock icon on tab holding `editingSessionId` |
| `+ New` | Starts new session when `non_terminal_count < 3` |
| At capacity | `+ New` disabled with tooltip: capacity reached |
| Tab click | Promote to selected + call `set_editing_session` |
| Tab close | Per-session disconnect (unsubscribe + stop/discard) |
| Unsaved mirror edits | Confirm dialog before tab switch (M2) |

---

## 5. Active workspace

Binds all T4 zones to `selectedSessionId`:

| Zone | Binding |
|------|---------|
| World column (SVG) | Selected session entities + editing lock |
| Viz column (Cesium) | Selected session markers + editing lock |
| Mirrors column | Selected session telemetry at full pull rate |
| Workflow strip | Selected session lifecycle + pull fault |
| Capture panel | Selected session capture readiness |

T1–T5 frozen behaviors preserved within selected session context.

---

## 6. Background diagnostics

Collapsed accordion listing non-selected connected sessions:

| Field per row | Source |
|---------------|--------|
| Session id suffix | Registry |
| Lifecycle state | `lifecycle_state` / `session_health` at 1 Hz |
| Entity count | `world_summary.entity_count` |
| Sync/telemetry health | Compact stale chip |
| Pull fault | Row-level indicator if `lastError` |

No full telemetry panels, Cesium view, or editing controls for background sessions.

---

## 7. Session workflow strip (selected session)

Same signals as [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md) §4, scoped to selected session:

| Signal | Source |
|--------|--------|
| Connection | Selected session connected |
| Lifecycle state | Selected session pull |
| Sim paused | Selected session `clock_mirror` |
| Editing allowed | Selected == editing + lifecycle |
| Multi-session count | `non_terminal_count` from `list_sessions` |
| Pull fault | Selected session `lastError` |

---

## 8. Session safety

- Each session maintains independent pull loop and snapshot state
- Per-tab disconnect clears only that session's UI state
- No `localStorage`, IndexedDB, or export-to-disk (unchanged)
- No SA imports or replay bundle loading (unchanged)
- No cross-session entity comparison or diff views

---

## 9. Explicit non-goals

- SA viewer integration
- Multi-bridge session picker
- Live staging status API
- Automatic SA import
- Federation UI
- Tactical/HITL/ops-dashboard semantics
- Session persistence across page reload

---

## 10. T1–T5 boundary

PLAN-RT-M1 **extends layout for multi-session** only. Frozen per-session behaviors preserved:

- T1: five telemetry channels, pull consumer, governance banners
- T2: SVG authoritative editing (editing session only)
- T3: Cesium mirror (selected session)
- T4: workstation zones, cognition hub, capture panel
- T5: Cesium interactive editing (editing session only)

---

## Related

- [rt_m1_multi_session_architecture_plan.md](../platform/rt_m1_multi_session_architecture_plan.md)
- [rt_multi_session_telemetry_routing_v1.md](rt_multi_session_telemetry_routing_v1.md)
- [rt_multi_session_editing_ownership_v1.md](rt_multi_session_editing_ownership_v1.md)
