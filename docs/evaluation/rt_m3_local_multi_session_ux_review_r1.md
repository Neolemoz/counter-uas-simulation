# RT-M3 — Local Multi-Session UX Review R1

**Phase:** PLAN-RT-M3 — local multi-session polish (read-only)  
**Prerequisite:** PLAT-RT-M2 frozen; PLAN-RT-C1 frozen  
**Plan:** [rt_m3_local_multi_session_polish_plan.md](../platform/rt_m3_local_multi_session_polish_plan.md)  
**Governance review:** [rt_m3_governance_review_r1.md](rt_m3_governance_review_r1.md)  
**Poll policy:** [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md)  
**PLAT roadmap:** [rt_roadmap_plat_rt_m3_v1.md](rt_roadmap_plat_rt_m3_v1.md)  
**Freeze audit:** [rt_m3_freeze_audit.md](rt_m3_freeze_audit.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| M1/M2 core delivery | **Pass** — cap=3, registry, tabs, background diagnostics, editing lock |
| Background polling vs contract | **Pass-with-conditions** — 1 Hz diagnostic subset; scheduler refinements deferred |
| Session inspection | **Gap** — no `rt_session_inspect.py`; design ready for PLAT |
| Tab / workspace polish | **Pass-with-conditions** — M1 provisional items (rename, reorder, confirm) open |
| PLAT-RT-M3 scope clarity | **Pass** — bounded polish; no distributed drift |

**Recommendation:** Freeze **PLAN-RT-M3**. Authorize **PLAT-RT-M3** only after separate implementation freeze.

---

## 1. Background polling

### 1.1 Implementation map

| Contract rule ([rt_multi_session_telemetry_routing_v1.md](rt_multi_session_telemetry_routing_v1.md)) | M2 implementation | Verdict |
|---------------------------------------------------------------------------------------------------|-------------------|---------|
| Active ≤ 10 Hz | `pullHz` capped by `MAX_PULL_HZ` in [useRtSessionWorkspace.ts](../../platform/rt-sandbox-ui/src/hooks/useRtSessionWorkspace.ts) | **Pass** |
| Background ≤ 1 Hz | `BACKGROUND_PULL_HZ = 1`; per-slot throttle via `lastPullUtc` | **Pass** |
| Background diagnostic channels only | `DIAGNOSTIC_TELEMETRY_CHANNELS` on connect; `selectTab` resubscribes siblings to diagnostic | **Pass** |
| Per-session drain | `pullTelemetry({ sessionId, subscriptionId })` | **Pass** |
| Background stale isolated from active banners | Separate slot maps; `BackgroundDiagnostics` per row | **Pass** |

### 1.2 Scheduler behavior

Single `setInterval` tick (interval from active `pullHz`) iterates all connected slots and pulls when `Date.now() - lastPullUtc >= slotInterval`.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| M3-UX-POLL-01 | Pass | Background 1 Hz enforced per slot |
| M3-UX-POLL-02 | Pass-with-conditions | Global `setPulling(true)` on any pull — active UI may flicker during background pull |
| M3-UX-POLL-03 | Pass-with-conditions | No adaptive backoff when background repeatedly stale — fixed 1 Hz continues |
| M3-UX-POLL-04 | Pass-with-conditions | `doPull()` refreshes **active** session only — background not on manual refresh |
| M3-UX-POLL-05 | Pass-with-conditions | Poll continues when `BackgroundDiagnostics` accordion collapsed — PLAT may gate |

### 1.3 Stale UI

[BackgroundDiagnostics.tsx](../../platform/rt-sandbox-ui/src/workstation/BackgroundDiagnostics.tsx) shows lifecycle, entity count, handoff phase, fidelity label, compact health badges, and fault when `lastError` set. Aligns with M1 §6 background diagnostics table.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| M3-UX-POLL-06 | Pass | Per-row stale/fault; no spill to active cognition hub |
| M3-UX-POLL-07 | Pass-with-conditions | Optional `lastPullUtc` age chip not shown — PLAT enhancement |

---

## 2. Session inspection

### 2.1 Current state

| Capability | Status |
|------------|--------|
| `list_sessions` bridge command | **Delivered** — [session_registry_handlers.py](../../platform/rt-sandbox-bridge/rt_sandbox/session_registry_handlers.py) |
| UI `listSessions()` | [client.ts](../../platform/rt-sandbox-ui/src/bridge/client.ts) |
| Maintainer inspect CLI | **Missing** |
| Per-session audit log | `runs/rt_sandbox/audit/{session_id}.json` via [audit_log.py](../../platform/rt-sandbox-bridge/rt_sandbox/audit_log.py) |

### 2.2 PLAT CLI design

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| M3-UX-CLI-01 | Pass | PLAN specifies read-only `rt_session_inspect.py` — no new bridge command |
| M3-UX-CLI-02 | Pass | `list` / `summary` use existing HTTP `list_sessions` |
| M3-UX-CLI-03 | Pass | `audit` reads filesystem audit JSON — no session mutation |
| M3-UX-CLI-04 | Pass-with-conditions | `show` may combine registry snapshot + last audit tail — document in PLAT help |

---

## 3. Tab / workspace polish

### 3.1 SessionTabBar

[SessionTabBar.tsx](../../platform/rt-sandbox-ui/src/workstation/SessionTabBar.tsx):

- Short session id suffix, accent color, lifecycle via snapshots
- Active `*` highlight, editing lock icon
- Handoff badge (`✓handoff` / capture count)
- `+ New` disabled at capacity
- Per-tab close → `disconnectSession`

| M1 contract item | Status |
|------------------|--------|
| Tab label + lifecycle badge | **Delivered** |
| Editing lock indicator | **Delivered** |
| Tab reorder | **Not delivered** |
| Session display name | **Not delivered** |
| Unsaved mirror confirm on switch | **Not delivered** |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| M3-UX-TAB-01 | Pass | Core tab rail matches M1 §4 |
| M3-UX-TAB-02 | Pass-with-conditions | Reorder + rename deferred to PLAT P1 |
| M3-UX-TAB-03 | Pass-with-conditions | Dirty local mirror confirm absent — PLAT should add before aggressive tab switching |

### 3.2 Tab switch flow

`selectTab`: calls `set_editing_session`, resubscribes active to full channels, demotes siblings to diagnostic channels, updates slot roles.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| M3-UX-TAB-04 | Pass | Channel demotion on demote matches telemetry routing contract |
| M3-UX-TAB-05 | Pass | `set_editing_session` tracks selected tab in normal flow |

### 3.3 Background diagnostics

Collapsed `<details>` accordion when ≥2 sessions — matches [rt_multi_session_workstation_ui_v1.md](rt_multi_session_workstation_ui_v1.md) §6.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| M3-UX-TAB-06 | Pass | Background health visible without promoting tab |

---

## 4. Focus / editing lock

| Surface | Binding |
|---------|---------|
| SVG world editing | `editingSessionId` + selected session command routing |
| Cesium interactive edit | Active globe for selected session; background flat registry z note in diagnostics |
| Entity commands | Bridge rejects mutations when session not editing lock holder |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| M3-UX-EDIT-01 | Pass | One editing session enforced at bridge |
| M3-UX-EDIT-02 | Pass-with-conditions | Selected tab can differ from editing lock briefly during async `set_editing_session` — rare; PLAT may disable commands until settled |

---

## 5. PLAT-RT-M3 recommended scope

Prioritized backlog for [rt_roadmap_plat_rt_m3_v1.md](rt_roadmap_plat_rt_m3_v1.md):

| Priority | Item |
|----------|------|
| **P0** | `rt_session_inspect.py` (list, show, summary, audit) |
| **P0** | Per-slot `pulling` flag; `lastPullUtc` age in background rows |
| **P1** | Optional pause background poll when accordion collapsed |
| **P1** | Tab switch confirm for dirty local entity mirror |
| **P1** | Session display name (localStorage, RT-only) |
| **P2** | Tab reorder persistence |
| **P2** | “Refresh all connected” control |

**Out of scope:** distributed bridge, new telemetry channels, background Cesium globe, cross-session drag.

---

## 6. Architecture summary (M3)

```text
Loopback bridge (cap=3)
  ├── Active session: full pull ≤10 Hz, full workstation
  ├── Background sessions: diagnostic pull 1 Hz, accordion diagnostics
  └── Maintainer: rt_session_inspect (PLAT) — list_sessions + audit read
```

Single process, single HTTP listener, no cross-machine coordination.

---

## Appendix — File cross-reference

| Concern | Contract | Implementation | Tests |
|---------|----------|----------------|-------|
| Registry | `rt_multi_session_registry_v1` | `session_registry.py` | bridge pytest multi-session |
| Telemetry routing | `rt_multi_session_telemetry_routing_v1` | `useRtSessionWorkspace.ts` | `useRtSessionWorkspace.test.ts` |
| Workstation UI | `rt_multi_session_workstation_ui_v1` | `SessionTabBar`, `BackgroundDiagnostics` | `SessionTabBar.test.tsx` |
| Poll policy | `rt_multi_session_poll_policy_v1` | constants.ts + hook | PLAT adds policy tests |
| Inspect CLI | PLAN-RT-M3 | (PLAT) `rt_session_inspect.py` | PLAT CLI smoke |
