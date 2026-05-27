# RT Multi-Session Editing Ownership (`rt_multi_session_editing_ownership_v1`)

**Phase:** PLAN-RT-M1 — multi-session architecture (docs only)  
**Authority:** [rt_authority_model_v1.md](rt_authority_model_v1.md); [rt_world_editing_ui_v1.md](rt_world_editing_ui_v1.md); [rt_cesium_interactive_editing_ui_v1.md](rt_cesium_interactive_editing_ui_v1.md)

World editing ownership rules for local multi-session RT sandbox. **No implementation in PLAN-RT-M1.**

---

## 1. Authority model (unchanged)

| Surface | Authority |
|---------|-----------|
| `EntityRegistry` / `world.revision` | **Command-authoritative** |
| `entity_pose_mirror` telemetry | **Explanatory** |
| `PoseSyncMirror` | **Explanatory** |
| UI local entity mirror | **Optimistic overlay** — reconciled against authoritative registry |

Mirrors ≠ authority. See [rt_authority_model_v1.md](rt_authority_model_v1.md).

---

## 2. Editing session lock

Exactly **one** session holds the editing lock at any time.

| Component | Field | Rule |
|-----------|-------|------|
| Bridge | `editing_session_id` | Authoritative mutation target |
| UI | `editingSessionId` | Set via `set_editing_session` on tab promote |
| UI | `selectedSessionId` | Active workspace display; defaults equal to editing |

**Recommendation:** bridge-enforced lock (not UI-only). Bridge rejects entity mutations targeting a non-editing session even if UI misroutes.

---

## 3. Entity command preconditions (revised)

All of `spawn_entity`, `move_entity`, `delete_entity` require:

| # | Precondition |
|---|--------------|
| 1 | `session_id` exists in registry |
| 2 | Session state is `running` or `paused` |
| 3 | Runtime alive |
| 4 | `session_id == editing_session_id` |
| 5 | Payload valid per entity catalog and world bounds |

Failure on (4) → `EDITING_SESSION_MISMATCH`.

Template apply (`apply_runtime_template`) and workflow mutations follow the same editing lock.

---

## 4. Background session read-only rule

| Surface | Background session | Editing session |
|---------|-------------------|-----------------|
| SVG grid | Display-only (telemetry mirror) | Full edit |
| Cesium globe | Display-only | Full interactive edit |
| Entity palette | Disabled | Enabled when `isEditingAllowed` |
| Edit history | Read-only view of past edits | Live append |
| Local entity mirror | Frozen / not updated | Optimistic updates |

Background sessions may still receive lifecycle commands (`pause_session`, `stop_session`) scoped to their `session_id`.

---

## 5. Dual-surface editing (SVG + Cesium)

PLAT-RT-T5 dual-surface rules preserved:

- Both surfaces bind to the **same editing session**
- Tab switch with unsaved local mirror edits → confirmation dialog (M2 UI)
- Cesium camera focus/follow scoped to editing session entities only
- Cross-session entity ID display in diagnostics includes session prefix to avoid confusion

---

## 6. Local entity mirror (M2 requirement)

Current `localEntityMirror.ts` is single-session. M2 must:

- Key `localEntities` map by `sessionId`
- Key `locallyDeletedIds` by `sessionId`
- Clear mirror state on per-session disconnect
- Not leak optimistic state across tab switches

---

## 7. UI editing enablement

```text
isEditingAllowed =
  connected
  && sessionId === editingSessionId
  && sessionId === selectedSessionId
  && sessionState in { running, paused }
  && !commandBusy
```

Workflow strip shows editing lock holder when multiple sessions connected.

---

## 8. Explicit non-goals

- Concurrent editing across multiple sessions
- Cross-session entity drag/copy
- Shared entity registry across sessions
- Operational fleet or order-of-battle semantics

---

## Related

- [rt_multi_session_registry_v1.md](rt_multi_session_registry_v1.md)
- [rt_multi_session_workstation_ui_v1.md](rt_multi_session_workstation_ui_v1.md)
- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §11 (M1 additive)
