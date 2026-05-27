# RT Tactical Modes Contract (`rt_tac1_tactical_modes_v1`)

**Phase:** PLAN-RT-TAC1 — tactical controller architecture (docs only)  
**Authority:** [rt_tac1_tactical_controller_architecture_plan.md](../platform/rt_tac1_tactical_controller_architecture_plan.md); [rt_authority_model_v1.md](rt_authority_model_v1.md); [rt_s1_interactive_sandbox_architecture_plan.md](../platform/rt_s1_interactive_sandbox_architecture_plan.md) UX lexicon

Defines Manual, Assisted, and Autonomous tactical modes for the RT interactive sandbox. **No implementation in PLAN-RT-TAC1.**

---

## 1. Mode definitions

### 1.1 Manual

| Aspect | Rule |
|--------|------|
| **Authority** | User-issued bridge commands are `command_authoritative` for entity/world state |
| **Controller** | Inactive, or pass-through diagnostics only (feasibility/TTI display without commits) |
| **Assignment** | User explicitly selects and assigns candidates via future bridge verbs (PLAT-RT-TAC2) |
| **Adapter** | Receives only bridge-approved entity motion intents |

**Required UX vocabulary:** assign candidate, select for review, simulate pose (existing entity commands).

**Forbidden UX vocabulary:** engage, fire, strike, neutralize, mission approve, target lock (operational).

### 1.2 Assisted

| Aspect | Rule |
|--------|------|
| **Authority** | Controller recommendations are `tactical_recommendation_explanatory`; **user approval** is `user_approval_authoritative` before any adapter motion intent |
| **Controller** | Emits recommendation packets (candidate id, TTI, feasibility summary) — does not auto-apply |
| **Bridge** | Remains command gate; rejects adapter intents without recorded approval |
| **Assignment** | Commits only after explicit `approve_recommendation` (future verb, PLAT-RT-TAC3) |

**Required UX vocabulary:** suggest, recommend, simulate outcome, review assignment.

**Forbidden UX vocabulary:** auto-engage, autonomous intercept, operator authorize, weapon release.

### 1.3 Autonomous

| Aspect | Rule |
|--------|------|
| **Authority** | Tactical controller is `tactical_controller_authoritative` for sandbox loop within session resource caps |
| **Controller** | May commit `assigned_candidate_id` and schedule adapter intents per policy — still subject to bridge allow-list and rate limits |
| **User override** | Pause or revert to Manual at any time — Manual transition clears autonomous scheduling |
| **Bridge** | Enforces caps; does not run selection geometry itself |

**Required UX vocabulary:** simulate autonomous loop, sandbox auto-assign.

**Forbidden UX vocabulary:** C2, HITL, weapon release, operational readiness, command authority grant.

---

## 2. Authority matrix

| Surface | Manual | Assisted | Autonomous |
|---------|--------|----------|------------|
| Entity registry poses | User via bridge | User via bridge (after approval) | Controller via bridge dispatch |
| Recommendation packet | N/A | Explanatory | N/A (or optional advisory) |
| `assigned_candidate_id` | User commit | User after approval | Controller commit |
| `selected_candidate_id` | User highlight | User or controller highlight | Controller highlight |
| TTI / feasibility display | Explanatory | Explanatory | Explanatory |
| Engine `[TACTICAL_*]` logs | Explanatory (if engine running) | Explanatory | Explanatory |
| SA replay authority | **Never** — mode is RT-session scoped only | Same | Same |

---

## 3. Mode transitions

| From | To | Initiator | Preconditions |
|------|-----|-----------|---------------|
| *(session start)* | Manual | Bridge default | `start_session` |
| Manual | Assisted | User (UI) | Session `running` or `paused`; editing session match |
| Assisted | Manual | User | Clears pending recommendations |
| Manual | Autonomous | User (UI) | Explicit confirm banner; PLAT-RT-TAC4 only |
| Autonomous | Manual | User | Immediate; cancels scheduled controller ticks |
| Assisted | Autonomous | User | Forbidden until PLAT-RT-TAC4 — must pass through Manual or explicit audit |
| Any | *(terminal)* | Lifecycle | `stop_session`, `capture_session`, `discard_session` — tactical state discarded |

Mode transitions **must** emit future audit events (`tactical_mode_changed`) — reserved in [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md) §6.

---

## 4. Default and invariants

1. **Default mode:** `manual` at every `start_session`.
2. **Mode ≠ SA replay authority** — tactical mode in RT session has no effect on SA corpus or parser contracts.
3. **Mode is per-session** — session A in Autonomous does not assign session B candidates.
4. **Background sessions** — tactical mode changes forbidden on non-editing sessions (align with [rt_multi_session_editing_ownership_v1.md](rt_multi_session_editing_ownership_v1.md)).
5. **Capture** — tactical mode at `capture_session` frozen into capture metadata (PLAT-RT-TAC5).

---

## 5. Alignment with RT-S1 UX lexicon

Forbidden terms from [rt_s1_interactive_sandbox_architecture_plan.md](../platform/rt_s1_interactive_sandbox_architecture_plan.md) apply in **all modes**:

`engage`, `intercept`, `strike`, `target lock`, `mission approval`, `operator authorization`, `tactical readiness`, defeat/kill/neutralize language.

TAC1 adds sandbox-safe alternatives in §1 — never substitute forbidden terms in UI copy, audit messages, or docs outside quoted governance examples.

---

## 6. Out of scope

- Bridge command implementations
- UI mode selector components
- Engine `interception_logic_node` mode coupling
- SA viewer tactical overlays

---

## Related

- [rt_tac1_tactical_controller_layers_v1.md](rt_tac1_tactical_controller_layers_v1.md)
- [rt_tac1_tactical_governance_v1.md](rt_tac1_tactical_governance_v1.md)
- [rt_tac1_tactical_telemetry_v1.md](rt_tac1_tactical_telemetry_v1.md)
