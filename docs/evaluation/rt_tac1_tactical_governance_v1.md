# RT Tactical Governance (`rt_tac1_tactical_governance_v1`)

**Phase:** PLAN-RT-TAC1 — tactical controller architecture (docs only)  
**Authority:** [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md); [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md); [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §13

Governance extensions for RT sandbox tactical controller planning. **No implementation in PLAN-RT-TAC1.**

---

## 1. Relationship to frozen governance

This document **supplements** [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) and [rt_multi_session_governance_v1.md](rt_multi_session_governance_v1.md). Frozen per-session caps, editing lock, and capture rules remain unchanged.

---

## 2. Deny-by-default tactical commands

All verbs in §3 return `COMMAND_FORBIDDEN` until an explicit **PLAT-RT-TAC*** wave adds them to [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md).

**TAC1 rule:** No allow-list extension. Bridge behavior unchanged.

### 2.1 Error semantics

Same as existing bridge contract: `COMMAND_FORBIDDEN` with message citing missing wave authorization — not silent ignore.

---

## 3. Reserved command namespace (documentation only)

| Verb | Intended wave | Mode | Notes |
|------|---------------|------|-------|
| `set_tactical_mode` | TAC2 (manual default) / TAC3 / TAC4 | All | Payload: `{ "mode": "manual" \| "assisted" \| "autonomous" }` |
| `select_candidate` | TAC2 | Manual+ | Highlight only — no assignment |
| `assign_candidate` | TAC2 | Manual | User-authoritative commit |
| `clear_assignment` | TAC2 | Manual | Clears `assigned_candidate_id` |
| `request_recommendation` | TAC3 | Assisted | Controller computes packet |
| `approve_recommendation` | TAC3 | Assisted | User approval gate |
| `reject_recommendation` | TAC3 | Assisted | Clears pending packet |
| `pause_autonomous_loop` | TAC4 | Autonomous | Revert scheduling |
| `resume_autonomous_loop` | TAC4 | Autonomous | Within caps |
| `get_tactical_state` | TAC2+ | All | Read-only snapshot pull |

Prefix `tactical_*` on any undeclared verb → **forbidden**.

---

## 4. No SA contamination

| Risk | Mitigation |
|------|------------|
| SA viewer live tactical hooks | **Forbidden** — viewer remains static JSON |
| Auto-import tactical state | **Forbidden** — maintainer pipeline only |
| Parser/topic/schema changes | **Forbidden** in all TAC waves without separate registry wave |
| Federation writes from tactical events | **Forbidden** |
| `session_id` as lineage parent | **Forbidden** — `ephemeral_session_ref` only |
| Tactical telemetry → corpus index | **Forbidden** without `replay_boundary_scoped` capture + import |
| Engine `[TACTICAL_*]` → SA bundle authority | **Forbidden** — explanatory only |

See [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md), [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md).

---

## 5. No operational semantics drift

### 5.1 Inherited forbidden terms (RT-S1)

`engage`, `intercept`, `strike`, `target lock`, `mission approval`, `operator authorization`, `tactical readiness`, defeat/kill/neutralize language.

### 5.2 TAC1 additive forbidden terms

| Forbidden | Use instead |
|-----------|-------------|
| engage / engagement | assign candidate, simulate engagement geometry |
| fire / weapon release | *(no RT equivalent — forbidden)* |
| neutralize / kill / defeat | remove entity (sandbox), end simulation |
| command authority | editing session, user approval |
| operator authorize | approve recommendation |
| readiness score | tactical health (explanatory) |
| C2 / command center | RT workstation, sandbox session |
| HITL | user approval (Assisted mode) |
| auto-intercept | sandbox auto-assign (Autonomous, bounded) |
| threat kill chain | candidate review pipeline |

Audit messages and UI copy **must** pass governance lint for forbidden terms (future PLAT — document requirement here).

---

## 6. RT-only scope

| Constraint | Rule |
|------------|------|
| Transport | Loopback HTTP to RT bridge only |
| Multi-session | Single bridge, `max_concurrent_sessions = 3` |
| Tactical state | Per `session_id` — no cross-session assignment |
| Distributed autonomy | **Forbidden** — no multi-bridge tactical sync |
| Cloud / multi-user | **Forbidden** |
| Federation orchestration | **Forbidden** |

---

## 7. Mirrors ≠ authority

| Mirror | Never overrides |
|--------|-----------------|
| Recommendation packet | Bridge registry without Assisted approval |
| TTI / feasibility display | User Manual commands |
| Engine `[TACTICAL_*]` logs | RT `assigned_candidate_id` |
| Pose sync mirror | Command registry pose |
| Telemetry `tactical_health` | Session lifecycle or capture approval |

---

## 8. Required banners (future PLAT)

When tactical mode ≠ `manual`, UI **must** show:

`TACTICAL SANDBOX — simulation only; not operational coordination`

When Autonomous (TAC4+):

`AUTONOMOUS LOOP — user may revert to Manual; not replay authority`

Additive to existing RT governance banners ([rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)).

---

## 9. Multi-session tactical rules

| Rule | Enforcement (future) |
|------|----------------------|
| Tactical commands on editing session only | `EDITING_SESSION_MISMATCH` |
| Background session tactical pull | Read-only `get_tactical_state` at ≤ 1 Hz |
| Capture | Target session tactical timeline only |
| Teardown | Clear controller state with session |

---

## 10. Out of scope

- Implementing §3 verbs
- CI lint for forbidden terms (proposed in PLAT-RT-TAC2)
- Changing `interception_logic_node` behavior

---

## Related

- [rt_tac1_tactical_modes_v1.md](rt_tac1_tactical_modes_v1.md)
- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §13
- [rt_authority_model_v1.md](rt_authority_model_v1.md) §5
