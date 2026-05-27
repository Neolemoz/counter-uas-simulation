# RT-TAC3 — Assisted Sandbox Recommendation (PLAT-RT-TAC3)

**Phase:** PLAT-RT-TAC3 — assisted tactical recommendation implementation  
**Prerequisite:** PLAT-RT-TAC2 frozen  
**Authority:** [AGENTS.md](../../AGENTS.md); [rt_tac1_tactical_controller_architecture_plan.md](rt_tac1_tactical_controller_architecture_plan.md)

**Companion artifacts:**

- [rt_tac3_governance_review_r1.md](../evaluation/rt_tac3_governance_review_r1.md)
- [rt_tac3_isolation_audit.md](../evaluation/rt_tac3_isolation_audit.md)
- [rt_tac3_freeze_audit.md](../evaluation/rt_tac3_freeze_audit.md)

---

## 1. Purpose

Implement **Assisted mode** tactical sandbox controls: recommendation ranking (cap-speed TTI), `tactical_recommendation` telemetry, approval-gated assignment (`approve_recommendation` / `reject_recommendation`), and RT UI recommendation panel. Recommendations are explanatory only; adapter motion occurs only after explicit user approval.

---

## 2. Deliverables

| Layer | Artifact |
|-------|----------|
| Bridge | `tactical_recommendation.py`, extended `tactical_controller.py`, `session_tactical_handlers.py`, governance allow-list |
| Telemetry | Channel `tactical_recommendation`, `pending_recommendation_id` on `tactical_state` |
| UI | `TacticalAssistedPanel`, assisted mode selector, `tacticalCommands.ts`, `useTacticalState` |
| Tests | `test_rt_tactical_assisted.py`, assisted panel tests |
| Docs | TAC3 reviews, freeze audit, bridge contract §13 update |

---

## 3. Mode / command matrix

| Command | Manual | Assisted |
|---------|--------|----------|
| `select_candidate`, `clear_assignment` | Allowed | Allowed |
| `assign_candidate` | Allowed + move | Forbidden — use `approve_recommendation` |
| `request/approve/reject_recommendation` | Forbidden | Allowed |
| `set_tactical_mode` | `manual` \| `assisted` | same |
| `autonomous` | Forbidden (TAC4) | Forbidden |

Switching away from assisted clears pending recommendations.

---

## 4. Stop line

No PLAT-RT-TAC4 (Autonomous loop) without TAC3 freeze audit.
