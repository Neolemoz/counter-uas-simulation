# RT-TAC2 — Manual Sandbox Assignment (PLAT-RT-TAC2)

**Phase:** PLAT-RT-TAC2 — manual tactical controller implementation  
**Prerequisite:** PLAN-RT-TAC1 frozen  
**Authority:** [AGENTS.md](../../AGENTS.md); [rt_tac1_tactical_controller_architecture_plan.md](rt_tac1_tactical_controller_architecture_plan.md)

**Companion artifacts:**

- [rt_tac2_governance_review_r1.md](../evaluation/rt_tac2_governance_review_r1.md)
- [rt_tac2_isolation_audit.md](../evaluation/rt_tac2_isolation_audit.md)
- [rt_tac2_freeze_audit.md](../evaluation/rt_tac2_freeze_audit.md)

---

## 1. Purpose

Implement **Manual mode only** tactical sandbox controls: bridge commands, per-session `TacticalController`, `tactical_state` telemetry, and RT UI candidate review panel. User actions remain authoritative; assign triggers a single `move_entity` to intercept geometry (no auto-move, no Assisted/Autonomous).

---

## 2. Deliverables

| Layer | Artifact |
|-------|----------|
| Bridge | `tactical_*.py`, `session_tactical_handlers.py`, governance allow-list |
| UI | `TacticalManualPanel`, `tacticalCommands.ts`, `useTacticalState` |
| Tests | `test_rt_tactical_manual.py`, `TacticalManualPanel.test.tsx` |
| Docs | TAC2 reviews, freeze audit, bridge contract §13 update |

---

## 3. Stop line

No PLAT-RT-TAC3 (Assisted) without TAC2 freeze audit.
