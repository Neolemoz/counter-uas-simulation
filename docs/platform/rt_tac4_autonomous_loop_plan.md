# RT-TAC4 — Autonomous Tactical Loop (PLAT-RT-TAC4)

**Phase:** PLAT-RT-TAC4 — autonomous sandbox loop  
**Prerequisite:** PLAT-RT-TAC3 frozen  
**Authority:** [AGENTS.md](../../AGENTS.md); [rt_tac1_tactical_controller_architecture_plan.md](rt_tac1_tactical_controller_architecture_plan.md)

**Companion artifacts:**

- [rt_tac4_governance_review_r1.md](../evaluation/rt_tac4_governance_review_r1.md)
- [rt_tac4_isolation_audit.md](../evaluation/rt_tac4_isolation_audit.md)
- [rt_tac4_autonomy_safety_review_r1.md](../evaluation/rt_tac4_autonomy_safety_review_r1.md)
- [rt_tac4_freeze_audit.md](../evaluation/rt_tac4_freeze_audit.md)

---

## 1. Purpose

Implement **Autonomous mode**: bounded per-session scheduler tick that ranks interceptor–target pairs, commits assignment, and applies one `move_entity` per cycle. User may pause/resume or revert to Manual at any time.

---

## 2. Tick policy

- Hook: [`BridgeSessionManager._tick_timeouts`](../../platform/rt-sandbox-bridge/rt_sandbox/session_manager.py) on each command wave
- Interval: `AUTONOMOUS_TICK_INTERVAL_S = 2.0`
- Assignment lock: `ASSIGNMENT_LOCK_DURATION_S = 1.5` after each autonomous commit
- Enter autonomous: loop starts **paused**; user calls `resume_autonomous_loop`

---

## 3. Stop line

No PLAT-RT-TAC5 (tactical capture annex) without TAC4 freeze audit.
