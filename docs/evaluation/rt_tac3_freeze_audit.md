# RT-TAC3 — Assisted Recommendation Freeze Audit (PLAT-RT-TAC3)

## Scope

- [rt_tac3_assisted_recommendation_plan.md](../platform/rt_tac3_assisted_recommendation_plan.md)
- `platform/rt-sandbox-bridge/rt_sandbox/tactical_recommendation.py`, extended `tactical_*.py`, `session_tactical_handlers.py`
- `platform/rt-sandbox-ui/src/components/TacticalAssistedPanel.tsx`, `TacticalManualPanel.tsx`, `tacticalCommands.ts`, `useTacticalState.ts`
- [rt_tac3_governance_review_r1.md](rt_tac3_governance_review_r1.md)
- [rt_tac3_isolation_audit.md](rt_tac3_isolation_audit.md)
- Additive [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §13

**Not in scope:** Autonomous mode, TAC5 capture annex, SA viewer, parser changes.

## Governance Result

**Verdict: frozen** for PLAT-RT-TAC3.

## Boundary Checks

| Check | Result |
|-------|--------|
| Assisted mode + recommendation verbs | Pass |
| Deny autonomous mode | Pass |
| No motion without approve in assisted | Pass |
| Manual assign unchanged | Pass |
| Editing lock on mutations | Pass |
| Per-session isolation | Pass |
| No SA contamination | Pass |

## Assisted architecture summary

- Browser issues `request_recommendation` → bridge ranks interceptor×target pairs by cap-speed TTI → `tactical_recommendation` telemetry (explanatory).
- User `approve_recommendation` → same assign/move path as manual with `user_approval_authoritative` label.
- User `reject_recommendation` → clears pending only.

## Isolation guarantees

- Recommendation packets are session-local and non-authoritative until approval.
- No cross-session tactical routing; no SA/parser/federation writes from recommendations.

## TAC4 prerequisites

- PLAT-RT-TAC3 frozen + registry row
- New wave plan for autonomous mode, scheduler tick, pause/resume verbs
- Assisted→Autonomous transition audit before implementation

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py src/counter_uas/test/test_rt_tactical_manual.py src/counter_uas/test/test_rt_tactical_assisted.py -q
(cd platform/rt-sandbox-ui && npm test && npm run build)
```

## Stop line

Do not start PLAT-RT-TAC4 without new wave audit.
