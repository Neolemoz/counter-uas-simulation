# RT-TAC2 — Manual Tactical Controller Freeze Audit (PLAT-RT-TAC2)

## Scope

- [rt_tac2_manual_intercept_plan.md](../platform/rt_tac2_manual_intercept_plan.md)
- `platform/rt-sandbox-bridge/rt_sandbox/tactical_*.py`, `session_tactical_handlers.py`
- `platform/rt-sandbox-ui/src/components/TacticalManualPanel.tsx`, `tacticalCommands.ts`, `useTacticalState.ts`
- [rt_tac2_governance_review_r1.md](rt_tac2_governance_review_r1.md)
- [rt_tac2_isolation_audit.md](rt_tac2_isolation_audit.md)
- Additive [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §13

**Not in scope:** Assisted/Autonomous modes, TAC5 capture annex, SA viewer, parser changes.

## Governance Result

**Verdict: frozen** for PLAT-RT-TAC2.

## Boundary Checks

| Check | Result |
|-------|--------|
| Manual mode only | Pass |
| Deny assisted/autonomous | Pass |
| No `intercept` bridge verb | Pass |
| Editing lock on mutations | Pass |
| Per-session isolation | Pass |
| No SA contamination | Pass |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py src/counter_uas/test/test_rt_tactical_manual.py -q
(cd platform/rt-sandbox-ui && npm test && npm run build)
```

## Stop line

Do not start PLAT-RT-TAC3 without new wave audit.
