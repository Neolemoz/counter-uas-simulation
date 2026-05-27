# RT-TAC4 — Autonomous Loop Freeze Audit (PLAT-RT-TAC4)

## Scope

- [rt_tac4_autonomous_loop_plan.md](../platform/rt_tac4_autonomous_loop_plan.md)
- `tactical_autonomous.py`, extended `tactical_controller.py`, `session_manager.py`
- `TacticalAutonomousPanel.tsx`, `tacticalCommands.ts`, `useTacticalState.ts`
- Governance reviews + bridge contract §13

**Not in scope:** TAC5 capture annex, SA viewer, parser changes.

## Governance Result

**Verdict: frozen** for PLAT-RT-TAC4.

## Boundary Checks

| Check | Result |
|-------|--------|
| Autonomous mode + pause/resume | Pass |
| Bounded tick + assignment lock | Pass |
| Manual/Assisted unchanged | Pass |
| No cross-session assignment | Pass |
| User revert to Manual | Pass |

## Autonomous architecture summary

- Internal tick on `_tick_timeouts` ranks pairs, commits, and moves once per cycle when loop is running.
- Telemetry uses `tactical_controller_authoritative` in autonomous mode.
- User pause or Manual mode stops commits immediately.

## Isolation guarantees

- Session-local scheduler; editing lock on user tactical commands; no SA/parser contamination.

## TAC5 prerequisites

- TAC4 freeze + registry row
- Plan for `rt_tactical_capture_annex_v1` and normalize hooks
- No capture schema until TAC5 wave audit

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py src/counter_uas/test/test_rt_tactical_manual.py src/counter_uas/test/test_rt_tactical_assisted.py src/counter_uas/test/test_rt_tactical_autonomous.py -q
(cd platform/rt-sandbox-ui && npm test && npm run build)
```

## Stop line

Do not start PLAT-RT-TAC5 without new wave audit.
