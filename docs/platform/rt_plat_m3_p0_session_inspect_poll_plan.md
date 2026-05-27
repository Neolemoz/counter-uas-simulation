# RT-M3 P0 — Session Inspect + Poll UX (PLAT-RT-M3 P0)

**Phase:** PLAT-RT-M3 P0 — local multi-session polish (session inspect + poll UX)  
**Prerequisite:** PLAN-RT-M3 frozen — [rt_m3_freeze_audit.md](../evaluation/rt_m3_freeze_audit.md); PLAT-RT-M2 frozen  
**Authority:** [rt_multi_session_poll_policy_v1.md](../evaluation/rt_multi_session_poll_policy_v1.md)

## Goal

Deliver maintainer `rt_session_inspect.py`, per-slot pull/`pulling` UX, and `BackgroundDiagnostics` polish — without bridge protocol changes, SA viewer changes, or P1 tab features.

## Delivered (P0)

| Item | Location |
|------|----------|
| Session inspect CLI | `scripts/rt/rt_session_inspect.py` |
| Per-slot pulling | `platform/rt-sandbox-ui/src/hooks/useRtSessionWorkspace.ts` |
| Pull age helpers | `platform/rt-sandbox-ui/src/telemetry/pullAge.ts` |
| Background diagnostics polish | `platform/rt-sandbox-ui/src/workstation/BackgroundDiagnostics.tsx` |
| Tests | `test_rt_session_inspect.py`, `pullAge.test.ts`, `BackgroundDiagnostics.test.tsx`, `useRtSessionWorkspace.test.ts` |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- SA viewer changes  
- Distributed multi-bridge  
- Tab reorder, display names, switch confirm, refresh-all — P1  
- Pause background poll when accordion collapsed — P1  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_session_inspect.py -q
cd platform/rt-sandbox-ui && npm run test && npm run build
scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/rt_session_inspect.py summary   # bridge running
```

## Stop line

PLAT-RT-M3 P0 frozen. Do not start **P1** without governance review + freeze audit.

## Related

- [rt_roadmap_plat_rt_m3_v1.md](../evaluation/rt_roadmap_plat_rt_m3_v1.md)
- [rt_plat_m3_p0_freeze_audit.md](../evaluation/rt_plat_m3_p0_freeze_audit.md)
- [rt_plat_m3_p0_governance_review_r1.md](../evaluation/rt_plat_m3_p0_governance_review_r1.md)
