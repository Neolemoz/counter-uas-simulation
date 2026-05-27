# RT-M3 P1 — Local Session UX Polish (PLAT-RT-M3 P1)

**Phase:** PLAT-RT-M3 P1 — local multi-session workstation UX  
**Prerequisite:** PLAT-RT-M3 P0 frozen — [rt_plat_m3_p0_freeze_audit.md](../evaluation/rt_plat_m3_p0_freeze_audit.md)  
**Authority:** [rt_multi_session_poll_policy_v1.md](../evaluation/rt_multi_session_poll_policy_v1.md)

## Goal

Pause background telemetry poll when diagnostics accordion is collapsed; advisory tab-switch confirm for unsynced local mirrors; optional session display names in `localStorage` — RT UI only.

## Delivered (P1)

| Item | Location |
|------|----------|
| Background poll pause | `useRtSessionWorkspace({ pauseBackgroundPoll })`, `shouldPullSlotInAutoRefresh` |
| Accordion + paused indicator | `BackgroundDiagnostics.tsx` |
| Tab-switch confirm | `sessionMirrorDirty.ts`, `handleSelectTab` in `App.tsx` |
| Display names | `sessionDisplayNameStore.ts`, `useSessionDisplayNames.ts` |
| Tab / diagnostics labels | `SessionTabBar.tsx`, `BackgroundDiagnostics.tsx` |

## Forbidden (unchanged)

- Bridge / `RUNTIME_SUBCOMMANDS` changes  
- SA viewer changes  
- Distributed multi-bridge  
- Tab reorder, refresh-all — P2  
- Tactical redesign  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
cd platform/rt-sandbox-ui && npm run test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-M3 P1 frozen. Do not start **P2** without governance review + freeze audit.

## Related

- [rt_roadmap_plat_rt_m3_v1.md](../evaluation/rt_roadmap_plat_rt_m3_v1.md)
- [rt_plat_m3_p1_freeze_audit.md](../evaluation/rt_plat_m3_p1_freeze_audit.md)
