# RT-M3 P2 — Session Reorder + Diagnostics Polish (PLAT-RT-M3 P2)

**Phase:** PLAT-RT-M3 P2 — completes local multi-session polish  
**Prerequisite:** PLAT-RT-M3 P1 frozen  
**Authority:** [rt_roadmap_plat_rt_m3_v1.md](../evaluation/rt_roadmap_plat_rt_m3_v1.md)

## Goal

Tab reorder persistence (`localStorage`), richer background diagnostic rows from existing telemetry, and workstation UX cohesion.

## Delivered (P2)

| Item | Location |
|------|----------|
| Tab order store | `sessionTabOrderStore.ts`, `useSessionTabOrder.ts` |
| Sort helper | `sortSessionSlots.ts` |
| Native drag reorder | `SessionTabBar.tsx` |
| Row cognition | `backgroundSessionRowCognition.ts` |
| Diagnostics polish | `BackgroundDiagnostics.tsx` |
| App cohesion | `App.tsx` |

## PLAN-RT-M3 / PLAT-RT-M3

With P2 frozen, **PLAN-RT-M3** and **PLAT-RT-M3** (P0–P2) are complete.

## Stop line

Do not start PLAN-RT-F7 or new PLAT waves without governance + freeze.

## Related

- [rt_plat_m3_p2_freeze_audit.md](../evaluation/rt_plat_m3_p2_freeze_audit.md)
