# RT UI Hardening (`rt_ui_hardening_v1`)

**Phase:** PLAN-RT-F2 / PLAT-RT-F2

## Multi-session edit state

When a session slot is removed from `useRtSessionWorkspace`, `App.tsx` must prune `editBySession[sessionId]` to prevent stale local entities on reconnect.

## Workbench

| Event | Action |
|-------|--------|
| Manifest import | Safe parse; prune annex cache to manifest run ids; reset invalid compare keys |
| Run remove | `clearAnnexForRun` + `pruneAnnexCacheForManifest` |
| Experiment id change | Batch spec sync (existing); continuity run picker rebind |

## Banners

No new user-facing banner for F2 — existing governance chrome sufficient.

## Related

- [rt_experiment_import_hardening_v1.md](rt_experiment_import_hardening_v1.md)
