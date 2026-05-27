# RT-F2 — Hardening Review R1 (PLAT-RT-F2)

**Phase:** PLAT-RT-F2  
**Status:** frozen

## Runtime teardown

| Item | Status |
|------|--------|
| `clear_tactical_state` resets buffer + drops controller | Done |
| Invoked on discard/capture/stop/fail teardown | Done |
| `bridge_ready_timeout` path clears tactical | Done |
| Audit `tactical_cleanup` with timeline counts | Done |
| Pytest `test_tactical_cleanup_on_discard` | Done |

## Experiment / annex

| Item | Status |
|------|--------|
| `safeParse*` guards (manifest, analytics, annex, bundle) | Done |
| Corrupt localStorage entry skipped per-run | Done |
| `pruneAnnexCacheForManifest` on import/remove | Done |
| Partial bundle import with per-entry errors | Done |
| Shared `shortId` in `experimentIds.ts` | Done |

## UI

| Item | Status |
|------|--------|
| `editBySession` pruned when sessions disconnect | Done |
| Compare keys normalized when runs/slots change | Done |
| Workbench remove run clears annex + prunes cache | Done |

## Maintainer (P1)

| Item | Status |
|------|--------|
| `rt_staging_integrity_audit.py` read-only report | Done |

## Deferred (unchanged debt)

- Full `App.tsx` decomposition — future wave.
- Distributed multi-session (M3) — not authorized.
