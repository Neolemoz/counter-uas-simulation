# PLAT-RT-F2 — Implementation Roadmap v1

**Phase:** PLAT-RT-F2 delivered  
**Prerequisite:** [rt_f2_freeze_audit.md](rt_f2_freeze_audit.md), [rt_plat_f2_freeze_audit.md](rt_plat_f2_freeze_audit.md)

## P0

| Item | Location |
|------|----------|
| `clear_tactical_state` | `session_teardown.py` |
| `experimentImportGuards.ts` | `rt-sandbox-ui` |
| Annex cache prune / partial bundle | `annexReviewStore.ts` |
| App `editBySession` prune | `App.tsx` |

## P1

| Item | Location |
|------|----------|
| `rt_staging_integrity_audit.py` | `scripts/rt/` |
| Workbench compare key reset | `ExperimentWorkbenchPanel.tsx` |
| `experimentIds.ts` | shared `shortId` |

## Stop line

PLAT-RT-F2 frozen after validation. Next advisory: **F4 realism** (not M3/F5 without audit).
