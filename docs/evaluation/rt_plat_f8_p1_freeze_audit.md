# RT-F8 P1 — Freeze Audit (PLAT-RT-F8 P1)

**Phase:** PLAT-RT-F8 P1 — advisory triage UX integration  
**Status:** frozen

**Plan:** [rt_plat_f8_p1_advisory_triage_integration_plan.md](../platform/rt_plat_f8_p1_advisory_triage_integration_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | `enrichRowsForTriage` v2 enrichment + focus highlight | Yes |
| 2 | Integrated `AdvisoryTriageQueuePanel` F8 hub | Yes |
| 3 | `cohort_v2` / `handoff_stage` grouping | Yes |
| 4 | Stand-up pass selector + template collapsible | Yes |
| 5 | Focus prev/next + group open memory | Yes |
| 6 | CaptureHandoffWorkflowPanel dedupe | Yes |
| 7 | Vitest | Yes |
| 8 | P1 reviews + registry | Yes |

No bridge/SA viewer changes.

---

## F8 P1 summary

**Triage hub:** Preset, focus, stand-up passes, v2 rollup bar, template preview, and triage table in one read-only panel.  
**Rows:** `readiness_cohort_v2`, focus ring, stale hint on triage lines.  
**Grouping:** Added `cohort_v2` and `handoff_stage` view modes.  
**Parent:** Session rollups unchanged; table respects preset/focus/cohort-hint filters.

---

## Regression evidence

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | pass |
| Vitest (rt-sandbox-ui) | 334 passed |
| `npm run build` | pass |
| `tier0-rt-ui` | pass |

---

## Recommended next (advisory)

| Option | When |
|--------|------|
| **PLAT-RT-F8 P2** | Default — corpus-preview refinement, dry-run v2 guards |
| **Platform checkpoint** | Optional if P2 deferred |

---

## Stop line

PLAT-RT-F8 P1 frozen. Do not start **P2** without scoped plan + contamination re-audit + freeze.
