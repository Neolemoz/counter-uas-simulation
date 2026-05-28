# RT-F8 P1 — Architecture Review R1

**Phase:** PLAT-RT-F8 P1  
**Plan:** [rt_plat_f8_p1_advisory_triage_integration_plan.md](../platform/rt_plat_f8_p1_advisory_triage_integration_plan.md)

---

## Summary

P1 consolidates F8 cognition into `AdvisoryTriageQueuePanel` as a read-only maintainer hub. Parent `CaptureHandoffWorkflowPanel` retains session-scope rollups (`buildSessionAdvisorySummaryV2`) and applies preset/focus/cohort-hint filters before passing rows to triage. No new bridge routes or Python batch changes.

---

## Layering

| Layer | Change | Verdict |
|-------|--------|---------|
| Bridge HTTP | None | Pass |
| Subcommand registry | None | Pass |
| F8 P0 batch/CLI | Unchanged | Pass |
| SA viewer | None | Pass |

---

## Verdict

**Pass** — PLAT-RT-F8 P1 may freeze.
