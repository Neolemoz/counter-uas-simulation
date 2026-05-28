# RT-F8 P0 — Architecture Review R1

**Phase:** PLAT-RT-F8 P0  
**Plan:** [rt_plat_f8_p0_advisory_v2_implementation_plan.md](../platform/rt_plat_f8_p0_advisory_v2_implementation_plan.md)  
**Contracts:** [rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md), [rt_advisory_maintainer_workflow_v2.md](rt_advisory_maintainer_workflow_v2.md)

---

## Architecture summary

F8 P0 extends frozen F7 advisory derive and batch paths additively:

1. **Python:** `advisory_queue.py` gains filter presets, focus intersection, `readiness_cohort_v2`, and v2 rollups (`multi_capture_cohorts`, `handoff_rollup`, `experiment_handoff_rollup`). `batch_advisory.py` emits `rt_advisory_batch_summary_v2` and render-only template packs.
2. **CLI:** `rt_handoff_batch_advisory.py` adds `--schema f8` distinct from F7 `--schema v2` review export; presets/focus apply before sort/export; `dry_run` default preserved.
3. **UI:** `advisoryAggregationV2.ts` mirrors Python for session cognition; read-only strips in `CaptureHandoffWorkflowPanel` — no new bridge routes.

```text
scan_staged → apply_filter_preset → apply_focus_set → enrich_row_dict_v2 → v2 rollups → doc / UI mirror
```

---

## Layering check

| Layer | Change | Verdict |
|-------|--------|---------|
| Bridge HTTP | None | Pass |
| Subcommand registry | None (lint unchanged) | Pass |
| F6 derive ladder | Unchanged precedence | Pass |
| F7 P0–P7 queue bands | Unchanged ranks | Pass |
| SA viewer | No scope | Pass |

---

## Debt / follow-ups (P1+)

- Triage panel could consume preset/focus props directly (P1).
- Golden fixture parity tests against `f8_advisory_examples/expected/` may expand in P1.
- Template pack file write path remains maintainer CLI opt-in only (P2 hardening).

---

## Verdict

**Pass** — additive v2 summary and read-only UI mirrors respect RT sandbox layering. PLAT-RT-F8 P0 may freeze.
