# F8 Advisory Aggregation Examples

Reference inputs for future PLAT-RT-F8 P0 golden tests. Documents expected `rt_advisory_batch_summary_v2` shapes when batch derive runs with F8 extensions.

**Not authoritative staging** — documentation fixtures only. **Not consumed** by `platform/sa-r0-viewer/`.

| File | Scenario |
|------|----------|
| `filter_preset_blocked_defer.json` | `blocked_today` + `defer_queue` preset semantics — filter only |
| `multi_capture_cohort_summary.json` | `multi_capture_cohorts` + `handoff_rollup.by_stage` |
| `experiment_handoff_rollup_warn.json` | X2 cohort index + F5 warn-only — no `import_ready` |
| `lineage_batch_v2_exemplar.json` | LIN-06+ exemplar cap — warn only |
| `readiness_cohorts_v2_stale.json` | `readiness_cohorts_v2` stale_review / multi_blocker |

Full golden outputs: `expected/<fixture-stem>.expected.json`.

See [rt_advisory_aggregation_v2.md](../../docs/evaluation/rt_advisory_aggregation_v2.md).
