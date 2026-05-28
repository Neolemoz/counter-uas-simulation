# F7 Advisory Aggregation Examples

Reference inputs for PLAT-RT-F7 P0 golden tests. Documents expected `rt_advisory_batch_summary_v1` shapes when batch derive runs with F7 extensions.

**Not authoritative staging** — documentation fixtures only.

| File | Scenario |
|------|----------|
| `queue_priority_mixed.json` | Multiple captures; stable queue band ordering |
| `blocker_groups_rollup.json` | Shared normalization + review blockers |
| `experiment_rollup_warn_only.json` | F5 eligible experiment; capture not `import_ready` |
| `lineage_warning_detect.json` | LIN-01 session_id parent hint — warn only |
| `readiness_cohorts_standup.json` | Cohort bucket counts for stand-up summary |

P2 stand-up export contract: `expected/standup_export_v2.expected.json`.

See [rt_advisory_aggregation_v1.md](../../docs/evaluation/rt_advisory_aggregation_v1.md).

Full golden outputs: `expected/<fixture-stem>.expected.json`.
