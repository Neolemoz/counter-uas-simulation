# Orchestration Recovery Report (`orchestration_recovery_report_v1`)

**Phase:** PLAT-SA-I3 — per-manifest recovery cognition artifact  
**Authority:** [AGENTS.md](../../AGENTS.md)

Produced by `replay_sa_orchestration_recovery.build_recovery_report()`. Explanatory only.

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `orchestration_recovery_report_v1` |
| `schema_version` | string | `1` |
| `manifest_id` | string | Target manifest |
| `generated_at` | string | ISO-8601 UTC |
| `governance` | object | `notice`, `anti_claims` |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `async_execution_status` | string | Current async status |
| `retry_chain` | array | Ordered claim/worker/lineage hops |
| `recovery_issues` | array | `{kind, message}` recovery audit findings |
| `recovery_warnings` | array | Non-fatal findings |
| `replay_reconciliation` | object | `verify_replay_reproducibility` summary |
| `recovery_continuity_ok` | boolean | Combined continuity pass |
| `superseded` | boolean | Whether manifest is superseded |
| `quarantine_hold` | boolean | Whether promote guard active |

---

## Storage

`fixtures/orchestration/recovery/{manifest_id}_recovery_report.json`

*End of orchestration recovery report v1.*
