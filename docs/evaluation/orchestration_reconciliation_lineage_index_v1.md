# Orchestration Reconciliation Lineage Index (`orchestration_reconciliation_lineage_index_v1`)

**Phase:** PLAT-SA-I3 — corpus supersede/retry graph for batch review  
**Authority:** [AGENTS.md](../../AGENTS.md)

Corpus index from `build_reconciliation_lineage_index()`. Read-only navigation aid for reviewers.

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `orchestration_reconciliation_lineage_index_v1` |
| `schema_version` | string | `1` |
| `generated_at` | string | ISO-8601 UTC |
| `governance` | object | `notice`, `anti_claims` |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `retry_groups` | array | `{manifest_id, worker_refs[], attempt_count}` |
| `supersede_edges` | array | `{from_manifest_id, to_manifest_id, notes}` |
| `quarantine_holds` | array | `{manifest_id, reason}` |
| `failed_executions` | array | `{manifest_id, async_execution_status}` |

---

## Storage

`fixtures/orchestration/reconciliation/reconciliation_lineage_index_v1.json`

*End of orchestration reconciliation lineage index v1.*
