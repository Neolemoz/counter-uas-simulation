# Worker Execution Record (`worker_execution_record_v1`)

**Phase:** PLAT-SA-I2 — deterministic worker provenance (CLI/fixture only)  
**Authority:** [AGENTS.md](../../AGENTS.md)

Per-attempt execution provenance for offline async bookkeeping. **Not** a live worker registry.

---

## File location

```
fixtures/orchestration/workers/<worker_id>_<execution_attempt>.json
```

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `worker_execution_record_v1` |
| `schema_version` | string | `1` |
| `worker_id` | string | CLI-assigned worker identity |
| `execution_attempt` | integer | 1-based attempt counter |
| `manifest_id` | string | Job manifest id |
| `claim_id` | string | Matching `queue_claim_token_v1.claim_id` |
| `snapshot_hash` | string | Queue snapshot hash at execution |
| `manifest_fingerprint` | string | Manifest hash at execution |
| `started_at` | string | ISO-8601 UTC |
| `finished_at` | string | ISO-8601 UTC |
| `governance` | object | `notice`, `anti_claims` |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `retry_lineage` | array | Prior attempt refs `{worker_id, execution_attempt}` |
| `retry_parent_ref` | string | Path to parent worker record |
| `deterministic_metadata` | object | `dry_run`, `allow_runtime_capture`, `allow_async_worker` flags only |
| `step_outcome_hashes` | object | Map step_id → outcome hash from report |
| `execution_fingerprint` | string | Computed fingerprint for this attempt |

---

## Related

- [queue_claim_token_v1.md](queue_claim_token_v1.md)
- [experiment_orchestration_async_manifest_v1.md](experiment_orchestration_async_manifest_v1.md)

*End of worker execution record v1.*
