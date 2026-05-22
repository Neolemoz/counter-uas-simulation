# Experiment Orchestration Async Manifest (`experiment_orchestration_async_manifest_v1`)

**Phase:** PLAT-SA-I2 — async orchestration foundations  
**Authority:** [AGENTS.md](../../AGENTS.md); [sa_i2_async_orchestration_plan.md](../platform/sa_i2_async_orchestration_plan.md)

Additive sidecar describing **async execution state** parallel to frozen `experiment_orchestration_ops_manifest_v1`. **Not** parser-visible. **Not** a substitute for I1 `operations_status`.

---

## File location

```
fixtures/orchestration/async/<manifest_id>_async.json
```

Optional: one async sidecar per manifest that uses async bookkeeping. Validation-only 12-pack manifests do not require async sidecars.

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | Must be `experiment_orchestration_async_manifest_v1` |
| `schema_version` | string | Must be `1` |
| `manifest_id` | string | Matches paired job manifest |
| `governance` | object | `notice` (string), `anti_claims` (string array) |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `async_execution_status` | string | See enum below (absent = no async activity) |
| `execution_fingerprint` | string | SHA-256 over frozen manifest + queue + report inputs |
| `replay_fingerprint` | string | SHA-256 over bundle `index.json` when replay verified |
| `queue_snapshot_hash` | string | Hash of frozen `experiment_run_queue_v1` file |
| `queue_claim_ref` | string | Repo-relative path to `queue_claim_token_v1` |
| `worker_execution_refs` | string[] | Paths to `worker_execution_record_v1` |
| `async_lineage` | array | Ordered async status events (CLI-only) |
| `async_execution_summary_ref` | string | Path to `async_execution_summary_v1` |
| `updated_at` | string | ISO-8601 UTC of last CLI write |

---

## `async_execution_status` enum

Transitions are **CLI-only** (`record_async_execution.py`). The browser never mutates async status.

| Status | Meaning |
|--------|---------|
| `retrying` | CLI re-queued after bounded failure |
| `failed` | Terminal step or manifest failure on async plane |
| `quarantined` | Integrity hold; blocks I1 forward promote |
| `superseded` | Replaced by newer snapshot/claim (read-only lineage) |

**Distinction from I1 `operations_status`:** I1 ladder remains authoritative for release bookkeeping. Async status is explanatory recovery/bookkeeping only unless `quarantined` (promote guard).

**Distinction from H3 queue job/step `status`:** Queue `running`/`completed`/`dry_run` remain H3 run bookkeeping.

---

## Related

- [experiment_orchestration_ops_manifest_v1.md](experiment_orchestration_ops_manifest_v1.md)
- [experiment_orchestration_async_model_v1.md](experiment_orchestration_async_model_v1.md)
- [queue_claim_token_v1.md](queue_claim_token_v1.md)
- [worker_execution_record_v1.md](worker_execution_record_v1.md)

*End of experiment orchestration async manifest v1.*
