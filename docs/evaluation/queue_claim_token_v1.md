# Queue Claim Token (`queue_claim_token_v1`)

**Phase:** PLAT-SA-I2 — async worker claim bookkeeping (CLI/fixture only)  
**Authority:** [AGENTS.md](../../AGENTS.md)

Records a **CLI worker claim** against a frozen queue snapshot. Not live distributed locking.

---

## File location

```
fixtures/orchestration/claims/<queue_id>_claim.json
```

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `queue_claim_token_v1` |
| `schema_version` | string | `1` |
| `claim_id` | string | Unique claim identifier |
| `manifest_id` | string | Paired job manifest |
| `queue_id` | string | Frozen queue id |
| `snapshot_hash` | string | SHA-256 of queue snapshot file |
| `manifest_fingerprint` | string | SHA-256 of job manifest at claim time |
| `worker_id` | string | Deterministic worker identity (CLI-assigned) |
| `claim_status` | string | `open` or `closed` |
| `governance` | object | `notice`, `anti_claims` |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `queue_snapshot_ref` | string | Repo-relative queue path |
| `claimed_at` | string | ISO-8601 UTC |
| `closed_at` | string | ISO-8601 UTC when terminal |
| `terminal_reason` | string | Closure reason code |

---

## Related

- [worker_execution_record_v1.md](worker_execution_record_v1.md)
- [experiment_orchestration_async_manifest_v1.md](experiment_orchestration_async_manifest_v1.md)

*End of queue claim token v1.*
