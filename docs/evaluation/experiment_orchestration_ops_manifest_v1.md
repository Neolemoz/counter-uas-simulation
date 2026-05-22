# Experiment Orchestration Ops Manifest (`experiment_orchestration_ops_manifest_v1`)

**Phase:** PLAN-SA-I1 — orchestration operations layer  
**Authority:** [AGENTS.md](../../AGENTS.md); [sa_i1_orchestration_operations_plan.md](../platform/sa_i1_orchestration_operations_plan.md)

Additive sidecar describing **orchestration operations state** for an `experiment_job_manifest_v1`. **Not** parser-visible. **Not** a substitute for `experiment_job_manifest_v1` job definitions.

---

## File location

```
fixtures/orchestration/ops/<manifest_id>_ops.json
```

One ops sidecar per job manifest. `manifest_id` must match the paired `experiment_job_manifest_v1` `manifest_id`.

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | Must be `experiment_orchestration_ops_manifest_v1` |
| `schema_version` | string | Must be `1` |
| `manifest_id` | string | Matches paired job manifest |
| `operations_status` | string | See enum below |
| `governance` | object | `notice` (string), `anti_claims` (string array) |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `scenario_pack_ids` | string[] | Pack ids from manifest jobs (explanatory) |
| `manifest_fingerprint` | string | SHA-256 of job manifest JSON at last validate |
| `queue_snapshot_ref` | string | Repo-relative path to `experiment_run_queue_v1` |
| `audit_report_ref` | string | Repo-relative path to `experiment_run_report_v1` |
| `authoring_pack_refs` | string[] | Catalog pack ids with authoring handoff |
| `operations_lineage` | array | Ordered ops promotion events |
| `operations_summary_ref` | string | Path to `orchestration_operations_summary_v1.json` |
| `updated_at` | string | ISO-8601 UTC of last CLI write |

---

## `operations_status` enum

Transitions are **CLI-only** (`promote_experiment_manifest.py`). The browser never mutates status.

| Status | Meaning |
|--------|---------|
| `pending` | Sidecar initialized; manifest lint not recorded |
| `validated` | Manifest lint + validation mirror OK for all jobs |
| `queued` | Frozen queue snapshot exists and fingerprint matches |
| `executed` | Queue run recorded (dry-run or live per ops policy) |
| `replay_generated` | Bundle/corpus outputs verified per manifest `outputs` |
| `archived` | Terminal retired manifest; no forward promote |

**Forward transitions (typical):** `pending` → `validated` → `queued` → `executed` → `replay_generated`

**Retirement:** `replay_generated` → `archived` (CLI-only)

**Coarse groups:** See [experiment_orchestration_operations_v1.md](experiment_orchestration_operations_v1.md).

**Stale downgrade:** If job manifest changes after `manifest_fingerprint`, downgrade to `pending` until re-validated.

---

## Distinction from queue job `status`

`experiment_run_queue_v1` job/step `status` values (`running`, `completed`, `failed`, `dry_run`, etc.) describe **run-time bookkeeping** from `run_experiment_queue.py`. `operations_status` describes **maintainer ops lifecycle** on the sidecar. Both may appear in the viewer; they must not be conflated.

---

## Related

- [experiment_job_manifest_v1.md](experiment_job_manifest_v1.md)
- [experiment_run_queue_v1.md](experiment_run_queue_v1.md)
- [experiment_orchestration_operations_v1.md](experiment_orchestration_operations_v1.md)

*End of experiment orchestration ops manifest v1.*
