# Experiment Run Queue Snapshot (`experiment_run_queue_v1`)

**Phase:** PLAT-SA-H3  
**Producer:** `scripts/evaluation/run_experiment_queue.py`

Frozen snapshot of offline job queue state for reviewer mirrors. **Not** a live message broker.

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `experiment_run_queue_v1` |
| `schema_version` | string | `experiment_run_queue_v1` |
| `queue_id` | string | Snapshot identifier |
| `manifest_ref` | string | Path to source manifest |
| `governance_banner` | string | `ORCHESTRATION MIRROR — not live execution state` |
| `jobs` | array | Job status records |
| `steps` | array | Optional flat step log for timeline UI |

## Job record

| Field | Type | Description |
|-------|------|-------------|
| `job_id` | string | From manifest |
| `scenario_pack_id` | string | Pack id |
| `status` | enum | `pending` \| `running` \| `completed` \| `failed` \| `skipped` |
| `phase` | string | Current or last `step_id` |
| `started_at` | string | ISO-8601 or empty |
| `finished_at` | string | ISO-8601 or empty |
| `artifact_refs` | array | Repo-relative paths produced |
| `error_hint` | string | Failure message (non-authoritative) |
| `provenance` | object | `run_id`, `log_path`, `bundle_path`, `corpus_ref` mirrors |

## Step record (timeline)

| Field | Type |
|-------|------|
| `step_id` | string |
| `step_type` | string |
| `status` | `pending` \| `running` \| `completed` \| `failed` \| `skipped` \| `dry_run` |
| `duration_ms` | number |
| `command` | string |
| `hint` | string |

## Viewer consumption

Loaded from `platform/sa-r0-viewer/public/demo/orchestration/<queue_id>.json` (static fetch only).

## Related

- H1 `sandbox_orchestration_context_v0` — thin view over this snapshot + manifest ref
