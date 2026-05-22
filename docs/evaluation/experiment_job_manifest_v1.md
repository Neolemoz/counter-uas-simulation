# Experiment Job Manifest (`experiment_job_manifest_v1`)

**Phase:** PLAT-SA-H3 — offline experiment orchestration foundations  
**Authority:** [AGENTS.md](../../AGENTS.md); [h3_offline_experiment_orchestration_plan.md](../platform/h3_offline_experiment_orchestration_plan.md)

Declarative offline workflow for scenario → replay bundle pipelines. **Not** parser-visible. **Not** live execution authority.

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `experiment_job_manifest_v1` |
| `schema_version` | string | `experiment_job_manifest_v1` |
| `manifest_id` | string | Stable manifest identifier |
| `title` | string | Human-readable label |
| `governance` | object | `notice`, `anti_claims[]` |
| `jobs` | array | One or more job definitions |

## Job object

| Field | Type | Description |
|-------|------|-------------|
| `job_id` | string | Unique within manifest |
| `scenario_pack_id` | string | Catalog pack id (e.g. `ridge_defense`) |
| `scenario_pack_ref` | string | Repo-relative path to `scenario_topology_v1` pack |
| `pipeline` | array | Ordered steps |
| `outputs` | object | Expected artifact paths (mirrors) |
| `lineage` | object | Optional parent refs (explanatory) |

## Pipeline step

| Field | Type | Description |
|-------|------|-------------|
| `step_id` | string | Stable step name |
| `step_type` | string | See table below |
| `args` | array | Optional extra CLI args |
| `enabled` | boolean | Default true |

### Step types

| `step_type` | CLI behavior |
|-------------|--------------|
| `validate_scenario` | `validate_scenario.py <pack_ref>` |
| `validation_mirror` | Write `experiment_validation_mirror_v1` JSON |
| `synthetic_demo_regen` | `sync_sa_catalog.py` (repack existing demo fixtures) |
| `observability` | `replay_observability.py single-run-report` (requires log paths in job context) |
| `narrative` | `replay_observability.py narrative` |
| `bundle_pack` | `replay_sa_bundle.py pack` |
| `catalog_sync` | `sync_sa_catalog.py` |
| `corpus_regen` | `run_replay_corpus_regen.py` (supports `--dry-run` via runner) |
| `runtime_capture` | `run_capture.py` template — **requires** `--allow-runtime-capture` on runner |

## Outputs object (typical)

```json
{
  "bundle_dir": "fixtures/sa_r0/demo_ridge_defense",
  "viewer_demo_pack_id": "ridge_defense",
  "corpus_entry_id": "demo_ridge_defense"
}
```

## Governance

Manifest copy must pass `governance_lint_sa.py`. Do not use operational readiness or certification language.

## Related

- [experiment_run_queue_v1.md](experiment_run_queue_v1.md)
- [experiment_workflow_scenario_to_replay_v1.md](experiment_workflow_scenario_to_replay_v1.md)
