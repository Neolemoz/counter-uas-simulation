# RT Experiment Workbench Contract (`rt_experiment_workbench_v1`)

**Phase:** PLAT-RT-X1  
**Authority:** [rt_x1_experimentation_workbench_plan.md](../platform/rt_x1_experimentation_workbench_plan.md)

Local experimentation artifacts and compare semantics. **Explanatory only** — not operational authority, rankings, or SA replay truth.

---

## 1. Manifest: `rt_experiment_manifest_v1`

| Field | Rule |
|-------|------|
| `schema` | `rt_experiment_manifest_v1` |
| `experiment_id` | Stable string id |
| `governance_banner` | Required explanatory line |
| `runs[]` | Pinned run records |

### Run record

| Field | Rule |
|-------|------|
| `run_id` | Unique within manifest |
| `label` | Human label |
| `session_id` | Ephemeral bridge session id at pin time |
| `recorded_at_utc` | ISO-8601 |
| `snapshot` | Pull channel payloads: `tactical_state`, `world_summary`, `lifecycle_state`, optional `entity_pose_mirror` |
| `terrain_context` | Optional V2 cognition line (layers flag, nearest ridge) |
| `capture_candidate_id` | Set by maintainer batch CLI |
| `capture_staging_ref` | Repo-relative path under `runs/rt_sandbox/captures/` |
| `tactical_annex_summary` | Denormalized annex counts (not full SA replay) |

---

## 2. Batch spec: `rt_experiment_batch_v1`

Maintainer YAML/JSON consumed by `rt_experiment_batch.py`.

| Field | Rule |
|-------|------|
| `experiment_id` | Matches manifest id |
| `runs[]` | Sequential queue |
| `dwell_s` | Sleep after start before stop |
| `template_id` | Optional `apply_runtime_template` |

Per run: `start_session` → optional template → dwell → `stop_session` → `capture_session` → update manifest.

---

## 3. Compare semantics

| Badge | Meaning |
|-------|---------|
| `mode_changed` | `tactical_mode` differs |
| `assignment_changed` | assigned target/interceptor differs |
| `tti_delta` | Numeric delta when both present |
| `pause_resume_delta` | Autonomous loop status differs |

**Forbidden:** winner labels, readiness scores, effectiveness claims.

---

## 4. UI boundaries

- Compare sources: live session slots (≤3) or pinned manifest runs
- No browser `capture_session`; batch panel shows CLI command only
- No SA viewer import; no federation writes

---

## Related

- [rt_experiment_model_v1.md](rt_experiment_model_v1.md) (PLAN-RT-F5 manifest supplements)
- [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md) (PLAN-RT-F5 maintainer pipeline)
- [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md) (PLAT-RT-F1 derive)
- [rt_experiment_sweep_catalog_v1.md](rt_experiment_sweep_catalog_v1.md) (PLAT-RT-F1 sweep browser)
- [rt_experiment_analytics_ui_v1.md](rt_experiment_analytics_ui_v1.md) (PLAT-RT-F1 panels)
- [rt_experiment_annex_review_ui_v1.md](rt_experiment_annex_review_ui_v1.md) (PLAT-RT-F3 annex timelines)
- [rt_experiment_continuity_review_v1.md](rt_experiment_continuity_review_v1.md) (PLAT-RT-F3 continuity hub)
- [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md)
- [rt_v2_terrain_realism_v1.md](rt_v2_terrain_realism_v1.md)
