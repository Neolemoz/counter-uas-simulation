# RT Audit Event Vocabulary (`rt_audit_event_vocabulary_v1`)

**Phase:** PLAT-RT-R1a — session audit taxonomy; PLAT-RT-R2e capture-pose events; PLAT-RT-R3b lifecycle audit table; PLAN-RT-R2f handoff events (specified only); PLAN-RT-TAC1 tactical events (reserved only)  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](rt_r1_architecture_stabilization_review_r1.md) appendix B; [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)

Disambiguates `rt_session_audit_log_v1` entries. **`command_type` is retained** for backward compatibility; **`event_kind`** classifies parser behavior.

---

## 1. Fields

| Field | Role |
|-------|------|
| `command_type` | Event verb (user commands and internal events share this field historically) |
| `event_kind` | Taxonomy class — see §2 |
| `issued_by` | `test`, client id, or `bridge` for automated events |
| `command_id` | User command UUID; `null` for bridge-initiated events |

Export boundary log (`rt_export_boundary_audit_v1`) uses `event_type` + `event_kind: export`.

---

## 2. Event kinds

| `event_kind` | Description |
|--------------|-------------|
| `user_command` | Client-invokable bridge commands |
| `adapter` | Adapter attach/detach/teardown/health/feedback loss |
| `sync` | Pose sync mirror updates, stale, mismatch |
| `telemetry` | Telemetry mirror, buffer trim, cleanup, snapshot |
| `lifecycle` | Timeouts, crashes, auto-cleanup, duration limits |
| `cleanup` | Entity/orphan cleanup |
| `export` | Export-boundary JSONL only (includes handoff events per §5) |
| `capture` | Capture-time pose cognition (PLAT-RT-R2e) |
| `handoff` | *(reserved)* — not used in JSONL today; handoff events use `event_kind: export` |
| `tactical` | *(reserved)* — PLAT-RT-TAC2+ sandbox tactical controller and mode events |

---

## 3. `command_type` → `event_kind` map

| `command_type` | `event_kind` |
|----------------|--------------|
| `start_session`, `pause_session`, `resume`, `stop_session`, `discard_session`, `reset_session`, `capture_session` | `user_command` |
| `spawn_entity`, `move_entity`, `delete_entity` | `user_command` |
| `subscribe_telemetry`, `unsubscribe_telemetry` | `user_command` |
| `list_runtime_templates`, `apply_runtime_template` | `user_command` |
| `start_workflow`, `advance_workflow`, `reset_workflow`, `reload_workflow`, `get_workflow_state` | `user_command` |
| `send_runtime_command` | `user_command` |
| `adapter_attach`, `adapter_detach`, `adapter_teardown`, `adapter_health`, `adapter_feedback_lost`, `adapter_poll_feedback`, `adapter_poll_telemetry`, `mock_inject_drift` | `adapter` |
| `sync_update`, `sync_stale`, `sync_mismatch` | `sync` |
| `template_resync_requested`, `template_resync_completed`, `template_resync_skipped`, `template_resync_stale` | `sync` |
| `telemetry_update`, `telemetry_stale`, `telemetry_feedback_lost`, `telemetry_cleanup`, `telemetry_buffer_trim`, `telemetry_snapshot` | `telemetry` |
| `fidelity_truth_update`, `fidelity_truth_stale` | `telemetry` |
| `fidelity_truth_mismatch` | `sync` |
| `entity_cleanup`, `orphan_cleanup` | `cleanup` |
| `auto_cleanup`, `bridge_ready_timeout`, `max_session_duration`, `runtime_crashed` | `lifecycle` |
| `capture_pose_authority`, `capture_pose_stale`, `capture_pose_mismatch` | `capture` |
| `fidelity_capture_snapshot` | `capture` |

Unknown types default to `lifecycle` (bridge-initiated).

Poll subcommands may emit `sync_*` or `telemetry_*` audit events via poll apply paths — see [rt_runtime_subcommand_registry_v1.md](rt_runtime_subcommand_registry_v1.md) §2.

### 3.2 Sync, telemetry, and capture audit events (PLAT-RT-R3d)

Revision context — full policy: [rt_world_revision_hint_policy_v1.md](rt_world_revision_hint_policy_v1.md).

| `command_type` / `event_type` | `event_kind` | Revision relevance |
|-------------------------------|--------------|-------------------|
| `sync_update` | `sync` | Feedback poll OK; `sync_seq` may advance |
| `sync_stale` | `sync` | Pose drift — not hint comparison |
| `sync_mismatch` | `sync` | Ref/entity map — not hint comparison |
| `telemetry_update` | `telemetry` | Updates `telemetry_revision`, `world_revision_hint` |
| `telemetry_stale` | `telemetry` | Poll age — explanatory only |
| `capture_normalized` | `export` | Normalization succeeded; sets `conversion_revision` |
| `export_pose_normalized` | `export` | Pose cognition; includes `conversion_revision` |

### 3.1 Lifecycle audit events (PLAT-RT-R3b)

Bridge-initiated lifecycle events (`issued_by: bridge`, `command_id: null`):

| `command_type` | `event_kind` | When | Typical `result` |
|----------------|--------------|------|-------------------|
| `bridge_ready_timeout` | `lifecycle` | `created` past deadline without reaching `running` | `failed` |
| `max_session_duration` | `lifecycle` | `running` exceeds wall-clock cap | `RESOURCE_LIMIT_EXCEEDED` |
| `runtime_crashed` | `lifecycle` | Runtime child not alive (passive detect or command path) | `RUNTIME_UNAVAILABLE` |
| `auto_cleanup` | `lifecycle` | `cleanup_after` elapsed; session not `captured` | `OK` |

See [rt_lifecycle_transitions_v1.md](rt_lifecycle_transitions_v1.md) §2–§3 for transition and teardown context.

---

## 4. Export-boundary `event_type` map (PLAT-RT-R2e)

| `event_type` | When |
|--------------|------|
| `capture_requested`, `capture_rejected`, `capture_validated` | Capture boundary lifecycle |
| `capture_normalized`, `provenance_injected`, `normalization_validation`, `normalization_rejected` | PLAT-RT-G5 normalization |
| `export_pose_normalized` | Pose cognition assessed after successful normalization |

All use `event_kind: export`. See [rt_runtime_export_semantics_v1.md](rt_runtime_export_semantics_v1.md).

---

## 5. Export-boundary handoff events (PLAN-RT-R2f / PLAT-RT-SA1)

Specified in PLAN-RT-R2f; **emitted by PLAT-RT-SA1** maintainer CLIs (`rt_handoff_review.py`, `rt_sa_import.py`) via `ExportAuditLog`.

| `event_type` | `event_kind` | When |
|--------------|--------------|------|
| `handoff_ready` | `export` | Normalized capture meets review preconditions (conceptually post-`capture_validated`) |
| `handoff_reviewed` | `export` | Maintainer completed review checklist; may precede `capture_approved` |
| `handoff_rejected` | `export` | SA import declined — distinct from `capture_rejected` (capture-time failure) |
| `handoff_import_deferred` | `export` | Import postponed without rejecting staging artifact |
| `handoff_import_prepared` | `export` | `rt_sa_import prepare` wrote handoff manifest |
| `handoff_import_committed` | `export` | Corpus commit completed via `rt_sa_import commit` |

### Mapping to current signals

| R2f concept | Existing artifact / `event_type` |
|-------------|----------------------------------|
| Ready | `capture_validated` + `normalization_status: normalized` |
| Reviewed + approved | `capture_approved`, `approval.json` |
| Conversion declared | `conversion_manifest_written`, `conversion.json` |
| Capture-time failure | `capture_rejected`, `normalization_rejected` |

See [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md).

---

## 6. Tactical audit events (PLAN-RT-TAC1 — reserved)

Specified in PLAN-RT-TAC1; **not emitted** until PLAT-RT-TAC2+. All use `event_kind: tactical` unless noted.

| `command_type` | When | Typical `issued_by` |
|----------------|------|---------------------|
| `tactical_mode_changed` | Manual / Assisted / Autonomous transition | `bridge` or client |
| `tactical_candidate_selected` | Highlight for review | client or `bridge` |
| `tactical_assignment_committed` | `assign_candidate` or autonomous commit | client or `bridge` |
| `tactical_assignment_cleared` | `clear_assignment` | client |
| `tactical_recommendation_issued` | Assisted `request_recommendation` | `bridge` |
| `tactical_recommendation_approved` | Assisted approval gate passed | client |
| `tactical_recommendation_rejected` | Assisted reject | client |
| `tactical_autonomous_paused` | Revert toward Manual | client |
| `tactical_capture_annex_written` | TAC5 — annex at capture | `bridge` |
| `tactical_capture_annex_empty` | TAC5 — no tactical history | `bridge` |
| `tactical_capture_snapshot` | TAC5 — capture rollup (mode, counts) | `bridge` |
| `tactical_switch` | TAC5 — mode switch rollup | `bridge` |
| `tactical_assignment` | TAC5 — assignment timeline rollup | `bridge` |
| `tactical_lock` | TAC5 — lock event rollup | `bridge` |
| `tactical_pause_resume` | TAC5 — pause/resume rollup | `bridge` |
| `tactical_engine_divergence_note` | RT mirror vs engine topic mismatch (reserved) | `bridge` |

See [rt_tac1_tactical_governance_v1.md](rt_tac1_tactical_governance_v1.md), [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md).

---

## 7. Parser-safe rules

1. Treat `event_kind != user_command` as **non-invokable** — never expose as bridge API.
2. Summarize capture audits by `event_kind` groups when building reports.
3. Do not infer replay authority from audit entries.

---

## Related

- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)
- [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md)
- [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md)
- [rt_runtime_export_semantics_v1.md](rt_runtime_export_semantics_v1.md)
- [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)
- [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md)
- [rt_tac1_tactical_governance_v1.md](rt_tac1_tactical_governance_v1.md)
