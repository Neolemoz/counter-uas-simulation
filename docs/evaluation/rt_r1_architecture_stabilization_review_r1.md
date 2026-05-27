# RT-R1 — Architecture Stabilization Review R1

**Phase:** PLAN-RT-R1 — runtime architecture stabilization (review-only)  
**Prerequisite:** PLAT-RT-G5 frozen  
**Plan:** [rt_r1_runtime_architecture_stabilization_plan.md](../platform/rt_r1_runtime_architecture_stabilization_plan.md)  
**Governance review:** [rt_r1_governance_review_r1.md](rt_r1_governance_review_r1.md)  
**Freeze audit:** [rt_r1_freeze_audit.md](rt_r1_freeze_audit.md)  
**Consolidation roadmap:** [rt_roadmap_post_g5_v1.md](rt_roadmap_post_g5_v1.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Item | Verdict |
|------|---------|
| Lifecycle governance safety | **Pass-with-conditions** — doc/code aligned; workflow/adapter overlap documented |
| Authority separation (RT vs mirrors vs SA) | **Pass** — command-authoritative registry; mirrors and capture non-authoritative |
| Sync vs telemetry semantics | **Pass-with-conditions** — parallel revision vocabulary and poll paths |
| Capture determinism | **Pass-with-conditions** — normalization deterministic for fixed inputs; tri-source pose history intentional |
| Runtime governance hardening | **Pass** — deny-by-default, isolation, ROS allow-list when adapter on |
| Auditability | **Pass-with-conditions** — dual logs append-only; audit `command_type` mixes user and internal events |
| Technical debt | **Debt** — `session_manager.py` concentration, triple telemetry/pose paths |
| RT↔SA boundary | **Pass** — export boundary, approval gate, no auto-import |

**Recommendation:** Freeze **PLAN-RT-R1** (docs only). Address **P0** findings in [rt_roadmap_post_g5_v1.md](rt_roadmap_post_g5_v1.md) via new scoped waves before telemetry UI, Cesium, SA bridge, or orchestration expansion.

### Architectural strengths

- Deny-by-default bridge with explicit forbidden command sets and `authority_scope` gate.
- Transient session model with terminal states, cleanup timeouts, and `CAPTURED` exempt from auto-cleanup.
- RT↔SA separation: writable roots limited to `runs/rt_sandbox/`; `reject_auto_sa_import`; normalized manifests cannot use `session_id` as lineage parent.
- Capture pipeline: staging → normalization → validation → maintainer approval without SA packager invocation.
- Gazebo path mock-by-default (`enable_gazebo_adapter=false`); `RuntimeStub` remains default.

### Unresolved architecture risks

- Revision counter proliferation (`world.revision`, `sync_seq`, `telemetry_revision`, `conversion_revision`) without a single maintainer glossary in contracts.
- Three telemetry backing stores and three pose sources for `entity_pose_mirror` increase misread risk under UI expansion.
- Telemetry mirror may lag between entity operations (feedback polled post-entity; telemetry poll on fewer transitions).
- Audit `command_type` vocabulary includes internal events not in `ALLOWED_COMMANDS` — safe for bridge, risky for future audit parsers.
- ~1900-line `session_manager.py` couples lifecycle, adapter, telemetry, capture, and normalization.

### Prerequisites before future RT expansion

1. PLAN-RT-R1 frozen; P0 roadmap items acknowledged.
2. Contract revision glossary published (P0) and referenced by mirror/capture docs.
3. Channel payload `source` / authority banner consistency for all mirror-fed telemetry (P0).
4. New expansion wave plan + governance review + freeze audit per feature (SA ingest, Cesium, full telemetry UI).
5. `RuntimeStub` default and adapter opt-in preserved unless explicitly re-opened.

**Stop line after RT-R1:** No telemetry UI, Cesium runtime viz, autonomous runtime, multi-runtime orchestration, or distributed infra until post-R1 implementation waves complete P0/P1 prerequisites.

---

## 1. Lifecycle consistency review

**Doc claim:** [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md) — happy path `created → running ↔ paused → stopped → captured|discarded`; failure states `failed`, `runtime_crashed`, `bridge_disconnected`, `cleanup_pending`.

**Code behavior:** [lifecycle.py](../../platform/rt-sandbox-bridge/rt_sandbox/lifecycle.py) `SessionState` matches doc strings. [session_manager.py](../../platform/rt-sandbox-bridge/rt_sandbox/session_manager.py) `_tick_timeouts` implements:

- `bridge_ready_timeout` from `CREATED` → `FAILED` + cleanup
- `max_session_duration` on `RUNNING` → `STOPPED`
- `runtime_crashed` when child not alive
- `cleanup_after` elapsed → `auto_cleanup` for non-`CAPTURED` sessions
- **`CAPTURED` skips auto-cleanup** (preserves session record until `discard_session`)

**Cleanup helpers (audited):**

| Helper | Clears |
|--------|--------|
| `_clear_world_with_audit` | `WorldStateStore` / entity registry |
| `_clear_telemetry_with_audit` | `TelemetrySubscriptionStore` |
| `_terminate_runtime_with_audit` | pose sync, telemetry mirror, runtime terminate, adapter teardown audits |
| `_clear_workflow_state` | `session.workflow = None` |

**Overlap:** Session lifecycle vs [workflow.py](../../platform/rt-sandbox-bridge/rt_sandbox/workflow.py) step state vs adapter attach on `start_session` / detach on teardown. Workflow does not own session terminal states; `reset_session` clears world and triggers adapter resync without ending session.

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-LIFE-01 | Pass | — | State enum and `_TRANSITIONS` align with contract for user commands |
| R1-LIFE-02 | Pass | P2 | **Closed by PLAT-RT-R3b** — [rt_lifecycle_transitions_v1.md](rt_lifecycle_transitions_v1.md); `bridge_disconnected` documented as reserved |
| R1-LIFE-03 | Pass | — | `capture_session` only from `stopped`; normalization failure blocks `captured` transition |
| R1-LIFE-04 | Pass-with-conditions | P1 | Workflow reset on capture/discard documented; template apply does not reset workflow automatically |
| R1-LIFE-05 | Pass | — | Orphan/runtime cleanup via `_terminate_runtime_with_audit` + `orphan_cleanup` audit |

---

## 2. Authority consistency review

**Authority matrix (extended from [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)):**

| Surface | Owner | Authoritative for replay? |
|---------|-------|---------------------------|
| `EntityRegistry` / bridge commands | Bridge | Command truth for sandbox world |
| `WorldStateStore.revision` | Bridge | Mutation counter; sent as `bridge_revision` |
| Adapter `MockSimState` / worker entities | Sim (when adapter on) | Sim-side pose truth; does not overwrite registry |
| `PoseSyncMirror` | Bridge (explanatory) | No — drift/stale/mismatch only |
| `TelemetryMirror` | Bridge (explanatory) | No — adapter-fed read model |
| `TelemetrySubscriptionStore` | Bridge | Pull ring; not SA replay |
| Staging capture / normalized manifest | RT staging | No — approval + export boundary required |

**Re-validation:**

- `session_id` must not be SA corpus lineage parent — enforced in [export_boundary.py](../../platform/rt-sandbox-bridge/rt_sandbox/export_boundary.py) `validate_normalized_manifest`.
- `reject_auto_sa_import` — no packager path from bridge.
- `authority_scope` must match `rt_sandbox_prototype` on commands.

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-AUTH-01 | Pass | — | Registry command-authoritative; mirrors labeled non-authoritative in contracts |
| R1-AUTH-02 | Pass-with-conditions | P0 | `entity_pose_mirror` channel may show adapter mirror while registry authoritative at command time — partial `source` on some payloads only |
| R1-AUTH-03 | Pass | — | `send_runtime_command` subcommands gated; no browser→ROS |
| R1-AUTH-04 | Risk | P1 | `apply_runtime_template` mutates world; adapter resync relies on subsequent entity ops / `reset_session` |
| R1-AUTH-05 | Pass | — | Capture approval requires `normalization_status: normalized` |

---

## 3. Sync vs telemetry semantics review

### Revision vocabulary (glossary)

| Name | Location | Meaning |
|------|----------|---------|
| `world.revision` | `WorldStateStore` | Increments on registry mutation |
| `bridge_revision` | Adapter IPC | Alias of `world.revision` at command time |
| `sync_revision` | `EntitySyncEntry` | World revision when command recorded in pose mirror |
| `sync_seq` | `adapter_worker` / `PoseSyncMirror.last_sync_seq` | Adapter feedback generation counter |
| `telemetry_seq` | Worker bundle | Adapter telemetry poll generation |
| `telemetry_revision` | `TelemetryMirror` | Copy of `telemetry_seq` from last poll |
| `registry_revision` | Normalization entity history | From snapshot entity state |
| `conversion_revision` | Normalization output | Monotonic per staging dir / input hash policy |
| `world_revision_hint` | Telemetry bundle | Adapter hint; parallel to bridge revision |

### Stale detection

- Pose: `adapter_feedback_stale_s` in [pose_sync.py](../../platform/rt-sandbox-bridge/rt_sandbox/pose_sync.py) — duplicate `_parse_utc`.
- Telemetry: `telemetry_stale_s` in [telemetry_bridge.py](../../platform/rt-sandbox-bridge/rt_sandbox/telemetry_bridge.py) — independent threshold.

### Poll orchestration

- Post-entity: `run_post_entity_sync` polls feedback after spawn/move/delete.
- Transitions: `_poll_telemetry_bridge` on subscribe and some session paths.
- Manual: `adapter_poll_feedback` / `adapter_poll_telemetry` via `send_runtime_command`.

`world_summary` exposes both `sync_health` (pose mirror) and `telemetry_health` / `telemetry_revision` (telemetry mirror).

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-SYNC-01 | Pass-with-conditions | P0 | Revision glossary not consolidated into single contract doc |
| R1-SYNC-02 | Pass-with-conditions | P1 | Duplicate `_parse_utc` / stale helpers in G3 and G4 modules |
| R1-SYNC-03 | Pass-with-conditions | P1 | Telemetry poll not unified with post-entity feedback poll — mirror freshness gap |
| R1-SYNC-04 | Pass | — | `sync_seq` vs `sync_revision` roles distinct in implementation |
| R1-SYNC-05 | Pass-with-conditions | P2 | `world_revision_hint` may diverge from `world.revision` under load — explanatory only |

---

## 4. Capture determinism review

**Path:** `capture_session` → staging artifacts → [capture_normalize.py](../../platform/rt-sandbox-bridge/rt_sandbox/capture_normalize.py) at capture time (pre-teardown) → validation manifest → export audit.

**Determinism:**

- `_hash_raw_inputs` fixes normalization input identity.
- `_next_conversion_revision` monotonic per staging dir and input hash.
- Entity pose history merges registry, feedback mirror, and telemetry mirror with explicit `source` / `telemetry_source` tags.
- Provenance strips external `audit_ref` from normalized output.

**Replay-boundary:** Normalized artifacts carry `NORMALIZATION_GOVERNANCE_BANNER`; not SA replay authority. [capture.py](../../platform/rt-sandbox-bridge/rt_sandbox/capture.py) audit summarization is explanatory.

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-CAP-01 | Pass | — | Stable artifact set and schema ids for stub/mock paths |
| R1-CAP-02 | Pass-with-conditions | P1 | Tri-source pose history is intentional but increases reviewer cognitive load |
| R1-CAP-03 | Pass | — | `conversion_revision` policy documented in code; re-run with same inputs yields same revision logic |
| R1-CAP-04 | Pass | — | Transient session metadata cleared on teardown after capture |
| R1-CAP-05 | Pass | — | Export audit append-only; four G5 event types in freeze audit |

---

## 5. Runtime governance hardening review

| Check | Implementation | Result |
|-------|----------------|--------|
| Deny-by-default commands | `classify_command`, `ALLOWED_COMMANDS` | Pass |
| Forbidden operational commands | `RT_FORBIDDEN_COMMANDS` | Pass |
| Entity/pose bounds | `WORLD_BOUNDS`, type limits | Pass |
| Rate limits | burst/sustained command + telemetry Hz cap | Pass |
| Session isolation | `max_concurrent_sessions=1` | Pass |
| Writable roots | [isolation.py](../../platform/rt-sandbox-bridge/rt_sandbox/isolation.py) | Pass |
| ROS allow-list | [ros_allowlist.py](../../platform/rt-sandbox-bridge/rt_sandbox/ros_allowlist.py) when adapter enabled | Pass |
| Default adapter off | `enable_gazebo_adapter=false` | Pass |

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-GOV-01 | Pass | — | Governance constants match `rt_runtime_governance_v1` prototype table |
| R1-GOV-02 | Pass-with-conditions | P0 | Recommend explicit `source` + authority banner on all mirror channel payloads (partial today) |
| R1-GOV-03 | Pass | — | No escalation path from failed session to corpus/federation |
| R1-GOV-04 | Pass-with-conditions | P2 | `send_runtime_command` surface area grows with adapter — maintain subcommand allow-list discipline |

---

## 6. Auditability consistency review

**Dual logs:**

1. `rt_session_audit_log_v1` — per-session JSON via [audit_log.py](../../platform/rt-sandbox-bridge/rt_sandbox/audit_log.py).
2. `rt_export_boundary_audit_v1` — JSONL via [export_audit_log.py](../../platform/rt-sandbox-bridge/rt_sandbox/export_audit_log.py) for capture/export/normalization.

Both are append-only. Capture report distills session audit; normalization removes external audit refs from provenance.

**Overlap:** Sync events (`sync_update`, `sync_stale`, `sync_mismatch`) vs telemetry events (`telemetry_update`, `telemetry_stale`, `telemetry_feedback_lost`) vs lifecycle (`bridge_ready_timeout`, `runtime_crashed`, `auto_cleanup`) vs adapter (`adapter_attach`, `adapter_teardown`).

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-AUDIT-01 | Pass | — | Append-only writes; no in-place mutation of audit files |
| R1-AUDIT-02 | Pass-with-conditions | P0 | `command_type` mixes user commands and internal events — document `event_kind` taxonomy for summarizers |
| R1-AUDIT-03 | Pass | — | Export audit separate from session audit — clear lineage |
| R1-AUDIT-04 | Pass-with-conditions | P1 | `issued_by="bridge"` and `command_id=None` on auto events — consistent but must stay parser-safe |
| R1-AUDIT-05 | Pass | — | Normalization rejection events recorded in export audit |

---

## 7. Technical debt review

| Debt area | Severity | Notes |
|-----------|----------|-------|
| `session_manager.py` size (~1900 lines) | High | Lifecycle, entity, telemetry, adapter, workflow, capture, normalization, timeouts |
| Triple telemetry stores | Medium | `TelemetryBuffer`, `TelemetrySubscriptionStore`, `TelemetryMirror` |
| Triple pose sources for mirrors | Medium | Registry, pose feedback, telemetry mirror — normalization merges |
| Duplicate stale UTC parsing | Low | `pose_sync.py` and `telemetry_bridge.py` |
| Adapter/bridge coupling | Medium | IPC and poll logic invoked from session manager |
| Audit vocabulary drift | Medium | Internal events share `command_type` with user commands |

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-DEBT-01 | Debt | P2 | Decompose `session_manager` in maintenance wave without behavior change |
| R1-DEBT-02 | Debt | P1 | Unify or deprecate heartbeat `TelemetryBuffer` vs subscription path |
| R1-DEBT-03 | Debt | P1 | Shared stale-check utility for G3/G4 |
| R1-DEBT-04 | Debt | P2 | Architecture entropy if expansion proceeds without P0/P1 closure |

---

## 8. RT↔SA boundary re-validation

Re-run of [rt_s1_architecture_readiness_review_r1.md](rt_s1_architecture_readiness_review_r1.md) contamination matrix against G5 surfaces:

| Vector | Mitigation | Residual |
|--------|------------|----------|
| Live telemetry → SA viewer | SA static JSON only; no viewer changes | **Low** |
| `session_id` as lineage parent | Export boundary lint + manifest rules | **Low** |
| capture → auto import | `reject_auto_sa_import`; approval CLI | **Low** |
| Federation live write | No RT path | **Low** |
| H3 queue merge | Separate entrypoint; `--allow-runtime-capture` | **Low** if maintainers read boundary doc |
| Normalized capture as replay truth | Banners + not SA schema | **Low** |
| Future Cesium / telemetry UI | Mirrors may be misread as truth | **Medium** without P0 banners |

| Finding ID | Verdict | Tier | Summary |
|------------|---------|------|---------|
| R1-SA-01 | Pass | — | `platform/sa-r0-viewer/` untouched by RT line |
| R1-SA-02 | Pass | — | No parser/topic/schema changes |
| R1-SA-03 | Pass-with-conditions | P0 | UI expansion must preserve mirror ≠ authority lexicon |
| R1-SA-04 | Pass | — | Federation/orchestration CLIs cannot start RT sessions |
| R1-SA-05 | Risk | P1 | Future SA bridge wave must be explicit; G5 stop line insufficient alone |

---

## Appendix A — Contract ↔ code cross-reference

| Layer | Contracts | Implementation | Tests / CLIs |
|-------|-----------|----------------|--------------|
| Bridge / session | `rt_bridge_contract_v1`, `rt_session_lifecycle_v1`, `rt_session_manager_ownership_v1` | `session_manager.py` (facade), `session_*` handlers, `lifecycle.py`, `bridge_server.py` | lifecycle, session commands |
| Governance | `rt_runtime_governance_v1` | `governance.py`, `isolation.py` | forbidden commands, bounds |
| World / workflow | S3/S6 plans, `rt_workflow_contract_v1` | `world_state.py`, `entity_registry.py`, `workflow.py`, `templates.py` | template/workflow tests |
| Adapter | `rt_gazebo_ros_boundary_v1` | `runtime_adapter.py`, `adapter_worker.py`, `adapter_ipc.py` | adapter inspect |
| Sync G3 | `rt_adapter_feedback_v1`, `rt_runtime_synchronization_v1` | `pose_sync.py`, `adapter_sync.py` | sync stale/mismatch |
| Telemetry G4 | `rt_adapter_telemetry_v1` | `telemetry_bridge.py`, `telemetry_subscriptions.py`, `telemetry.py` | telemetry pull |
| Capture G5 | `rt_capture_normalization_v1`, `rt_sa_export_boundary_v1`, `rt_capture_continuity_v1` | `capture.py`, `capture_normalize.py`, `export_boundary.py` | normalization, approval |
| Audit | wave audits (implicit) | `audit_log.py`, `export_audit_log.py` | capture summarization |

---

## Appendix B — Audit `command_type` taxonomy

| `command_type` | Kind | In `ALLOWED_COMMANDS`? |
|----------------|------|------------------------|
| `start_session`, `pause_session`, `resume`, `stop_session`, `discard_session`, `reset_session`, `capture_session` | user_command | Yes |
| `spawn_entity`, `move_entity`, `delete_entity` | user_command | Yes |
| `subscribe_telemetry`, `unsubscribe_telemetry` | user_command | Yes |
| `list_runtime_templates`, `apply_runtime_template` | user_command | Yes |
| `start_workflow`, `advance_workflow`, `reset_workflow`, `reload_workflow`, `get_workflow_state` | user_command | Yes |
| `send_runtime_command` (user) | user_command | Yes |
| `adapter_attach`, `adapter_detach`, `adapter_teardown`, `adapter_health` | adapter | Subcommand / internal |
| `adapter_feedback_lost`, `telemetry_feedback_lost` | adapter | Internal |
| `sync_update`, `sync_stale`, `sync_mismatch` | sync | Internal |
| `telemetry_update`, `telemetry_stale`, `telemetry_feedback_lost` | telemetry | Internal |
| `telemetry_cleanup`, `telemetry_buffer_trim`, `telemetry_snapshot` | telemetry | Internal |
| `mock_inject_drift` | adapter | Subcommand |
| `entity_cleanup`, `orphan_cleanup` | cleanup | Internal |
| `auto_cleanup`, `bridge_ready_timeout`, `max_session_duration`, `runtime_crashed` | lifecycle | Internal |
| `reset_workflow` | lifecycle | Internal (also user `reset_workflow` — name collision risk) |

**Drift flag (R1-AUDIT-02):** Summarizers must treat non-`ALLOWED_COMMANDS` types as internal `event_kind`, not client-invokable commands.

---

## Consolidation recommendations (summary)

See [rt_roadmap_post_g5_v1.md](rt_roadmap_post_g5_v1.md) for tiered waves linked to finding IDs.

**P0 (before any expansion):** R1-SYNC-01, R1-AUTH-02, R1-GOV-02, R1-AUDIT-02, R1-SA-03  
**P1 (before UI/Cesium/SA bridge):** R1-SYNC-02, R1-SYNC-03, R1-CAP-02, R1-AUTH-04, R1-DEBT-02, R1-DEBT-03, R1-SA-05  
**P2 (maintenance):** R1-LIFE-02, R1-DEBT-01, R1-SYNC-05, R1-GOV-04
