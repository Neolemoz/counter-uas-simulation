# RT Capture Pose Cognition (`rt_capture_pose_cognition_v1`)

**Phase:** PLAT-RT-R2e — capture pose interpretation at export boundary  
**Authority:** [rt_r2e_capture_pose_cognition_plan.md](../platform/rt_r2e_capture_pose_cognition_plan.md)

Defines how reviewers interpret tri-source pose data at `capture_session` and in normalized exports. Closes R1-CAP-02.

---

## 1. Authority at capture time

| Layer | Term | Label | Role |
|-------|------|-------|------|
| Bridge registry | **Bridge intent** | `command_authoritative` | Accepted commands and `EntityRegistry` poses frozen into `snapshot.json` |
| G3 mirror | **Adapter runtime truth** (sync) | `explanatory_sync` | `feedback_pose`, `drift_m`, `sync_health` at capture instant |
| G4 mirror | **Adapter runtime truth** (telemetry) | `explanatory_telemetry` | `telemetry_mirror_pose`; may lag last entity poll |
| Staging bundle | **Captured runtime truth** | `replay_boundary_scoped` | Immutable artifacts under `runs/rt_sandbox/captures/` after capture |

**Normative rule:** `command_pose` in `entity_pose_history` is the **only authoritative pose** for normalized RT replay-boundary staging. Feedback and telemetry poses are **explanatory-only**; divergence must not be merged into `command_pose`.

See [rt_authority_model_v1.md](rt_authority_model_v1.md).

---

## 2. Source comparison

| Source | Artifact / field | When captured | Authoritative? |
|--------|------------------|---------------|----------------|
| Registry snapshot | `snapshot.json` → `entity_states[].pose` | `capture_session` pre-teardown | Yes (same values as `command_pose`) |
| Normalized history | `entity_pose_history[].command_pose` | Normalization from snapshot | Yes |
| G3 feedback | `entity_pose_history[].feedback_pose` | From `pose_sync_summary` in `NormalizationContext` | No |
| G4 telemetry | `entity_pose_history[].telemetry_mirror_pose` | From `telemetry_mirror_summary` | No |

`snapshot.json` and `command_pose` must agree for a given `entity_id` at capture. Mirrors may diverge; that divergence is **explanatory-only mismatch**, not a reason to rewrite registry poses.

---

## 3. Stale pose at capture

| Condition | Meaning | Surfaced as |
|-----------|---------|-------------|
| `pose_sync_summary.sync_health == "stale"` | Drift above configured threshold on one or more entities | `capture_pose_stale` audit; `session_flags` includes `sync_stale` |
| `sync_health == "mismatch"` | Unknown feedback entity or sim ref mismatch | `capture_pose_mismatch` audit |
| `sync_health == "feedback_lost"` | Adapter poll lost | `capture_pose_stale` when adapter attached |
| Missing `feedback_pose` with adapter on | Partial feedback at capture | `partial_feedback` per-entity or session flag |

Assessment uses existing G3 thresholds and summaries — no new drift constants in R2e.

When `adapter_attached` is false (stub default), missing feedback is **expected** — not stale and not ambiguous.

---

## 4. Partial feedback during capture

| Scenario | Interpretation |
|----------|----------------|
| Entity in registry, no `feedback_pose`, adapter on | Partial feedback — explanatory; `command_pose` remains authoritative |
| Entity in registry, no mirrors, adapter off | Normal stub path |
| Feedback entity not in registry | Mismatch — `capture_pose_mismatch` |
| `telemetry_mirror_pose` present without `feedback_pose` | Telemetry-only mirror lag — flag `telemetry_without_sync` (informational) |

Normalization **must not fail** on pose ambiguity flags.

---

## 5. Manifest block and audit events

### `capture_pose_cognition` (additive on `rt_normalized_capture_v1`)

Written by [capture_pose_cognition.py](../../platform/rt-sandbox-bridge/rt_sandbox/capture_pose_cognition.py):

| Field | Description |
|-------|-------------|
| `assessment_utc` | ISO-8601 UTC |
| `adapter_attached` | bool at assessment |
| `sync_health` | Session `sync_health` from G3 summary when present |
| `session_flags` | e.g. `sync_stale`, `partial_feedback` |
| `per_entity` | `entity_id`, `authoritative_field` (`command_pose`), `flags[]` |
| `governance_banner` | Capture cognition banner |

### Session audit (`event_kind: capture`)

| `command_type` | When |
|----------------|------|
| `capture_pose_authority` | After successful normalization — records authoritative field per entity |
| `capture_pose_stale` | Stale sync or missing/stale feedback when adapter attached |
| `capture_pose_mismatch` | Mismatch or per-entity drift flagged at capture |

### Export boundary audit (`event_kind: export`)

| `event_type` | When |
|--------------|------|
| `export_pose_normalized` | After normalization — cognition summary + `conversion_revision` (distinct from `capture_normalized`) |

---

## 6. Non-goals

- SA replay import or corpus lineage authority
- Telemetry UI / Cesium runtime visualization
- Changing which pose is written to `entity_pose_history.command_pose`
- Operational readiness or HITL assessment from pose flags

---

## Related

- [rt_runtime_export_semantics_v1.md](rt_runtime_export_semantics_v1.md)
- [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md)
- [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md)
- [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md)
- [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md)
