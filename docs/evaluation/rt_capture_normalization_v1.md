# RT Capture Normalization Contract (`rt_capture_normalization_v1`)

**Phase:** PLAT-RT-G5 — runtime capture normalization  
**Authority:** [rt_g5_capture_normalization_plan.md](../platform/rt_g5_capture_normalization_plan.md)

Normalized captures are **replay-ready** RT-side artifacts — not replay authority. SA import remains explicit maintainer workflow only.

---

## 1. Authority

| Layer | Role |
|-------|------|
| `EntityRegistry` / `snapshot.json` | Command-authoritative poses in normalized history |
| `PoseSyncMirror` (G3) | Explanatory sync health + feedback overlays |
| `TelemetryMirror` (G4) | Explanatory telemetry snapshots |
| `rt_normalized_capture_v1` | Replay-ready index — not SA corpus lineage |
| `capture_pose_cognition` (R2e) | Explanatory ambiguity flags — see [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md) |

`session_id` is **never** authoritative lineage `parent_ref`.

---

## 2. Staging artifacts

Written under `runs/rt_sandbox/captures/<capture_candidate_id>/` (additive to PLAT-RT-S5 raw files).

### `normalized_manifest.json` — `rt_normalized_capture_v1`

| Field | Description |
|-------|-------------|
| `capture_candidate_id` | UUID |
| `conversion_revision` | Monotonic int; starts at `1` |
| `normalization_utc` | ISO-8601 UTC |
| `origin` | `rt_sandbox_capture_v1` |
| `entity_pose_history` | List of per-entity pose records with `source` tag |
| `lifecycle_summary` | Session state, workflow/templates pointers |
| `telemetry_summary_snapshot` | Channels, health from capture |
| `sync_health_summary` | G3 mirror summary when present |
| `telemetry_health_summary` | G4 mirror summary when present |
| `capture_timestamps` | `capture_utc`, snapshot revision hints |
| `tactical_annex` | Optional `rt_tactical_capture_annex_v1` (PLAT-RT-TAC5) |
| `governance_banner` | Normalization banner |

### `provenance.json` — `rt_capture_provenance_v1`

| Field | Description |
|-------|-------------|
| `adapter_attached` | bool |
| `adapter_mode` | e.g. `mock` / `stub` |
| `normalization_utc` | ISO-8601 |
| `source_artifact_refs` | Staging-local refs only (no external audit paths); may include `tactical_annex_ref` |
| `input_content_hash` | SHA-256 of canonical raw inputs |
| `entity_sim_mapping` | `entity_id` → optional `sim_entity_ref` (explanatory) |

### `normalization_validation.json` — `rt_normalization_validation_v1`

| Field | Description |
|-------|-------------|
| `valid` | bool |
| `checks` | Named pass/fail list |
| `boundary_lint` | Export-boundary validation results |
| `governance_banner` | Validation banner |

### `candidate.json` updates

| Field | Values |
|-------|--------|
| `normalization_status` | `pending` \| `normalized` \| `rejected` |
| `staging_refs` | Adds `normalized_manifest_ref`, `provenance_ref`, `validation_ref` |

---

## 3. Revision and timestamp policy

- Timestamps: UTC ISO-8601, microsecond-stripped.
- `conversion_revision`: `1` on first successful normalization; increment on re-normalize when `input_content_hash` changes; unchanged when hash matches (idempotent re-run).
- `snapshot_revision` in `capture_timestamps`: bridge `world.revision` at capture snapshot — distinct from `conversion_revision`.
- `world_revision_hint` in `telemetry_health_summary`: adapter `{telemetry_seq, sync_seq}` at capture — explanatory; may diverge from `snapshot_revision`.

Full policy: [rt_world_revision_hint_policy_v1.md](rt_world_revision_hint_policy_v1.md).

---

## 4. Redaction policy

Omit or strip from normalized/provenance surfaces:

- Absolute `audit_ref` paths outside staging directory
- `session_id` as `parent_ref` / `authoritative_parent_ref`
- Adapter PID, ROS domain id, IPC handles
- Undeclared live telemetry blobs

Retain as explanatory: `ephemeral_session_ref`, sync/telemetry health enums, workflow summary pointers.

---

## 5. Export audit events

| Event | When |
|-------|------|
| `capture_normalized` | Successful normalization |
| `export_pose_normalized` | Pose cognition assessed (PLAT-RT-R2e) |
| `provenance_injected` | `provenance.json` written |
| `normalization_validation` | Validation pass recorded |
| `normalization_rejected` | Normalization failure |

---

## 6. Conversion manifest linkage

`runtime_to_replay_conversion_v1` must include:

- `normalization_status: normalized` (via candidate check at approve time)
- `staging_refs.normalized_manifest_ref`
- `conversion_revision`, `normalization_utc`

---

## Related

- [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md)
- [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)
- [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md)
- [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md)
- [rt_authority_model_v1.md](rt_authority_model_v1.md)
- [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md)
- [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md)
- [rt_runtime_export_semantics_v1.md](rt_runtime_export_semantics_v1.md)
