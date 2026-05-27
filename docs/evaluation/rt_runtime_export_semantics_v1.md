# RT Runtime Export Semantics (`rt_runtime_export_semantics_v1`)

**Phase:** PLAT-RT-R2e — export lineage and interpretation  
**Authority:** [rt_r2e_capture_pose_cognition_plan.md](../platform/rt_r2e_capture_pose_cognition_plan.md); [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)

Clarifies ownership, lineage boundaries, and maintainer interpretation expectations for RT capture → SA replay handoff. Does not authorize SA import.

---

## 1. Layered artifacts

| Layer | Schemas / files | Owner | Replay authority |
|-------|-----------------|-------|------------------|
| **Raw capture** | `rt_capture_candidate_v1`, `sandbox_session_snapshot_v1`, `runtime_capture_report_v1`, `rt_telemetry_capture_summary_v1` | RT bridge at `capture_session` | **No** |
| **Normalized** | `rt_normalized_capture_v1`, `rt_capture_provenance_v1`, `rt_normalization_validation_v1` | RT normalization (capture-time or `rt_capture_normalize.py`) | Replay-*ready* RT staging only |
| **Conversion ref** | `runtime_to_replay_conversion_v1` | Maintainer after `rt_capture_approval_v1` | Declares external pipeline steps only |
| **SA bundle** | `replay_sa_bundle` chain | SA tooling (out of bridge) | Corpus authority **only** after explicit import |

**Invariant:** `capture_session ≠ SA replay import` ([rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md) §1).

---

## 2. Ownership and lineage boundaries

| ID / ref | May appear in staging? | Lineage parent? |
|----------|------------------------|-----------------|
| `session_id` / `ephemeral_session_ref` | Yes (explanatory) | **Never** authoritative `parent_ref` |
| `capture_candidate_id` | Yes | RT staging key only |
| `conversion_revision` | Normalized + conversion manifest | RT normalization revision |
| `run_id` / `bundle_path` / `corpus_ref` | SA bundle only | Authoritative **after** maintainer import |

Provenance `source_artifact_refs` must stay under the capture staging directory (no external `audit_ref` paths in normalized surfaces).

---

## 3. Maintainer gate ordering

1. **Raw capture** — `capture_session` writes S5 artifacts.
2. **Normalization** — automatic at capture or `rt_capture_normalize.py`; emits `capture_normalized`, `export_pose_normalized`, pose cognition block.
3. **Approval** — `rt_capture_approve.py` requires `normalization_status: normalized`.
4. **Conversion manifest** — `runtime_to_replay_conversion_v1` links normalized refs.
5. **External SA packager** — `validate_scenario.py`, `replay_observability`, `replay_sa_bundle_pack`, governance lint (out of RT bridge).

Failed or non-`stopped`/`captured` sessions must not produce importable conversion manifests ([rt_capture_continuity_v1.md](rt_capture_continuity_v1.md) §6).

---

## 4. Interpretation expectations

### Raw bundle

- `snapshot.json` — command-authoritative entity poses at capture instant; banner: non-replay truth for SA viewer.
- `capture_report.json` — explanatory command summary; pairs with session audit log.
- `telemetry_summary.json` — subscription ring summary; not operational state.

### Normalized bundle

- `entity_pose_history` — tri-source poses; read with [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md).
- `capture_pose_cognition` — ambiguity flags and authoritative field per entity.
- `sync_health_summary` / `telemetry_health_summary` — G3/G4 snapshots at capture; explanatory.

### Export audit JSONL (`runs/rt_sandbox/export_audit/export_boundary.jsonl`)

| `event_type` | Meaning |
|--------------|---------|
| `capture_requested` / `capture_validated` | Boundary lifecycle |
| `capture_normalized` | Normalization succeeded — sets `conversion_revision` on manifest |
| `export_pose_normalized` | Pose cognition assessed and attached (R2e) — includes `conversion_revision` in detail; distinct event from `capture_normalized` |
| `provenance_injected` / `normalization_validation` | G5 validation path |
| `normalization_rejected` / `capture_rejected` | Failure — no SA import |

Summarize by `event_kind` groups; do not treat export audit as replay authority.

### CLI inspection

- `rt_capture_inspect` — staging refs, normalization status.
- `rt_capture_inspect normalization-status` — validation doc read-only.

---

## 5. Pose semantics in export (R2e)

| Export surface | Authoritative pose | Explanatory overlays |
|--------------|-------------------|----------------------|
| `snapshot.json` | `entity_states[].pose` | — |
| `rt_normalized_capture_v1` | `entity_pose_history[].command_pose` | `feedback_pose`, `telemetry_mirror_pose` |
| SA bundle (if ever imported) | Parser/corpus contracts only | RT mirrors must not become SA compare clocks |

**Explanatory-only mismatch:** recorded in `capture_pose_cognition` and capture audit events; never promotes feedback/telemetry into authoritative export fields.

---

## 6. RT→SA handoff (PLAN-RT-R2f)

Maintainer manual import after RT authority stop line: [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md), [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md), [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md).

---

## 7. Related

- [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md)
- [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md)
- [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md)
- [rt_roadmap_post_g5_v1.md](rt_roadmap_post_g5_v1.md)
