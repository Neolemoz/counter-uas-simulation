# RT Manual SA Import Workflow (`rt_manual_sa_import_workflow_v1`)

**Phase:** PLAN-RT-R2f — maintainer-only import workflow (docs only)  
**Authority:** [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md); [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)

Human-reviewed workflow from RT staging to SA replay corpus. **No automation** in this contract wave.

---

## 1. Workflow overview

| Phase | Actor | Outcome |
|-------|-------|---------|
| A. Capture complete | RT bridge | Raw + normalized staging |
| B. Review | Maintainer | Checklist pass or defer/reject |
| C. Approve | Maintainer CLI | `approval.json`, `conversion.json` |
| D. SA packaging | Maintainer (external CLIs) | Validation artifacts + bundle |
| E. Import | Maintainer | Corpus commit — SA lineage starts here |

Phases D–E are **never** triggered by `capture_session` or bridge HTTP commands.

---

## 2. Review checklist (phase B)

Before `rt_capture_approve.py`, maintainer confirms:

| Check | Requirement |
|-------|-------------|
| Normalization | `candidate.json` → `normalization_status: normalized` |
| Validation doc | `normalization_validation.json` → `valid: true` |
| Pose cognition | `capture_pose_cognition` reviewed; `command_pose` treated as authoritative |
| Origin | `origin` includes `rt_sandbox_capture_v1` |
| Session state | Capture not from `failed` / `discarded` / non-importable lifecycle ([rt_capture_continuity_v1.md](rt_capture_continuity_v1.md) §6) |
| Scenario pack | If `scenario_pack_ref` present, path exists under `fixtures/scenarios/` |
| Export audit | `capture_validated`, `capture_normalized`, `export_pose_normalized` present |
| Lineage | No plan to use `session_id` as corpus `parent_ref` |

Failure → reject or defer (§5–§6). Do not approve.

---

## 3. Validation requirements (phase D)

Maintainer runs **outside** the RT bridge (order may vary per local runbook; normative set from [export_boundary.py](../../platform/rt-sandbox-bridge/rt_sandbox/export_boundary.py) `CONVERSION_STEPS`):

1. `validate_scenario_pack` — `validate_scenario.py` on referenced pack
2. `replay_observability` — observability chain on logs/artifacts
3. `replay_sa_bundle_pack` — produces SA bundle (not invoked from bridge)
4. `governance_lint` — `replay_observability.py governance-lint` on packaged output

Each step produces SA-side artifacts. RT staging refs in `conversion.json` are **inputs declared**, not authoritative replay state.

---

## 4. Approval flow (phase C)

**CLI:** `scripts/rt/rt_capture_approve.py <capture_candidate_id>`

| Action | Artifact | Export audit (today) |
|--------|----------|----------------------|
| Approve | `approval.json` (`rt_capture_approval_v1`) | `capture_approved` |
| Write conversion | `conversion.json` (`runtime_to_replay_conversion_v1`) | `conversion_manifest_written` |
| Update candidate | `approval_status: approved` | — |

 Preconditions enforced by CLI:

- `normalization_status: normalized` (unless `--skip-normalization-check` for legacy test fixtures only)
- Valid `rt_capture_candidate_v1` candidate

Approval records **maintainer intent** — not automatic corpus promotion.

---

## 5. Rejection flow

When SA import is **declined** for a capture candidate:

| Action | Semantics |
|--------|-----------|
| Do not run | `rt_capture_approve.py` for import intent |
| Optional | Set `candidate.json` → `approval_status: rejected` (maintainer edit or future CLI) |
| Document | `handoff_rejected` export event meaning (vocabulary) — distinct from `capture_rejected` (capture-time bridge failure) |
| Do not | Run SA packager or write corpus/federation paths |

Rejected captures remain in `runs/rt_sandbox/captures/` for audit; they must not enter SA corpus.

---

## 6. Defer flow

When import is **postponed** without rejecting the capture artifact:

| Action | Semantics |
|--------|-----------|
| Decision | `handoff_import_deferred` — vocabulary only in R2f |
| Optional sidecar | `rt_handoff_review_v1` in staging (docs stub — **no writer in R2f**) |

### `rt_handoff_review_v1` (stub schema)

```json
{
  "schema": "rt_handoff_review_v1",
  "capture_candidate_id": "uuid",
  "decision": "deferred",
  "reviewer": "maintainer_id",
  "review_utc": "ISO-8601",
  "notes": "optional free text",
  "governance_banner": "HANDOFF REVIEW — explanatory; not lineage authority"
}
```

Allowed `decision` values: `deferred`, `rejected`, `ready_for_approval` (explanatory workflow states only).

Defer does **not** imply approval or SA packaging.

---

## 7. Provenance handoff

**May transfer** (via `conversion.json` `staging_refs` and external packaging):

- `normalized_manifest_ref`, `provenance_ref`, `validation_ref`
- `conversion_revision`, `normalization_utc`
- Redacted `rt_capture_provenance_v1` fields (staging-local refs only)

**Must not transfer as authoritative lineage:**

- `session_id` / `ephemeral_session_ref` as `parent_ref`
- Absolute `audit_ref` paths outside staging
- Live telemetry blobs without conversion declaration
- RT mirror poses as SA compare clocks

---

## 8. Inspection CLIs (read-only)

| CLI | Use |
|-----|-----|
| `rt_capture_inspect` | Staging refs, candidate status |
| `rt_capture_inspect normalization-status` | Validation doc summary |

---

## Related

- [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md)
- [rt_runtime_export_semantics_v1.md](rt_runtime_export_semantics_v1.md)
- [experiment_workflow_scenario_to_replay_v1.md](experiment_workflow_scenario_to_replay_v1.md)
