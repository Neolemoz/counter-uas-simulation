# RT SA Import Bridge Contract (`rt_sa_import_bridge_v1`)

**Phase:** PLAT-RT-SA1 — manual RT→SA import implementation  
**Authority:** [rt_sa1_manual_import_bridge_plan.md](../platform/rt_sa1_manual_import_bridge_plan.md); [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)

Normative contract for maintainer-only RT→SA import tooling.

---

## 1. Core invariant

`capture_session ≠ SA replay import`. RT authority stops before `replay_sa_bundle_pack`. SA lineage begins only on **explicit maintainer commit**.

---

## 2. Workflow phases

| Phase | CLI | Export event |
|-------|-----|--------------|
| A. Capture + normalize | Bridge / `rt_capture_normalize` | `capture_validated`, `capture_normalized` |
| B. Handoff ready | `rt_handoff_review.py ready` | `handoff_ready` |
| C. Review | `rt_handoff_review.py reviewed` | `handoff_reviewed` |
| D. Approve | `rt_capture_approve.py` | `capture_approved`, `conversion_manifest_written` |
| E. Prepare | `rt_sa_import.py prepare` | `handoff_import_prepared` |
| F. SA steps | `rt_sa_import.py run-step` | Step logs under `sa_handoff/steps/` |
| G. Commit | `rt_sa_import.py commit --corpus-dest` | `handoff_import_committed` |

Reject: `rt_handoff_review.py reject` → `handoff_rejected`  
Defer: `rt_handoff_review.py defer` → `handoff_import_deferred`

---

## 3. Staging paths

| Path | Writer |
|------|--------|
| `runs/rt_sandbox/captures/<id>/` | RT capture (unchanged) |
| `runs/rt_sandbox/sa_handoff/<id>/` | SA1 handoff CLIs only |
| `fixtures/sa_r0/...` | `rt_sa_import commit` only |

---

## 4. Schemas

### `rt_handoff_review_v1`

Written to capture staging and/or `sa_handoff/<id>/handoff_review.json`.

| Field | Required |
|-------|----------|
| `schema` | `rt_handoff_review_v1` |
| `capture_candidate_id` | yes |
| `decision` | `ready_for_approval`, `deferred`, `rejected` |
| `reviewer` | yes |
| `review_utc` | ISO-8601 |
| `notes` | optional |
| `governance_banner` | HANDOFF REVIEW — explanatory; not lineage authority |

### `rt_sa_handoff_manifest_v1`

Written to `sa_handoff/<id>/handoff_manifest.json`.

References capture staging refs, conversion manifest, approval — **no** `session_id` as lineage parent.

### `rt_sa_import_record_v1`

Written on commit. Fields: `corpus_ref`, `bundle_path`, `imported_at`, `rt_capture_ref` (non-authoritative), `imported_by`.

---

## 5. Export audit events (PLAT-RT-SA1)

| `event_type` | When |
|--------------|------|
| `handoff_ready` | Preconditions pass |
| `handoff_reviewed` | Maintainer checklist complete |
| `handoff_rejected` | Import declined |
| `handoff_import_deferred` | Postponed |
| `handoff_import_prepared` | Handoff manifest written |
| `handoff_import_committed` | Corpus copy complete |

All use `event_kind: export`.

---

## 6. CLI surface

| Script | Subcommands |
|--------|-------------|
| `rt_handoff_review.py` | `ready`, `reviewed`, `reject`, `defer` |
| `rt_sa_import.py` | `prepare`, `status`, `run-step`, `run-pipeline`, `commit` |
| `rt_capture_inspect.py` | `handoff-status` |

---

## 7. Lineage protection

- `session_id` / `ephemeral_session_ref` must not be SA `parent_ref`
- `capture_candidate_id` is cross-ref only
- `validate_sa_import_record()` at commit
- Bridge `reject_auto_sa_import()` unchanged

---

## 8. Explicit non-goals

- Automatic replay import; federation auto-publish
- SA viewer live hooks; new bridge HTTP commands
- RT-T3 / Cesium changes

---

## 9. R2f boundary

Implements vocabulary specified in PLAN-RT-R2f without changing `capture_session` behavior.
