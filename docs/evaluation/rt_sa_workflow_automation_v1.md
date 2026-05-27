# RT SA Workflow Automation Contract (`rt_sa_workflow_automation_v1`)

**Phase:** PLAN-RT-F6 — workflow automation advisory (docs only)  
**Prerequisite:** PLAT-RT-SA1, PLAT-RT-SA2, PLAT-RT-F5 frozen  
**Authority:** [rt_sa_import_bridge_v1.md](rt_sa_import_bridge_v1.md); [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md); [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md)

Normative contract for **advisory** RT→SA workflow automation. Composes frozen SA1 maintainer CLIs and SA2 read-only mirror — does **not** replace them.

---

## 1. Core invariant

```text
capture_session ≠ SA replay import
SA corpus authority begins only at rt_sa_import commit --corpus-dest
```

Advisory states are **explanatory mirrors** of staging + export audit signals. They are **not** approval authority and **not** SA replay lineage.

---

## 2. Advisory state ladder (`rt_sa_workflow_advisory_state_v1`)

Five rungs in strict order. Prefix `advisory_*` distinguishes from export event `handoff_ready` (pre-approval gate).

| Advisory state | Meaning | Derived from (read-only) | Maintainer checkpoint |
|----------------|---------|--------------------------|----------------------|
| `capture_ready` | Staging candidate exists and normalization path is viable | `candidate.json` present; `normalization_status: normalized`; lifecycle importable per [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md); `normalization_validation.json` → `valid: true` when present | After `rt_capture_normalize.py`; before handoff review |
| `review_complete` | Maintainer checklist satisfied | Export event `handoff_reviewed` OR `handoff_review.json` with reviewed decision; pose cognition acknowledged in review notes or attestation | `rt_handoff_review.py reviewed` |
| `approval_ready` | Preconditions for approval CLI met | `check_handoff_preconditions()` would pass; not blocked by reject/defer; `review_complete` signals present; **not yet** `capture_approved` | Ready for `rt_capture_approve.py` |
| `handoff_ready` | Approved + conversion manifest written; SA packaging may begin | Export events `capture_approved`, `conversion_manifest_written`; `approval_status: approved`; `conversion.json` present | Post-approve; pre-`rt_sa_import prepare` |
| `import_ready` | SA pipeline prepared; corpus commit is the only remaining gate | Export event `handoff_import_prepared`; `handoff_manifest.json` present; advisory pass on `CONVERSION_STEPS` logs; lineage lint clean | Maintainer may run `run-pipeline`; **must** explicitly `commit --corpus-dest` |

**Terminal authority (not an advisory state):** `handoff_import_committed` → SA lineage begins under `fixtures/sa_r0/`.

### 2.1 Naming disambiguation

| Term | Role | Same as F6 advisory? |
|------|------|----------------------|
| Export event `handoff_ready` | Emitted by `rt_handoff_review.py ready` — pre-approval preconditions | **No** — precedes `review_complete` |
| Advisory `handoff_ready` | Post-approval packaging readiness | **No** — different semantic |
| `workflow_phase: ready` (SA2 mirror) | Mirror chip from staging files | Maps near advisory `approval_ready` / pre-approve — see §3 |
| F5 `handoff_eligibility.experiment_level` | Experiment batch rollup | Never overrides per-capture advisory |
| `captureReadinessFromLifecycle` | Session UI cognition | Explanatory only — never `capture_ready` authority |

**UI copy rule:** Post-approval advisory state displays as **"Advisory: handoff packaging ready"** to avoid conflating with export event `handoff_ready`.

### 2.2 Derivation precedence

When multiple signals conflict, advisory derive uses this order (highest wins for **blocking**):

1. `handoff_rejected` / `handoff_import_deferred` → block downstream advisory states
2. Missing normalization / validation fail → cap at pre-`capture_ready`
3. Missing review / approve events → cap at corresponding rung
4. F5 experiment `ineligible` → **warn only** — does not block per-capture advisory

---

## 3. Mapping to existing vocabulary

### 3.1 SA2 `workflow_phase`

| `workflow_phase` | Typical advisory state | Gap notes |
|------------------|------------------------|-----------|
| `none` | (none) | No staging |
| `staged` | pre-`capture_ready` | Candidate exists; may lack normalize |
| `normalized` | `capture_ready` | Normalized but review not started |
| `review_pending` | `capture_ready` → `approval_ready` | Between `handoff_ready` export event and `handoff_reviewed` |
| `ready` | `approval_ready` | SA2 "ready" ≈ pre-approve; not advisory `handoff_ready` |
| `prepared` | `import_ready` (partial) | Manifest written; pipeline steps may be incomplete |
| `committed` | terminal (not advisory) | SA lineage active |
| `rejected` | blocked | Ladder reset — §5 |
| `deferred` | blocked at defer | Import advisory shows reason |

### 3.2 Maintainer checkpoint sequence

Overlay on [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md):

| Step | SA1 phase | Export / CLI | Advisory state after |
|------|-----------|--------------|----------------------|
| A | Capture + normalize | `capture_normalized` | `capture_ready` |
| B | Handoff review ready | `rt_handoff_review ready` → `handoff_ready` event | still `capture_ready` until reviewed |
| C | Review complete | `rt_handoff_review reviewed` → `handoff_reviewed` | `review_complete` |
| D | Approve | `rt_capture_approve.py` | `approval_ready` → `handoff_ready` |
| E | SA prepare + pipeline | `rt_sa_import prepare` / `run-step` | `import_ready` (advisory) |
| F | Corpus commit | `rt_sa_import commit --corpus-dest` | terminal — not advisory |

---

## 4. Automation boundaries

### 4.1 What may be automated (PLAT-RT-F6)

| Category | Allowed | Authority |
|----------|---------|-----------|
| Read-only aggregation | Join mirror rows + export audit + F5 eligibility + staging integrity audit | Advisory derive only |
| Checklist derivation | Pass/fail/warn per manual workflow §2 checklist item | No writes |
| Batch status reports | JSON listing advisory states for N capture IDs or experiment manifest refs | Read-only CLI |
| Corpus diff preview (P2) | Read-only diff vs `fixtures/sa_r0/` before commit | Never writes corpus |
| Maintainer CLI helpers (P2) | Dry-run wrappers calling existing SA1 modules sequentially | `--dry-run` default; no bridge HTTP |

### 4.2 What must stay manual

| Action | Reason |
|--------|--------|
| `rt_handoff_review.py` write paths | Maintainer attestation |
| `rt_capture_approve.py` | Approval intent |
| `rt_sa_import prepare` / `run-step` / `run-pipeline` | Handoff manifest and SA step logs |
| `rt_sa_import commit --corpus-dest` | **SA lineage gate** — always explicit per capture |
| `replay_sa_bundle_pack` | Never bridge-invoked |
| Browser approve / import / capture | Forbidden (X1, SA2, F5) |

### 4.3 Explicit approval gates (hard stops)

| Gate | Blocks advisory state |
|------|----------------------|
| Normalization invalid or missing | `approval_ready`, `handoff_ready`, `import_ready` |
| Reject active | `approval_ready`, `handoff_ready`, `import_ready` |
| Defer active | `handoff_ready`, `import_ready` |
| Missing `conversion.json` | `handoff_ready`, `import_ready` |
| Lineage lint fail | `import_ready` |
| `reject_auto_sa_import()` | Any automated pipeline trigger from bridge/browser |
| Federation / orchestration auto-publish | All automation |

---

## 5. Failure and rollback rules

### 5.1 Reject (`handoff_rejected`)

- Advisory ladder resets to pre-`review_complete`
- Mirror `workflow_phase` → `rejected`
- Re-entry: fix issues → `rt_handoff_review ready` → `reviewed` → continue from `review_complete`

### 5.2 Defer (`handoff_import_deferred`)

- Advisory frozen at defer; `import_ready` blocked
- Import advisory strip shows defer reason from `handoff_review.json`
- Re-entry: maintainer clears defer via new review cycle

### 5.3 Pipeline step failure

- `import_ready` revoked until failing step re-run successfully
- No auto-retry without explicit maintainer CLI invocation
- Partial step logs retained under `sa_handoff/<id>/steps/` for audit

### 5.4 Partial batch (P2)

- Per-capture isolation: one capture failure does **not** auto-commit siblings
- Batch report lists per-ID advisory state and blockers
- No batch `--commit-all` flag in PLAT scope

---

## 6. Advisory derive schema (PLAT reference)

Normative shape for PLAT-RT-F6 P0 derive output:

```json
{
  "schema": "rt_sa_workflow_advisory_status_v1",
  "capture_candidate_id": "cap-2026-01-01-abc",
  "advisory_state": "approval_ready",
  "advisory_state_label": "Approval ready (advisory)",
  "blocked": false,
  "block_reasons": [],
  "checklist": [
    {"id": "normalization", "status": "pass"},
    {"id": "validation_doc", "status": "pass"},
    {"id": "pose_cognition", "status": "warn", "detail": "maintainer attestation required"}
  ],
  "upstream": {
    "workflow_phase": "ready",
    "last_export_event": "handoff_reviewed",
    "approval_status": "pending"
  },
  "governance_banner": "SA WORKFLOW ADVISORY — explanatory; maintainer CLIs are authority"
}
```

| Field | Required | Notes |
|-------|----------|-------|
| `schema` | yes | `rt_sa_workflow_advisory_status_v1` |
| `advisory_state` | yes | One of five rungs or `blocked` |
| `blocked` | yes | true when reject/defer active |
| `checklist` | optional | Derived from §2 of manual workflow |
| `governance_banner` | yes | Fixed advisory banner string |

---

## 7. Governance rules

| Rule | Enforcement |
|------|-------------|
| Advisory ≠ SA replay authority | Banner on all derive outputs and UI surfaces |
| Advisory ≠ approve/import | No CLI side effects in derive |
| F5 eligibility ≠ import | Cross-ref [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md) phase I |
| Lineage protection | Reuse [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md) validators in derive |
| Forbidden lexicon | No `readiness_score`, `auto_import`, `operational_ready`, `tactical readiness` |

---

## 8. PLAT advisory (implementation status)

- **P0 delivered:** `advisory_derive.py`, `deriveAdvisoryState.ts`, `rt_handoff_advisory_status.py`, golden fixtures, read-only UI strips — see [rt_plat_f6_p0_freeze_audit.md](rt_plat_f6_p0_freeze_audit.md)
- **P1 delivered:** `SaWorkflowAdvisoryPanel`, checklist chips, `BANNER_SA_WORKFLOW_ADVISORY`, import advisory strip — see [rt_plat_f6_p1_freeze_audit.md](rt_plat_f6_p1_freeze_audit.md)
- **P2 delivered:** Batch advisory report, dry-run pipeline wrapper, corpus diff preview — see [rt_plat_f6_p2_freeze_audit.md](rt_plat_f6_p2_freeze_audit.md)

---

## 9. Related

- [rt_sa_workflow_advisory_ui_v1.md](rt_sa_workflow_advisory_ui_v1.md)
- [rt_sa2_multi_session_handoff_ui_v1.md](rt_sa2_multi_session_handoff_ui_v1.md)
- [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) — F5 `handoff_eligibility`
- [rt_f6_handoff_contamination_review_r1.md](rt_f6_handoff_contamination_review_r1.md)
