# RT Advisory Aggregation (`rt_advisory_aggregation_v1`)

**Phase:** PLAN-RT-F7 — multi-capture and experiment rollups (docs only)  
**Prerequisite:** [rt_sa_workflow_automation_v1.md](rt_sa_workflow_automation_v1.md); PLAT-RT-F6 P2 `batch_advisory` frozen  
**Composes:** [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md), [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md)

Normative schemas and rules for **advisory aggregation** atop F6 P2 batch reports — not SA corpus authority.

---

## 1. Core invariant

```text
aggregate_summary ≠ batch_commit
experiment_rollup ≠ import_ready
readiness_cohort ≠ readiness_score
```

---

## 2. Schema evolution

### 2.1 F6 baseline: `rt_handoff_batch_review_v1`

PLAT-RT-F6 P2 emits:

- `summary.total`, `summary.by_advisory_state`, `summary.block_reason_rollup`, `summary.derive_errors`
- `captures[]` with per-row `advisory`, `next_cli`

### 2.2 F7 extension: `rt_advisory_batch_summary_v1`

Superset document — compatible with F6 fields:

```json
{
  "schema": "rt_advisory_batch_summary_v1",
  "generated_at": "2026-05-28T12:00:00+00:00",
  "repo_root": "/path/to/repo",
  "dry_run": true,
  "governance_banner": "SA WORKFLOW ADVISORY — explanatory; maintainer CLIs are authority",
  "summary": {
    "total": 4,
    "by_advisory_state": { "approval_ready": 2, "import_ready": 1 },
    "block_reason_rollup": { "handoff_import_deferred": 1 },
    "derive_errors": [],
    "blocker_groups": {
      "review_attestation": { "count": 1, "exemplar_capture_ids": ["cap-a"] },
      "experiment_warn": { "count": 2, "exemplar_capture_ids": ["cap-b", "cap-c"] }
    },
    "readiness_cohorts": {
      "needs_review": 1,
      "needs_approve": 2,
      "ready_for_commit_advisory": 1
    },
    "experiment_rollup": {
      "manifest_ref": "fixtures/rt_experiment/example_manifest.json",
      "handoff_eligibility": "partial",
      "warn_capture_ids": ["cap-c"],
      "note": "experiment eligibility is warn-only"
    }
  },
  "captures": [
    {
      "capture_candidate_id": "cap-a",
      "advisory": { "schema": "rt_sa_workflow_advisory_status_v1", "advisory_state": "approval_ready" },
      "queue_priority": { "rank": 410, "band": "P4_approve", "rationale": "approval_ready" },
      "blocker_groups": ["review_attestation"],
      "readiness_cohort": "needs_approve",
      "next_cli": "scripts/rt/rt_capture_approve.py cap-a"
    }
  ]
}
```

| Field | Required | Notes |
|-------|----------|-------|
| `schema` | yes | `rt_advisory_batch_summary_v1` |
| `governance_banner` | yes | Same string as F6 derive |
| `summary.blocker_groups` | optional | Counts + exemplar IDs per group |
| `summary.readiness_cohorts` | optional | Bucket counts — **not** scores |
| `summary.experiment_rollup` | optional | Present when manifest ref supplied |
| `captures[].queue_priority` | optional | Per [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md) §2 |
| `captures[].readiness_cohort` | optional | Single primary cohort per row |
| `captures[].lineage_warnings` | optional | Detect-only — [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) §4 |

PLAT may emit F6 schema for backward compatibility; F7 fields are additive.

---

## 3. Multi-capture summaries

### 3.1 Stand-up report (normative minimum)

Maintainer stand-up JSON SHOULD include:

| Section | Content |
|---------|---------|
| `summary.total` | Row count |
| `summary.by_advisory_state` | Ladder distribution |
| `summary.blocker_groups` | Top groups by `count` |
| `summary.readiness_cohorts` | Lane occupancy |
| `captures` (sorted) | By `queue_priority.rank` ascending |
| Per row | `next_cli`, `blocker_groups`, primary `readiness_cohort` |

### 3.2 Sort order

Default capture list order: **`queue_priority.rank` ascending** (see maintainer workflow §2). PLAT flag: `--sort queue` (advisory).

### 3.3 Relation to F6 `aggregate_report()`

F6 P2 `aggregate_report()` populates `by_advisory_state` and `block_reason_rollup`. F7 adds `blocker_groups`, `readiness_cohorts`, per-row `queue_priority` — implemented in PLAT-RT-F7 P0, specified here.

---

## 4. Readiness grouping (cohorts)

**Not** operational readiness. Advisory cohort labels for bulk cognition:

| Cohort ID | Maps from advisory state / blockers |
|-----------|-------------------------------------|
| `needs_normalize` | Below `capture_ready`; normalization group primary |
| `needs_review` | `capture_ready`, `review_complete`; review queue |
| `needs_approve` | `approval_ready` |
| `needs_prepare` | `handoff_ready` (advisory packaging) |
| `ready_for_commit_advisory` | `import_ready` — **commit still manual** |
| `blocked` | `blocked: true` |
| `terminal` | `handoff_import_committed` |
| `error` | Derive error / missing staging |

Primary cohort = lowest triage lane with active blocker, else advisory state mapping.

Forbidden field names: `readiness_score`, `operational_ready`, `tactical_readiness`.

---

## 5. Experiment → handoff rollups

### 5.1 Inputs

- Experiment manifest ref (F5 workbench / `rt_experiment_spec_compile` output)
- Per-capture rows from batch scan
- F5 `handoff_eligibility` per run (read-only)

### 5.2 Rollup rules

| Rule | Behavior |
|------|----------|
| Precedence | Per-capture F6 ladder wins on conflict |
| Experiment `eligible` | Adds `experiment_warn` group; does not set `import_ready` |
| Experiment `ineligible` | Warn in rollup; does not block sibling captures |
| Partial manifest | `handoff_eligibility: partial` + `warn_capture_ids` |
| No manifest | Omit `experiment_rollup` section |

### 5.3 Example rollup block

```json
{
  "experiment_rollup": {
    "manifest_ref": "runs/rt_sandbox/experiments/exp-001/manifest.json",
    "handoff_eligibility": "partial",
    "eligible_count": 1,
    "ineligible_count": 1,
    "warn_capture_ids": ["cap-run-2"],
    "note": "experiment eligibility is warn-only; per-capture advisory is authority"
  }
}
```

---

## 6. Blocker group rollup algorithm (normative)

For each capture row:

1. Classify `block_reasons` and checklist into groups per [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md) §3
2. Increment `summary.blocker_groups[<id>].count`
3. Append capture_id to `exemplar_capture_ids` (max 5 per group in summary; full list in `captures[]`)

Sort groups in report footer by **descending count**.

---

## 7. Governance

| Rule | Enforcement |
|------|-------------|
| Banner on every aggregate | Required |
| No export event emission | Rollup is read-only |
| No corpus writes | Preview sections dry-run only |
| SA viewer | No consumption of live rollup |

---

## 8. Reference fixtures

See [fixtures/rt_handoff/f7_advisory_examples/](../../fixtures/rt_handoff/f7_advisory_examples/) — queue order, blocker groups, experiment warn-only, lineage warning cases.

## 9. PLAT implementation status

- **P0 delivered:** `advisory_queue.py`, `build_advisory_batch_summary_document`, CLI extensions, TS mirror, read-only UI chips — see [rt_plat_f7_p0_freeze_audit.md](rt_plat_f7_p0_freeze_audit.md)
- **P1 delivered:** `AdvisoryTriageQueuePanel`, grouped blocker strip, triage grouping helpers — see [rt_plat_f7_p1_freeze_audit.md](rt_plat_f7_p1_freeze_audit.md)
- **P2 delivered:** `rt_advisory_batch_review_v2`, stand-up/grouped export CLIs, `dry-run-review`, dry-run guardrails — see [rt_plat_f7_p2_freeze_audit.md](rt_plat_f7_p2_freeze_audit.md)

### 2.3 F7 P2 extension: `rt_advisory_batch_review_v2`

Stand-up / grouped maintainer export — superset of `rt_advisory_batch_summary_v1`:

| Field | Required | Notes |
|-------|----------|-------|
| `schema` | yes | `rt_advisory_batch_review_v2` |
| `dry_run` | yes | Must be `true` |
| `grouped` | yes | `by_queue_band`, `by_blocker_group`, `by_readiness_cohort` (capture id lists) |
| `standup` | yes | `priority_capture_ids`, `top_blocker_groups`, `cohort_counts`, `warn_only_notes` |

CLI: `standup-export`, `grouped-export`, `export --schema v2`. Default `export` remains `rt_advisory_batch_summary_v1` (`--schema f7`).

---

## 9. Related

- [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md)
- [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md)
- [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) — F5 `handoff_eligibility`
- [rt_roadmap_plat_rt_f7_v1.md](rt_roadmap_plat_rt_f7_v1.md)
