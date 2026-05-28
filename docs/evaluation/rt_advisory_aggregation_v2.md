# RT Advisory Aggregation v2 (`rt_advisory_aggregation_v2`)

**Phase:** PLAN-RT-F8 — multi-capture cohort and handoff rollups (docs only)  
**Prerequisite:** [rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md) (F7 frozen); PLAT-RT-F7 P0–P2 frozen  
**Composes:** [rt_advisory_maintainer_workflow_v2.md](rt_advisory_maintainer_workflow_v2.md), [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md), [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)

Normative schemas and rules for **advisory aggregation v2** atop F7 `rt_advisory_batch_summary_v1` and `rt_advisory_batch_review_v2` — not SA corpus authority.

---

## 1. Core invariants

```text
cohort_summary_v2 ≠ readiness_score
experiment_handoff_rollup ≠ import_ready
multi_capture_summary ≠ batch_commit
handoff_rollup ≠ export_audit
```

---

## 2. Schema evolution

### 2.1 F7 baseline (unchanged)

- `rt_handoff_batch_review_v1` (F6)
- `rt_advisory_batch_summary_v1` (F7 P0)
- `rt_advisory_batch_review_v2` (F7 P2 stand-up / grouped export)

PLAT continues to emit v1/v2 F7 schemas for backward compatibility.

### 2.2 F8 extension: `rt_advisory_batch_summary_v2`

Additive superset of `rt_advisory_batch_summary_v1` — all F7 fields remain valid.

```json
{
  "schema": "rt_advisory_batch_summary_v2",
  "generated_at": "2026-05-28T14:00:00+00:00",
  "repo_root": "/path/to/repo",
  "dry_run": true,
  "governance_banner": "SA WORKFLOW ADVISORY — explanatory; maintainer CLIs are authority",
  "preset_applied": "review_backlog",
  "focus_capture_ids": ["cap-a", "cap-b"],
  "summary": {
    "total": 4,
    "by_advisory_state": { "approval_ready": 2, "import_ready": 1 },
    "block_reason_rollup": {},
    "derive_errors": [],
    "blocker_groups": {
      "review_attestation": { "count": 1, "exemplar_capture_ids": ["cap-a"] }
    },
    "readiness_cohorts": {
      "needs_review": 1,
      "needs_approve": 2,
      "ready_for_commit_advisory": 1
    },
    "readiness_cohorts_v2": {
      "stale_review": 1,
      "multi_blocker": 1,
      "experiment_handoff_warn": 1
    },
    "multi_capture_cohorts": {
      "by_primary_lane": {
        "review": 2,
        "approve": 1,
        "import_advisory": 1
      },
      "stale_age_warn_count": 1,
      "note": "lane counts are advisory cognition only"
    },
    "experiment_rollup": {
      "manifest_ref": "fixtures/rt_experiment/example_manifest.json",
      "handoff_eligibility": "partial",
      "warn_capture_ids": ["cap-c"],
      "note": "experiment eligibility is warn-only"
    },
    "experiment_handoff_rollup": {
      "manifest_ref": "fixtures/rt_experiment/example_manifest.json",
      "cohort_index_ref": "fixtures/rt_experiments/x2_cohort_index_example.json",
      "cohort_status": "indexed",
      "handoff_eligibility": "partial",
      "warn_capture_ids": ["cap-c"],
      "review_packet_paths": [],
      "note": "X2 cohort and packet paths are read-only adjacency; not commit authority"
    },
    "handoff_rollup": {
      "by_stage": {
        "normalize": 0,
        "review": 1,
        "approve": 2,
        "prepare": 0,
        "import_advisory": 1
      },
      "blocked_count": 0,
      "terminal_count": 0
    }
  },
  "captures": [
    {
      "capture_candidate_id": "cap-a",
      "advisory": {
        "schema": "rt_sa_workflow_advisory_status_v1",
        "advisory_state": "approval_ready"
      },
      "queue_priority": { "rank": 410, "band": "P4_approve", "rationale": "approval_ready" },
      "blocker_groups": ["review_attestation"],
      "readiness_cohort": "needs_approve",
      "readiness_cohort_v2": "multi_blocker",
      "stale_age_hours": null,
      "in_focus_set": true,
      "next_cli": "scripts/rt/rt_capture_approve.py cap-a"
    }
  ]
}
```

| Field | Required | Notes |
|-------|----------|-------|
| `schema` | yes | `rt_advisory_batch_summary_v2` |
| `governance_banner` | yes | Same string as F6/F7 derive |
| `preset_applied` | optional | Filter preset ID from workflow v2 §2.2 |
| `focus_capture_ids` | optional | Focus set — cognition only |
| `summary.readiness_cohorts_v2` | optional | Additive buckets — **not** scores |
| `summary.multi_capture_cohorts` | optional | Lane occupancy + stale warn count |
| `summary.experiment_handoff_rollup` | optional | X2 + F5 adjacency — warn-only |
| `summary.handoff_rollup` | optional | Per-stage counts by maintainer lane |
| `captures[].readiness_cohort_v2` | optional | Primary v2 cohort per row |
| `captures[].stale_age_hours` | optional | Warn-only when threshold exceeded |
| `captures[].in_focus_set` | optional | Boolean when focus set active |

F7 fields (`queue_priority`, `blocker_groups`, `readiness_cohort`, `lineage_warnings`, `experiment_rollup`) remain optional as in v1.

### 2.3 Compatibility with `rt_advisory_batch_review_v2`

PLAT may embed v2 `summary` inside v2 review export:

| Approach | Rule |
|----------|------|
| Nested | `batch_review_v2.summary_v2` holds `rt_advisory_batch_summary_v2` subset |
| Parallel | `export --schema v2` emits summary v2; `standup-export` adds grouped indexes |
| Default | Unspecified flag → F7 v1 summary (backward compatible) |

---

## 3. Multi-capture cohort summaries

### 3.1 `summary.multi_capture_cohorts`

| Subfield | Content |
|----------|---------|
| `by_primary_lane` | Counts keyed by maintainer lane: `normalize`, `review`, `approve`, `prepare`, `import_advisory` |
| `stale_age_warn_count` | Rows exceeding advisory stale threshold (warn-only) |
| `note` | Required disclaimer — not operational readiness |

**Stale threshold (advisory):** 24 hours since `candidate.json` `generated_at` unless maintainer overrides in PLAT config — document-only default in PLAN wave.

### 3.2 Stand-up minimum (v2)

Extends F7 §3.1:

| Section | Content |
|---------|---------|
| F7 minimum | `total`, `by_advisory_state`, `blocker_groups`, `readiness_cohorts`, sorted `captures` |
| v2 additions | `readiness_cohorts_v2`, `multi_capture_cohorts.by_primary_lane`, `handoff_rollup.by_stage` |
| Warn footer | `experiment_handoff_rollup.note`, lineage warn count |

### 3.3 Exemplar caps

| Rollup | Max exemplar IDs in summary |
|--------|----------------------------|
| `blocker_groups` | 5 per group (F7) |
| `readiness_cohorts_v2` | 3 per bucket in summary footer |
| Full detail | Always in `captures[]` |

---

## 4. Readiness grouping v2 (cohorts)

**Not** operational readiness. Additive labels atop F7 cohorts:

| Cohort v2 ID | Maps from |
|--------------|-----------|
| `stale_review` | `needs_review` + stale age warn |
| `stale_approve` | `needs_approve` + stale age warn |
| `multi_blocker` | ≥2 entries in `blocker_groups` |
| `experiment_handoff_warn` | `experiment_handoff_rollup` warn capture |
| `focus_highlight` | `in_focus_set: true` (when focus active) |

Primary v2 cohort = highest-priority v2 signal, else fall back to F7 `readiness_cohort`.

Forbidden field names: `readiness_score`, `operational_ready`, `tactical_readiness`.

---

## 5. Experiment + handoff rollups

### 5.1 `summary.experiment_handoff_rollup`

Combines F7 `experiment_rollup` with X2 read-only adjacency:

| Subfield | Source | Authority |
|----------|--------|-----------|
| `manifest_ref` | F5 / workbench manifest | No |
| `cohort_index_ref` | X2 cohort index store path | No |
| `cohort_status` | Index entry status string | No |
| `handoff_eligibility` | F5 derived | Warn-only |
| `warn_capture_ids` | Per-capture ladder + F5 | Warn-only |
| `review_packet_paths` | X2 export paths if known | No — not invoked by batch |
| `note` | Required disclaimer | — |

### 5.2 Rollup rules

| Rule | Behavior |
|------|----------|
| Precedence | Per-capture F6 ladder wins on conflict |
| X2 cohort `complete` | Does not set `import_ready` |
| X2 packet path present | Informational only — F8-CONT-06 |
| No manifest | Omit `experiment_handoff_rollup`; may retain F7 `experiment_rollup` |
| No cohort index | Omit `cohort_index_ref` and `cohort_status` |

### 5.3 `summary.handoff_rollup`

Per-stage counts aligned to maintainer workflow v2 lanes:

| Stage key | Maps from advisory / cohort |
|-----------|----------------------------|
| `normalize` | `needs_normalize` / P2_normalize band |
| `review` | `needs_review` |
| `approve` | `needs_approve` |
| `prepare` | `needs_prepare` |
| `import_advisory` | `ready_for_commit_advisory` |

Plus `blocked_count`, `terminal_count` — informational.

---

## 6. Blocker and cohort algorithms (normative)

1. Run F7 classification per capture ([rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md) §3)
2. Apply v2 cohort rules (§4) per row
3. Aggregate `multi_capture_cohorts.by_primary_lane` from primary maintainer lane
4. Build `handoff_rollup.by_stage` from cohort mapping
5. If manifest ref supplied, build `experiment_handoff_rollup` with X2 read-only fields
6. Apply preset/focus filters **before** aggregation when `--preset` / `--focus-captures` set

Sort capture list: `queue_priority.rank` ascending unless `--sort-profile` overrides.

---

## 7. Governance

| Rule | Enforcement |
|------|-------------|
| Banner on every aggregate | Required |
| `dry_run: true` on batch-derived v2 | Required when emitted from helper |
| No export event emission | Rollup read-only |
| No corpus writes | Preview/template depth 3 only |
| SA viewer | No consumption of live v2 rollup |
| No scoring fields | Ban `readiness_score` |

---

## 8. Reference fixtures

See [fixtures/rt_handoff/f8_advisory_examples/](../../fixtures/rt_handoff/f8_advisory_examples/).

---

## 9. PLAT implementation status

- **F7 delivered:** v1 summary, v2 review export — see [rt_plat_f7_p2_freeze_audit.md](rt_plat_f7_p2_freeze_audit.md)
- **F8 P0 delivered:** `rt_advisory_batch_summary_v2`, filter presets, focus sets, render-only template packs — see [rt_plat_f8_p0_freeze_audit.md](rt_plat_f8_p0_freeze_audit.md)
- **F8 P1 delivered:** integrated triage hub with v2 row chips, cohort_v2/handoff_stage grouping, stand-up pass selector — see [rt_plat_f8_p1_freeze_audit.md](rt_plat_f8_p1_freeze_audit.md)
- **F8 P2 delivered:** corpus-preview refinement + dry-run v2 guardrails — see [rt_plat_f8_p2_freeze_audit.md](rt_plat_f8_p2_freeze_audit.md)

---

## 10. Related

- [rt_advisory_maintainer_workflow_v2.md](rt_advisory_maintainer_workflow_v2.md)
- [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md)
- [rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md)
- [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md)
- [rt_roadmap_plat_rt_f8_v1.md](rt_roadmap_plat_rt_f8_v1.md)
