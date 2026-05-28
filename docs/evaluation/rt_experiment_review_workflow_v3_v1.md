# RT Experiment Review Workflow v3 (`rt_experiment_review_workflow_v3_v1`)

**Phase:** PLAN-RT-X3 — review lane and packet ergonomics planning  
**Prerequisite:** [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md), PLAT-RT-X2 P1–P2 frozen  
**Authority:** [rt_x3_experiment_workbench_v3_plan.md](../platform/rt_x3_experiment_workbench_v3_plan.md)

Extends frozen unified review with **step completion affordances**, **grouped report dock**, and **packet section catalog**. Ordering and join keys unchanged from X2.

---

## 1. Review lane steps (unchanged ids)

Step ids remain per [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md) §1:

`select_scope` → `f1_analytics` → `f3_continuity` → `f5_metrics` → `f5b_fidelity` → `compare` → `export_packet`

Skipping steps remains allowed. Backward navigation does not invalidate imported reports.

---

## 2. Step completion affordances (v3)

| Step | Completion states | UI signal |
|------|-------------------|-----------|
| `select_scope` | `complete` when primary manifest loaded | check |
| `f1_analytics` | `imported` \| `derived` \| `missing` | slot badge in lane |
| `f3_continuity` | `opened` \| `skipped` \| `n/a` | optional step — hollow when skipped |
| `f5_metrics` | `imported` \| `derived` \| `missing` | slot badge |
| `f5b_fidelity` | `imported` \| `derived` \| `missing` \| `n/a` | `n/a` when no fidelity supplement |
| `compare` | `active` \| `complete` | ties to compare stage mode |
| `export_packet` | `draft` \| `exported` | packet tab focus |

**Rule:** Completion is **explanatory** — not a gate for capture, import, or handoff.

**Keyboard-safe navigation (PLAT):** Prev/next step buttons move `review_step` only; no implicit subprocess or bridge call.

---

## 3. Report dock groups (v3)

Reorganizes frozen [report dock slots](rt_experiment_unified_review_v1.md) without changing slot ids or schemas.

| Group | Slot ids | Collapse default |
|-------|----------|------------------|
| Analytics | `f1_analytics` | expanded when `review_step` = `f1_analytics` |
| Continuity | `f3_annex` (optional) | collapsed until step 3 |
| Metrics | `f5_metrics` | expanded when step 4 active |
| Fidelity | `f5b_fidelity` | expanded when step 5 active |

| Rule | Detail |
|------|--------|
| Import | Same JSON prompt flow as X2 / C4 `useJsonPromptImport` — **no** new MIME types |
| Preview | Truncate at 2000 chars (existing P1 behavior) |
| Export slot | Copy/download derived JSON — **not** SA import |

---

## 4. Packet organization — `sections[]` (additive)

`rt_experiment_review_packet_v1` remains the schema id. PLAT may add optional **`sections[]`** for organized exports without breaking P2 consumers that ignore unknown keys.

### 4.1 Section catalog

| Section id | Title | Content summary |
|------------|-------|-----------------|
| `scope` | Review scope | `scope` object + cohort label + manifest focus labels |
| `reports` | Imported reports | Summarized `artifact_refs[]` with kind + path |
| `compare_summary` | Compare snapshot | `compare_mode`, `compare_run_ids`, mode coach one-liner |
| `advisory_refs` | Handoff advisory | Optional F6/F7 rollup ids — **display refs only** |
| `cli_hints` | Maintainer CLIs | Copy-only command strings per step |

### 4.2 Section entry shape

```json
{
  "section_id": "compare_summary",
  "title": "Compare snapshot",
  "body_markdown": "Mode: pairwise_pinned. Runs: run-a, run-b. Metadata-only across manifests when in multi_manifest_diff.",
  "refs": []
}
```

| Rule | Detail |
|------|--------|
| `body_markdown` | Maintainer prose + factual labels — no winner language |
| `refs` | Optional path strings — same allow-list as `artifact_refs` |
| Required top-level fields | Unchanged from X2 packet schema |

**Forbidden on packet:** `sa_bundle_ref`, `import_commit_id`, `readiness_verdict`, `winner_run_id`.

Reference: [fixtures/rt_experiments/x3_review_packet_sections_example.json](../../fixtures/rt_experiments/x3_review_packet_sections_example.json).

---

## 5. Review packet workflow (v3)

| Stage | Maintainer action | Authority |
|-------|-------------------|-----------|
| Draft | Build preview in report dock packet tab | Browser-local |
| Review | Read sections + governance banner | Explanatory |
| Export file | Download JSON via existing export helper | Local file only |
| Share | Paste into ticket/notes | **Not** SA import |

**Banner (required on export):**

`RT EXPERIMENT REVIEW PACKET — advisory export only; not SA import authority`

---

## 6. Join keys (unchanged)

Per [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md) §3 — **no** cross-manifest `run_id` join.

---

## 7. F6/F7 adjacency (display only)

| Rule | Detail |
|------|--------|
| `advisory_refs` section | May list handoff row ids or advisory queue keys — read-only |
| Scope | **Does not** extend F7 queue, batch summary, or contamination gates |
| PLAT | Contamination review required at PLAT-RT-X3 P1 |

---

## 8. Maintainer CLI map (unchanged — copy-only)

| Step | CLI (representative) |
|------|----------------------|
| Batch / capture | `python3 scripts/rt/rt_experiment_batch.py` |
| F1 | `python3 scripts/rt/rt_experiment_analytics.py` |
| F3 annex | `python3 scripts/rt/rt_experiment_annex_pack.py` |
| F5 | `python3 scripts/rt/rt_experiment_metrics.py` |
| F5b | `python3 scripts/rt/rt_experiment_fidelity_metrics.py` |

Optional offline ref validation (documented only — **not** new subcommand in PLAN):

```bash
# Advisory: verify manifest path exists before cohort import (maintainer shell)
test -f fixtures/rt_experiments/f5_metrics_golden/manifest.json
```

---

## Related

- [rt_experiment_workbench_v3_v1.md](rt_experiment_workbench_v3_v1.md)
- [rt_experiment_compare_workflow_v3_v1.md](rt_experiment_compare_workflow_v3_v1.md)
- [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md)
- [rt_experiment_import_hardening_v1.md](rt_experiment_import_hardening_v1.md)
