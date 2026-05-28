# RT Experiment Unified Review Flow (`rt_experiment_unified_review_v1`)

**Phase:** PLAN-RT-X2 — metrics and continuity integration planning  
**Prerequisite:** [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md), [rt_experiment_continuity_review_v1.md](rt_experiment_continuity_review_v1.md), [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md)  
**Authority:** [rt_x2_experiment_workbench_v2_plan.md](../platform/rt_x2_experiment_workbench_v2_plan.md)

Normative **maintainer review lane** sequencing F1, F3, F5, and F5b surfaces in one workflow. Ordering is advisory — skipping steps is allowed. **No** new derive algorithms in PLAN wave.

---

## 1. Review lane steps

| Step | Id | Action | Primary artifact |
|------|-----|--------|------------------|
| 1 | `select_scope` | Import cohort index or single manifest | `rt_experiment_cohort_index_v1` or `rt_experiment_manifest_v1` |
| 2 | `f1_analytics` | Derive or import F1 report | `rt_experiment_analytics_report_v1` |
| 3 | `f3_continuity` | Optional — open continuity hub per run | [rt_experiment_continuity_review_v1.md](rt_experiment_continuity_review_v1.md) |
| 4 | `f5_metrics` | Optional — derive/import F5 metrics report | `rt_experiment_metrics_report_v1` |
| 5 | `f5b_fidelity` | Optional — derive/import F5b fidelity report | `rt_experiment_fidelity_metrics_report_v1` |
| 6 | `compare` | Compare stage per workflow v2 | [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md) |
| 7 | `export_packet` | Export advisory review packet | `rt_experiment_review_packet_v1` |

**UI:** `review_step` in workbench v2 reflects current step; backward navigation does not invalidate imported reports.

---

## 2. Integration rules

### 2.1 F1 analytics (required for full review)

| Rule | Detail |
|------|--------|
| Derive | `deriveExperimentAnalytics(manifest)` in browser or `rt_experiment_analytics.py` offline |
| Primary table | `per_run[]` keyed by `run_id` |
| Filters | Use F5 [rt_experiment_advanced_ui_v1.md](rt_experiment_advanced_ui_v1.md) filter semantics — do not duplicate filter enum in X2 |
| Authority | Report is derived — not operational |

### 2.2 F3 continuity (optional)

| Rule | Detail |
|------|--------|
| Inputs | Manifest run + F1 row + optional `rt_experiment_annex_cache_v1` |
| Compare strip | Count-only annex compare when in 2-run compare — per F3 §3 |
| Full timeline | Continuity hub only — not embedded in review packet binary |
| Capture lineage | Phases A–E vocabulary read-only — no CLI from browser |

### 2.3 F5 metrics (optional)

| Rule | Detail |
|------|--------|
| Prerequisite | F1 report should exist (metrics extend F1) |
| Schema | Separate `rt_experiment_metrics_report_v1` — do not mutate F1 schema |
| Handoff hints | `handoff_eligibility_hint` per run — **display only**; no auto-import |
| Matrix class | When `experiment_class` = `parameter_matrix`, matrix panel available from compare stage |

### 2.4 F5b fidelity (optional)

| Rule | Detail |
|------|--------|
| Prerequisite | Manifest runs with fidelity supplement fields when coupling was on |
| Schema | `rt_experiment_fidelity_metrics_report_v1` per [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) §11 |
| Labels | `truth_attested` vs `explanatory` per [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md) |
| Coexistence | F5b strip does not replace F5 metrics strip |

### 2.5 F6/F7 handoff advisory (optional display)

| Rule | Detail |
|------|--------|
| Display | Existing `CaptureHandoffWorkflowPanel` / F7 triage may show in workbench footer |
| X2 scope | **Does not** extend F7 queue, batch summary, or contamination gates |
| Reference | [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) for copy discipline |

---

## 3. Join keys

| Join | Key | Scope |
|------|-----|-------|
| Manifest ↔ F1/F5/F5b | `run_id` | Within single manifest |
| Manifest ↔ F3 annex cache | `run_id` | Within single manifest |
| Cohort ↔ manifests | `experiment_id` + `manifest_ref` | Cross-manifest metadata only |
| Review packet | Paths + checksums | Export snapshot |

**Forbidden:** Cross-manifest `run_id` join (run ids may collide across experiments).

---

## 4. Schema: `rt_experiment_review_packet_v1`

Lightweight export for maintainer handoff to notes or ticket — **not** SA bundle, **not** import commit.

```json
{
  "schema": "rt_experiment_review_packet_v1",
  "packet_id": "review-2026-05-28-ridge-001",
  "created_at_utc": "2026-05-28T14:30:00.000Z",
  "governance_banner": "RT EXPERIMENT REVIEW PACKET — advisory export only; not SA import authority",
  "scope": {
    "cohort_id": "cohort-2026-05-28-ridge-program",
    "primary_manifest_ref": "fixtures/rt_experiments/f5_metrics_golden/manifest.json",
    "secondary_manifest_ref": null
  },
  "artifact_refs": [
    {
      "kind": "f1_analytics",
      "path": "runs/rt_sandbox/experiments/exp-2026/reports/analytics.json",
      "sha256": "optional"
    }
  ],
  "compare_mode": "pairwise_pinned",
  "compare_run_ids": ["m-radar-north-arc-v1-1", "m-radar-north-arc-v1-2"],
  "review_steps_completed": ["select_scope", "f1_analytics", "compare"]
}
```

| Field | Rule |
|-------|------|
| `artifact_refs[].kind` | `f1_analytics` \| `f5_metrics` \| `f5b_fidelity` \| `f3_annex_cache` \| `cohort_index` |
| `sha256` | Optional integrity hint |
| `compare_run_ids` | Explanatory selection record only |

**Forbidden:** `sa_bundle_ref`, `import_commit_id`, `readiness_verdict`.

---

## 5. Maintainer CLI map (copy-only in UI)

| Step | Maintainer CLI (representative) |
|------|----------------------------------|
| Batch / capture | `python3 scripts/rt/rt_experiment_batch.py` |
| F1 | `python3 scripts/rt/rt_experiment_analytics.py` |
| F3 annex | `python3 scripts/rt/rt_experiment_annex_pack.py` |
| F5 | `python3 scripts/rt/rt_experiment_metrics.py` |
| F5b | `python3 scripts/rt/rt_experiment_fidelity_metrics.py` |

---

## Related

- [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md)
- [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md)
- [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md)
