# RT Experiment Cohort Contract (`rt_experiment_cohort_v1`)

**Phase:** PLAN-RT-X2 — cohort organization planning  
**Prerequisite:** [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)  
**Authority:** [rt_x2_experiment_workbench_v2_plan.md](../platform/rt_x2_experiment_workbench_v2_plan.md)

Defines **`rt_experiment_cohort_index_v1`** for grouping multiple experiment manifests under one maintainer review program. **References only** — cohort index is not operational authority and does not merge manifests.

---

## 1. Governance

| Rule | Detail |
|------|--------|
| Banner | Required on every cohort index document |
| Cohort | Advisory label for related manifests — **not** readiness cohort, **not** SA federation corpus |
| Merge | **Forbidden** — UI may load manifests side-by-side; must not produce merged `rt_experiment_manifest_v1` as authority |
| SA paths | Forbidden in `manifest_refs` — use repo-relative RT paths only |

**Normative banner:**

`RT EXPERIMENT COHORT — index references only; per-manifest authority unchanged`

---

## 2. Schema: `rt_experiment_cohort_index_v1`

```json
{
  "schema": "rt_experiment_cohort_index_v1",
  "cohort_id": "cohort-2026-05-28-ridge-program",
  "label": "Ridge comparison program (May 2026)",
  "created_at_utc": "2026-05-28T12:00:00.000Z",
  "governance_banner": "RT EXPERIMENT COHORT — index references only; per-manifest authority unchanged",
  "tags": ["terrain_comparison", "parameter_matrix"],
  "manifest_refs": [
    {
      "manifest_ref": "fixtures/rt_experiments/f5_metrics_golden/manifest.json",
      "experiment_id": "exp-2026-05-26-template-dwell-matrix",
      "label": "Dwell matrix (golden)",
      "run_count_hint": 4
    }
  ],
  "notes": "Optional maintainer prose — not evaluated by tooling"
}
```

### 2.1 Core fields

| Field | Required | Rule |
|-------|----------|------|
| `schema` | Yes | `rt_experiment_cohort_index_v1` |
| `cohort_id` | Yes | Stable string; snake_case or kebab |
| `label` | Yes | Human-readable cohort title |
| `governance_banner` | Yes | §1 normative line or equivalent |
| `manifest_refs` | Yes | Non-empty array |
| `created_at_utc` | No | ISO-8601 |
| `tags` | No | Maintainer taxonomy strings |
| `notes` | No | Free text; forbidden lexicon per workbench v1 |

### 2.2 Manifest reference entry

| Field | Required | Rule |
|-------|----------|------|
| `manifest_ref` | Yes | Repo-relative path to exported `rt_experiment_manifest_v1` JSON **or** path under `runs/rt_sandbox/experiments/` |
| `experiment_id` | Yes | Must match manifest `experiment_id` when loaded (validation advisory) |
| `label` | Yes | Short label within cohort |
| `run_count_hint` | No | Expected `runs.length` for UI badges |
| `spec_fingerprint` | No | Rollup hint from F5 supplement — explanatory |
| `experiment_class` | No | Copy from manifest supplement when present |

**Forbidden fields:** `sa_corpus_ref`, `import_ready`, `readiness_score`, `winner_run_id`.

---

## 3. Multi-manifest review rules

| Rule | Detail |
|------|--------|
| Min manifests | 1 (degenerate cohort = single manifest shortcut) |
| Max manifests (UX advisory) | 8 — PLAT may warn above cap |
| Load order | Order in `manifest_refs[]` is display order only |
| Cross-manifest compare | Metadata diff per [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md) `multi_manifest_diff` |
| Run join | Join analytics/metrics by `run_id` **within** one manifest only |

---

## 4. Storage (PLAT advisory)

| Store | Rule |
|-------|------|
| Key | `rt_experiment_cohort_index_v1` in localStorage map by `cohort_id` |
| Export | Maintainer may export cohort JSON alongside manifest exports |
| Import | Validate schema + banner; block SA path patterns |

---

## 5. Reference fixture

[fixtures/rt_experiments/x2_cohort_index_example.json](../../fixtures/rt_experiments/x2_cohort_index_example.json) — two manifests, six runs total (hints).

---

## Related

- [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md)
- [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md)
- [rt_experiment_model_v1.md](rt_experiment_model_v1.md)
