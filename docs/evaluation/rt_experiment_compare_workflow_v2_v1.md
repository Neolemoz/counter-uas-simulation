# RT Experiment Compare Workflow v2 (`rt_experiment_compare_workflow_v2_v1`)

**Phase:** PLAN-RT-X2 — compare ergonomics planning  
**Prerequisite:** [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md) §3, [rt_experiment_advanced_ui_v1.md](rt_experiment_advanced_ui_v1.md)  
**Authority:** [rt_x2_experiment_workbench_v2_plan.md](../platform/rt_x2_experiment_workbench_v2_plan.md)

Extends frozen compare semantics with **modes** and **navigation** rules. Does not add winner language or new badge types beyond F1/F5 extensions already frozen.

---

## 1. Compare modes

| Mode | Id | Runs | Primary UI host |
|------|-----|------|-----------------|
| Pairwise pinned | `pairwise_pinned` | 2 | `ExperimentComparePanel` (X1) |
| Extended N-run | `extended_n_run` | 2–4 | `ExperimentExtendedComparePanel` (F5) |
| Cohort matrix | `cohort_matrix` | N (matrix cells) | `ExperimentMatrixPanel` (F5) |
| Multi-manifest diff | `multi_manifest_diff` | 2 manifests (metadata) | V2 metadata table (PLAT) |

Default mode: `pairwise_pinned`.

---

## 2. Mode rules

### 2.1 `pairwise_pinned`

| Rule | Detail |
|------|--------|
| Sources | Two pinned manifest runs, or one live slot + one pinned |
| Badges | X1 §3: `mode_changed`, `assignment_changed`, `tti_delta`, `pause_resume_delta` |
| Annex | F3 compare strip (counts) when both runs have annex summary |
| Live compare | Allowed when ≤3 sessions — same as X1 |

### 2.2 `extended_n_run`

| Rule | Detail |
|------|--------|
| Cap | **4** runs (F5 UX cap) |
| Inputs | `rt_experiment_manifest_v1`, F1 report, optional F5 metrics report |
| Badges | F1 badges + F5 extended badges per [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) |
| Terrain / LOS | Prefix `cognition:` on strip lines |
| When to use | Single manifest, same experiment class, parameter or repeat sweep |

**Navigation (X2):** Cohort navigator filters run list → user selects up to 4 → hand off to extended panel.

### 2.3 `cohort_matrix`

| Rule | Detail |
|------|--------|
| Prerequisite | `experiment_class` = `parameter_matrix` on primary manifest |
| Layout | F5 matrix table — rows/cols from `matrix_axes` |
| Missing cells | Show `missing` per F5 rollup — no heat color = winner |
| X2 addition | Link from cohort entry to manifest that owns matrix |

**Navigation:** Select cohort → select matrix manifest → open matrix mode.

### 2.4 `multi_manifest_diff`

| Rule | Detail |
|------|--------|
| Inputs | Primary + secondary manifest from cohort index |
| Compare surface | **Metadata only** — not merged entity snapshots |

| Compared field | Source |
|----------------|--------|
| `experiment_id` | manifest root |
| `runs.length` | manifest |
| `experiment_class` | manifest supplement (if uniform) |
| `spec_fingerprint` set | unique fingerprints across runs |
| `capture_count` | runs with `capture_candidate_id` |
| Tag overlap | cohort tags vs manifest supplements |

**Forbidden:** Diffing tactical snapshots across manifests as operational truth; merged manifest export.

---

## 3. Mode selection guidance

| Maintainer intent | Recommended mode |
|-------------------|------------------|
| Quick A/B two runs | `pairwise_pinned` |
| Sweep / repeatability scan | `extended_n_run` |
| Template × dwell matrix | `cohort_matrix` |
| Two related experiments same program | `multi_manifest_diff` then drill into each manifest |

---

## 4. Forbidden compare semantics

- Winner column, best-run highlight, red/green outcome colors  
- Readiness scores, effectiveness claims, threat neutralization language  
- Auto handoff when `import_ready` or F5 `eligible`  
- Cross-manifest `run_id` pairing  
- Live bridge re-pull to refresh pinned snapshots during compare  

---

## 5. Banners

Stack when multiple modes visible:

1. Workbench v2 banner ([rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md))  
2. `BANNER_ANALYTICS` when F1-derived content shown  
3. `BANNER_EXPERIMENT_F5` when F5 panels active  
4. F5b fidelity banner when fidelity metrics visible  

---

## Related

- [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md)
- [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md)
- [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)
