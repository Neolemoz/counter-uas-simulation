# RT Experiment Advanced UI Contract (`rt_experiment_advanced_ui_v1`)

**Phase:** PLAN-RT-F5 — UI planning (docs); PLAT-RT-F5 implements panels  
**Prerequisite:** PLAT-RT-X1, PLAT-RT-F1, PLAT-RT-F3 workbench; PLAN-RT-F5 contracts frozen  
**Authority:** [rt_f5_advanced_runtime_experiments_plan.md](../platform/rt_f5_advanced_runtime_experiments_plan.md)

Defines **read-only cognition surfaces** for advanced experiments: richer compare, matrix review, filters, and trend visualization. Extends F1 analytics UI — does not replace X1 compare/batch or F3 continuity hub.

---

## 1. Governance chrome

| Constant | Value |
|----------|-------|
| `BANNER_EXPERIMENT_F5` | `RT EXPERIMENT — derived summaries only; not operational authority` |

Display on: matrix panel, extended compare, filter bar, trend strip (F5 mode), handoff eligibility strip.

Additive only — **do not modify** frozen `BANNER_ANALYTICS`, T1/V2/X1 banners.

When F1 and F5 panels visible together, show both banners stacked or concatenated with separator.

---

## 2. Surfaces

### 2.1 Richer compare (`ExperimentExtendedComparePanel`)

| Rule | Detail |
|------|--------|
| Selection | Up to **4** pinned manifest runs (UX cap) |
| Inputs | `rt_experiment_manifest_v1`, `rt_experiment_analytics_report_v1`, `rt_experiment_metrics_report_v1` |
| Display | Badge grid: F1 compare badges + F5 extended badges (§6.2 of metrics contract) |
| Strips | Per-run terrain ridge/band text; LOS label text — prefix `cognition:` |
| Forbidden | Winner column, score column, red/green outcome colors |

Reuse [ExperimentComparePanel](../../platform/rt-sandbox-ui/src/experiment/ExperimentComparePanel.tsx) patterns for 2-run; extended panel is superset for N-run.

### 2.2 Matrix review (`ExperimentMatrixPanel`)

| Rule | Detail |
|------|--------|
| When visible | `experiment_class` = `parameter_matrix` |
| Layout | Table/heatmap: rows/cols from two primary axes (maintainer selects which axes in UI); cell = `run_id` or empty |
| Cell content | Compact badge list from `compare_pairs_extended` vs neighbors — no heat color = winner |
| Missing cells | Show `missing` from `rollup_extended.matrix_rollup` |

### 2.3 Experiment filters (`ExperimentFilterBar`)

Filter manifest run list (workbench) by:

| Filter | Source |
|--------|--------|
| `experiment_class` | manifest supplement |
| `template_id` | F1 per_run / batch join |
| `tactical_mode` | F1 per_run |
| `has_capture` | F1 per_run |
| `handoff_eligibility_hint` | F5 per_run |
| `spec_fingerprint` | F5 per_run |

Filters are **client-side** on imported manifest — no live bridge query.

### 2.4 Trend visualization (`ExperimentRepeatabilityTrendStrip`)

Extends PLAT-RT-F1 `ExperimentTrendStrip`:

| Rule | Detail |
|------|--------|
| Primary use | `repeatability_sweep` class |
| X-order | `repeat_index` if present; else `recorded_at_utc`; else `run_id` |
| Y labels | `entity_count`, optional `tti_s`, optional `occlusion_marker_count` — numeric text only |
| Annotation | Show `spec_fingerprint` when repeat group has >1 run |

Not time-series authority — explanatory scan only.

### 2.5 Handoff eligibility strip (`ExperimentHandoffEligibilityStrip`)

| Rule | Detail |
|------|--------|
| Input | `handoff_eligibility` block from metrics report |
| Display | Gate table: id, pass/fail, detail text |
| Actions | **None** — link text to SA1 workflow doc only; no approve/import buttons |
| Attestation | PLAT may add read-only checkbox mirror for `maintainer_ack_pose_reviewed` (local UI state only until metrics re-derived) |

---

## 3. Planned components (PLAT-RT-F5)

| Component | Path (planned) | Role |
|-----------|----------------|------|
| `ExperimentExtendedComparePanel` | `src/experiment/ExperimentExtendedComparePanel.tsx` | N-run compare |
| `ExperimentMatrixPanel` | `src/experiment/ExperimentMatrixPanel.tsx` | Matrix grid |
| `ExperimentFilterBar` | `src/experiment/ExperimentFilterBar.tsx` | Manifest filters |
| `ExperimentRepeatabilityTrendStrip` | `src/experiment/ExperimentRepeatabilityTrendStrip.tsx` | Repeat trend |
| `ExperimentHandoffEligibilityStrip` | `src/experiment/ExperimentHandoffEligibilityStrip.tsx` | Eligibility gates |

### Integration

- Collapsible tier in [ExperimentWorkbenchPanel](../../platform/rt-sandbox-ui/src/experiment/ExperimentWorkbenchPanel.tsx) below F1 analytics / F3 continuity.  
- `RuntimeCognitionHub`: one line when F5 visible — "Advanced experiment metrics — derived summaries only".  
- Spec import: file picker for `rt_experiment_spec_v1` JSON (optional); show compiled batch CLI snippet.

---

## 4. Explicit non-features

- SA compare mode import  
- Replay scrubber sync  
- Auto handoff commit or `rt_capture_approve` from UI  
- `federation_register`  
- Browser `capture_session` or subprocess batch  
- Live bridge re-pull for metrics  
- Distributed queue UI  
- Tactical mode switching from experiment panel  
- Readiness / effectiveness / winner UI  

---

## 5. Import paths

| Artifact | Import method |
|----------|----------------|
| Metrics report JSON | File picker / paste (maintainer) |
| Spec JSON | File picker / paste |
| Manifest / F1 report | Reuse X1/F1 import |
| Example specs | Path hint `fixtures/rt_experiments/f5_spec_examples/` |

---

## 6. Related

- [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md)
- [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md)
- [rt_experiment_analytics_ui_v1.md](rt_experiment_analytics_ui_v1.md)
- [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)
- [rt_experiment_continuity_review_v1.md](rt_experiment_continuity_review_v1.md)
