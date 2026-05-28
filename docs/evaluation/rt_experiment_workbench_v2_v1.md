# RT Experiment Workbench v2 Contract (`rt_experiment_workbench_v2_v1`)

**Phase:** PLAN-RT-X2 — workbench workspace planning  
**Prerequisite:** PLAT-RT-X1, PLAT-RT-F1, PLAT-RT-F3, PLAT-RT-F5, PLAT-RT-F5b frozen  
**Authority:** [rt_x2_experiment_workbench_v2_plan.md](../platform/rt_x2_experiment_workbench_v2_plan.md)  
**Supplements:** [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)

Normative **workspace layout** for RT experiment review after X1/F5 panels. **Planning only** — no implementation in PLAN wave.

---

## 0. Core invariant

| Rule | Detail |
|------|--------|
| Scope | `platform/rt-sandbox-ui/` experiment workbench region only |
| Authority | [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md) manifest + maintainer CLIs remain truth |
| SA | No SA viewer layout or replay bundle changes |
| Capture | No browser `capture_session`; maintainer CLI only |

**Governance banner (normative):**

`RT EXPERIMENT v2 — cohort review is explanatory; maintainer CLIs remain authority`

Display on workbench v2 chrome when any cohort or unified review mode is active. Stack with frozen `BANNER_ANALYTICS`, `BANNER_EXPERIMENT_F5`, F5b fidelity banners — do not replace.

---

## 1. Workstation zones

Extends frozen X1 workbench conceptually (`ExperimentWorkbenchPanel` and nested panels).

```text
+------------------------------------------------------------------+
| Experiment workbench v2 header (banners + cohort label)            |
+------------------------------------------------------------------+
| Cohort navigator | Active review lane | Report dock (collapsible)|
+------------------+-------------------+---------------------------+
| Compare stage (mode selector + panel host)                        |
+------------------------------------------------------------------+
| Maintainer actions strip (CLI hints only)                         |
+------------------------------------------------------------------+
| Frozen X1 panels (pin, batch guidance) — unchanged authority      |
+------------------------------------------------------------------+
| Optional: F6/F7 handoff advisory footer (read-only, no F8 scope)   |
+------------------------------------------------------------------+
```

### 1.1 Cohort navigator

| Element | Rule |
|---------|------|
| Input | `rt_experiment_cohort_index_v1` or single manifest import |
| Display | Cohort label → manifest refs → run count per manifest |
| Selection | Sets **primary manifest** for review lane; optional **secondary manifest** for multi-manifest diff |
| Actions | Import JSON only — no merge into authoritative manifest |

### 1.2 Active review lane

| Field | Rule |
|-------|------|
| `primary_manifest_ref` | Repo-relative path or in-browser imported copy |
| `secondary_manifest_ref` | Optional; used only for `multi_manifest_diff` compare mode |
| `active_run_id` | Optional focus run within primary manifest |
| `review_step` | One of unified review steps per [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md) |

Lane state is **UI-local** (localStorage or session store). Not bridge authority.

### 1.3 Report dock

Slots for **imported or derived** JSON (read-only):

| Slot | Schema | Source |
|------|--------|--------|
| F1 analytics | `rt_experiment_analytics_report_v1` | `deriveExperimentAnalytics` or `rt_experiment_analytics.py` output |
| F5 metrics | `rt_experiment_metrics_report_v1` | `deriveExperimentMetrics` or `rt_experiment_metrics.py` |
| F5b fidelity | `rt_experiment_fidelity_metrics_report_v1` | `rt_experiment_fidelity_metrics.py` |
| F3 annex cache | `rt_experiment_annex_cache_v1` | Per-run cache keys — optional |

Missing slot shows dashed placeholder — **no** live bridge re-pull for stored runs.

### 1.4 Compare stage

Hosts compare modes per [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md):

- Default: `pairwise_pinned` (X1-compatible)
- Optional: `extended_n_run`, `cohort_matrix`, `multi_manifest_diff`

### 1.5 Maintainer actions strip

| Action | UI behavior |
|--------|-------------|
| Batch run | Copy-only `rt_experiment_batch.py` command |
| Analytics derive | Copy-only `rt_experiment_analytics.py` |
| Annex pack | Copy-only `rt_experiment_annex_pack.py` |
| F5 metrics | Copy-only `rt_experiment_metrics.py` / fidelity metrics CLI |
| Export review packet | Writes `rt_experiment_review_packet_v1` JSON locally — **not** SA import |

**Forbidden:** Run subprocess, `capture_session`, `rt_sa_import`, federation job submit.

---

## 2. Coexistence with frozen panels

| Frozen panel | V2 relationship |
|--------------|-----------------|
| X1 pin / compare | Remains; compare stage may delegate to `ExperimentComparePanel` for 2-run |
| F1 analytics panel | May collapse into report dock when unified review active |
| F3 continuity hub | Opened from review lane step 3 — not replaced |
| F5 matrix / extended compare / filters | Opened from compare stage — navigation per X2 |
| F5b fidelity compare strip | Linked from report dock fidelity slot |

PLAT may use accordion gating to reduce duplicate panels — PLAN does not mandate removal of frozen panels.

---

## 3. Explicit non-goals

- SA replay workstation (PLAN-SA-H*) layout or corpus browser
- Offline orchestration queue authority (PLAN-SA-H3)
- Federation manifest writes from browser
- Experiment class or spec schema changes (F5 model frozen)
- Distributed multi-session compare across bridges

---

## Related

- [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md)
- [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md)
- [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md)
- [rt_experiment_advanced_ui_v1.md](rt_experiment_advanced_ui_v1.md)
