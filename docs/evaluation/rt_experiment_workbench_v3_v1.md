# RT Experiment Workbench v3 Contract (`rt_experiment_workbench_v3_v1`)

**Phase:** PLAN-RT-X3 — workbench ergonomics planning  
**Prerequisite:** PLAT-RT-X2, PLAT-RT-C4 frozen; [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md)  
**Authority:** [rt_x3_experiment_workbench_v3_plan.md](../platform/rt_x3_experiment_workbench_v3_plan.md)  
**Supplements:** [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md), [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md)

Normative **ergonomics** for RT experiment review after X2 zones and C4 section extraction. **Planning only** — no implementation in PLAN wave.

---

## 0. Core invariant

| Rule | Detail |
|------|--------|
| Scope | `platform/rt-sandbox-ui/` experiment workbench region only |
| Authority | [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md) manifest + maintainer CLIs remain truth |
| X2 zones | Cohort navigator, review lane, report dock, compare stage — **layout preserved**; v3 adds navigation affordances |
| SA | No SA viewer layout or replay bundle changes |
| Capture | No browser `capture_session`; maintainer CLI only |

**Governance banner (normative — stack with X2):**

`RT EXPERIMENT v3 — navigation and packets are explanatory; maintainer CLIs remain authority`

Display when any v3 navigation affordance is active (program context, breadcrumb, manifest roster). Do not replace X2 or F5b banners.

---

## 1. Workstation zones (v3 extensions)

Extends frozen v2 layout (`ExperimentWorkbenchV2Shell` and nested panels). PLAT may render v3 affordances inside existing zones without relocating authority controls.

```text
+------------------------------------------------------------------+
| v3 program context strip (cohort label, tags, breadcrumb)         |
+------------------------------------------------------------------+
| Cohort navigator | Active review lane | Report dock (grouped)    |
+------------------+-------------------+---------------------------+
| Manifest roster (within active cohort) | manifest focus label    |
+------------------------------------------------------------------+
| Compare stage (mode coach + panel host)                           |
+------------------------------------------------------------------+
| Maintainer actions strip (CLI hints only)                         |
+------------------------------------------------------------------+
| Frozen X1 / C4 sections (toolbar, compare, F5) — unchanged        |
+------------------------------------------------------------------+
```

### 1.1 Program context strip

| Element | Rule |
|---------|------|
| Cohort label | From active `rt_experiment_cohort_index_v1.label` or “single manifest” when no cohort |
| Tag chips | Render `tags[]` from cohort index; click filters manifest roster (UI-local) |
| Breadcrumb | `cohort → manifest → run` — each segment clickable for scope change |
| Secondary manifest | Explicit picker — not only “second manifest in list order” |

**Forbidden:** readiness cohort labels; SA corpus paths; merge into single manifest authority.

### 1.2 Manifest roster (within cohort)

| Column | Source |
|--------|--------|
| Label | `manifest_refs[].label` |
| `experiment_id` | manifest ref entry |
| Run count | `run_count_hint` or loaded manifest `runs.length` |
| Primary / secondary | UI badges from `workbenchV2State` |
| Ref status | `ok` \| `missing` \| `id_mismatch` — advisory validation on load |

| Rule | Detail |
|------|--------|
| Selection | Sets primary or secondary manifest ref per user action |
| Validation | `experiment_id` mismatch → warn chip only — no auto-fix |
| Cap | Warn when cohort has more than **8** manifests (X2 cohort UX cap) |

### 1.3 Manifest focus label (review lane)

| Field | Rule |
|-------|------|
| Display | `Primary: {label} ({experiment_id})` and optional `Secondary: …` |
| `active_run_id` | Append `→ run {run_id}` when run focus set |
| Storage | `WorkbenchV2State` — UI-local; not bridge |

### 1.4 Coexistence with PLAT-RT-C4

| C4 extraction | v3 relationship |
|---------------|-----------------|
| `ExperimentManifestToolbar` | Remains authoritative pin/import/export entry |
| `ExperimentCompareSection` | Compare stage hosts; v3 adds coach strip above |
| `ExperimentF5MetricsSection` | Opened from compare or review lane — not removed |
| `useJsonPromptImport` | Shared import path — v3 does not add parallel import flows |

---

## 2. UI-local state (additive keys)

Optional keys in `WorkbenchV2State` (PLAT — not required in PLAN freeze):

| Key | Type | Purpose |
|-----|------|---------|
| `review_session_id` | string | Correlates dock + packet exports in one maintainer session |
| `cohort_tag_filter` | string \| null | Active tag filter for roster |
| `breadcrumb_focus` | `cohort` \| `manifest` \| `run` | Highlights active breadcrumb segment |

**Forbidden:** persisting merged manifests; bridge session ids as review authority.

---

## 3. Explicit non-goals

- SA replay workstation (PLAN-SA-H*) layout or corpus browser
- Offline orchestration queue authority (PLAN-SA-H3)
- Federation manifest writes from browser
- New cohort schema version (remains `rt_experiment_cohort_index_v1`)
- Distributed multi-session compare across bridges
- Runtime or bridge coupling for ref validation (offline CLI hints only in review workflow v3)

---

## Related

- [rt_experiment_review_workflow_v3_v1.md](rt_experiment_review_workflow_v3_v1.md)
- [rt_experiment_compare_workflow_v3_v1.md](rt_experiment_compare_workflow_v3_v1.md)
- [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md)
- [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md)
