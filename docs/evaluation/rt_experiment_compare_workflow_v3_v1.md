# RT Experiment Compare Workflow v3 (`rt_experiment_compare_workflow_v3_v1`)

**Phase:** PLAN-RT-X3 — compare readability planning  
**Prerequisite:** [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md), PLAT-RT-X2 P2 frozen  
**Authority:** [rt_x3_experiment_workbench_v3_plan.md](../platform/rt_x3_experiment_workbench_v3_plan.md)

Extends frozen compare v2 with **mode coach**, **compare-status vocabulary**, and **multi-manifest drill-down** rules. Compare mode ids unchanged.

---

## 1. Compare modes (unchanged)

| Mode | Id | Unchanged from v2 |
|------|-----|-------------------|
| Pairwise pinned | `pairwise_pinned` | Yes |
| Extended N-run | `extended_n_run` | Yes |
| Cohort matrix | `cohort_matrix` | Yes |
| Multi-manifest diff | `multi_manifest_diff` | Yes |

Default mode: `pairwise_pinned`.

---

## 2. Mode coach (v3)

One-line coach strip above compare stage (explanatory):

| Mode | Coach text (normative) |
|------|------------------------|
| `pairwise_pinned` | Two runs — quick A/B within one manifest or one live slot + one pin. |
| `extended_n_run` | Up to 4 runs — sweep or repeatability within one manifest. |
| `cohort_matrix` | Parameter matrix layout — requires `experiment_class` = `parameter_matrix`. |
| `multi_manifest_diff` | Manifest metadata only — not run outcome compare; drill into each manifest separately. |

**Forbidden:** “best mode”, “recommended winner”, readiness language.

---

## 3. Compare-status vocabulary (v3)

Shared labels for pairwise, extended, fidelity, session, and multi-manifest surfaces (aligns with CHECKPOINT-RT-POST-V4 duplication note). **Display only.**

| Status id | Label | Use when |
|-----------|-------|----------|
| `aligned` | aligned | Keys match; no advisory mismatch |
| `divergent` | divergent | Metadata or badge mismatch |
| `missing` | missing | Expected artifact or run not present |
| `explanatory` | explanatory | F5b or cognition-prefixed line |
| `not_comparable` | not comparable | Cross-manifest run pairing attempted — **forbidden** — show error state |

**Forbidden:** `winner`, `better`, `passed`, `failed`, red/green outcome colors.

PLAT may implement as `formatCompareStatus(id)` helper — not required in PLAN wave.

---

## 4. Compare readability rules

### 4.1 Table density

| Surface | Cap | Overflow |
|---------|-----|----------|
| Extended N-run | 4 runs | horizontal scroll |
| Cohort matrix | F5 matrix dims | `missing` cell label |
| Multi-manifest diff | 2 manifests | fixed column order (see §5) |

### 4.2 Badge stacking

When multiple reports visible, stack banners in order:

1. Workbench v3 banner  
2. Workbench v2 banner  
3. `BANNER_ANALYTICS` / `BANNER_EXPERIMENT_F5` / F5b fidelity banner per v2 §5  

---

## 5. Multi-manifest diff refinement (v3)

Extends [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md) §2.4.

### 5.1 Column order (normative)

| Order | Field |
|-------|-------|
| 1 | Field name |
| 2 | Primary manifest value |
| 3 | Secondary manifest value |
| 4 | Status (`aligned` \| `divergent` \| `missing`) |

### 5.2 Mismatch emphasis

| Rule | Detail |
|------|--------|
| Divergent row | Bold label + `divergent` status chip — **no** red/green |
| Missing secondary | `missing` — not an error verdict |
| Tooltip | “Metadata compare only — open manifest for run-level review” |

### 5.3 Drill-down flow

```text
multi_manifest_diff table
  → [Open primary] loads primary manifest into review scope
  → [Open secondary] loads secondary manifest (does not merge)
  → optional switch to pairwise_pinned within chosen manifest
```

**Forbidden:** merged manifest export; cross-manifest `run_id` pairing; tactical snapshot diff as operational truth.

---

## 6. Report dock ergonomics (compare adjacency)

| Rule | Detail |
|------|--------|
| Compare stage | May focus report dock “Metrics” or “Analytics” group when mode needs F1/F5 |
| Packet | `compare_summary` section captures mode + run ids per [rt_experiment_review_workflow_v3_v1.md](rt_experiment_review_workflow_v3_v1.md) |

---

## 7. Forbidden compare semantics (carried forward)

Per v2 §4 plus v3:

- Winner column, best-run highlight, outcome colors  
- Readiness scores, effectiveness claims  
- Auto handoff when `import_ready` or F5 `eligible`  
- Cross-manifest `run_id` pairing  
- Live bridge re-pull to refresh pinned snapshots during compare  

---

## Related

- [rt_experiment_workbench_v3_v1.md](rt_experiment_workbench_v3_v1.md)
- [rt_experiment_review_workflow_v3_v1.md](rt_experiment_review_workflow_v3_v1.md)
- [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md)
- [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md)
