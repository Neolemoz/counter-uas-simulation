# RT-X2 — Experiment Review R1

**Phase:** PLAN-RT-X2 — experiment workbench v2 (docs only)  
**Plan:** [rt_x2_experiment_workbench_v2_plan.md](../platform/rt_x2_experiment_workbench_v2_plan.md)  
**Contracts:** [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md), [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md)

Experiment semantics and mentor-facing value review — **not** operational effectiveness claims.

---

## Executive summary

| Topic | Verdict |
|-------|---------|
| Compare modes cover X1 + F5 gaps | **Pass** |
| Cohort model avoids merged authority | **Pass** |
| Unified review lane ordering | **Pass** |
| Forbidden winner/readiness language | **Pass** |
| Mentor value vs complexity | **Pass-with-conditions** |

---

## 1. Compare workflow v2

| Mode | Assessment |
|------|------------|
| `pairwise_pinned` | Preserves X1 mentor A/B workflow |
| `extended_n_run` | Aligns with F5 cap (4) — suitable for sweep review |
| `cohort_matrix` | Correctly delegates layout to F5 matrix panel |
| `multi_manifest_diff` | Metadata-only — avoids false cross-run pairing |

**Condition:** PLAT must label `multi_manifest_diff` as manifest metadata compare, not run outcome compare.

---

## 2. Cohort organization

| Rule | Assessment |
|------|------------|
| `manifest_refs[]` | Clear program-level navigation |
| 8-manifest UX cap | Reasonable warn threshold |
| Tags | Useful for maintainer taxonomy without readiness semantics |

Example fixture ([x2_cohort_index_example.json](../../fixtures/rt_experiments/x2_cohort_index_example.json)) validates two-manifest, six-run program shape.

---

## 3. Unified review flow

| Step | Value |
|------|-------|
| F1 required path | Ensures analytics before deep compare |
| F3/F5/F5b optional | Matches real maintainer workflows |
| Review packet | Useful stand-up artifact — must stay non-import |

**Gap closed:** Prior waves exposed panels in isolation; X2 defines one lane without new derive math.

---

## 4. Forbidden semantics audit

Contracts explicitly forbid:

- winner labels, readiness scores, effectiveness claims  
- auto handoff on `import_ready` / F5 `eligible`  
- cross-manifest `run_id` join  

| Finding ID | Verdict |
|------------|---------|
| X2-EXP-01 | Pass |

---

## 5. Residual conditions (PLAT)

| ID | Condition |
|----|-----------|
| X2-EXP-C1 | Extended compare must not imply statistical proof from repeatability trend |
| X2-EXP-C2 | Fidelity lines remain `explanatory` unless F5b coupling was on at capture |

---

## Recommendation

Freeze **PLAN-RT-X2**. Proceed to **PLAT-RT-X2 P0** when experiment cohort/compare is the binding maintainer constraint.
