# RT-X2 P0 — Governance Review R1

**Phase:** PLAT-RT-X2 P0  
**Plan:** [rt_plat_x2_p0_cohort_index_plan.md](../platform/rt_plat_x2_p0_cohort_index_plan.md)  
**Freeze audit:** [rt_plat_x2_p0_freeze_audit.md](rt_plat_x2_p0_freeze_audit.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| New bridge commands? | **No** |
| SA viewer changes? | **No** |
| Auto-import from cohort? | **No** |
| Browser capture/import? | **No** |
| Experiment cohort = F7 readiness cohort? | **No** — distinct labels and UI copy |
| Cohort = authority? | **No** — per-manifest authority unchanged |

**Recommendation:** Freeze **PLAT-RT-X2 P0**.

---

## Authority boundaries

| Artifact | Authoritative? |
|----------|----------------|
| `rt_experiment_manifest_v1` | Explanatory mirror at pin time |
| `rt_experiment_cohort_index_v1` | Reference catalog only |
| F1/F5/F5b reports in dock | Derived / imported — display only |
| Workbench v2 lane state | UI-local |

| Finding ID | Verdict |
|------------|---------|
| X2-P0-GOV-01 | Pass |

---

## SA isolation

| Rule | Verdict |
|------|---------|
| No `platform/sa-r0-viewer/` changes | Pass |
| `assertCohortManifestRefAllowed` blocks SA paths | Pass |
| Forbidden cohort fields rejected | Pass |

| Finding ID | Verdict |
|------------|---------|
| X2-P0-GOV-SA-01 | Pass |

---

## Verdict

**Pass** — PLAT-RT-X2 P0 suitable for freeze. Contamination review deferred to **P1** when review lane becomes interactive.
