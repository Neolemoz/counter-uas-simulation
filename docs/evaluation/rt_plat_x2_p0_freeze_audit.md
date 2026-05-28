# RT-X2 P0 — Freeze Audit (PLAT-RT-X2 P0)

**Phase:** PLAT-RT-X2 P0 — cohort index store + read-only workbench v2 shells  
**Status:** frozen

**Plan:** [rt_plat_x2_p0_cohort_index_plan.md](../platform/rt_plat_x2_p0_cohort_index_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | `cohortSchema.ts` + `cohortImportGuards.ts` | Yes |
| 2 | `cohortIndexStore.ts` | Yes |
| 3 | `workbenchV2State.ts` + `experimentUnifiedReview.ts` | Yes |
| 4 | V2 shell components + `ExperimentWorkbenchV2Shell.tsx` | Yes |
| 5 | `ExperimentWorkbenchPanel` read-only wiring | Yes |
| 6 | `BANNER_EXPERIMENT_V2` | Yes |
| 7 | Fixture parity + store tests | Yes |
| 8 | P0 reviews | Yes |
| 9 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`. No changes under `platform/sa-r0-viewer/`.

---

## P0 summary

**Cohort store:** `rt_experiment_cohort_index_v1` map in localStorage; import/export with SA path guards.  
**Workbench v2:** Collapsible shells for cohort navigator, unified review lane (read-only), report dock presence, compare stage placeholder.  
**Frozen X1:** Pin, compare, analytics, F5 panels unchanged below v2 region.

---

## Boundary guarantees

- Experiment cohort ≠ F7 readiness cohort (explicit UI copy)  
- Cohort index ≠ merged manifest authority  
- No bridge / derive / report schema changes  
- No browser capture or SA import  
- Report dock display-only in P0 — import wiring is P1  

---

## Regression evidence

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | pass |
| `test_rt_experiment_batch.py` | 4 passed |
| Vitest (rt-sandbox-ui) | 297 passed |
| `npm run build` | pass |
| `tier0-rt-ui` | pass |

---

## Recommended next (advisory)

| Option | When |
|--------|------|
| **PLAT-RT-X2 P1** | Default — unified review panel, report dock import, step navigation to frozen F1/F3/F5 panels |
| **Platform checkpoint review** | Optional if workbench panel extraction desired before P1 |

Default: **PLAT-RT-X2 P1** per [rt_roadmap_plat_rt_x2_v1.md](rt_roadmap_plat_rt_x2_v1.md).

---

## Stop line

PLAT-RT-X2 P0 frozen. Do not start **P1** without implementation plan + governance + freeze. Stop before P2 in this wave.
