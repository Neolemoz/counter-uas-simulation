# RT — PLAT-RT-X2 Implementation Roadmap v1

**Phase:** PLAN-RT-X2 frozen → **PLAT-RT-X2** P0–P2 advisory  
**Prerequisite:** [rt_x2_freeze_audit.md](rt_x2_freeze_audit.md) (PLAN-RT-X2 docs frozen)  
**Contracts:** [rt_experiment_workbench_v2_v1.md](rt_experiment_workbench_v2_v1.md), [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md), [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md), [rt_experiment_compare_workflow_v2_v1.md](rt_experiment_compare_workflow_v2_v1.md)

---

## P0 — Cohort index + multi-manifest import

**Prerequisite:** PLAN-RT-X2 frozen

| Item | Location | Status |
|------|----------|--------|
| `cohortIndexStore.ts` | `platform/rt-sandbox-ui/src/experiment/` | Done |
| Fixture parity test | `x2_cohort_index_example.json` | Done |
| Cohort navigator UI (read-only) | Workbench v2 zone | Done |
| Governance: no SA path refs | Vitest | Done |
| `tier0-rt-ui` | CI | Done |

**PLAT plan:** `docs/platform/rt_plat_x2_p0_cohort_index_plan.md` (create at P0 start)

**Freeze target:** `docs/evaluation/rt_plat_x2_p0_freeze_audit.md`

---

## P1 — Unified review panel

**Prerequisite:** P0 frozen

| Item | Location | Status |
|------|----------|--------|
| `ExperimentUnifiedReviewPanel.tsx` | `src/experiment/` | Done |
| Report dock (F1/F5/F5b import slots) | Panel | Done |
| Review lane stepper | Panel | Done |
| Wire existing derive helpers | `analyticsDerive.ts`, `metricsDerive.ts` | Done |
| F3 continuity entry | Link to frozen continuity hub | Done |
| Contamination review (F6/F7 adjacency) | Docs | Done |

**Freeze target:** `docs/evaluation/rt_plat_x2_p1_freeze_audit.md`

---

## P2 — Compare workflow v2 + workbench refactor

**Prerequisite:** P1 frozen

| Item | Location | Status |
|------|----------|--------|
| Compare mode selector | `ExperimentCompareStagePanel` | Done |
| `multi_manifest_diff` metadata table | `multiManifestDiff.ts`, `MultiManifestDiffTable.tsx` | Done |
| Review packet export | `reviewPacketExport.ts` | Done |
| Workbench v2 hook | `useExperimentWorkbenchV2.ts` | Done |
| `tier0-rt-ui` + build | CI | Done |

**PLAT plan:** [rt_plat_x2_p2_multi_manifest_export_plan.md](../platform/rt_plat_x2_p2_multi_manifest_export_plan.md)

**Freeze target:** [rt_plat_x2_p2_freeze_audit.md](rt_plat_x2_p2_freeze_audit.md) — **PLAT-RT-X2 complete**

---

## Regression matrix (all PLAT phases)

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

---

## Stop line

Do not start PLAT-RT-X2 without:

1. PLAN-RT-X2 frozen  
2. Per-phase PLAT plan in `docs/platform/`  
3. Governance review + freeze audit per phase  
4. Contamination review at P1 when F6/F7 strips adjacent to unified review  
