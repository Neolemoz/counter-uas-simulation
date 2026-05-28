# RT — PLAT-RT-X3 Implementation Roadmap v1

**Phase:** PLAN-RT-X3 frozen → **PLAT-RT-X3** P0–P2 advisory  
**Prerequisite:** [rt_x3_freeze_audit.md](rt_x3_freeze_audit.md) (PLAN-RT-X3 docs frozen); PLAT-RT-X2 and PLAT-RT-C4 complete  
**Contracts:** [rt_experiment_workbench_v3_v1.md](rt_experiment_workbench_v3_v1.md), [rt_experiment_review_workflow_v3_v1.md](rt_experiment_review_workflow_v3_v1.md), [rt_experiment_compare_workflow_v3_v1.md](rt_experiment_compare_workflow_v3_v1.md)

**Not authorized by PLAN-RT-X3.** Per-phase PLAT plan + governance + freeze required.

---

## P0 — Cohort navigation ergonomics

**Prerequisite:** PLAN-RT-X3 frozen

| Item | Location (proposed) |
|------|---------------------|
| Program context strip | `ExperimentCohortNavigator.tsx` or `ExperimentProgramContextStrip.tsx` |
| Manifest roster table | `ExperimentManifestRoster.tsx` |
| Breadcrumb + tag filter | `workbenchV2State.ts` additive keys |
| Explicit secondary picker | `ExperimentCohortNavigator.tsx` |
| Fixture parity | `x3_review_packet_sections_example.json` (reference only) |
| Governance: no SA path refs | Vitest |
| `tier0-rt-ui` | CI |

**PLAT plan:** `docs/platform/rt_plat_x3_p0_cohort_navigation_plan.md` (create at P0 start)

**Freeze target:** `docs/evaluation/rt_plat_x3_p0_freeze_audit.md`

---

## P1 — Review lane + grouped dock + packet sections

**Prerequisite:** P0 frozen

| Item | Location (proposed) |
|------|---------------------|
| Step completion badges | `ExperimentUnifiedReviewPanel.tsx` or lane subcomponent |
| Grouped report dock | `ExperimentReportDockPanel.tsx` |
| Optional `sections[]` on packet export | `reviewPacketSchema.ts`, `reviewPacketExport.ts` |
| Contamination review (F6/F7 adjacency) | Docs |

**Freeze target:** `docs/evaluation/rt_plat_x3_p1_freeze_audit.md`

---

## P2 — Compare coach + multi-manifest readability

**Prerequisite:** P1 frozen

| Item | Location (proposed) |
|------|---------------------|
| Mode coach strip | `ExperimentCompareStagePanel.tsx` |
| `formatCompareStatus` helper | `experiment/compareStatusVocabulary.ts` |
| Multi-manifest column order + drill-down | `MultiManifestDiffTable.tsx` |
| `tier0-rt-ui` + build | CI |

**Freeze target:** `docs/evaluation/rt_plat_x3_p2_freeze_audit.md` — **PLAT-RT-X3 complete**

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

Do not start PLAT-RT-X3 without:

1. PLAN-RT-X3 frozen  
2. Per-phase PLAT plan in `docs/platform/`  
3. Governance review + freeze audit per phase  
4. Contamination review at P1 when F6/F7 strips adjacent to unified review  
