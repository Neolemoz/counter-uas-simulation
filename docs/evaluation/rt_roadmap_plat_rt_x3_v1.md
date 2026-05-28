# RT — PLAT-RT-X3 Implementation Roadmap v1

**Phase:** PLAN-RT-X3 frozen → **PLAT-RT-X3** P0–P2 advisory  
**Prerequisite:** [rt_x3_freeze_audit.md](rt_x3_freeze_audit.md) (PLAN-RT-X3 docs frozen); PLAT-RT-X2 and PLAT-RT-C4 complete  
**Contracts:** [rt_experiment_workbench_v3_v1.md](rt_experiment_workbench_v3_v1.md), [rt_experiment_review_workflow_v3_v1.md](rt_experiment_review_workflow_v3_v1.md), [rt_experiment_compare_workflow_v3_v1.md](rt_experiment_compare_workflow_v3_v1.md)

**Not authorized by PLAN-RT-X3.** Per-phase PLAT plan + governance + freeze required.

---

## P0 — Workbench v3 shell (frozen)

**Prerequisite:** PLAN-RT-X3 frozen
**Status:** **frozen** — [rt_plat_x3_p0_freeze_audit.md](rt_plat_x3_p0_freeze_audit.md)

| Item | Location |
|------|----------|
| Program context strip | `ExperimentProgramContextStrip.tsx` |
| Manifest roster table | `ExperimentManifestRoster.tsx` |
| Breadcrumb + tag filter | `workbenchV2State.ts` additive keys |
| Explicit secondary picker | `ExperimentSecondaryManifestPicker.tsx` |
| V3 shell wrapper | `ExperimentWorkbenchV3Shell.tsx` |
| Packet sections (UI preview) | `reviewPacketSections.ts` |
| Compare coach + status vocabulary | `compareModeCoach.ts`, `compareStatusVocabulary.ts` |
| Fixture parity | `x3_review_packet_sections_example.json` |

**PLAT plan:** [rt_plat_x3_p0_workbench_v3_shell_plan.md](../platform/rt_plat_x3_p0_workbench_v3_shell_plan.md)

---

## P1 — Review lane + grouped dock + packet sections (frozen)

**Prerequisite:** P0 frozen
**Status:** **frozen** — [rt_plat_x3_p1_freeze_audit.md](rt_plat_x3_p1_freeze_audit.md)

| Item | Location |
|------|----------|
| Step completion badges | `reviewStepCompletion.ts`, `ExperimentUnifiedReviewPanel.tsx` |
| Grouped report dock | `reportDockGroups.ts`, `ExperimentReportDockPanel.tsx` |
| Packet section cards (preview only) | `ReviewPacketSectionCard.tsx`, `reviewPacketSections.ts` |
| Compare status chips | `CompareStatusChip.tsx`, compare panels |
| Contamination review (F6/F7) | [rt_plat_x3_p1_governance_review_r1.md](rt_plat_x3_p1_governance_review_r1.md) |

**Note:** Export/copy JSON intentionally omits `sections[]` (UI preview only).

**PLAT plan:** [rt_plat_x3_p1_review_workflow_plan.md](../platform/rt_plat_x3_p1_review_workflow_plan.md)

---

## P2 — Compare readability polish (advisory)

**Prerequisite:** P1 frozen

**Note:** P0 already delivered compare mode coach, `compareStatusVocabulary`, and multi-manifest status column + drill-down. P2 is residual polish only if needed.

| Item | Location (proposed) |
|------|---------------------|
| Extended compare / matrix readability | compare panels |
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
