# RT-X3 P1 — Review Workflow Ergonomics (PLAT-RT-X3 P1)

**Phase:** PLAT-RT-X3 P1 — review lane completion badges, grouped report dock, packet section preview polish, compare status chips  
**Prerequisite:** PLAT-RT-X3 P0 frozen — [rt_plat_x3_p0_freeze_audit.md](../evaluation/rt_plat_x3_p0_freeze_audit.md)  
**Authority:** [rt_experiment_review_workflow_v3_v1.md](../evaluation/rt_experiment_review_workflow_v3_v1.md)

## Goal

Extend frozen P0 v3 shell with review workflow ergonomics: per-step completion badges, collapsible dock groups (Analytics / Continuity / Metrics / Fidelity), packet section cards with completion hints, and shared compare-status chips on compare panels — without changing export/copy JSON shape, bridge/runtime, or SA boundaries.

## Delivered (P1)

| Item | Location |
|------|----------|
| Step completion resolver | `reviewStepCompletion.ts`, `ReviewStepCompletionBadge.tsx` |
| Unified review lane badges | `ExperimentUnifiedReviewPanel.tsx` |
| Grouped report dock | `reportDockGroups.ts`, `ExperimentReportDockPanel.tsx` |
| Packet section cards | `ReviewPacketSectionCard.tsx`, `reviewPacketSections.ts` (completion_hint) |
| Compare status chips | `compareBadgeStatus.ts`, `CompareStatusChip.tsx` |
| Compare panel wiring | `ExperimentComparePanel.tsx`, `ExperimentExtendedComparePanel.tsx`, `ExperimentFidelityCompareStrip.tsx`, `ExperimentCompareSection.tsx` |

## Preserved

- `buildReviewPacketPreview` / `exportReviewPacketJson` — **no** `sections[]` in export
- P0 navigation shell, compare coach, multi-manifest status column
- No derive / bridge / SA / `App.tsx` changes

## Forbidden

- Export `sections[]` on download/copy (deferred; clipboard unchanged)
- F7 queue / contamination gate changes
- Cross-manifest run pairing

## Validation

```bash
cd platform/rt-sandbox-ui && npm test && npm run build
cd ../.. && scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
git diff --check
```

## Stop line

PLAT-RT-X3 P1 frozen. Do not start **PLAT-RT-X3 P2** without scoped plan + freeze audit.

## Related

- [rt_plat_x3_p1_freeze_audit.md](../evaluation/rt_plat_x3_p1_freeze_audit.md)
- [rt_roadmap_plat_rt_x3_v1.md](../evaluation/rt_roadmap_plat_rt_x3_v1.md)
