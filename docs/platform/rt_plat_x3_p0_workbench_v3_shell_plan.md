# RT-X3 P0 — Workbench v3 Shell (PLAT-RT-X3 P0)

**Phase:** PLAT-RT-X3 P0 — behavior-neutral experiment workbench v3 shell ergonomics  
**Prerequisite:** PLAN-RT-X3 frozen — [rt_x3_freeze_audit.md](../evaluation/rt_x3_freeze_audit.md); PLAT-RT-X2 and PLAT-RT-C4 complete  
**Authority:** [rt_x3_experiment_workbench_v3_plan.md](rt_x3_experiment_workbench_v3_plan.md), [rt_experiment_workbench_v3_v1.md](../evaluation/rt_experiment_workbench_v3_v1.md)

## Goal

Add workbench v3 navigation and readability affordances on frozen X2 zones: program context strip, manifest roster, explicit secondary picker, packet section preview helpers, compare mode coach, and shared compare-status vocabulary — without bridge/runtime changes, SA contamination, or import/export semantic changes.

## Delivered (P0)

| Item | Location |
|------|----------|
| V3 state keys | `workbenchV2State.ts` (`cohort_tag_filter`, `breadcrumb_focus`, `review_session_id`) |
| V3 banner | `governance/banners.ts` (`BANNER_EXPERIMENT_V3`) |
| Program context strip | `ExperimentProgramContextStrip.tsx` |
| Manifest roster | `ExperimentManifestRoster.tsx` |
| Secondary picker | `ExperimentSecondaryManifestPicker.tsx` |
| V3 shell wrapper | `ExperimentWorkbenchV3Shell.tsx` |
| Slim cohort navigator | `ExperimentCohortNavigator.tsx` |
| Packet sections (UI) | `reviewPacketSections.ts`; optional `sections[]` on `reviewPacketSchema` |
| Compare vocabulary | `compareStatusVocabulary.ts` |
| Compare mode coach | `compareModeCoach.ts`, `ExperimentCompareModeCoach.tsx` |
| Multi-manifest status column + drill-down prep | `multiManifestDiff.ts`, `MultiManifestDiffTable.tsx` |
| Integration | `ExperimentWorkbenchV2Shell.tsx`, `ExperimentCompareStagePanel.tsx`, `ExperimentReportDockPanel.tsx` |

## Preserved

- `buildReviewPacketPreview` / download/copy export JSON **without** `sections[]`
- X2 review lane step ids, compare mode ids, derive modules, import guards
- C4 `ExperimentManifestToolbar`, compare/F5 sections in parent panel
- No `App.tsx`, bridge, or SA viewer changes

## Forbidden (unchanged)

- Bridge API / runtime / subcommand changes
- SA viewer layout or import automation
- Browser capture/import authority
- Cross-manifest `run_id` pairing
- P1 step completion badges, grouped dock collapse, export `sections[]`

## Validation

```bash
cd platform/rt-sandbox-ui && npm test && npm run build
cd ../.. && scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
git diff --check
git diff --name-only | grep -E 'rt-sandbox-bridge|sa-r0-viewer|src/counter_uas' || true
```

## Stop line

PLAT-RT-X3 P0 frozen. Do not start **PLAT-RT-X3 P1** (review lane step badges, grouped dock, packet export sections) without scoped plan + governance + contamination review + freeze audit.

## Related

- [rt_plat_x3_p0_freeze_audit.md](../evaluation/rt_plat_x3_p0_freeze_audit.md)
- [rt_roadmap_plat_rt_x3_v1.md](../evaluation/rt_roadmap_plat_rt_x3_v1.md)
