# RT-C4 P1 — Experiment Section Cleanup (PLAT-RT-C4 P1)

**Phase:** PLAT-RT-C4 P1 — behavior-neutral compare + F5 section extraction  
**Prerequisite:** PLAT-RT-C4 P0 frozen — [rt_plat_c4_p0_freeze_audit.md](../evaluation/rt_plat_c4_p0_freeze_audit.md)  
**Authority:** [rt_c4_checkpoint_cleanup_plan.md](rt_c4_checkpoint_cleanup_plan.md)

## Goal

Continue reducing `ExperimentWorkbenchPanel` concentration by extracting compare and F5 presentation blocks without changing runtime behavior, bridge authority, or SA boundaries.

## Delivered (P1)

| Item | Location |
|------|----------|
| Compare section | `platform/rt-sandbox-ui/src/experiment/ExperimentCompareSection.tsx` |
| F5 metrics section | `platform/rt-sandbox-ui/src/experiment/ExperimentF5MetricsSection.tsx` |
| Parent refactor | `platform/rt-sandbox-ui/src/experiment/ExperimentWorkbenchPanel.tsx` (~671 LOC) |

## Preserved

- `useExperimentWorkbenchV2` orchestration unchanged
- Advisory rollup `useEffect` in parent
- Compare state/effects (`normalizeCompareKey`, `resolveSide`, `compareOptions`) in parent
- F5 derive state, import handlers, CLI hints in parent
- Child `data-testid` values unchanged

## Forbidden (unchanged)

- `App.tsx`, bridge, SA viewer, `useExperimentWorkbenchV2` internals
- PLAT-RT-C4 P2 / PLAN-RT-X3

## Validation

```bash
cd platform/rt-sandbox-ui && npm test && npm run build
cd ../.. && scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
git diff --check
```

## Stop line

PLAT-RT-C4 P1 frozen. Do not start **PLAT-RT-C4 P2** without scoped plan + governance + freeze audit.

## Related

- [rt_plat_c4_p1_freeze_audit.md](../evaluation/rt_plat_c4_p1_freeze_audit.md)
- [rt_plat_c4_p1_architecture_review_r1.md](../evaluation/rt_plat_c4_p1_architecture_review_r1.md)
- [rt_plat_c4_p1_governance_review_r1.md](../evaluation/rt_plat_c4_p1_governance_review_r1.md)
