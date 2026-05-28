# RT-C4 P0 — Experiment Workbench Cleanup (PLAT-RT-C4 P0)

**Phase:** PLAT-RT-C4 P0 — behavior-neutral experiment import helper + manifest toolbar extraction  
**Prerequisite:** PLAN-RT-C4 frozen — [rt_c4_freeze_audit.md](../evaluation/rt_c4_freeze_audit.md)  
**Authority:** [rt_c4_checkpoint_cleanup_plan.md](rt_c4_checkpoint_cleanup_plan.md)

## Goal

Reduce concentration in `ExperimentWorkbenchPanel.tsx` by extracting shared JSON prompt import logic and the manifest toolbar block — without changing runtime behavior, bridge authority, import validation, or SA boundaries.

## Delivered (P0)

| Item | Location |
|------|----------|
| JSON prompt import helper | `platform/rt-sandbox-ui/src/experiment/useJsonPromptImport.ts` |
| Manifest toolbar | `platform/rt-sandbox-ui/src/experiment/ExperimentManifestToolbar.tsx` |
| Parent refactor | `platform/rt-sandbox-ui/src/experiment/ExperimentWorkbenchPanel.tsx` |
| Tests | `useJsonPromptImport.test.ts`, `isolation.test.ts` (module presence) |

## Preserved

- `useExperimentWorkbenchV2` orchestration and `ExperimentWorkbenchV2Shell`
- Advisory rollup `useEffect` in parent
- Compare block (A/B selectors + `ExperimentComparePanel`)
- F5 metrics section (import/export buttons remain in parent F5 block)
- All `safeParse*` functions in `experimentImportGuards.ts` unchanged
- Exact prompt strings and `Invalid <label>:` alert format
- `data-testid="experiment-workbench"` on parent root

## Forbidden (unchanged)

- `App.tsx` edits  
- `platform/rt-sandbox-bridge/` changes  
- `platform/sa-r0-viewer/` changes  
- `ExperimentCompareSection` / `ExperimentF5MetricsSection` (P1)  
- Advisory rollup relocation  
- Import guard semantic changes  
- Other panels migrated to `useJsonPromptImport` (deferred)

## Validation

```bash
cd platform/rt-sandbox-ui && npm test && npm run build
cd ../.. && scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
git diff --check
git diff --name-only | grep -E 'App\.tsx|rt-sandbox-bridge|sa-r0-viewer' || true
```

## Stop line

PLAT-RT-C4 P0 frozen. Do not start **PLAT-RT-C4 P1** (`ExperimentCompareSection`, `ExperimentF5MetricsSection`) or **P2** (`App.tsx` hooks) without scoped plan + governance + freeze audit.

## Related

- [rt_plat_c4_p0_freeze_audit.md](../evaluation/rt_plat_c4_p0_freeze_audit.md)
- [rt_plat_c4_p0_architecture_review_r1.md](../evaluation/rt_plat_c4_p0_architecture_review_r1.md)
- [rt_plat_c4_p0_governance_review_r1.md](../evaluation/rt_plat_c4_p0_governance_review_r1.md)
