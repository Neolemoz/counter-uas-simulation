# RT-C4 P2 — App Cleanup (PLAT-RT-C4 P2)

**Phase:** PLAT-RT-C4 P2 — behavior-neutral `App.tsx` concentration relief  
**Prerequisite:** PLAT-RT-C4 P1 frozen — [rt_plat_c4_p1_freeze_audit.md](../evaluation/rt_plat_c4_p1_freeze_audit.md)  
**Authority:** [rt_c4_checkpoint_cleanup_plan.md](rt_c4_checkpoint_cleanup_plan.md)

## Goal

Reduce `App.tsx` concentration by extracting per-session entity editing and workstation shell slot assembly without changing runtime behavior, bridge authority, or SA boundaries.

## Delivered (P2)

| Item | Location |
|------|----------|
| Entity editing hook | `platform/rt-sandbox-ui/src/hooks/useSessionEntityEditing.ts` |
| Workstation slots | `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx` |
| Thin orchestrator | `platform/rt-sandbox-ui/src/App.tsx` (~280 LOC) |

## Preserved

- `useRtSessionWorkspace` session/telemetry ownership in `App`
- Layer visibility memory (`rt_session_layer_visibility_v1`) in `App`
- Experiment chrome state + `experimentRollup` handoff callback in `App`
- `useTacticalState` + `handleSelectEntity` tactical coupling in `App`
- M3 tab-switch dirty confirm (`hasUnsyncedLocalMirror`) unchanged
- `RuntimeWorkstationShell` layout-only boundary unchanged
- All `data-testid` values in child components unchanged

## Forbidden (unchanged)

- Bridge, SA viewer, experiment derive / `useExperimentWorkbenchV2` internals
- PLAN-RT-X3 implementation

## Validation

```bash
cd platform/rt-sandbox-ui && npm test && npm run build
cd ../.. && scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
git diff --check
```

## Stop line

**PLAT-RT-C4 complete** (P0–P2 frozen). Do not start **PLAN-RT-X3** implementation without scoped plan + governance + freeze audit.

## Related

- [rt_plat_c4_p2_freeze_audit.md](../evaluation/rt_plat_c4_p2_freeze_audit.md)
- [rt_plat_c4_p2_architecture_review_r1.md](../evaluation/rt_plat_c4_p2_architecture_review_r1.md)
- [rt_plat_c4_p2_governance_review_r1.md](../evaluation/rt_plat_c4_p2_governance_review_r1.md)
