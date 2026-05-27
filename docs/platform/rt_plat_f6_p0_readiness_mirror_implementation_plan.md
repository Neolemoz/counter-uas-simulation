# RT-F6 P0 — Readiness Mirror (PLAT-RT-F6 P0)

**Phase:** PLAT-RT-F6 P0 — advisory readiness mirror  
**Prerequisite:** PLAN-RT-F6 frozen — [rt_f6_freeze_audit.md](../evaluation/rt_f6_freeze_audit.md)  
**Authority:** [rt_sa_workflow_automation_v1.md](../evaluation/rt_sa_workflow_automation_v1.md)

## Goal

Implement read-only advisory readiness mirror: derive layer (Python + TypeScript), maintainer CLI, golden tests, and minimal workbench/handoff UI surfaces — without bridge protocol changes, auto-import, or SA viewer changes.

## Delivered (P0)

| Item | Location |
|------|----------|
| Advisory derive (Python) | `platform/rt-sandbox-bridge/rt_sandbox/advisory_derive.py` |
| Advisory derive (TypeScript) | `platform/rt-sandbox-ui/src/handoff/deriveAdvisoryState.ts` |
| Mirror row input mapper | `platform/rt-sandbox-ui/src/handoff/advisoryInputFromMirrorRow.ts` |
| Labels / tones | `platform/rt-sandbox-ui/src/handoff/advisoryLabels.ts` |
| Types | `platform/rt-sandbox-ui/src/handoff/advisoryTypes.ts` |
| Maintainer CLI | `scripts/rt/rt_handoff_advisory_status.py` |
| UI strip | `platform/rt-sandbox-ui/src/handoff/HandoffAdvisoryMirrorStrip.tsx` |
| Handoff panel wiring | `CaptureHandoffWorkflowPanel.tsx` |
| Workbench wiring | `ExperimentWorkbenchPanel.tsx`, `App.tsx` |
| Golden fixtures | `fixtures/rt_handoff/f6_advisory_examples/expected/` |
| Tests | `deriveAdvisoryState.test.ts`, `test_rt_handoff_advisory.py` |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- SA viewer changes  
- Auto-import / browser commit  
- Checklist UI panel (`SaWorkflowAdvisoryPanel`) — P1  
- Batch maintainer helpers — P2  
- Tactical redesign  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_handoff_advisory.py -q
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -k handoff -q
cd platform/rt-sandbox-ui && npm run test && npm run build
python3 scripts/rt/rt_handoff_advisory_status.py status <capture_id> --json
```

## Stop line

PLAT-RT-F6 P0 frozen. Do not start **P1 advisory checklist UI** without `rt_plat_f6_p1_*` implementation plan + governance review + freeze audit.

## Related

- [rt_roadmap_plat_rt_f6_v1.md](../evaluation/rt_roadmap_plat_rt_f6_v1.md)
- [rt_plat_f6_p0_freeze_audit.md](../evaluation/rt_plat_f6_p0_freeze_audit.md)
- [rt_plat_f6_p0_governance_review_r1.md](../evaluation/rt_plat_f6_p0_governance_review_r1.md)
