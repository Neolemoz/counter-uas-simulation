# RT-F6 P1 — Advisory Checklist UI (PLAT-RT-F6 P1)

**Phase:** PLAT-RT-F6 P1 — advisory checklist UI  
**Prerequisite:** PLAT-RT-F6 P0 frozen — [rt_plat_f6_p0_freeze_audit.md](../evaluation/rt_plat_f6_p0_freeze_audit.md)  
**Authority:** [rt_sa_workflow_advisory_ui_v1.md](../evaluation/rt_sa_workflow_advisory_ui_v1.md), [rt_sa_workflow_automation_v1.md](../evaluation/rt_sa_workflow_automation_v1.md)

## Goal

Wire read-only advisory checklist cognition on frozen P0 derive — `SaWorkflowAdvisoryPanel`, import advisory strip, workbench per-run badges, `BANNER_SA_WORKFLOW_ADVISORY` — without bridge protocol, SA viewer, or write-path changes.

## Delivered (P1)

| Item | Location |
|------|----------|
| `advisoryChecklist.ts` | `platform/rt-sandbox-ui/src/handoff/` |
| `SaWorkflowAdvisoryPanel.tsx` | `platform/rt-sandbox-ui/src/handoff/` |
| `AdvisoryStateBadge.tsx` | `platform/rt-sandbox-ui/src/handoff/` |
| `AdvisoryRunBadge.tsx` | `platform/rt-sandbox-ui/src/handoff/` |
| `ExperimentImportAdvisoryStrip.tsx` | `platform/rt-sandbox-ui/src/experiment/` |
| `BANNER_SA_WORKFLOW_ADVISORY` | `src/governance/banners.ts` |
| Handoff panel wiring | `CaptureHandoffWorkflowPanel.tsx` |
| Workbench wiring | `ExperimentWorkbenchPanel.tsx` |
| Checklist derive parity | `advisory_derive.py` |
| Vitest / pytest | panel tests + golden fixtures |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- `platform/sa-r0-viewer/` changes  
- Approve / import / capture buttons in browser  
- Auto-import; batch `--commit-all`  
- Parser/topic/schema changes  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_handoff_advisory.py -q
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -k handoff -q
cd platform/rt-sandbox-ui && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F6 P1 frozen. Do not start **P2** maintainer batch helpers without contamination re-check + `rt_plat_f6_p2_*` governance review.

## Related

- [rt_plat_f6_p1_freeze_audit.md](../evaluation/rt_plat_f6_p1_freeze_audit.md)
- [rt_plat_f6_p1_governance_review_r1.md](../evaluation/rt_plat_f6_p1_governance_review_r1.md)
- [rt_roadmap_plat_rt_f6_v1.md](../evaluation/rt_roadmap_plat_rt_f6_v1.md)
