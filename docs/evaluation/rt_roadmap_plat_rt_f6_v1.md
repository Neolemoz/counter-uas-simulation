# RT — PLAT-RT-F6 Implementation Roadmap v1

**Phase:** PLAN-RT-F6 frozen → **PLAT-RT-F6** advisory backlog  
**Prerequisite:** [rt_f6_freeze_audit.md](rt_f6_freeze_audit.md) (PLAN-RT-F6 docs frozen)  
**Contracts:** [rt_sa_workflow_automation_v1.md](rt_sa_workflow_automation_v1.md), [rt_sa_workflow_advisory_ui_v1.md](rt_sa_workflow_advisory_ui_v1.md)

---

## P0 — Readiness mirror

**Freeze:** [rt_plat_f6_p0_freeze_audit.md](rt_plat_f6_p0_freeze_audit.md)

| Item | Location | Status |
|------|----------|--------|
| `advisory_derive.py` | `platform/rt-sandbox-bridge/rt_sandbox/` | Done (P0) |
| `deriveAdvisoryState.ts` | `platform/rt-sandbox-ui/src/handoff/` | Done (P0) |
| `rt_handoff_advisory_status.py` | `scripts/rt/` | Done (P0) |
| Golden expected outputs | `fixtures/rt_handoff/f6_advisory_examples/expected/` | Done (P0) |
| `HandoffAdvisoryMirrorStrip` + panel wiring | `CaptureHandoffWorkflowPanel`, `ExperimentWorkbenchPanel` | Done (P0) |

---

## P1 — Advisory checklist UI

**Freeze:** [rt_plat_f6_p1_freeze_audit.md](rt_plat_f6_p1_freeze_audit.md)

| Item | Location | Status |
|------|----------|--------|
| `advisoryChecklist.ts` | `platform/rt-sandbox-ui/src/handoff/` | Done (P1) |
| `SaWorkflowAdvisoryPanel` | `platform/rt-sandbox-ui/src/handoff/` | Done (P1) |
| Checklist chips (manual workflow §2) | `SaWorkflowAdvisoryPanel` | Done (P1) |
| Workbench per-run advisory badge | `AdvisoryRunBadge.tsx` | Done (P1) |
| Staging mirror advisory column | `CaptureHandoffWorkflowPanel.tsx` | Done (P1) |
| `ExperimentImportAdvisoryStrip` | `platform/rt-sandbox-ui/src/experiment/` | Done (P1) |
| `BANNER_SA_WORKFLOW_ADVISORY` | `governance/banners.ts` | Done (P1) |
| `tier0-rt-ui` gate | CI | Done (P1) |

---

## P2 — Optional maintainer automation helpers

**Freeze:** [rt_plat_f6_p2_freeze_audit.md](rt_plat_f6_p2_freeze_audit.md)  
**Prerequisite:** P0 + P1 frozen; [rt_f6_handoff_contamination_review_p2_r1.md](rt_f6_handoff_contamination_review_p2_r1.md)

| Item | Location | Status |
|------|----------|--------|
| `batch_advisory.py` | `platform/rt-sandbox-bridge/rt_sandbox/` | Done (P2) |
| `rt_handoff_batch_advisory.py` | `scripts/rt/` | Done (P2) |
| `rt_sa_import_dry_run.py` | `scripts/rt/` | Done (P2) |
| `corpus-preview` subcommand | `rt_handoff_batch_advisory.py` | Done (P2) |
| `annotate-review` (gated write) | `rt_handoff_batch_advisory.py` | Done (P2) |

---

## Explicit out of scope (PLAT-RT-F6)

- Bridge protocol changes (except optional additive read-only `advisory_state` field — audited separately)  
- Automatic import; browser-triggered pipeline  
- `platform/sa-r0-viewer/` changes  
- Federation / orchestration authority from RT sessions  
- Tactical controller redesign  
- Distributed multi-bridge (M3)  
- Parser/topic/schema changes  
- Batch `--commit-all` or implicit corpus writes  
- Operational readiness scoring  

---

## Validation (PLAT wave)

```bash
lint_rt_runtime_subcommands
pytest platform/rt-sandbox-bridge/tests/ src/counter_uas/test/test_rt_sandbox_bridge.py
cd platform/rt-sandbox-ui && npm run test
python3 scripts/rt/rt_handoff_advisory_status.py status cap-example-normalized --json
```

P1 additionally requires `tier0-rt-ui`.

---

## Next frontier (advisory)

After PLAT-RT-F6 P0–P2: re-evaluate [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md) — F7/M3 distributed runtime remains deferred.

---

## Stop line

PLAN-RT-F6 frozen. **PLAT-RT-F6 complete** (P0–P2 frozen). Re-evaluate [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md); M3/F7 deferred.
