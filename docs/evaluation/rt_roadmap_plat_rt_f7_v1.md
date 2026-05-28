# RT — PLAT-RT-F7 Implementation Roadmap v1

**Phase:** PLAN-RT-F7 frozen → **PLAT-RT-F7** advisory backlog (not authorized until PLAT freeze)  
**Prerequisite:** [rt_f7_freeze_audit.md](rt_f7_freeze_audit.md) (PLAN-RT-F7 docs frozen)  
**Contracts:** [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md), [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md), [rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md)

---

## P0 — Queue, groups, aggregation schema

**Prerequisite:** PLAN-RT-F7 frozen; [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md) closed for P0

| Item | Location | Status |
|------|----------|--------|
| `queue_priority` derive | `platform/rt-sandbox-bridge/rt_sandbox/advisory_queue.py` | Done (P0) |
| `blocker_groups` + `readiness_cohort` | `advisory_queue.py`, `batch_advisory.py` | Done (P0) |
| `rt_advisory_batch_summary_v1` emitter | `batch_advisory.py`, `rt_handoff_batch_advisory.py` | Done (P0) |
| `report --sort queue` / `--group-by` | `scripts/rt/rt_handoff_batch_advisory.py` | Done (P0) |
| Lineage warn detect (LIN-*) | `advisory_derive.py` (additive field) | Done (P0) |
| Golden fixtures | `fixtures/rt_handoff/f7_advisory_examples/expected/` | Done (P0) |
| Pytest / vitest | `test_advisory_queue.py`, `advisoryQueue.test.ts` | Done (P0) |
| UI mirror chips | `CaptureHandoffWorkflowPanel`, F7 chip components | Done (P0) |

**Freeze:** [rt_plat_f7_p0_freeze_audit.md](rt_plat_f7_p0_freeze_audit.md)

---

## P1 — Triage queue UI (read-only)

**Prerequisite:** P0 frozen

| Item | Location | Status |
|------|----------|--------|
| `AdvisoryTriageQueuePanel` | `platform/rt-sandbox-ui/src/handoff/` | Done (P1) |
| `advisoryTriageGrouping.ts` | `platform/rt-sandbox-ui/src/handoff/` | Done (P1) |
| Grouped blocker strip | `AdvisoryGroupedBlockerStrip.tsx` | Done (P1) |
| Experiment rollup → handoff | `App.tsx`, `ExperimentWorkbenchPanel.tsx` | Done (P1) |
| `BANNER_SA_WORKFLOW_ADVISORY` reuse | triage panel | Done (P1) |
| `tier0-rt-ui` | CI | Done (P1) |

No action buttons; no browser commit or pipeline.

**Freeze:** [rt_plat_f7_p1_freeze_audit.md](rt_plat_f7_p1_freeze_audit.md)

---

## P2 — Bulk workflow hardening

**Prerequisite:** P0 + P1 frozen; P2 contamination re-audit (F6 pattern)  
**Debt:** [rt_c1_technical_debt_audit_r1.md](rt_c1_technical_debt_audit_r1.md) §6 batch helper hardening

| Item | Location | Status |
|------|----------|--------|
| `rt_advisory_batch_review_v2` | `batch_advisory.py` | Done (P2) |
| `standup-export` / `grouped-export` | `rt_handoff_batch_advisory.py` | Done (P2) |
| `dry-run-review` | `rt_handoff_batch_advisory.py` | Done (P2) |
| Stricter dry-run guards | `rt_sa_import_dry_run.py` | Done (P2) |
| Client JSON preview | `advisoryBatchExportPreview.ts` | Done (P2) |

**Freeze:** [rt_plat_f7_p2_freeze_audit.md](rt_plat_f7_p2_freeze_audit.md) — **PLAT-RT-F7 complete**

---

## Explicit out of scope (PLAT-RT-F7)

- Bridge protocol / telemetry channel changes (except additive read-only rollup fields — audited)
- Automatic import; browser-triggered pipeline
- `platform/sa-r0-viewer/` changes
- Federation / orchestration authority from RT
- Distributed multi-bridge
- Parser/topic/schema changes
- `--commit-all` or batch implicit corpus writes
- Operational readiness scoring (`readiness_score`)
- Re-specifying F6 five-rung ladder semantics

---

## Validation (PLAT wave)

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
cd platform/rt-sandbox-ui && npm run test
scripts/ci_eval.sh tier0-rt-ui
```

P0 additionally requires golden fixture parity for F7 expected JSON.

---

## Stop line

PLAN-RT-F7 frozen. **PLAT-RT-F7** is **not** authorized until scoped implementation plan + governance + contamination review + freeze per phase.
