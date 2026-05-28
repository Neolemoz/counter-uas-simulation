# RT — PLAT-RT-F8 Implementation Roadmap v1

**Phase:** PLAN-RT-F8 frozen → **PLAT-RT-F8** complete (P0–P2 frozen)  
**Prerequisite:** [rt_f8_freeze_audit.md](rt_f8_freeze_audit.md) (PLAN-RT-F8 docs frozen)  
**Contracts:** [rt_advisory_maintainer_workflow_v2.md](rt_advisory_maintainer_workflow_v2.md), [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md), [rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md)

---

## P0 — Summary v2, presets, golden fixtures

**Prerequisite:** PLAN-RT-F8 frozen; [rt_f8_handoff_contamination_review_r1.md](rt_f8_handoff_contamination_review_r1.md) closed for P0

| Item | Location | Status |
|------|----------|--------|
| `build_advisory_batch_summary_v2_document` | `platform/rt-sandbox-bridge/rt_sandbox/batch_advisory.py` | **Delivered** |
| Filter preset catalog | `advisory_queue.py` | **Delivered** |
| `report/export --preset`, `--focus-captures`, `--schema f8`, `--template-pack` | `scripts/rt/rt_handoff_batch_advisory.py` | **Delivered** |
| `readiness_cohorts_v2`, `multi_capture_cohorts`, `handoff_rollup` | `advisory_queue.py` | **Delivered** |
| TS mirror + read-only UI strips | `advisoryAggregationV2.ts`, `CaptureHandoffWorkflowPanel.tsx` | **Delivered** |
| Golden fixtures | `fixtures/rt_handoff/f8_advisory_examples/` | Docs ready |
| Pytest + vitest | `test_advisory_queue.py`, `test_rt_handoff_batch_advisory.py`, F8 vitest | **Delivered** |

**Frozen:** [rt_plat_f8_p0_freeze_audit.md](rt_plat_f8_p0_freeze_audit.md)

---

## P1 — Triage preset UI, cohort v2 chips, experiment-handoff strip

**Prerequisite:** P0 frozen

| Item | Location | Status |
|------|----------|--------|
| Integrated triage hub | `AdvisoryTriageQueuePanel.tsx` | **Delivered** |
| Preset + focus + pass + template in panel | `AdvisoryStandupPassSelector`, toolbar | **Delivered** |
| `readiness_cohort_v2` on triage rows | `enrichRowsForTriage`, `ReadinessCohortV2Chip` | **Delivered** |
| `cohort_v2` / `handoff_stage` grouping | `advisoryTriageGrouping.ts` | **Delivered** |
| Rollup bar (session scope) | `AdvisoryRollupSummaryBar.tsx` | **Delivered** |
| Vitest + `tier0-rt-ui` | CI | **Delivered** |

No action buttons; no browser commit or pipeline.

**Frozen:** [rt_plat_f8_p1_freeze_audit.md](rt_plat_f8_p1_freeze_audit.md)

---

## P2 — Template packs, corpus-preview refinements, dry-run guards

**Prerequisite:** P0 + P1 frozen; P2 contamination re-audit

| Item | Location | Status |
|------|----------|--------|
| Stand-up template packs (`standup_md_daily`, etc.) | `batch_advisory.py` | **Delivered** |
| `--template-pack` CLI | `rt_handoff_batch_advisory.py` | **Delivered** |
| Corpus-preview refinement (read-only) | `batch_advisory.py`, `rt_handoff_batch_advisory.py` | **Delivered** |
| Stricter v2 dry-run guards | `batch_advisory.py`, `rt_handoff_batch_advisory.py` | **Delivered** |
| Client preview hook (optional) | `advisoryBatchExportPreview.ts` | **Delivered** |

**Frozen:** [rt_plat_f8_p2_freeze_audit.md](rt_plat_f8_p2_freeze_audit.md) — **PLAT-RT-F8 complete**

---

## Explicit out of scope (PLAT-RT-F8)

- Bridge protocol / telemetry channel changes (except additive read-only rollup fields — audited per phase)
- Automatic import; browser-triggered pipeline
- `platform/sa-r0-viewer/` changes
- Federation / orchestration authority from RT
- Distributed multi-bridge
- Parser/topic/schema changes
- `--commit-all` or batch implicit corpus writes
- Operational readiness scoring (`readiness_score`)
- Re-specifying F6 ladder or F7 P0–P7 band ranks
- Batch helper generating X2 review packet as authority artifact

---

## Validation (PLAT wave)

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_advisory_queue.py \
  src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

---

## Related

- [rt_roadmap_next_frontiers_v8.md](rt_roadmap_next_frontiers_v8.md)
- [rt_roadmap_plat_rt_f7_v1.md](rt_roadmap_plat_rt_f7_v1.md)
- [rt_c3_technical_debt_audit_r1.md](rt_c3_technical_debt_audit_r1.md)
