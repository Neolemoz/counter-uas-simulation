# RT-F3 — Tactical Annex Review (PLAT-RT-F3)

**Phase:** PLAT-RT-F3 — annex & continuity review UI  
**Prerequisite:** PLAN-RT-F3 frozen; PLAT-RT-F1 frozen; PLAT-RT-TAC5 frozen  
**Authority:** [rt_experiment_annex_review_ui_v1.md](../evaluation/rt_experiment_annex_review_ui_v1.md), [rt_experiment_continuity_review_v1.md](../evaluation/rt_experiment_continuity_review_v1.md)

## Goal

Read-only tactical annex timeline review integrated with F1 analytics in the experiment workbench — without bridge, SA viewer, or authority changes.

## Delivered

| Item | Location |
|------|----------|
| Annex Zod + cache | `tacticalAnnexSchema.ts`, `annexReviewStore.ts` |
| Timeline UI | `TacticalAnnexReviewPanel`, `TacticalAnnexTimelineTable` |
| Continuity hub | `ExperimentContinuityReviewPanel` |
| Compare strip | `TacticalAnnexCompareStrip` |
| Maintainer pack CLI | `scripts/rt/rt_experiment_annex_pack.py` |
| `BANNER_ANNEX_REVIEW` | `governance/banners.ts` |

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_experiment_analytics.py \
  src/counter_uas/test/test_rt_experiment_annex_pack.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F3 frozen. Do not start post-F3 expansion without new wave audit.

## Related

- [rt_plat_f3_freeze_audit.md](../evaluation/rt_plat_f3_freeze_audit.md)
