# RT-F2 — Runtime Platform Hardening (PLAT-RT-F2)

**Phase:** PLAT-RT-F2 — stability-only maintenance  
**Prerequisite:** PLAN-RT-F2 frozen; PLAT-RT-F3 frozen  
**Authority:** [rt_runtime_cleanup_hardening_v1.md](../evaluation/rt_runtime_cleanup_hardening_v1.md), [rt_ui_hardening_v1.md](../evaluation/rt_ui_hardening_v1.md), [rt_experiment_import_hardening_v1.md](../evaluation/rt_experiment_import_hardening_v1.md)

## Goal

Harden RT sandbox teardown, experiment/annex import paths, and multi-session UI orphan state — without bridge protocol changes, SA viewer changes, or new runtime features.

## Delivered

| Item | Location |
|------|----------|
| Tactical teardown + audit | `session_teardown.clear_tactical_state`, lifecycle timeout path |
| Import guards | `experimentImportGuards.ts`, `experimentIds.ts` |
| Annex cache lifecycle | `annexReviewStore.ts` prune/partial bundle |
| UI orphan prune | `App.tsx` `editBySession`, workbench compare key normalize |
| Staging integrity audit (P1) | `scripts/rt/rt_staging_integrity_audit.py` |

## Forbidden

- New bridge commands / telemetry channels
- `platform/sa-r0-viewer/` changes
- Browser `capture_session`
- `App.tsx` full decomposition
- Capture staging deletion automation

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_experiment_analytics.py \
  src/counter_uas/test/test_rt_experiment_annex_pack.py \
  src/counter_uas/test/test_rt_staging_integrity_audit.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F2 frozen. Next advisory: **F4 realism** or defer **M3** / **F5** indefinitely — new wave audit required.

## Related

- [rt_plat_f2_governance_review_r1.md](../evaluation/rt_plat_f2_governance_review_r1.md)
- [rt_plat_f2_hardening_review_r1.md](../evaluation/rt_plat_f2_hardening_review_r1.md)
- [rt_plat_f2_freeze_audit.md](../evaluation/rt_plat_f2_freeze_audit.md)
