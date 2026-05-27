# RT-X1 — Experimentation Workbench (PLAT-RT-X1)

**Phase:** PLAT-RT-X1 — local RT experimentation workbench  
**Prerequisite:** PLAT-RT-M2, PLAT-RT-TAC2–TAC5, PLAT-RT-V2, PLAT-RT-SA2 frozen  
**Authority:** [rt_experiment_workbench_v1.md](../evaluation/rt_experiment_workbench_v1.md)

## Goal

Turn the RT sandbox into a **local experimentation workbench**: pin session snapshots, compare two runs side-by-side (tactical / telemetry / terrain context), and run sequential batch sessions via maintainer CLI with per-run capture.

## Architecture

Pull telemetry → `rt_experiment_manifest_v1` (localStorage + export) → compare UI. Batch: `scripts/rt/rt_experiment_batch.py` → bridge lifecycle + `capture_session` (maintainer only; browser forbidden).

## Allowed

| Item | Location |
|------|----------|
| `platform/rt-sandbox-ui/src/experiment/*` | Manifest, compare, batch panel |
| `scripts/rt/rt_experiment_batch.py` | Maintainer batch runner |
| `fixtures/rt_experiments/*` | Example batch spec |

## Forbidden

- `platform/rt-sandbox-bridge/` protocol changes
- `platform/sa-r0-viewer/` changes
- Browser `capture_session` / SA import / federation
- Parser/topic changes; distributed runtime

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-X1 frozen. Do not start post-X1 expansion (deeper annex UI, template sweeps, PLAT-RT-M3) without explicit new wave audit.

## Related

- [rt_x1_governance_review_r1.md](../evaluation/rt_x1_governance_review_r1.md)
- [rt_x1_freeze_audit.md](../evaluation/rt_x1_freeze_audit.md)
