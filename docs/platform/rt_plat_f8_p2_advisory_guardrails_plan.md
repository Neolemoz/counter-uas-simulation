# RT-F8 P2 — Advisory Guardrails + Corpus Preview Refinement (PLAT-RT-F8 P2)

**Phase:** PLAT-RT-F8 P2 — corpus-preview refinement + dry-run v2 guardrails + preview polish  
**Prerequisite:** PLAT-RT-F8 P1 frozen — [rt_plat_f8_p1_freeze_audit.md](../evaluation/rt_plat_f8_p1_freeze_audit.md)  
**Authority:** [rt_advisory_contamination_gates_v2.md](../evaluation/rt_advisory_contamination_gates_v2.md), [rt_advisory_maintainer_workflow_v2.md](../evaluation/rt_advisory_maintainer_workflow_v2.md)

## Goal

Complete the F8 advisory wave by hardening **read-only** maintainer previews:

- Corpus preview destination policy + clearer preview state (still no corpus writes)
- Dry-run review output buckets and CLI messaging (still `dry_run: true`)
- Preview export polish (client-side only)

## Delivered (P2)

| Item | Location |
|------|----------|
| Corpus preview dest validation (`fixtures/sa_r0/**` only) + richer fields | `platform/rt-sandbox-bridge/rt_sandbox/batch_advisory.py` |
| Dry-run review status buckets (`ran/skipped/error`) | `batch_advisory.py` dry-run helpers |
| CLI messaging for dry-run + corpus-preview | `scripts/rt/rt_handoff_batch_advisory.py`, `scripts/rt/rt_sa_import_dry_run.py` |
| Preview export v2 rollups | `platform/rt-sandbox-ui/src/handoff/advisoryBatchExportPreview.ts` |
| Pytest + vitest updates | `test_rt_handoff_batch_advisory.py`, `advisoryBatchExportPreview.test.ts` |

## Forbidden (unchanged)

- Bridge HTTP / subcommand registry changes  
- SA viewer scope  
- Auto-import / browser commit  
- Preset/template causing writes  
- `readiness_score` / operational readiness scoring  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_advisory_queue.py \
  src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
cd platform/rt-sandbox-ui && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F8 P2 frozen. Do not start post-F8 advisory expansion without new PLAN/PLAT + governance.

