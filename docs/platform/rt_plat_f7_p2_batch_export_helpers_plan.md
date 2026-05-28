# RT-F7 P2 — Batch Export Helpers (PLAT-RT-F7 P2)

**Phase:** PLAT-RT-F7 P2 — advisory batch export + dry-run hardening  
**Prerequisite:** PLAT-RT-F7 P0 + P1 frozen — [rt_plat_f7_p0_freeze_audit.md](../evaluation/rt_plat_f7_p0_freeze_audit.md), [rt_plat_f7_p1_freeze_audit.md](../evaluation/rt_plat_f7_p1_freeze_audit.md)  
**Authority:** [rt_advisory_aggregation_v1.md](../evaluation/rt_advisory_aggregation_v1.md), [rt_advisory_contamination_gates_v1.md](../evaluation/rt_advisory_contamination_gates_v1.md)

## Goal

Complete F7 maintainer bulk workflow: `rt_advisory_batch_review_v2` export, stand-up/grouped CLI subcommands, batch `dry-run-review`, stricter dry-run guards — without bridge protocol, SA viewer, or write-path expansion.

## Delivered (P2)

| Item | Location |
|------|----------|
| `build_advisory_batch_review_v2_document` | `batch_advisory.py` |
| `validate_advisory_batch_review_v2` | `batch_advisory.py` |
| `build_grouped_export_indexes` / `build_standup_section` | `batch_advisory.py` |
| `run_dry_run_review_for_capture` | `batch_advisory.py` |
| CLI `standup-export`, `grouped-export`, `dry-run-review` | `rt_handoff_batch_advisory.py` |
| `--schema v2` on `export` | `rt_handoff_batch_advisory.py` |
| `rt_sa_import_dry_run.py` hardening | `scripts/rt/` |
| `advisoryBatchExportPreview.ts` | `platform/rt-sandbox-ui/src/handoff/` |
| Pytest + vitest + golden fixture | tests + `fixtures/rt_handoff/f7_advisory_examples/` |
| Reviews + freeze | `docs/evaluation/rt_plat_f7_p2_*` |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- `platform/sa-r0-viewer/` changes  
- `--commit-all`; auto-import; federation writes  
- Browser commit / import automation  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_advisory_queue.py src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
cd platform/rt-sandbox-ui && npm run test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F7 P2 frozen — **PLAT-RT-F7 complete**. No post-F7 expansion without new PLAN wave.

## Related

- [rt_plat_f7_p2_freeze_audit.md](../evaluation/rt_plat_f7_p2_freeze_audit.md)
- [rt_roadmap_plat_rt_f7_v1.md](../evaluation/rt_roadmap_plat_rt_f7_v1.md)
