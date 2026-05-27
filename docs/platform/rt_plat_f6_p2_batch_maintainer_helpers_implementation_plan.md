# RT-F6 P2 — Batch Maintainer Helpers (PLAT-RT-F6 P2)

**Phase:** PLAT-RT-F6 P2 — batch maintainer helpers  
**Prerequisite:** PLAT-RT-F6 P0 + P1 frozen  
**Authority:** [rt_sa_workflow_automation_v1.md](../evaluation/rt_sa_workflow_automation_v1.md)

## Goal

Maintainer-only CLIs for batch advisory scan/report/export, import pipeline dry-run preview, and read-only corpus diff preview — dry-run default, no auto-import.

## Delivered (P2)

| Item | Location |
|------|----------|
| `batch_advisory.py` | `platform/rt-sandbox-bridge/rt_sandbox/` |
| `rt_handoff_batch_advisory.py` | `scripts/rt/` |
| `rt_sa_import_dry_run.py` | `scripts/rt/` |
| `corpus-preview` subcommand | `rt_handoff_batch_advisory.py` |
| Pytest | `test_rt_handoff_batch_advisory.py` |

## Forbidden (unchanged)

- Bridge HTTP / `RUNTIME_SUBCOMMANDS` changes  
- `platform/sa-r0-viewer/` changes  
- `--commit-all`; auto-import; federation writes  
- Browser automation  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_handoff_advisory.py -q
python3 -m pytest src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
python3 scripts/rt/rt_handoff_batch_advisory.py report --json
```

## Stop line

PLAT-RT-F6 complete (P0–P2). Recommend platform consolidation review; defer M3.

## Related

- [rt_plat_f6_p2_freeze_audit.md](../evaluation/rt_plat_f6_p2_freeze_audit.md)
- [rt_plat_f6_p2_governance_review_r1.md](../evaluation/rt_plat_f6_p2_governance_review_r1.md)
- [rt_f6_handoff_contamination_review_p2_r1.md](../evaluation/rt_f6_handoff_contamination_review_p2_r1.md)
