# RT-SA1 — RT→SA Manual Import Bridge (PLAT-RT-SA1)

**Phase:** PLAT-RT-SA1 — RT→SA manual import bridge implementation  
**Prerequisite:** PLAN-RT-R2f frozen; PLAT-RT-T3 frozen  
**Authority:** [rt_rt_sa_bridge_handoff_v1.md](../evaluation/rt_rt_sa_bridge_handoff_v1.md); [rt_sa_import_bridge_v1.md](../evaluation/rt_sa_import_bridge_v1.md)

## Goal

Implement the first **manual RT→SA import bridge**: maintainer CLIs, handoff staging, `handoff_*` export audit events, and explicit corpus commit — without automatic import, SA viewer changes, or federation writes.

## Architecture

See [rt_sa_import_bridge_v1.md](../evaluation/rt_sa_import_bridge_v1.md).

## Allowed

| Item | Location |
|------|----------|
| `rt_sandbox/sa_handoff.py` | Bridge helpers |
| `scripts/rt/rt_handoff_review.py` | Review / reject / defer |
| `scripts/rt/rt_sa_import.py` | prepare / run-step / commit |
| `rt_capture_inspect handoff-status` | Read-only status |
| `runs/rt_sandbox/sa_handoff/` | Handoff workspace |
| Handoff export events | `export_boundary.jsonl` |
| Lineage validators | `export_boundary.py` extensions |

## Forbidden

- Bridge HTTP commands for import
- Automatic import on `capture_session`
- Writes to SA corpus from bridge session code
- Federation / `sync_sa_catalog` from handoff code
- `platform/sa-r0-viewer/` changes
- RT-T3 / telemetry UI changes

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-SA1 frozen. Do not start RT-T4 or deeper SA workflow integration without explicit new wave audit.

## Related

- [rt_sa1_governance_review_r1.md](../evaluation/rt_sa1_governance_review_r1.md)
- [rt_sa1_freeze_audit.md](../evaluation/rt_sa1_freeze_audit.md)
