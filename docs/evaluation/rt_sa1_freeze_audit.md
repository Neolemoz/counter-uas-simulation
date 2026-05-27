# RT-SA1 — Manual Import Bridge Freeze Audit (PLAT-RT-SA1)

**Scope:** RT→SA manual import bridge implementation.

## In scope

- Plan: [rt_sa1_manual_import_bridge_plan.md](../platform/rt_sa1_manual_import_bridge_plan.md)
- Contract: [rt_sa_import_bridge_v1.md](rt_sa_import_bridge_v1.md)
- Module: `platform/rt-sandbox-bridge/rt_sandbox/sa_handoff.py`
- CLIs: `rt_handoff_review.py`, `rt_sa_import.py`, `rt_capture_inspect handoff-status`
- Handoff staging: `runs/rt_sandbox/sa_handoff/`
- Export events: `handoff_ready`, `handoff_reviewed`, `handoff_rejected`, `handoff_import_deferred`, `handoff_import_prepared`, `handoff_import_committed`

## Not in scope

- Automatic import; federation writes
- SA viewer changes; bridge HTTP import commands
- RT-T4; telemetry/Cesium expansion

**Prerequisite:** PLAN-RT-R2f frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-SA1.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Manual import only | Pass |
| RT authority stops before SA pack | Pass |
| Lineage starts at commit | Pass |
| Session ID non-authoritative | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `sa_handoff.py` | Yes |
| 2 | `rt_handoff_review.py` | Yes |
| 3 | `rt_sa_import.py` | Yes |
| 4 | Handoff export events | Yes |
| 5 | Lineage validators | Yes |
| 6 | Tests + CI | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## RT→SA bridge architecture summary

| Step | Layer |
|------|-------|
| 1 | RT capture + normalization in `captures/` |
| 2 | `rt_handoff_review` emits `handoff_*` review events |
| 3 | `rt_capture_approve` writes approval + conversion |
| 4 | `rt_sa_import prepare` creates `sa_handoff/` manifest |
| 5 | Maintainer runs `run-step` for SA tooling (subprocess) |
| 6 | `rt_sa_import commit` copies bundle to corpus with import record |

## Isolation guarantees

- Bridge session code cannot write SA corpus paths
- Only `rt_sa_import commit` performs corpus copy (maintainer-invoked)
- No SA viewer imports; no federation auto-update
- `session_id` never SA lineage authority

## Stop Line

PLAT-RT-SA1 frozen. Next expansion requires explicit new wave audit:

| Frontier | Notes |
|----------|-------|
| **RT-T4** | Runtime UI expansion |
| **Deeper SA workflow** | Integrated SA import UX |

Do **not** start RT-T4 or auto-ingest without new audit.
