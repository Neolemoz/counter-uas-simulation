# RT-SA2 — Multi-Session Handoff UX Freeze Audit (PLAT-RT-SA2)

**Scope:** Read-only handoff staging mirror + multi-session RT sandbox UI.

## In scope

- Plan: [rt_sa2_multi_session_handoff_workflow_plan.md](../platform/rt_sa2_multi_session_handoff_workflow_plan.md)
- Contract: [rt_sa2_multi_session_handoff_ui_v1.md](rt_sa2_multi_session_handoff_ui_v1.md)
- Bridge: `capture_handoff_mirror.py`, `session_handoff_handlers.py`, `list_capture_handoff_status`
- UI: handoff mirror hook, capture table, multi-session overview, tab/diagnostics badges
- Banner: `MANUAL HANDOFF ONLY`
- CLI: `rt_capture_inspect list --session-id`
- Reviews: [rt_sa2_governance_review_r1.md](rt_sa2_governance_review_r1.md)

## Not in scope

- SA viewer / automatic replay import / federation
- Bridge import or commit commands
- RT-V1 richer visualization
- PLAT-RT-M3 polish
- Distributed multi-bridge

**Prerequisite:** PLAT-RT-SA1, PLAT-RT-M2, PLAT-RT-T4 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-SA2.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Manual handoff only | Pass |
| Read-only mirror (no staging writes) | Pass |
| Session-scoped capture rows | Pass |
| RT authority ends before corpus commit | Pass |
| Loopback only | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `capture_handoff_mirror.py` | Yes |
| 2 | `list_capture_handoff_status` | Yes |
| 3 | `rt_sa2_multi_session_handoff_ui_v1.md` | Yes |
| 4 | Handoff UI modules + panel | Yes |
| 5 | `MANUAL HANDOFF ONLY` banner | Yes |
| 6 | Bridge + UI tests + CI | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

**Pass counts (SA2 freeze):** bridge pytest 143 passed; UI Vitest 75 passed; tier0 + tier0-rt-ui OK.

## RT→SA workflow architecture summary

| Layer | Behavior |
|-------|----------|
| Runtime | `capture_session` writes `runs/rt_sandbox/captures/<id>/` |
| Maintainer | normalize → review → approve → prepare → SA steps (CLIs) |
| Corpus | `rt_sa_import commit` — SA lineage begins |
| UI mirror | `list_capture_handoff_status` — read-only, session-scoped rows |

## Isolation guarantees

- Mirror responses filtered by requested `session_id` / `ephemeral_session_ref`
- No cross-session capture rows in API responses
- Browser cannot invoke import or capture commands
- Redacted rows exclude corpus paths and full candidate blobs
- `session_id` remains non-authoritative in UI copy

## Stop Line

| Frontier | Notes |
|----------|-------|
| RT-V1 | Richer visualization — not authorized |
| RT-M3 | Multi-session polish — not authorized |
| RT-SA3+ / auto-import | Requires new audit |
