# RT-SA2 — Multi-Session RT→SA Workflow UX (PLAT-RT-SA2)

**Phase:** PLAT-RT-SA2 — multi-session handoff workflow visibility  
**Prerequisite:** PLAT-RT-SA1 frozen; PLAT-RT-M2 frozen; PLAT-RT-T4 frozen  
**Authority:** [rt_sa2_multi_session_handoff_ui_v1.md](../evaluation/rt_sa2_multi_session_handoff_ui_v1.md); [rt_sa_import_bridge_v1.md](../evaluation/rt_sa_import_bridge_v1.md)

## Goal

Improve RT→SA workflow **visibility and review ergonomics** via read-only bridge staging mirror and multi-session RT sandbox UI — without SA viewer changes, automatic import, or corpus writes from the browser.

## Architecture

See [rt_sa2_multi_session_handoff_ui_v1.md](../evaluation/rt_sa2_multi_session_handoff_ui_v1.md).

## Allowed

| Item | Location |
|------|----------|
| `rt_sandbox/capture_handoff_mirror.py` | Read-only staging mirror |
| `list_capture_handoff_status` | Bridge command (read-only) |
| `session_handoff_handlers.py` | Command handler |
| `platform/rt-sandbox-ui/` handoff modules | UI mirror consumer |
| `MANUAL HANDOFF ONLY` banner | `governance/banners.ts` |
| `rt_capture_inspect list --session-id` | CLI parity filter |
| Contract + freeze docs | `docs/evaluation/rt_sa2_*` |

## Forbidden

- SA viewer (`platform/sa-r0-viewer/`) changes
- Automatic `rt_sa_import` / normalize / approve / review from browser
- Bridge HTTP import or commit commands
- SA corpus writes from bridge session code or UI
- Federation / distributed multi-bridge
- New telemetry channels or WebSocket push
- UI invocation of `capture_session`, `rt_sa_import`

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-SA2 frozen. Do not start PLAT-RT-M3, RT-V1, RT-SA3, or automatic SA import without explicit new wave audit.

## Related

- [rt_sa2_governance_review_r1.md](../evaluation/rt_sa2_governance_review_r1.md)
- [rt_sa2_freeze_audit.md](../evaluation/rt_sa2_freeze_audit.md)
