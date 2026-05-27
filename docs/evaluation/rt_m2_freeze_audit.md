# RT-M2 — Multi-Session Freeze Audit (PLAT-RT-M2)

**Scope:** Local single-bridge multi-session bridge + workstation UI (cap=3).

## In scope

- Plan: [rt_m2_multi_session_implementation_plan.md](../platform/rt_m2_multi_session_implementation_plan.md)
- M1 contracts: `rt_multi_session_*_v1` (implemented per bridge §11)
- Bridge: `session_registry.py`, `session_registry_handlers.py`, manager refactor
- UI: `useRtSessionWorkspace.ts`, `SessionTabBar`, `BackgroundDiagnostics`, per-session `App.tsx` maps
- Seventh banner: `MULTI-SESSION — local prototype; not operational coordination`
- Reviews: [rt_m2_governance_review_r1.md](rt_m2_governance_review_r1.md), [rt_m2_isolation_audit.md](rt_m2_isolation_audit.md)

## Not in scope

- SA viewer / replay ingestion / federation
- Distributed multi-bridge
- RT-SA2 deeper SA workflow
- RT-V1 richer visualization
- PLAT-RT-M3 polish wave
- WebSocket push telemetry
- New telemetry channels

**Prerequisite:** PLAN-RT-M1 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-M2.

Bridge contract [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §11 marked **implemented** (no rewrite of frozen §3).

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Parser/topic/schema unchanged | Pass |
| Single bridge, cap=3 | Pass |
| Editing lock bridge-enforced | Pass |
| Capture per-session isolation | Pass |
| No federation writes | Pass |
| Loopback only | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `SessionRegistry` module | Yes |
| 2 | `list_sessions` / `set_editing_session` | Yes |
| 3 | Editing gate + aggregate entity cap | Yes |
| 4 | Per-session telemetry rate limits | Yes |
| 5 | `useRtSessionWorkspace` + tab bar | Yes |
| 6 | Background diagnostics panel | Yes |
| 7 | Multi-session governance banner | Yes |
| 8 | Bridge + UI tests + CI | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

**Pass counts (M2 freeze):** bridge pytest 138 passed; UI Vitest 70 passed; tier0 + tier0-rt-ui OK.

## Architecture summary

| Layer | Behavior |
|-------|----------|
| Bridge | Registry of up to 3 `SessionRecord`; one `editing_session_id` |
| Active UI | Full telemetry channels, SVG + Cesium editing on selected tab |
| Background UI | Diagnostic channels at 1 Hz; accordion diagnostics panel |
| Isolation | Separate world, mirrors, audit, capture, rate limits per session |

## Isolation guarantees

- No shared world/mirror/audit/capture paths across sessions
- Bridge rejects entity/template mutations on non-editing session
- Sibling session survives individual runtime crash
- UI edit history and local entity maps keyed by `sessionId`

## Stop Line

PLAT-RT-M2 frozen. Next expansion requires explicit new wave audit:

| Frontier | Notes |
|----------|-------|
| **PLAT-RT-M3** | Optional polish per [rt_roadmap_m1_m2_v1.md](rt_roadmap_m1_m2_v1.md) |
| **RT-SA2** | Deeper SA workflow — not authorized |
| **RT-V1** | Richer visualization — provisional |
| **Distributed multi-bridge** | Forbidden |

Do **not** start these without new audit.
