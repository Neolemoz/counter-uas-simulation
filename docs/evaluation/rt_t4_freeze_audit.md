# RT-T4 — Runtime Session Workspace UX Polish Freeze Audit (PLAT-RT-T4)

**Scope:** Runtime workstation layout and workflow visibility extending PLAT-RT-T1/T2/T3.

## In scope

- Plan: [rt_t4_runtime_session_workspace_plan.md](../platform/rt_t4_runtime_session_workspace_plan.md)
- Contract: [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md)
- UI: `platform/rt-sandbox-ui/src/workstation/`, `src/workflow/`, `CaptureHandoffWorkflowPanel`
- `useRtSession` hook; `RuntimeWorkstationShell` layout
- Governance chrome polish (banner text unchanged)
- T1–T3 panels and behaviors preserved

## Not in scope

- Bridge command or channel changes
- Browser `capture_session` or SA import execution
- Live staging filesystem polling
- SA viewer / replay ingestion / federation
- Multi-session UI
- WebSocket push telemetry

**Prerequisite:** PLAT-RT-T3 and PLAT-RT-SA1 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-T4.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Pull-only telemetry unchanged | Pass |
| Registry commands authoritative (T2 SVG) | Pass |
| Cesium mirrors explanatory | Pass |
| Capture/handoff explanatory only | Pass |
| No browser staging reads | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `RuntimeWorkstationShell` + session workflow strip | Yes |
| 2 | `RuntimeCognitionHub` | Yes |
| 3 | `CaptureHandoffWorkflowPanel` + `captureHandoffCognition.ts` | Yes |
| 4 | `useRtSession` hook | Yes |
| 5 | Disconnected idle UX | Yes |
| 6 | Vitest + pytest workstation boundaries | Yes |
| 7 | CI tier0-rt-ui | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## Workstation UX architecture summary

| Layer | Role |
|-------|------|
| 1 | Governance banners — transient runtime only |
| 2 | Session rail — connect, pull, workflow badges |
| 3 | World column — T2 SVG authoritative editing |
| 4 | Viz column — T3 Cesium read-only mirror |
| 5 | Mirrors — cognition hub + T1 channel panels |
| 6 | Pipeline footer — capture/handoff maintainer guidance |

## Isolation guarantees

- Same `platform/rt-sandbox-ui/` package; no `sa-r0-viewer` imports
- Bridge `src/bridge/` excludes capture/SA import command strings
- Loopback bridge only; no browser→ROS
- No federation writes; no SA bundle load
- No localStorage / session persistence beyond React state

## Stop Line

PLAT-RT-T4 frozen. Next expansion requires explicit new wave audit:

| Frontier | Notes |
|----------|-------|
| **Multi-session UI** | Not authorized |
| **Richer RT visualization** | Beyond Cesium mirror scope |
| **Deeper SA workflow integration** | Browser staging poll, SA viewer hooks, auto-import |
| **Bridge staging status API** | Separate bridge wave |

Do **not** start these frontiers without new audit.
