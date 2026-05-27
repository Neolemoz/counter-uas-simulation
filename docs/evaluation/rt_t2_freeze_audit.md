# RT-T2 — Drag/Drop + Runtime World Editing Freeze Audit (PLAT-RT-T2)

**Scope:** Interactive world editing UI extending PLAT-RT-T1.

## In scope

- Plan: [rt_t2_world_editing_ui_plan.md](../platform/rt_t2_world_editing_ui_plan.md)
- Contract: [rt_world_editing_ui_v1.md](rt_world_editing_ui_v1.md)
- UI: `platform/rt-sandbox-ui/` world editor components
- Entity commands: `spawn_entity`, `move_entity`, `delete_entity`
- Fourth banner: `WORLD EDITING ACTIVE`
- Edit history UI (session-local)

## Not in scope

- Bridge command changes
- Cesium / SA viewer / federation
- WebSocket push telemetry
- Hidden persistence

**Prerequisite:** PLAT-RT-T1 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-T2.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Pull-only telemetry unchanged | Pass |
| Registry commands authoritative | Pass |
| Mirrors explanatory | Pass |
| Session-scoped edits | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | World editing grid + palette | Yes |
| 2 | `entityCommands.ts` | Yes |
| 3 | Editing cognition helpers | Yes |
| 4 | Fourth governance banner | Yes |
| 5 | Edit history panel | Yes |
| 6 | Vitest + pytest tests | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## World editing architecture summary

| Step | Layer |
|------|-------|
| 1 | Entity palette selects type |
| 2 | SVG grid: click spawn, drag move, delete |
| 3 | `POST /v1/command` entity ops |
| 4 | Local edit history append |
| 5 | `GET /v1/telemetry/pull` reconcile mirror |
| 6 | Editing cognition strip |

## Isolation guarantees

- Same `platform/rt-sandbox-ui/` package as T1
- No SA imports; path blocking unchanged
- Commands via loopback bridge only
- No browser→ROS; no federation writes

## Stop Line

PLAT-RT-T2 frozen. Next expansion requires explicit new wave audit:

| Frontier | Notes |
|----------|-------|
| **RT-T3** | Cesium runtime visualization |
| **PLAT RT→SA bridge** | Manual import handoff implementation |

Do **not** start RT-T3, Cesium, or SA bridge implementation without new audit.
