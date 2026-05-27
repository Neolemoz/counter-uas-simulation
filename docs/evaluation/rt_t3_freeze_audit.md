# RT-T3 — Cesium Runtime Visualization Freeze Audit (PLAT-RT-T3)

**Scope:** Cesium 3D runtime visualization extending PLAT-RT-T1/T2.

## In scope

- Plan: [rt_t3_cesium_runtime_visualization_plan.md](../platform/rt_t3_cesium_runtime_visualization_plan.md)
- Contract: [rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md)
- UI: `platform/rt-sandbox-ui/src/cesium/`, `CesiumRuntimePanel`
- Fifth banner: `CESIUM RUNTIME VIEW`
- Fictional georef + bounds overlay + entity markers from pull mirror
- T2 SVG world editor retained

## Not in scope

- Bridge command or channel changes
- SA viewer / replay ingestion / federation
- Cesium-based entity editing
- WebSocket push telemetry
- Hidden persistence
- Cesium Ion / operational geography

**Prerequisite:** PLAT-RT-T2 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-T3.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Pull-only telemetry unchanged | Pass |
| Registry commands authoritative (T2 SVG) | Pass |
| Cesium mirrors explanatory | Pass |
| Session-scoped rendering | Pass |
| Fictional georef only | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | Cesium runtime view + panel | Yes |
| 2 | `src/cesium/` modules | Yes |
| 3 | Cesium cognition helpers | Yes |
| 4 | Fifth governance banner | Yes |
| 5 | Viz toggles + camera reset | Yes |
| 6 | Vitest + CI tier0-rt-ui | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## Cesium runtime architecture summary

| Step | Layer |
|------|-------|
| 1 | Pull loop updates `entity_pose_mirror` + `world_summary` |
| 2 | `CesiumRuntimeView` projects ENU → fictional georef Cartesian3 |
| 3 | Entity markers + bounds overlay render mirror state |
| 4 | T2 SVG grid issues spawn/move/delete via POST |
| 5 | Cognition strips show authority, source, health badges |
| 6 | Disconnect destroys viewer and clears entities |

## Isolation guarantees

- Same `platform/rt-sandbox-ui/` package; no `sa-r0-viewer` imports
- RT-local Cesium modules; no shared SA replay code
- Loopback bridge only; no browser→ROS
- No federation writes; no SA bundle load
- No localStorage / session persistence beyond React state

## Stop Line

PLAT-RT-T3 frozen. Next expansion requires explicit new wave audit:

| Frontier | Notes |
|----------|-------|
| **PLAT RT→SA bridge** | Manual import handoff **implementation** ([rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)) |
| **RT-T4+** | Any further runtime UI expansion |

Do **not** start SA bridge implementation or RT-T4 without new audit.
