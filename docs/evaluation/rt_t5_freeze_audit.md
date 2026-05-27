# RT-T5 — Cesium Interactive Editing Freeze Audit (PLAT-RT-T5)

**Scope:** Interactive Cesium entity editing extending PLAT-RT-T3/T4.

## In scope

- Plan: [rt_t5_cesium_interactive_editing_plan.md](../platform/rt_t5_cesium_interactive_editing_plan.md)
- Contract: [rt_cesium_interactive_editing_ui_v1.md](rt_cesium_interactive_editing_ui_v1.md)
- UI: `platform/rt-sandbox-ui/src/cesium/cesiumEditing.ts`, `cameraHelpers.ts`, `entityId.ts`
- `CesiumEditingCognitionStrip`, extended `CesiumRuntimePanel` / `CesiumRuntimeView`
- Sixth banner: `INTERACTIVE EDITING`
- Dual edit surfaces: SVG + Cesium share App command pipeline

## Not in scope

- Bridge command or channel changes
- SA viewer / replay ingestion / federation
- WebSocket push telemetry
- Multi-session UI
- Tactical/HITL semantics

**Prerequisite:** PLAT-RT-T4 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-T5.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Existing entity commands only | Pass |
| Registry authoritative | Pass |
| Mirrors explanatory | Pass |
| Bounds/caps enforced | Pass |
| No hidden persistence | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | Cesium edit handlers | Yes |
| 2 | Camera focus/follow helpers | Yes |
| 3 | Cesium editing cognition | Yes |
| 4 | Sixth governance banner | Yes |
| 5 | Dual-surface App wiring | Yes |
| 6 | Vitest + CI tier0-rt-ui | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## Cesium editing architecture summary

| Step | Layer |
|------|-------|
| 1 | Globe pick → ENU via `cartographicToWorld` |
| 2 | `clampPose` / `canSpawn` / `isEditingAllowed` gates |
| 3 | `spawn_entity` / `move_entity` / `delete_entity` via POST |
| 4 | Markers refresh from pull; drag-local override during gesture |
| 5 | SVG grid and Cesium share `selectedEntityId` in App |

## Isolation guarantees

- No `sa-r0-viewer` imports
- Bridge `src/bridge/` unchanged command set
- Loopback only; no browser→ROS
- No federation writes

## Stop Line

PLAT-RT-T5 frozen. Next expansion requires explicit new wave audit:

| Frontier | Notes |
|----------|-------|
| **RT-G6** | Visual fidelity / sim realism |
| **Multi-session UI** | Not authorized |
| **Bridge expansion** | Staging APIs, new channels |

Do **not** start these without new audit.
