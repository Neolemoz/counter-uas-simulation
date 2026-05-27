# RT-V1 — Runtime Visualization Fidelity (PLAT-RT-V1)

**Phase:** PLAT-RT-V1 — runtime visualization fidelity  
**Prerequisite:** PLAT-RT-T3, PLAT-RT-T5, PLAT-RT-M2, PLAT-RT-T1, PLAT-RT-SA2 frozen  
**Authority:** [rt_v1_runtime_visualization_v1.md](../evaluation/rt_v1_runtime_visualization_v1.md)

## Goal

Improve RT sandbox **runtime visualization quality and readability** (Cesium markers, bounds depth, camera presets, cognition overlays, multi-session chrome) — UI-only, no bridge or SA viewer changes.

## Architecture

See [rt_v1_runtime_visualization_v1.md](../evaluation/rt_v1_runtime_visualization_v1.md).

## Allowed

| Item | Location |
|------|----------|
| `platform/rt-sandbox-ui/src/cesium/visualStyle.ts` | Marker/bounds style tokens |
| `entityMarkers.ts`, `boundsLayer.ts`, `cameraHelpers.ts` | Fidelity + camera presets |
| Cognition/strip/hub polish | `telemetry/cognition.ts`, panels |
| `sessionVisualIdentity.ts` | Multi-session accent colors |
| Governance layout polish | `GovernanceChrome.tsx` (text unchanged) |

## Forbidden

- `platform/rt-sandbox-bridge/` changes
- `platform/sa-r0-viewer/` changes
- Multi-globe / background entities on Cesium
- Bridge camera commands; camera persistence
- Banner text changes; tactical overlays

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-V1 frozen. Do not start PLAT-RT-M3 or automatic SA import without explicit new wave audit.

## Related

- [rt_v1_governance_review_r1.md](../evaluation/rt_v1_governance_review_r1.md)
- [rt_v1_freeze_audit.md](../evaluation/rt_v1_freeze_audit.md)
