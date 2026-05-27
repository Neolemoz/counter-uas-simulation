# RT-V1 — Runtime Visualization Fidelity Freeze Audit (PLAT-RT-V1)

**Scope:** Cesium readability, camera presets, cognition overlays, multi-session chrome (UI-only).

## In scope

- Plan: [rt_v1_runtime_visualization_fidelity_plan.md](../platform/rt_v1_runtime_visualization_fidelity_plan.md)
- Contract: [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md)
- UI: `cesium/visualStyle.ts`, `entityMarkers.ts`, `boundsLayer.ts`, `cameraHelpers.ts`
- Workstation: `sessionVisualIdentity.ts`, tab accents, background health chips
- Reviews: [rt_v1_governance_review_r1.md](rt_v1_governance_review_r1.md)

## Not in scope

- Bridge / SA viewer / federation
- Multi-globe rendering
- PLAT-RT-M3 polish wave
- Tactical overlays

**Prerequisite:** PLAT-RT-T3, T5, M2, SA2 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-V1.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| Zero bridge diff | Pass |
| SA viewer untouched | Pass |
| Banner text unchanged | Pass |
| Single selected-session globe | Pass |
| No camera bridge commands | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `visualStyle.ts` + marker/bounds fidelity | Yes |
| 2 | Camera presets (fit/tight/session fly) | Yes |
| 3 | Session strip + tab accents | Yes |
| 4 | Cognition overlay polish | Yes |
| 5 | Vitest + CI | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```

**Pass counts (V1 freeze):** bridge pytest 143 passed; UI Vitest 79 passed; tier0 + tier0-rt-ui OK.

## Runtime visualization architecture summary

| Surface | Authority | V1 role |
|---------|-----------|---------|
| SVG grid | Command-authoritative | Unchanged |
| Cesium globe | Explanatory mirror | Readability, bounds depth, camera presets |
| Telemetry hub/strips | Explanatory pull | Authority/health/stale emphasis |
| Tabs/diagnostics | UI chrome | Per-session accent + background health |

## Stop Line

| Frontier | Notes |
|----------|-------|
| RT-M3 | Multi-session polish — not authorized |
| RT-SA3+ / auto-import | Forbidden |
| Distributed / tactical viz | Forbidden |
