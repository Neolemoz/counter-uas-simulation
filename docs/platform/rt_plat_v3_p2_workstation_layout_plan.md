# RT-V3 P2 — Workstation Visualization Layout (PLAT-RT-V3 P2)

**Phase:** PLAT-RT-V3 P2 — cognition rail, compact diagnostics, session chrome, toggle memory  
**Prerequisite:** PLAT-RT-V3 P1 frozen — [rt_plat_v3_p1_freeze_audit.md](../evaluation/rt_plat_v3_p1_freeze_audit.md)  
**Authority:** [rt_cesium_workstation_visualization_v3_v1.md](../evaluation/rt_cesium_workstation_visualization_v3_v1.md)

## Goal

Complete V3 workstation visualization: cognition rail column, compact background diagnostics below the globe grid, multi-session chrome polish, session-scoped layer toggle memory, and warn-only registry budget summary — without bridge/SA changes or post-V3 expansion.

## Delivered (P2)

| Item | Location |
|------|----------|
| Cognition rail + 3-6-3 grid | `RuntimeWorkstationShell.tsx`, `App.tsx` |
| Compact background diagnostic row | `BackgroundDiagnosticsCompact.tsx` |
| Shared diagnostic chips | `backgroundDiagnosticChips.ts` |
| Session layer toggle memory | `sessionLayerVisibilityStore.ts` |
| Inactive tab dimming | `SessionTabBar.tsx` |
| Cesium session chrome + budget line | `CesiumRuntimePanel.tsx`, `VisualLayerToggleRail.tsx` |
| Marker muted emphasis | `entityMarkers.ts`, `CesiumRuntimeView.tsx` |
| Registry budget helpers | `visualLayerRegistry.ts`, `RuntimeCognitionHub.tsx` |
| Reviews + freeze | `docs/evaluation/rt_plat_v3_p2_*` |

## Forbidden (unchanged)

- `platform/rt-sandbox-bridge/` changes  
- `platform/sa-r0-viewer/` changes  
- Capture/import or bridge commands from compact diagnostic row  
- Performance budget enforcement (warn-only)  
- Distributed multi-bridge; poll policy changes beyond M3  
- PLAN-RT-X2 / PLAN-RT-F8 implementation  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

**PLAT-RT-V3 complete** after P2 freeze audit. Do not start post-V3 PLAT without new scoped plan + governance.

## Related

- [rt_roadmap_plat_rt_v3_v1.md](../evaluation/rt_roadmap_plat_rt_v3_v1.md)
- [rt_plat_v3_p2_freeze_audit.md](../evaluation/rt_plat_v3_p2_freeze_audit.md)
