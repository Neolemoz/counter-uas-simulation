# RT - PLAT-RT-V4 P1 Architecture Review R1

**Phase:** PLAT-RT-V4 P1
**Status:** accepted for freeze

## Review summary

P1 is an additive visualization layer over P0. It extends the V4 registry with four default-off display layers, adds one Cesium-local display module, and threads explanatory cognition into existing workstation surfaces.

## Architecture decisions

| Decision | Rationale | Boundary |
|----------|-----------|----------|
| Reuse `visualLayerRegistry.ts` | Keeps layer metadata in the frozen V3/V4 registry path | No command registry changes |
| Add `visibilityOverlayV4.ts` | Isolates corridor, occlusion band, and terrain label rendering | Display-only Cesium entities |
| Extend `VisibilityCognitionStrip` | Keeps visibility wording in existing cognition surface | Explanatory text only |
| Extend session comparison cognition | Supports dimmed compare emphasis without session authority changes | Selected session remains commandable |
| Count P1 overlays in density budget | Preserves P0 warn-only density policy | Advisory only, not enforcement |

## Coupling assessment

- **Bridge/runtime coupling:** none. No files under `platform/rt-sandbox-bridge/` were changed.
- **SA coupling:** none. No files under `platform/sa-r0-viewer/` were changed.
- **Cesium coupling:** low to medium. P1 adds decorative entities through the existing viewer sync pattern.
- **Registry coupling:** additive. New rows use `v4_p1` phase and default-off visibility keys.
- **Governance coupling:** low. All new labels and summaries state heuristic/explanatory semantics.

## Accepted residual risk

The primary risk is reviewer over-reading visual cues as coverage or occlusion proof. The mitigation is default-off toggles, explicit disclaimers, cognition wording, and freeze audit boundaries.
