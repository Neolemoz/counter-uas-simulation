# RT - PLAT-RT-V4 P1 Visibility Overlay Plan

**Phase:** PLAT-RT-V4 P1
**Status:** implemented and frozen
**Prerequisite:** PLAT-RT-V4 P0 frozen

## Scope

P1 adds advanced visibility and terrain cognition on top of the V4 P0 registry and density foundations. The work is RT UI local and display-only.

Delivered surfaces:

| Area | Implementation |
|------|----------------|
| V4 visibility registry rows | `visibility_corridor_v4`, `occlusion_bands_v4`, `terrain_relation_labels_v4`, `compare_emphasis_v4` |
| Cesium display module | `visibilityOverlayV4.ts` corridor rays, occlusion band cue, terrain relation label |
| Cognition refinement | `VisibilityCognitionStrip`, `RuntimeCognitionHub`, `SessionComparisonCognitionStrip` |
| Density awareness | P1 layers participate in existing warn-only overlay budget counting |
| Tests | registry, visibility overlay, and session comparison cognition tests |

## Architecture

P1 reuses existing V3/V4 visualization patterns:

1. `visualLayerRegistry.ts` remains the source for display layer metadata.
2. `visibilityOverlayV4.ts` derives explanatory hints and syncs Cesium decorative entities with the `rt-v4-visibility-` prefix.
3. `CesiumRuntimeView.tsx` calls the sync module after terrain and stacked LOS synchronization.
4. `RuntimeCognitionHub.tsx` and `VisibilityCognitionStrip.tsx` summarize active V4 hints as heuristic/explanatory.
5. `sessionComparisonCognition.ts` adds a dimmed display mode when compare emphasis is enabled.

All P1 layer defaults are off. P1 does not change command routing, bridge contracts, telemetry contracts, or session authority.

## Governance boundaries

- Registry rows are display-layer metadata only.
- Browser controls toggle local display state only.
- Corridor, band, and label entities are visual cues, not coverage proof.
- Compare emphasis dims secondary/background sessions only; selected session remains the only commandable session.
- No bridge, ROS/Gazebo, SA viewer, import, federation, or distributed runtime changes.

## P2 stop line

PLAT-RT-V4 P1 is frozen. Do not start P2 multi-session comparison layout/geometry work without a scoped P2 plan, governance review, validation, and freeze audit.
