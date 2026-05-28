# RT - PLAT-RT-V4 P1 Visualization Realism Review R1

**Phase:** PLAT-RT-V4 P1
**Status:** accepted

## Realism posture

P1 improves visual cognition, not simulation realism. Corridor, occlusion, and terrain relation cues are derived from existing fictional terrain and selected-session mirror data. They are useful for maintainer interpretation but are not sensor truth, coverage authority, or tactical readiness evidence.

## Visual elements

| Element | Source | Realism claim |
|---------|--------|---------------|
| Visibility corridor | Selected entity pose and heading | Heuristic display cue only |
| Occlusion band | Selected pose and fictional terrain context | Possible masking emphasis only |
| Terrain relation label | Existing terrain cognition helpers | Explanatory AGL/terrain relation only |
| Compare emphasis | Local session comparison rows | Visual separation only |

## Density and readability

P1 overlays are default off and participate in the existing warn-only overlay budget. Density warnings do not enforce display choices and do not change Cesium or bridge behavior.

## Misread controls

- Layer disclaimers include heuristic/explanatory wording.
- Workstation cognition repeats display-only boundaries.
- Compare rows keep non-selected sessions non-commandable.
- No browser surface claims authority over ROS/Gazebo state.

## Verdict

Visualization realism is acceptable for P1 because it increases visual fidelity without implying physics, sensor coverage, or command authority.
