# RT Planning Mode Evolution V1

## Scope

RT Planning Mode Evolution V1 adds a UI-local planning surface that coexists with the existing Grid Mode in `platform/rt-sandbox-ui/`.

Included surfaces:

- Grid Mode / Planning Mode selector, defaulting to Grid Mode.
- Planning-only defense area polygon drawing.
- Planning-only radar site placement and preset editing.
- Planning-only heuristic coverage visualization, blind spot hints, status values, legend, and reset controls.

This wave does not change runtime contracts, bridge behavior, entity semantics, scenario application, telemetry schemas, parser surfaces, or Gazebo/ROS behavior.

## UI-local boundary

Planning artifacts are explanatory only and remain browser UI state:

- Defense polygons are not written to runtime world state.
- Planning radar sites are not runtime entities.
- Planning radar placement does not call spawn commands.
- Coverage overlays do not call bridge commands.
- Planning Mode does not touch `apply_scenario` or session entity mutation paths.
- Planning artifacts are not runtime authority and are not SA replay authority.

Grid Mode remains the default and preserves existing entity editing semantics.

## Coverage limitations

Coverage is a visual estimate only. It uses simple 2D geometry from the completed defense polygon and UI-local radar detection ranges.

Coverage does not use:

- terrain masking
- LOS
- sensor truth
- live telemetry
- radar physics
- probability of detection
- operational readiness scoring

The UI labels the output as heuristic, a visual estimate, not validated sensing, and not runtime authority.

## Future work

Future Planning Mode work should remain additive and governance-safe:

- Optional export/import of planning artifacts only after a separate governance review.
- Better polygon/radar editing ergonomics without runtime persistence.
- Explicit planning artifact schemas if persistence is approved.
- Coverage improvements only if they keep clear visual-estimate wording and do not imply sensor truth.

## Freeze verdict

Frozen as a UI-local, explanatory Planning Mode surface. Runtime authority, bridge contracts, world state, scenario application, entity semantics, telemetry schemas, parser contracts, and SA replay authority remain unchanged.
