# RT Planning Terrain Visualization V1

Status: frozen

## Scope

Planning Terrain Visualization V1 adds presentation-only terrain and location
controls to RT Sandbox Planning Mode. It is bounded to browser-side Cesium
visualization and Planning Mode UX.

Included:

- Terrain provider selection with ellipsoid as the default.
- Optional Cesium 3D terrain provider path.
- Planning Mode terrain on/off, source display, and status indicator.
- Planning camera preset controls that reuse existing Cesium camera helpers.
- Planning location presets for Bangkok, Chiang Mai, Phuket, and custom
  latitude/longitude entry.
- Visual-only governance copy for terrain and real-world locations.
- Tests for provider defaults, optional terrain path, Planning Mode visibility,
  location presets, custom coordinate validation, and governance wording.

Excluded:

- Runtime state mutation.
- Bridge command or API changes.
- Planning metric changes.
- LOS behavior changes.
- Sensor behavior changes.
- Monte Carlo behavior changes.
- Telemetry, schema, capture, or parser changes.
- Persistence of custom coordinates.

## Architecture Summary

The implementation keeps Planning Mode UI-local. Terrain and location controls
live in the RT Sandbox UI and feed only Cesium viewer configuration or camera
requests.

The relevant UI path is:

`App.tsx -> AppWorkstationSlots -> PlanningModePanel -> CesiumRuntimePanel -> CesiumRuntimeView`

Planning Mode artifacts remain local UI state:

- defense polygon state
- planning radar sites
- heuristic coverage cells
- blind spot markers

The existing planar coverage calculation remains unchanged in
`planningDrawing.ts`. Location presets and terrain provider mode do not enter
that calculation.

## Terrain Provider Model

`terrainProviderConfig.ts` defines the provider model:

- `ellipsoid`: default, existing behavior.
- `cesium_world_terrain`: optional 3D terrain path using Cesium World Terrain.

The provider mode is selected in UI state and passed to Cesium viewer
initialization. The terrain provider changes the visual globe surface only. It
does not become an authority source for runtime poses, planning metrics, LOS,
sensors, capture, telemetry, or MC.

## Planning Mode Integration

Planning Mode surfaces the terrain model through:

- Terrain On/Off.
- Terrain Source display.
- Terrain status indicator.
- Terrain governance legend.

Camera controls surface existing helpers only:

- Overview -> `terrainOverview`
- Ridge -> `ridgeLine`
- Valley -> `valleyFloor`
- Sensor Context -> `sensorContext`

Location presets are presentation-only camera recenter requests:

- Bangkok
- Chiang Mai
- Phuket
- Custom Coordinates

Custom latitude/longitude values are validated in the UI only. Invalid custom
coordinates block the camera jump affordance. No coordinates are persisted and
no runtime/world state is updated.

## Visual Validation Review

Validated by source inspection and automated tests:

| Surface | Ellipsoid Terrain | 3D Terrain |
|---------|-------------------|------------|
| Terrain toggle | Default/off path preserved; optional path selectable | Optional/on path selectable |
| Camera presets | Existing helper calls surfaced in Planning Mode | Same helper calls; terrain remains visual-only |
| Planning polygon | Existing Cesium entity path unchanged | Existing Cesium entity path unchanged |
| Radar sites | Existing Planning radar entity path unchanged | Existing Planning radar entity path unchanged |
| Coverage overlay | Existing planar coverage cells unchanged | Existing planar coverage cells unchanged |
| Blind spot markers | Existing marker rendering unchanged | Existing marker rendering unchanged |
| Location jumps | Camera-only recenter request | Camera-only recenter request |

Limitations:

- No live browser screenshot or manual Cesium scene inspection was captured in
  this freeze note.
- 3D terrain availability depends on Cesium terrain provider/network behavior.
- Planning overlays remain local ENU display overlays; they do not reproject or
  recompute against real-world terrain heights.
- Real-world location presets are presentation framing only and do not alter the
  fictional sandbox georef, world registry, or Planning Mode metric model.

## Governance Audit

Confirmed boundaries:

- Terrain remains presentation-only.
- Real-world locations remain presentation-only.
- No runtime mutation.
- No bridge changes.
- No LOS behavior changes.
- No sensor effects.
- No MC effects.
- No telemetry changes.
- No schema changes.
- No parser, capture, or replay contract changes.

Authoritative state remains in the existing RT session/runtime surfaces.
Planning terrain and locations are explanatory browser presentation only.

## Validation

Commands run:

- `npm test`
- `npm run build`
- `git diff --check`

Validation result at freeze: pass.

Build note: Vite still emits the existing chunk-size warning.

## Future Work

Future work must remain additive and separately scoped:

- Manual browser screenshot validation for both terrain modes.
- Optional screenshot-based regression checks for polygon, radar, coverage, and
  blind spot readability.
- More location presets if needed for presentation demos.
- UI polish for provider availability/error messaging.
- Documentation of Cesium terrain provider token/network expectations.

Future work must not couple terrain or real-world locations to runtime state,
LOS, sensors, MC, telemetry, capture, parser contracts, or bridge commands
without a separate governed plan.
