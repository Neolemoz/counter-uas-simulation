# RT Tactical Visualization Smoke Checklist v1

Manual validation for unified ±7000 m tactical visualization (display-only).

**Fixture reference:** [fixtures/rt_visualization/tactical_view_7km_golden_v1.json](../../fixtures/rt_visualization/tactical_view_7km_golden_v1.json)

**Governance:** Tactical overlays are visualization only — no command authority, no intercept assignment authority, and no autonomous engagement authority.

## Preconditions

- RT sandbox UI running with an active session (`platform/rt-sandbox-ui`)
- Cesium globe connected to loopback bridge
- Unified world bounds ±7000 m visible (optional: enable bounds layer)

## Scenario setup

1. Spawn **defender** (`interceptor`) inside the defense ring, e.g. near `(400, -200)` — within 3000 m of origin.
2. Spawn **attacker** (`drone`) in the spawn band, e.g. near `(6000, 3500)` — between 5000 m and 7000 m from origin.
3. In tactical panel (manual mode): select interceptor and target, then assign (or select-only to verify provisional geometry).

## Enable tactical view

1. Click **Enable Tactical View** on the Cesium toolbar (`data-testid="enable-tactical-view"`).
2. Confirm governance strip appears: *visualization only; no command authority; no autonomous engagement*.
3. Confirm layers enabled together:
   - Tactical path
   - Solution point
   - Timing labels
   - Threat corridor
   - Tactical target emphasis
   - Ranking cues
4. Confirm **Tactical compare** remains off unless manually toggled.

## Verify geometry (city-core camera)

1. Use **Tight bounds** or **Focus** on defender — city-core camera (~2200 m height).
2. **Predicted path:** dashed telemetry path from defender toward intercept (~2800, 1800); width readable at medium zoom.
3. **Solution point:** marker at intercept with “solution point · display only” label.
4. **Threat corridor:** ribbon from attacker spawn-band position toward solution; visible but not oversized.
5. **ETA / TTI:** timing label near solution when assignment/selection provides `tti_s` / `eta_s`.
6. **Tactical target emphasis:** halo/label on assigned/selected attacker (unless same entity is edit-selected).
7. **Ranking cue:** `#1` (or recommendation cue) label on primary target when ranking/recommendation telemetry exists.

## Verify geometry (world-fit camera)

1. Use **Reset** or **Fit** to frame full ±7000 m world (~10150 m camera height).
2. Path, corridor, and labels remain visible at long range (scaled widths/offsets — see fixture `camera_validation`).
3. Attacker in spawn band and defender inside defense ring both visible relative to operational rings (if bounds/rings enabled).

## Negative checks (governance)

- Overlays do **not** change bridge assignment, mode, or autonomous loop state.
- Toggling layers off returns globe to prior state without side effects.
- No SA import, MC execution, or runtime command side effects from visualization alone.

## Pass criteria

- All six preset layers render for assigned defender/target pair at spawn-band range.
- Intercept derives from `predicted_path_enu_m` endpoint when `last_intercept_pose` absent (selection-only smoke).
- Corridor spans attacker → solution at 7 km world scale.
- Governance copy visible when tactical layers active.
- No bridge/runtime/MC/SA mutations observed.

## Automated companion

Run: `npm test -- tactical` (includes `tacticalViewGoldenFixture.test.ts`).
