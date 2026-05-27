/** RT-only fictional georef anchor (display-only; not operational geography). */

export const FICTIONAL_GEOREF_ANCHOR = {
  lon_deg: 12.5,
  lat_deg: 41.9,
  h_m: 120,
} as const;

export const CESIUM_SCENARIO_CAVEAT =
  "Scenario-local fictional georef — not deployed geography. Cesium markers are pull mirrors only.";

/** Default camera offset above bounds center (meters). */
export const DEFAULT_CAMERA_HEIGHT_M = 1800;

export const BOUNDS_LAYER_ID = "rt-world-bounds";
