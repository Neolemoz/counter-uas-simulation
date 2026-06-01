/** RT-only fictional georef anchor (display-only; not operational geography). */

export const FICTIONAL_GEOREF_ANCHOR = {
  // Passo Gardena / Sella Group, Dolomites, Italy (display anchor only).
  lon_deg: 11.806,
  lat_deg: 46.549,
  h_m: 2120,
} as const;

export const CESIUM_SCENARIO_CAVEAT =
  "Scenario-local fictional georef anchored near Passo Gardena / Sella Group, Dolomites — not deployed geography. Cesium markers are pull mirrors only.";

/** Default camera offset above bounds center (meters). */
export const DEFAULT_CAMERA_HEIGHT_M = 1880;

export const BOUNDS_LAYER_ID = "rt-world-bounds";
