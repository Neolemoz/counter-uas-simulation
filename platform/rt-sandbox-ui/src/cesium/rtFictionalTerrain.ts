import terrainProfile from "./fixtures/rt_ridge_terrain_v1.json";
export const TERRAIN_CAUTION =
  "Fictional sandbox terrain — not deployed geography or sensor truth";

export const CONTOUR_CAUTION = "Explanatory contour — not survey data";

export const DEFAULT_TERRAIN_EXAGGERATION = 1.0;

export const NOMINAL_SENSOR_DOME_RADIUS_M = 200;

export interface RidgeFeature {
  ridge_id: string;
  label: string;
  polyline_enu_m: [number, number, number][];
}

export interface OcclusionMarker {
  marker_id: string;
  position_enu_m: [number, number, number];
  label: string;
}

export type VegetationKind = "scrub" | "canopy";

export interface VegetationMarker {
  marker_id: string;
  position_enu_m: [number, number, number];
  kind: VegetationKind;
  label: string;
}

export interface ContourPolyline {
  level_m: number;
  points_enu_m: [number, number][];
}

export interface RtFictionalTerrainProfile {
  schema: string;
  caveat: string;
  elevation_bands_m?: number[];
  grid_enu_m: {
    origin: [number, number];
    spacing_m: number;
    size: number;
    heights_m: number[][];
  };
  ridge_features: RidgeFeature[];
  occlusion_markers: OcclusionMarker[];
  vegetation_markers?: VegetationMarker[];
}

export const RT_RIDGE_TERRAIN = terrainProfile as RtFictionalTerrainProfile;

const BAND_COLORS = [
  "rgba(120, 113, 108, 0.75)",
  "rgba(161, 98, 7, 0.8)",
  "rgba(180, 83, 9, 0.85)",
  "rgba(217, 119, 6, 0.88)",
  "rgba(251, 191, 36, 0.92)",
] as const;

export function hasRtFictionalTerrain(): boolean {
  return RT_RIDGE_TERRAIN.schema === "rt_fictional_terrain_v1";
}

export function listElevationBandsM(): number[] {
  return RT_RIDGE_TERRAIN.elevation_bands_m ?? [10, 20, 30, 40];
}

/** Bilinear sample of terrain height at ENU x/y (meters). */
export function sampleTerrainHeight(x_m: number, y_m: number): number {
  if (!hasRtFictionalTerrain()) return 0;
  const grid = RT_RIDGE_TERRAIN.grid_enu_m;
  const ox = grid.origin[0] ?? 0;
  const oy = grid.origin[1] ?? 0;
  const spacing = grid.spacing_m;
  const heights = grid.heights_m;
  const size = grid.size;
  const fx = (x_m - ox) / spacing;
  const fy = (y_m - oy) / spacing;
  if (fx < 0 || fy < 0 || fx >= size - 1 || fy >= heights.length - 1) {
    return 0;
  }
  const ix = Math.floor(fx);
  const iy = Math.floor(fy);
  const tx = fx - ix;
  const ty = fy - iy;
  const h00 = heights[iy]?.[ix] ?? 0;
  const h10 = heights[iy]?.[Math.min(ix + 1, size - 1)] ?? 0;
  const h01 = heights[Math.min(iy + 1, heights.length - 1)]?.[ix] ?? 0;
  const h11 =
    heights[Math.min(iy + 1, heights.length - 1)]?.[Math.min(ix + 1, size - 1)] ?? 0;
  return (1 - tx) * (1 - ty) * h00 + tx * (1 - ty) * h10 + (1 - tx) * ty * h01 + tx * ty * h11;
}

/** Visual-only globe z: registry z is AGL above flat plane; add local terrain when enabled. */
export function applyTerrainDisplayOffset(
  x_m: number,
  y_m: number,
  registry_z_m: number,
  exaggeration = DEFAULT_TERRAIN_EXAGGERATION,
): number {
  return registry_z_m + sampleTerrainHeight(x_m, y_m) * exaggeration;
}

export function displayAglM(x_m: number, y_m: number, registry_z_m: number): number {
  const terrain = sampleTerrainHeight(x_m, y_m);
  return registry_z_m - terrain;
}

export function nearestRidgeLabel(x_m: number, y_m: number): string | null {
  let best: { label: string; dist2: number } | null = null;
  for (const ridge of RT_RIDGE_TERRAIN.ridge_features) {
    for (const pt of ridge.polyline_enu_m) {
      const dx = pt[0] - x_m;
      const dy = pt[1] - y_m;
      const d2 = dx * dx + dy * dy;
      if (!best || d2 < best.dist2) {
        best = { label: ridge.label, dist2: d2 };
      }
    }
  }
  return best?.label ?? null;
}

export function contourLevelUnderM(x_m: number, y_m: number): number | null {
  const h = sampleTerrainHeight(x_m, y_m);
  const bands = listElevationBandsM();
  let under: number | null = null;
  for (const level of bands) {
    if (h >= level) under = level;
  }
  return under;
}

export function ridgeElevationBandColor(height_m: number): string {
  const bands = listElevationBandsM();
  let idx = 0;
  for (const level of bands) {
    if (height_m >= level) idx += 1;
  }
  return BAND_COLORS[Math.min(idx, BAND_COLORS.length - 1)];
}

export function listOcclusionMarkers(): OcclusionMarker[] {
  return RT_RIDGE_TERRAIN.occlusion_markers ?? [];
}

export function listVegetationMarkers(): VegetationMarker[] {
  return RT_RIDGE_TERRAIN.vegetation_markers ?? [];
}

export function primaryRidgePolyline(): [number, number, number][] {
  const north = RT_RIDGE_TERRAIN.ridge_features.find((r) => r.ridge_id === "north_ridge");
  return north?.polyline_enu_m ?? RT_RIDGE_TERRAIN.ridge_features[0]?.polyline_enu_m ?? [];
}

export function valleyFloorPolyline(): [number, number, number][] {
  const valley = RT_RIDGE_TERRAIN.ridge_features.find((r) => r.ridge_id === "valley_floor");
  return valley?.polyline_enu_m ?? [];
}

function interpEdge(
  level: number,
  ha: number,
  hb: number,
  xa: number,
  ya: number,
  xb: number,
  yb: number,
): [number, number] | null {
  if ((ha - level) * (hb - level) > 0) return null;
  if (Math.abs(ha - hb) < 1e-6) return [xa, ya];
  const t = (level - ha) / (hb - ha);
  return [xa + (xb - xa) * t, ya + (yb - ya) * t];
}

function collectRowContoursAtLevel(level: number): [number, number][][] {
  const grid = RT_RIDGE_TERRAIN.grid_enu_m;
  const ox = grid.origin[0];
  const oy = grid.origin[1];
  const step = grid.spacing_m;
  const heights = grid.heights_m;
  const size = grid.size;
  const polylines: [number, number][][] = [];

  for (let iy = 0; iy < size; iy++) {
    let current: [number, number][] = [];
    const y = oy + iy * step;
    for (let ix = 0; ix < size - 1; ix++) {
      const x0 = ox + ix * step;
      const x1 = x0 + step;
      const h0 = heights[iy]?.[ix] ?? 0;
      const h1 = heights[iy]?.[ix + 1] ?? 0;
      const pt = interpEdge(level, h0, h1, x0, y, x1, y);
      if (pt) {
        current.push(pt);
      } else if (current.length >= 2) {
        polylines.push(current);
        current = [];
      }
    }
    if (current.length >= 2) polylines.push(current);
  }

  for (let ix = 0; ix < size; ix++) {
    let current: [number, number][] = [];
    const x = ox + ix * step;
    for (let iy = 0; iy < size - 1; iy++) {
      const y0 = oy + iy * step;
      const y1 = y0 + step;
      const h0 = heights[iy]?.[ix] ?? 0;
      const h1 = heights[iy + 1]?.[ix] ?? 0;
      const pt = interpEdge(level, h0, h1, x, y0, x, y1);
      if (pt) {
        current.push(pt);
      } else if (current.length >= 2) {
        polylines.push(current);
        current = [];
      }
    }
    if (current.length >= 2) polylines.push(current);
  }

  return polylines;
}

/** Iso-height contour segments from the fictional heightmap (explanatory only). */
export function generateContourPolylines(levels?: number[]): ContourPolyline[] {
  const targetLevels = levels ?? listElevationBandsM();
  const out: ContourPolyline[] = [];
  for (const level of targetLevels) {
    for (const points of collectRowContoursAtLevel(level)) {
      if (points.length >= 2) {
        out.push({ level_m: level, points_enu_m: points });
      }
    }
  }
  return out;
}
