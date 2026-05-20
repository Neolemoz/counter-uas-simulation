import type { ReplaySaBundle } from "@/replay/bundleSchema";

export function sampleFictionalHeight(bundle: ReplaySaBundle, x_m: number, y_m: number): number {
  const model = bundle.scenario.terrain_model;
  if (!model || model.type !== "fictional_heightmap") return 0;
  const grid = model.grid_enu_m;
  const origin = grid.origin;
  const spacing = grid.spacing_m;
  const heights = grid.heights_m;
  const size = grid.size;
  const ox = origin[0] ?? 0;
  const oy = origin[1] ?? 0;
  const fx = (x_m - ox) / spacing;
  const fy = (y_m - oy) / spacing;
  if (fx < 0 || fy < 0 || fx >= size - 1 || fy >= heights.length - 1) return 0;
  const ix = Math.floor(fx);
  const iy = Math.floor(fy);
  const tx = fx - ix;
  const ty = fy - iy;
  const h00 = heights[iy]?.[ix] ?? 0;
  const h10 = heights[iy]?.[Math.min(ix + 1, size - 1)] ?? 0;
  const h01 = heights[Math.min(iy + 1, heights.length - 1)]?.[ix] ?? 0;
  const h11 = heights[Math.min(iy + 1, heights.length - 1)]?.[Math.min(ix + 1, size - 1)] ?? 0;
  return (1 - tx) * (1 - ty) * h00 + tx * (1 - ty) * h10 + (1 - tx) * ty * h01 + tx * ty * h11;
}

export function applyTerrainOffset(
  bundle: ReplaySaBundle,
  x_m: number,
  y_m: number,
  z_m: number,
  exaggeration = 1.0,
): number {
  return z_m + sampleFictionalHeight(bundle, x_m, y_m) * exaggeration;
}

export function hasFictionalTerrain(bundle: ReplaySaBundle): boolean {
  return bundle.scenario.terrain_model?.type === "fictional_heightmap";
}
