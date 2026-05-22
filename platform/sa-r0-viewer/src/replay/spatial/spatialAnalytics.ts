import type { ReplayMcSweep } from "../sweepSchema";
import type { ReplaySaBundle } from "../bundleSchema";

export type SpatialGrid = {
  origin_enu_m: [number, number];
  spacing_m: number;
  size: [number, number];
};

export type SpatialLayers = ReplayMcSweep["spatial_aggregate"]["layers"];

export function normalizeGrid(grid: SpatialGrid["origin_enu_m"] extends never ? never : ReplayMcSweep["spatial_aggregate"]["grid"]): SpatialGrid {
  const origin = grid.origin_enu_m;
  const size = grid.size;
  return {
    origin_enu_m: [Number(origin[0]), Number(origin[1])],
    spacing_m: Number(grid.spacing_m),
    size: [Number(size[0]), Number(size[1])],
  };
}

export function maxGridCount(layers: SpatialLayers): number {
  let max = 0;
  for (const key of ["ambiguity_density", "los_degraded", "topology_sensitivity", "intercept_outcome"] as const) {
    const layer = layers[key];
    if (layer && "counts" in layer) {
      for (const c of layer.counts) max = Math.max(max, c);
    }
  }
  return max || 1;
}

export function spatialFromBundle(bundle: ReplaySaBundle): SpatialLayers | null {
  const sa = (bundle as ReplaySaBundle & { spatial_analytics?: { layers: SpatialLayers } }).spatial_analytics;
  return sa?.layers ?? null;
}

export function spatialFromSweep(sweep: ReplayMcSweep): SpatialLayers {
  return sweep.spatial_aggregate.layers;
}
