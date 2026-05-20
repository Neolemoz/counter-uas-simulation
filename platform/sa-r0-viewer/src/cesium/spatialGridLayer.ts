import { Cartesian3, Color, Entity, Viewer } from "cesium";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import type { ReplayMcSweep } from "@/replay/sweepSchema";
import type { SpatialDeclutterMode } from "@/replay/useSweepStore";
import type { SpatialLayers } from "@/replay/spatial/spatialAnalytics";
import { maxGridCount, normalizeGrid } from "@/replay/spatial/spatialAnalytics";
import { enuToCartographic } from "./coordinates";
import { applyTerrainOffset, hasFictionalTerrain } from "./fictionalTerrain";

export const SPATIAL_LAYER_CAVEAT =
  "Replay concentration overlay — explanatory only, not operational prediction.";

function toCartesian3(bundle: ReplaySaBundle, x: number, y: number, z = 0): Cartesian3 {
  const zAdj = hasFictionalTerrain(bundle) ? applyTerrainOffset(bundle, x, y, z, 0.2) : z;
  const c = enuToCartographic(bundle, x, y, zAdj);
  return Cartesian3.fromRadians(c.longitude, c.latitude, c.height);
}

type SpatialLayerKey = "spatialAmbiguity" | "spatialLos" | "spatialSensitivity";

const LAYER_MAP: Record<SpatialLayerKey, keyof SpatialLayers> = {
  spatialAmbiguity: "ambiguity_density",
  spatialLos: "los_degraded",
  spatialSensitivity: "topology_sensitivity",
};

const LAYER_COLORS: Record<SpatialLayerKey, string> = {
  spatialAmbiguity: "#a78bfa",
  spatialLos: "#f87171",
  spatialSensitivity: "#fbbf24",
};

function visibleCellIndices(
  counts: number[],
  declutter: SpatialDeclutterMode,
): Set<number> {
  const nonzero = counts
    .map((c, i) => ({ c, i }))
    .filter((x) => x.c > 0);
  if (!nonzero.length || declutter === "off") {
    return new Set(nonzero.map((x) => x.i));
  }
  const sorted = [...nonzero].sort((a, b) => b.c - a.c);
  if (declutter === "top_k") {
    return new Set(sorted.slice(0, 12).map((x) => x.i));
  }
  const vals = sorted.map((x) => x.c);
  const p75 = vals[Math.floor(vals.length * 0.25)] ?? vals[0]!;
  return new Set(sorted.filter((x) => x.c >= p75).map((x) => x.i));
}

export function syncSpatialGridLayer(
  viewer: Viewer,
  bundle: ReplaySaBundle,
  grid: ReplayMcSweep["spatial_aggregate"]["grid"],
  layers: SpatialLayers,
  visibility: Record<SpatialLayerKey, boolean>,
  entityTag: string,
  declutter: SpatialDeclutterMode = "off",
): void {
  const existing = viewer.entities.values.filter((e) => e.id?.startsWith(entityTag));
  for (const e of existing) viewer.entities.remove(e);

  const norm = normalizeGrid(grid);
  const max = maxGridCount(layers);
  const [cols, rows] = norm.size;
  const [ox, oy] = norm.origin_enu_m;
  const spacing = norm.spacing_m;

  for (const visKey of Object.keys(LAYER_MAP) as SpatialLayerKey[]) {
    if (!visibility[visKey]) continue;
    const dataKey = LAYER_MAP[visKey];
    const layer = layers[dataKey];
    if (!layer || !("counts" in layer)) continue;
    const counts = layer.counts;
    const visible = visibleCellIndices(counts, declutter);
    const baseColor = Color.fromCssColorString(LAYER_COLORS[visKey]);

    for (let idx = 0; idx < counts.length; idx++) {
      const count = counts[idx] ?? 0;
      if (count <= 0 || !visible.has(idx)) continue;
      const row = Math.floor(idx / cols);
      const col = idx % cols;
      if (row >= rows) continue;
      const alpha = 0.15 + 0.55 * (count / max);
      const x0 = ox + col * spacing;
      const y0 = oy + row * spacing;
      const positions = [
        toCartesian3(bundle, x0, y0),
        toCartesian3(bundle, x0 + spacing, y0),
        toCartesian3(bundle, x0 + spacing, y0 + spacing),
        toCartesian3(bundle, x0, y0 + spacing),
      ];
      viewer.entities.add(
        new Entity({
          id: `${entityTag}-${visKey}-${col}-${row}`,
          polygon: {
            hierarchy: positions,
            material: baseColor.withAlpha(alpha),
            outline: false,
          },
          description: SPATIAL_LAYER_CAVEAT,
        }),
      );
    }
  }

  const fd = layers.first_detection;
  if (fd?.points_enu_m) {
    for (let i = 0; i < fd.points_enu_m.length; i++) {
      const pt = fd.points_enu_m[i];
      if (!pt || pt.length < 2) continue;
      viewer.entities.add(
        new Entity({
          id: `${entityTag}-fd-${i}`,
          position: toCartesian3(bundle, Number(pt[0]), Number(pt[1])),
          point: {
            pixelSize: 8,
            color: Color.CYAN.withAlpha(0.85),
            outlineColor: Color.WHITE,
            outlineWidth: 1,
          },
          description: "First-detection replay point — explanatory only.",
        }),
      );
    }
  }
}
