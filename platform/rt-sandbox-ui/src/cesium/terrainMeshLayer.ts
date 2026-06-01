import { Color, Entity, Viewer } from "cesium";
import { WORLD_BOUNDS } from "@/world/bounds";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { RT_RIDGE_TERRAIN, sampleTerrainHeight } from "./rtFictionalTerrain";

const MESH_PREFIX = "rt-terrain-mesh-";

function removeMeshEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(MESH_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

/** Coarse terrain mesh as shaded ground quads (display-only). */
export function syncTerrainMeshLayer(
  viewer: Viewer | null | undefined,
  show: boolean,
): void {
  if (!isViewerUsable(viewer)) return;
  removeMeshEntities(viewer);
  if (!show) return;

  const grid = RT_RIDGE_TERRAIN.grid_enu_m;
  const ox = grid.origin[0];
  const oy = grid.origin[1];
  const step = grid.spacing_m;
  const { x, y } = WORLD_BOUNDS;

  let quadIndex = 0;
  for (let iy = 0; iy < grid.size - 1; iy++) {
    for (let ix = 0; ix < grid.size - 1; ix++) {
      const x0 = ox + ix * step;
      const y0 = oy + iy * step;
      const x1 = x0 + step;
      const y1 = y0 + step;
      if (x1 < x.min || x0 > x.max || y1 < y.min || y0 > y.max) continue;

      const h00 = sampleTerrainHeight(x0, y0);
      const h10 = sampleTerrainHeight(x1, y0);
      const h01 = sampleTerrainHeight(x0, y1);
      const h11 = sampleTerrainHeight(x1, y1);
      const avg = (h00 + h10 + h01 + h11) / 4;
      const alpha = 0.035 + Math.min(0.09, avg / 420);
      const warm = Math.round(95 + Math.min(45, avg / 3));
      const cool = Math.round(92 + Math.min(35, avg / 4));

      viewer.entities.add(
        new Entity({
          id: `${MESH_PREFIX}${quadIndex++}`,
          polygon: {
            hierarchy: [
              worldToCartesian(x0, y0, h00),
              worldToCartesian(x1, y0, h10),
              worldToCartesian(x1, y1, h11),
              worldToCartesian(x0, y1, h01),
            ],
            material: Color.fromCssColorString(
              `rgba(${warm}, ${cool}, 82, ${alpha.toFixed(3)})`,
            ),
            outline: false,
            perPositionHeight: true,
          },
        }),
      );
    }
  }
}

export function clearTerrainMeshLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeMeshEntities(viewer);
}
