import { Color, Entity, PolylineDashMaterialProperty, Viewer } from "cesium";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { CONTOUR_CAUTION, generateContourPolylines } from "./rtFictionalTerrain";

const CONTOUR_PREFIX = "rt-terrain-contour-";

function removeContourEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(CONTOUR_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

export function syncTerrainContourLayer(
  viewer: Viewer | null | undefined,
  show: boolean,
): void {
  if (!isViewerUsable(viewer)) return;
  removeContourEntities(viewer);
  if (!show) return;

  let idx = 0;
  let labeled = false;
  for (const contour of generateContourPolylines()) {
    const positions = contour.points_enu_m.map(([x, y]) =>
      worldToCartesian(x, y, contour.level_m + 0.5),
    );
    viewer.entities.add(
      new Entity({
        id: `${CONTOUR_PREFIX}${idx++}`,
        polyline: {
          positions,
          width: 1.5,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString("rgba(148, 163, 184, 0.55)"),
            dashLength: 6,
          }),
        },
        label:
          !labeled
            ? {
                text: CONTOUR_CAUTION,
                font: "10px sans-serif",
                fillColor: Color.fromCssColorString("rgba(203, 213, 225, 0.9)"),
                showBackground: true,
                backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.85)"),
              }
            : undefined,
      }),
    );
    labeled = true;
  }
}

export function clearTerrainContourLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeContourEntities(viewer);
}
