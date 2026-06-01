import {
  Color,
  Entity,
  LabelStyle,
  PolylineDashMaterialProperty,
  Viewer,
} from "cesium";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import {
  RT_RIDGE_TERRAIN,
  sampleTerrainHeight,
} from "./rtFictionalTerrain";

const OVERLAY_PREFIX = "rt-terrain-overlay-";

function removeOverlayEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(OVERLAY_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

export function syncTerrainOverlays(
  viewer: Viewer | null | undefined,
  show: boolean,
  showElevationBands = true,
): void {
  if (!isViewerUsable(viewer)) return;
  removeOverlayEntities(viewer);
  if (!show) return;

  for (const ridge of RT_RIDGE_TERRAIN.ridge_features) {
    const positions = ridge.polyline_enu_m.map(([x, y, zHint]) => {
      const z = zHint > 0 ? zHint : sampleTerrainHeight(x, y) + 2;
      return worldToCartesian(x, y, z);
    });
    const avgZ =
      ridge.polyline_enu_m.reduce((s, p) => s + (p[2] > 0 ? p[2] : sampleTerrainHeight(p[0], p[1])), 0) /
      Math.max(1, ridge.polyline_enu_m.length);
    const bandColor = avgZ > 55 ? "rgba(180, 134, 73, 0.24)" : "rgba(148, 163, 184, 0.2)";

    viewer.entities.add(
      new Entity({
        id: `${OVERLAY_PREFIX}ridge-${ridge.ridge_id}`,
        polyline: {
          positions,
          width: showElevationBands ? 3 : 2,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString(bandColor),
            dashLength: 18,
          }),
        },
        label: {
          text: `${ridge.label} (explanatory)`,
          font: "9px sans-serif",
          fillColor: Color.fromCssColorString("rgba(203, 213, 225, 0.58)"),
          outlineColor: Color.BLACK,
          outlineWidth: 1,
          style: LabelStyle.FILL_AND_OUTLINE,
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.45)"),
        },
      }),
    );
  }
}

export function clearTerrainOverlays(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeOverlayEntities(viewer);
}
