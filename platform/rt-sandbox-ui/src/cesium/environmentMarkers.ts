import { Color, Entity, LabelStyle, VerticalOrigin, Viewer } from "cesium";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import {
  listOcclusionMarkers,
  listVegetationMarkers,
  sampleTerrainHeight,
  type VegetationKind,
} from "./rtFictionalTerrain";

const ENV_PREFIX = "rt-terrain-env-";

function removeEnvEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(ENV_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function vegetationColor(kind: VegetationKind): { point: string; outline: string } {
  if (kind === "canopy") {
    return {
      point: "rgba(120, 113, 108, 0.32)",
      outline: "rgba(168, 162, 158, 0.38)",
    };
  }
  return {
    point: "rgba(161, 98, 7, 0.24)",
    outline: "rgba(202, 138, 4, 0.34)",
  };
}

export function syncEnvironmentMarkers(
  viewer: Viewer | null | undefined,
  options: { showOcclusion: boolean; showVegetation: boolean },
): void {
  if (!isViewerUsable(viewer)) return;
  removeEnvEntities(viewer);

  if (options.showOcclusion) {
    for (const marker of listOcclusionMarkers()) {
      const [x, y] = marker.position_enu_m;
      const z = sampleTerrainHeight(x, y) + 8;
      viewer.entities.add(
        new Entity({
          id: `${ENV_PREFIX}occ-${marker.marker_id}`,
          position: worldToCartesian(x, y, z),
          point: {
            pixelSize: 7,
            color: Color.fromCssColorString("rgba(148, 163, 184, 0.28)"),
            outlineColor: Color.fromCssColorString("rgba(203, 213, 225, 0.34)"),
            outlineWidth: 1,
          },
          label: {
            text: marker.label,
            font: "9px sans-serif",
            fillColor: Color.fromCssColorString("rgba(203, 213, 225, 0.54)"),
            outlineColor: Color.BLACK,
            outlineWidth: 1,
            style: LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: VerticalOrigin.BOTTOM,
            showBackground: true,
            backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.42)"),
          },
        }),
      );
    }
  }

  if (options.showVegetation) {
    for (const marker of listVegetationMarkers()) {
      const [x, y] = marker.position_enu_m;
      const z = sampleTerrainHeight(x, y) + 6;
      const colors = vegetationColor(marker.kind);
      viewer.entities.add(
        new Entity({
          id: `${ENV_PREFIX}veg-${marker.marker_id}`,
          position: worldToCartesian(x, y, z),
          point: {
            pixelSize: 7,
            color: Color.fromCssColorString(colors.point),
            outlineColor: Color.fromCssColorString(colors.outline),
            outlineWidth: 1,
          },
          label: {
            text: marker.label,
            font: "9px sans-serif",
            fillColor: Color.fromCssColorString("rgba(217, 119, 6, 0.5)"),
            outlineColor: Color.BLACK,
            outlineWidth: 1,
            style: LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: VerticalOrigin.BOTTOM,
            showBackground: true,
            backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.42)"),
          },
        }),
      );
    }
  }
}

export function clearEnvironmentMarkers(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeEnvEntities(viewer);
}
