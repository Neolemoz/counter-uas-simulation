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
      point: "rgba(34, 197, 94, 0.9)",
      outline: "rgba(21, 128, 61, 0.95)",
    };
  }
  return {
    point: "rgba(132, 204, 22, 0.85)",
    outline: "rgba(77, 124, 15, 0.9)",
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
            pixelSize: 10,
            color: Color.fromCssColorString("rgba(74, 222, 128, 0.85)"),
            outlineColor: Color.fromCssColorString("rgba(22, 101, 52, 0.9)"),
            outlineWidth: 2,
          },
          label: {
            text: marker.label,
            font: "10px sans-serif",
            fillColor: Color.fromCssColorString("rgba(187, 247, 208, 0.95)"),
            outlineColor: Color.BLACK,
            outlineWidth: 2,
            style: LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: VerticalOrigin.BOTTOM,
            showBackground: true,
            backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.88)"),
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
            pixelSize: 12,
            color: Color.fromCssColorString(colors.point),
            outlineColor: Color.fromCssColorString(colors.outline),
            outlineWidth: 2,
          },
          label: {
            text: marker.label,
            font: "10px sans-serif",
            fillColor: Color.fromCssColorString("rgba(190, 242, 100, 0.95)"),
            outlineColor: Color.BLACK,
            outlineWidth: 2,
            style: LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: VerticalOrigin.BOTTOM,
            showBackground: true,
            backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.88)"),
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
