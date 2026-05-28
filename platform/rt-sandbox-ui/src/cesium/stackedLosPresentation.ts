import type { Viewer } from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { isViewerUsable } from "./cesiumEditing";
import { syncLosSegmentLayer } from "./losSegmentLayer";
import { anyTerrainLayerEnabled, type TerrainLayerVisibility } from "./terrainLayers";
import type { VisualLayerVisibility } from "./visualLayerRegistry";
import { clearVisibilityWedgeLayer, syncVisibilityWedgeLayer } from "./visibilityWedgeLayer";

export interface StackedLosFlags {
  showStackedLos: boolean;
  showVisibilityWedge: boolean;
}

export function syncStackedLosPresentation(
  viewer: Viewer | null | undefined,
  selected: MirrorEntity | null,
  entities: MirrorEntity[],
  visibility: VisualLayerVisibility,
  terrainLayers: TerrainLayerVisibility,
): void {
  if (!isViewerUsable(viewer)) return;

  const terrainOn = anyTerrainLayerEnabled(terrainLayers);
  const wedgeFromStacked =
    visibility.showStackedLos && terrainOn && selected != null;
  const wedgeFromToggle = visibility.showVisibilityWedge && selected != null;
  const showWedge = wedgeFromStacked || wedgeFromToggle;

  if (showWedge) {
    syncVisibilityWedgeLayer(
      viewer,
      selected,
      true,
      terrainLayers.showTerrainMesh,
    );
  } else {
    clearVisibilityWedgeLayer(viewer);
  }

  if (visibility.showStackedLos && terrainOn && selected) {
    syncLosSegmentLayer(
      viewer,
      selected,
      entities,
      true,
      terrainLayers.showTerrainMesh,
      true,
    );
  }
}

export function shouldUseLegacyLosPath(
  visibility: VisualLayerVisibility,
  terrainLayers: TerrainLayerVisibility,
  selected: MirrorEntity | null,
): boolean {
  if (visibility.showStackedLos) return false;
  return anyTerrainLayerEnabled(terrainLayers) && selected != null;
}
