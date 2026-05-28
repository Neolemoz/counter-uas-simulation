import type { Viewer } from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { clearEnvironmentMarkers, syncEnvironmentMarkers } from "./environmentMarkers";
import { clearSensorDomeLayer, syncSensorDomeLayer } from "./sensorDomeLayer";
import { clearTerrainContourLayer, syncTerrainContourLayer } from "./terrainContourLayer";
import { clearTerrainMeshLayer, syncTerrainMeshLayer } from "./terrainMeshLayer";
import { clearTerrainOverlays, syncTerrainOverlays } from "./terrainOverlays";

export interface TerrainLayerVisibility {
  showTerrainMesh: boolean;
  showRidgeOverlays: boolean;
  showContourOverlays: boolean;
  showVegetationMarkers: boolean;
  showEnvironmentMarkers: boolean;
  showSensorDomes: boolean;
}

import {
  defaultVisibilityFromRegistry,
  toTerrainLayerVisibility,
} from "./visualLayerRegistry";

export const DEFAULT_TERRAIN_LAYERS: TerrainLayerVisibility = toTerrainLayerVisibility(
  defaultVisibilityFromRegistry(),
);

export function syncTerrainLayers(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[],
  layers: TerrainLayerVisibility,
): void {
  syncTerrainMeshLayer(viewer, layers.showTerrainMesh);
  syncTerrainOverlays(viewer, layers.showRidgeOverlays, layers.showTerrainMesh);
  syncTerrainContourLayer(viewer, layers.showContourOverlays);
  syncEnvironmentMarkers(viewer, {
    showOcclusion: layers.showEnvironmentMarkers,
    showVegetation: layers.showVegetationMarkers,
  });
  syncSensorDomeLayer(
    viewer,
    entities,
    layers.showSensorDomes,
    layers.showTerrainMesh,
  );
}

export function clearAllTerrainLayers(viewer: Viewer | null | undefined): void {
  clearTerrainMeshLayer(viewer);
  clearTerrainOverlays(viewer);
  clearTerrainContourLayer(viewer);
  clearEnvironmentMarkers(viewer);
  clearSensorDomeLayer(viewer);
}

export function anyTerrainLayerEnabled(layers: TerrainLayerVisibility): boolean {
  return (
    layers.showTerrainMesh ||
    layers.showRidgeOverlays ||
    layers.showContourOverlays ||
    layers.showVegetationMarkers ||
    layers.showEnvironmentMarkers ||
    layers.showSensorDomes
  );
}

export function activeTerrainLayerLabels(layers: TerrainLayerVisibility): string[] {
  const labels: string[] = [];
  if (layers.showTerrainMesh) labels.push("mesh");
  if (layers.showRidgeOverlays) labels.push("ridges");
  if (layers.showContourOverlays) labels.push("contours");
  if (layers.showVegetationMarkers) labels.push("vegetation");
  if (layers.showEnvironmentMarkers) labels.push("occlusion");
  if (layers.showSensorDomes) labels.push("domes");
  return labels;
}
