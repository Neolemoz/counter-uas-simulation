import type { Viewer } from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import type { DefenseZoneRenderOptions } from "./defenseZoneConfig";
import { clearDefenseZoneLayer, syncDefenseZoneLayer } from "./defenseZoneLayer";
import { clearEnvironmentMarkers, syncEnvironmentMarkers } from "./environmentMarkers";
import { clearSensorDomeLayer, radarPreviewLayerActive, syncSensorDomeLayer, type SensorDomeRenderOptions } from "./sensorDomeLayer";
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
  shouldShowDefenseZones,
  shouldShowRadarZones,
  type SensorDomeZoneMode,
} from "./sensorDomeZoneMode";

import {
  defaultVisibilityFromRegistry,
  toTerrainLayerVisibility,
} from "./visualLayerRegistry";

export type { SensorDomeZoneMode };
export { DEFAULT_SENSOR_DOME_ZONE_MODE } from "./sensorDomeZoneMode";

export const DEFAULT_TERRAIN_LAYERS: TerrainLayerVisibility = toTerrainLayerVisibility(
  defaultVisibilityFromRegistry(),
);

export function syncTerrainLayers(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[],
  layers: TerrainLayerVisibility,
  sensorDomeOptions: SensorDomeRenderOptions = {},
  defenseZoneOptions: DefenseZoneRenderOptions = {},
  zoneMode: SensorDomeZoneMode = "both",
): void {
  syncTerrainMeshLayer(viewer, layers.showTerrainMesh);
  syncTerrainOverlays(viewer, layers.showRidgeOverlays, layers.showTerrainMesh);
  syncTerrainContourLayer(viewer, layers.showContourOverlays);
  syncEnvironmentMarkers(viewer, {
    showOcclusion: layers.showEnvironmentMarkers,
    showVegetation: layers.showVegetationMarkers,
  });
  const domesOn = layers.showSensorDomes;
  const applyTerrainGrounding = shouldApplyTerrainGrounding(layers);
  syncSensorDomeLayer(
    viewer,
    entities,
    domesOn && shouldShowRadarZones(zoneMode) && radarPreviewLayerActive(sensorDomeOptions),
    applyTerrainGrounding,
    sensorDomeOptions,
  );
  syncDefenseZoneLayer(
    viewer,
    entities,
    domesOn &&
      defenseZoneOptions.show !== false &&
      shouldShowDefenseZones(zoneMode),
    applyTerrainGrounding,
    defenseZoneOptions,
  );
}

export function clearAllTerrainLayers(viewer: Viewer | null | undefined): void {
  clearTerrainMeshLayer(viewer);
  clearTerrainOverlays(viewer);
  clearTerrainContourLayer(viewer);
  clearEnvironmentMarkers(viewer);
  clearSensorDomeLayer(viewer);
  clearDefenseZoneLayer(viewer);
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

/** Ground markers/radar/defense to sampled terrain when mesh or domes are visible. */
export function shouldApplyTerrainGrounding(layers: TerrainLayerVisibility): boolean {
  return layers.showTerrainMesh || layers.showSensorDomes;
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
