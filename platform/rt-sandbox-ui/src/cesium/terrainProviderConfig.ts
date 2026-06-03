import { EllipsoidTerrainProvider, Terrain } from "cesium";

export type CesiumTerrainProviderMode = "ellipsoid" | "cesium_world_terrain";

export const DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE: CesiumTerrainProviderMode =
  "ellipsoid";

export const CESIUM_TERRAIN_PROVIDER_OPTIONS: {
  mode: CesiumTerrainProviderMode;
  label: string;
}[] = [
  { mode: "ellipsoid", label: "Ellipsoid Terrain" },
  { mode: "cesium_world_terrain", label: "3D Terrain" },
];

export const TERRAIN_PROVIDER_VISUAL_ONLY_COPY =
  "Terrain is visual-only; it does not affect sensors, LOS, runtime simulation, or MC.";

export function isOptionalCesiumTerrainMode(
  mode: CesiumTerrainProviderMode,
): boolean {
  return mode === "cesium_world_terrain";
}

export function createCesiumTerrainProvider(
  mode: CesiumTerrainProviderMode = DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE,
): Terrain {
  if (isOptionalCesiumTerrainMode(mode)) {
    return Terrain.fromWorldTerrain({ requestVertexNormals: true });
  }
  return new Terrain(Promise.resolve(new EllipsoidTerrainProvider()));
}

export function terrainProviderModeLabel(
  mode: CesiumTerrainProviderMode,
): string {
  return (
    CESIUM_TERRAIN_PROVIDER_OPTIONS.find((option) => option.mode === mode)?.label ??
    CESIUM_TERRAIN_PROVIDER_OPTIONS[0].label
  );
}

export function terrainProviderRuntimeMutationAllowed(
  _mode: CesiumTerrainProviderMode,
): false {
  return false;
}
