import { Terrain } from "cesium";
import { describe, expect, it } from "vitest";
import {
  CESIUM_TERRAIN_PROVIDER_OPTIONS,
  DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE,
  TERRAIN_PROVIDER_VISUAL_ONLY_COPY,
  createCesiumTerrainProvider,
  isOptionalCesiumTerrainMode,
  terrainProviderModeLabel,
  terrainProviderRuntimeMutationAllowed,
} from "./terrainProviderConfig";
import {
  EMPTY_PLANNING_RADARS,
  estimatePlanningCoverage,
  type PlanningPolygonState,
} from "./planningDrawing";

describe("terrainProviderConfig", () => {
  it("defaults to ellipsoid terrain", () => {
    expect(DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE).toBe("ellipsoid");
    expect(terrainProviderModeLabel(DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE)).toBe(
      "Ellipsoid Terrain",
    );
    expect(isOptionalCesiumTerrainMode(DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE)).toBe(false);
    expect(createCesiumTerrainProvider()).toBeInstanceOf(Terrain);
  });

  it("supports optional 3D terrain as a visual-only provider path", () => {
    expect(CESIUM_TERRAIN_PROVIDER_OPTIONS.map((option) => option.mode)).toEqual([
      "ellipsoid",
      "cesium_world_terrain",
    ]);
    expect(isOptionalCesiumTerrainMode("cesium_world_terrain")).toBe(true);
    expect(terrainProviderModeLabel("cesium_world_terrain")).toBe("3D Terrain");
    expect(createCesiumTerrainProvider("cesium_world_terrain")).toBeInstanceOf(Terrain);
  });

  it("documents visual-only governance boundaries", () => {
    expect(TERRAIN_PROVIDER_VISUAL_ONLY_COPY).toContain("visual-only");
    expect(TERRAIN_PROVIDER_VISUAL_ONLY_COPY).toContain("sensors");
    expect(TERRAIN_PROVIDER_VISUAL_ONLY_COPY).toContain("LOS");
    expect(TERRAIN_PROVIDER_VISUAL_ONLY_COPY).toContain("runtime simulation");
    expect(TERRAIN_PROVIDER_VISUAL_ONLY_COPY).toContain("MC");
    expect(terrainProviderRuntimeMutationAllowed("ellipsoid")).toBe(false);
    expect(terrainProviderRuntimeMutationAllowed("cesium_world_terrain")).toBe(false);
  });

  it("does not change planar Planning Mode coverage calculations", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1000, y: 0 },
        { x: 1000, y: 1000 },
        { x: 0, y: 1000 },
      ],
    };

    const defaultCoverage = estimatePlanningCoverage(polygon, EMPTY_PLANNING_RADARS, 8);
    createCesiumTerrainProvider("cesium_world_terrain");
    const optionalCoverage = estimatePlanningCoverage(polygon, EMPTY_PLANNING_RADARS, 8);

    expect(optionalCoverage).toEqual(defaultCoverage);
  });
});
