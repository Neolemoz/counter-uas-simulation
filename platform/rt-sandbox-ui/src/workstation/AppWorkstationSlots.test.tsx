import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import {
  DEFAULT_PLANNING_COVERAGE_OPTIONS,
  EMPTY_PLANNING_POLYGON,
  EMPTY_PLANNING_RADARS,
  addPlanningRadarSite,
  addPlanningVertex,
  cancelPlanningDrawing,
  clearPlanningPolygon,
  clearPlanningRadarSites,
  deletePlanningRadarSite,
  estimatePlanningCoverage,
  finishPlanningPolygon,
  planningToolAllowsDrawing,
  planningToolAllowsRadarPlacement,
  planningToolUsesCesiumClick,
  updatePlanningRadarPreset,
  type PlanningCoverageEstimate,
  type PlanningCoverageLayerOptions,
  type PlanningPolygonState,
  type PlanningRadarState,
  type PlanningTool,
} from "@/cesium/planningDrawing";
import {
  DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE,
  type CesiumTerrainProviderMode,
} from "@/cesium/terrainProviderConfig";
import {
  DEFAULT_PLANNING_LOCATION_PRESET_ID,
  planningLocationPreset,
  type PlanningLocationPresetId,
} from "@/cesium/planningLocations";
import {
  DEFAULT_RUNTIME_WORKSPACE_MODE,
  PlanningModePanel,
  RuntimeWorkspaceModeSelector,
  editingEnabledForWorkspaceMode,
  workspaceModeShowsPlanningPlaceholder,
} from "./AppWorkstationSlots";

function renderPlanningPanel({
  tool = "select",
  polygon = EMPTY_PLANNING_POLYGON,
  radars = EMPTY_PLANNING_RADARS,
  coverage = estimatePlanningCoverage(polygon, radars),
  coverageOptions = DEFAULT_PLANNING_COVERAGE_OPTIONS,
  terrainProviderMode = DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE,
  locationPresetId = DEFAULT_PLANNING_LOCATION_PRESET_ID,
  customLatitude = String(planningLocationPreset(DEFAULT_PLANNING_LOCATION_PRESET_ID).latitudeDeg),
  customLongitude = String(planningLocationPreset(DEFAULT_PLANNING_LOCATION_PRESET_ID).longitudeDeg),
}: {
  tool?: PlanningTool;
  polygon?: PlanningPolygonState;
  radars?: PlanningRadarState;
  coverage?: PlanningCoverageEstimate;
  coverageOptions?: PlanningCoverageLayerOptions;
  terrainProviderMode?: CesiumTerrainProviderMode;
  locationPresetId?: PlanningLocationPresetId;
  customLatitude?: string;
  customLongitude?: string;
} = {}) {
  return renderToStaticMarkup(
    <PlanningModePanel
      tool={tool}
      polygon={polygon}
      radars={radars}
      coverage={coverage}
      coverageOptions={coverageOptions}
      terrainProviderMode={terrainProviderMode}
      locationPresetId={locationPresetId}
      customLatitude={customLatitude}
      customLongitude={customLongitude}
      onTerrainProviderModeChange={vi.fn()}
      onLocationPresetChange={vi.fn()}
      onCustomLatitudeChange={vi.fn()}
      onCustomLongitudeChange={vi.fn()}
      onApplyLocation={vi.fn()}
      onCameraPreset={vi.fn()}
      onToolChange={vi.fn()}
      onFinishPolygon={vi.fn()}
      onCancelDrawing={vi.fn()}
      onClearPolygon={vi.fn()}
      onSelectRadarSite={vi.fn()}
      onDeleteRadarSite={vi.fn()}
      onRadarPresetChange={vi.fn()}
      onClearRadarSites={vi.fn()}
      onCoverageOptionsChange={vi.fn()}
      onResetCoverageState={vi.fn()}
    />,
  );
}

describe("AppWorkstationSlots planning mode shell", () => {
  it("defaults to Grid Mode", () => {
    expect(DEFAULT_RUNTIME_WORKSPACE_MODE).toBe("grid");
    expect(workspaceModeShowsPlanningPlaceholder(DEFAULT_RUNTIME_WORKSPACE_MODE)).toBe(false);
  });

  it("renders mode selector state for Grid and Planning modes", () => {
    const onModeChange = vi.fn();
    const gridMarkup = renderToStaticMarkup(
      <RuntimeWorkspaceModeSelector mode="grid" onModeChange={onModeChange} />,
    );
    const planningMarkup = renderToStaticMarkup(
      <RuntimeWorkspaceModeSelector mode="planning" onModeChange={onModeChange} />,
    );

    expect(gridMarkup).toContain("Grid Mode");
    expect(gridMarkup).toContain("Planning Mode");
    expect(gridMarkup).toContain("aria-pressed=\"true\"");
    expect(planningMarkup).toContain("Planning Mode");
    expect(planningMarkup).toContain("aria-pressed=\"true\"");
  });

  it("keeps entity editing semantics unchanged across modes", () => {
    expect(editingEnabledForWorkspaceMode("grid", true)).toBe(true);
    expect(editingEnabledForWorkspaceMode("planning", true)).toBe(true);
    expect(editingEnabledForWorkspaceMode("grid", false)).toBe(false);
    expect(editingEnabledForWorkspaceMode("planning", false)).toBe(false);
  });

  it("shows Planning placeholders only for Planning Mode", () => {
    expect(workspaceModeShowsPlanningPlaceholder("grid")).toBe(false);
    expect(workspaceModeShowsPlanningPlaceholder("planning")).toBe(true);

    const markup = renderPlanningPanel();
    expect(markup).toContain("data-testid=\"planning-toolbar-placeholder\"");
    expect(markup).toContain("data-testid=\"planning-panel-placeholder\"");
    expect(markup).toContain("UI-local");
    expect(markup).toContain("explanatory only");
    expect(markup).toContain("not runtime authority");
    expect(markup).toContain("not validated sensing");
    expect(markup).toContain("no simulation behavior change");
    expect(markup).toContain("no bridge commands");
    expect(markup).toContain("no apply_scenario path");
  });

  it("renders Planning terrain controls and default terrain-off status", () => {
    const markup = renderPlanningPanel();

    expect(markup).toContain('data-testid="planning-terrain-controls"');
    expect(markup).toContain("Terrain controls");
    expect(markup).toContain("Terrain Off");
    expect(markup).toContain("Source Ellipsoid Terrain");
    expect(markup).toContain("status off");
    expect(markup).toContain("Terrain Source");
    expect(markup).toContain("3D Terrain");
  });

  it("renders Planning terrain-on status when optional terrain is selected", () => {
    const markup = renderPlanningPanel({ terrainProviderMode: "cesium_world_terrain" });

    expect(markup).toContain("Terrain On");
    expect(markup).toContain("Source 3D Terrain");
    expect(markup).toContain("status on");
  });

  it("renders Planning location preset controls", () => {
    const markup = renderPlanningPanel();

    expect(markup).toContain('data-testid="planning-location-controls"');
    expect(markup).toContain("Location presets");
    expect(markup).toContain("Current Bangkok");
    expect(markup).toContain("Bangkok");
    expect(markup).toContain("Chiang Mai");
    expect(markup).toContain("Phuket");
    expect(markup).toContain("Custom Coordinates");
    expect(markup).toContain("Jump Camera");
    expect(markup).toContain("Latitude");
    expect(markup).toContain("Longitude");
  });

  it("renders custom coordinate validation status", () => {
    const invalidMarkup = renderPlanningPanel({
      locationPresetId: "custom",
      customLatitude: "91",
      customLongitude: "100",
    });
    const validMarkup = renderPlanningPanel({
      locationPresetId: "custom",
      customLatitude: "13.7563",
      customLongitude: "100.5018",
    });

    expect(invalidMarkup).toContain("Custom coordinates must use latitude -90..90 and longitude -180..180.");
    expect(validMarkup).toContain("Real-world locations are presentation-only");
  });

  it("renders Planning camera preset controls", () => {
    const markup = renderPlanningPanel();

    expect(markup).toContain('data-testid="planning-camera-presets"');
    expect(markup).toContain("Overview");
    expect(markup).toContain("Ridge");
    expect(markup).toContain("Valley");
    expect(markup).toContain("Sensor Context");
  });

  it("renders terrain governance and overlay visibility copy", () => {
    const markup = renderPlanningPanel();

    expect(markup).toContain('data-testid="planning-terrain-legend"');
    expect(markup).toContain("visual-only");
    expect(markup).toContain("does not affect sensors");
    expect(markup).toContain("LOS");
    expect(markup).toContain("MC");
    expect(markup).toContain("runtime simulation");
    expect(markup).toContain("Real-world locations are presentation-only");
    expect(markup).toContain("location presets do not affect planning metrics");
    expect(markup).toContain("Planning polygon");
    expect(markup).toContain("radar sites");
    expect(markup).toContain("coverage overlay");
    expect(markup).toContain("blind spot markers");
    expect(markup).toContain("both terrain modes");
  });

  it("enters draw mode only while Planning Mode is active", () => {
    expect(planningToolAllowsDrawing(true, "draw_defense_area")).toBe(true);
    expect(planningToolAllowsDrawing(false, "draw_defense_area")).toBe(false);
    expect(planningToolAllowsDrawing(true, "select")).toBe(false);

    const markup = renderPlanningPanel({ tool: "draw_defense_area" });
    expect(markup).toContain("Draw Defense Area");
    expect(markup).toContain("aria-pressed=\"true\"");
  });

  it("adds draft polygon vertices without runtime state", () => {
    const oneVertex = addPlanningVertex(EMPTY_PLANNING_POLYGON, { x: 10, y: 20 });
    const twoVertices = addPlanningVertex(oneVertex, { x: 30, y: 40 });

    expect(oneVertex.draftVertices).toEqual([{ x: 10, y: 20 }]);
    expect(twoVertices.draftVertices).toEqual([
      { x: 10, y: 20 },
      { x: 30, y: 40 },
    ]);
    expect(twoVertices.completedVertices).toBeNull();
  });

  it("finishes polygons only after three vertices", () => {
    const twoVertices = addPlanningVertex(
      addPlanningVertex(EMPTY_PLANNING_POLYGON, { x: 0, y: 0 }),
      { x: 1, y: 0 },
    );
    const threeVertices = addPlanningVertex(twoVertices, { x: 0, y: 1 });

    expect(finishPlanningPolygon(twoVertices)).toBe(twoVertices);
    expect(finishPlanningPolygon(threeVertices)).toEqual({
      draftVertices: [],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1, y: 0 },
        { x: 0, y: 1 },
      ],
    });
  });

  it("cancels drawing while preserving a completed polygon", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [{ x: 5, y: 6 }],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1, y: 0 },
        { x: 0, y: 1 },
      ],
    };

    expect(cancelPlanningDrawing(polygon)).toEqual({
      draftVertices: [],
      completedVertices: polygon.completedVertices,
    });
  });

  it("clears Planning polygons and isolates Grid Mode", () => {
    expect(clearPlanningPolygon()).toEqual(EMPTY_PLANNING_POLYGON);
    expect(planningToolAllowsDrawing(false, "draw_defense_area")).toBe(false);
    expect(workspaceModeShowsPlanningPlaceholder("grid")).toBe(false);
  });

  it("places UI-local radar sites with the default preset", () => {
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 100, y: 200 });

    expect(radars.sites).toEqual([
      {
        id: "planning-radar-1",
        position: { x: 100, y: 200 },
        radar_type: "Medium Radar",
        detection_range_m: 1500,
      },
    ]);
    expect(radars.selectedSiteId).toBe("planning-radar-1");
    expect(radars.nextSiteId).toBe(2);
  });

  it("renders and updates radar preset selection", () => {
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 0, y: 0 });
    const updated = updatePlanningRadarPreset(radars, "planning-radar-1", "long");

    expect(updated.sites[0]).toMatchObject({
      radar_type: "Long Range",
      detection_range_m: 3000,
    });
    const markup = renderPlanningPanel({ tool: "place_radar_site", radars: updated });
    expect(markup).toContain("Place Radar Site");
    expect(markup).toContain("Long Range (3000m)");
    expect(markup).toContain("Short Range (500m)");
  });

  it("deletes and clears selected planning radar sites", () => {
    const first = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 0, y: 0 });
    const second = addPlanningRadarSite(first, { x: 10, y: 20 });
    const deleted = deletePlanningRadarSite(second, "planning-radar-2");
    const cleared = clearPlanningRadarSites(second);

    expect(deleted.sites.map((site) => site.id)).toEqual(["planning-radar-1"]);
    expect(deleted.selectedSiteId).toBeNull();
    expect(deleted.nextSiteId).toBe(3);
    expect(cleared).toEqual({ sites: [], selectedSiteId: null, nextSiteId: 3 });
  });

  it("isolates radar placement to Planning Mode Cesium clicks", () => {
    expect(planningToolAllowsRadarPlacement(true, "place_radar_site")).toBe(true);
    expect(planningToolAllowsRadarPlacement(false, "place_radar_site")).toBe(false);
    expect(planningToolAllowsRadarPlacement(true, "select")).toBe(false);
    expect(planningToolUsesCesiumClick(false, "place_radar_site")).toBe(false);
    expect(planningToolUsesCesiumClick(true, "place_radar_site")).toBe(true);
  });


  it("renders visual legend and reset affordances", () => {
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 0, y: 0 });
    const markup = renderPlanningPanel({ radars });

    expect(markup).toContain('data-testid="planning-visual-legend"');
    expect(markup).toContain("defense area");
    expect(markup).toContain("radar range");
    expect(markup).toContain("covered cells");
    expect(markup).toContain("uncovered cells");
    expect(markup).toContain("blind spot hints");
    expect(markup).toContain("Clear Polygon");
    expect(markup).toContain("Clear Radar Sites");
    expect(markup).toContain("Reset Coverage View");
  });

  it("planning artifacts do not call bridge, spawn, or apply_scenario paths", () => {
    const bridgeCommand = vi.fn();
    const onSpawn = vi.fn();
    const applyScenario = vi.fn();
    const polygon = finishPlanningPolygon({
      draftVertices: [
        { x: 0, y: 0 },
        { x: 100, y: 0 },
        { x: 0, y: 100 },
      ],
      completedVertices: null,
    });
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 10, y: 10 });

    estimatePlanningCoverage(polygon, radars, 6);
    clearPlanningRadarSites(radars);
    clearPlanningPolygon();

    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(onSpawn).not.toHaveBeenCalled();
    expect(applyScenario).not.toHaveBeenCalled();
  });

  it("does not create runtime entities for planning radar sites", () => {
    const onSpawn = vi.fn();
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 5, y: 6 });

    expect(onSpawn).not.toHaveBeenCalled();
    expect(radars.sites[0]).not.toHaveProperty("entity_id");
    expect(radars.sites[0]).not.toHaveProperty("entity_type");
  });

  it("computes heuristic coverage for a completed defense polygon", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1000, y: 0 },
        { x: 1000, y: 1000 },
        { x: 0, y: 1000 },
      ],
    };
    const radars: PlanningRadarState = {
      ...EMPTY_PLANNING_RADARS,
      sites: [
        {
          id: "planning-radar-1",
          position: { x: 500, y: 500 },
          radar_type: "Medium Radar",
          detection_range_m: 800,
        },
      ],
    };

    const coverage = estimatePlanningCoverage(polygon, radars, 10);

    expect(coverage.radarCount).toBe(1);
    expect(coverage.totalPolygonAreaM2).toBe(1_000_000);
    expect(coverage.estimatedCoveredAreaM2).toBeGreaterThan(0);
    expect(coverage.estimatedUncoveredAreaM2).toBeLessThan(1_000_000);
    expect(coverage.coveragePercent).toBeGreaterThan(0);
  });

  it("reports fully uncovered visual estimate when no radar is placed", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1000, y: 0 },
        { x: 1000, y: 1000 },
        { x: 0, y: 1000 },
      ],
    };

    const coverage = estimatePlanningCoverage(polygon, EMPTY_PLANNING_RADARS, 8);

    expect(coverage.radarCount).toBe(0);
    expect(coverage.estimatedCoveredAreaM2).toBe(0);
    expect(coverage.estimatedUncoveredAreaM2).toBe(1_000_000);
    expect(coverage.coveragePercent).toBe(0);
    expect(coverage.blindSpotHints.length).toBeGreaterThan(0);
  });

  it("handles multiple radar overlap without double-counting polygon coverage", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1000, y: 0 },
        { x: 1000, y: 1000 },
        { x: 0, y: 1000 },
      ],
    };
    const radars: PlanningRadarState = {
      ...EMPTY_PLANNING_RADARS,
      sites: [
        {
          id: "planning-radar-1",
          position: { x: 450, y: 500 },
          radar_type: "Medium Radar",
          detection_range_m: 900,
        },
        {
          id: "planning-radar-2",
          position: { x: 550, y: 500 },
          radar_type: "Medium Radar",
          detection_range_m: 900,
        },
      ],
    };

    const coverage = estimatePlanningCoverage(polygon, radars, 12);

    expect(coverage.radarCount).toBe(2);
    expect(coverage.estimatedCoveredAreaM2).toBeLessThanOrEqual(
      coverage.totalPolygonAreaM2,
    );
    expect(coverage.coveragePercent).toBeLessThanOrEqual(100);
  });

  it("renders Planning coverage visibility toggles", () => {
    const markup = renderPlanningPanel({
      coverageOptions: { showCoverage: false, showBlindSpots: true },
    });

    expect(markup).toContain('data-testid="planning-coverage-status"');
    expect(markup).toContain("show coverage");
    expect(markup).toContain("show blind spots");
    expect(markup).toContain("heuristic");
    expect(markup).toContain("visual estimate");
    expect(markup).toContain("not validated sensing");
  });

  it("isolates coverage overlays from Grid Mode", () => {
    expect(workspaceModeShowsPlanningPlaceholder("grid")).toBe(false);
    expect(planningToolUsesCesiumClick(false, "select")).toBe(false);
    expect(estimatePlanningCoverage(EMPTY_PLANNING_POLYGON, EMPTY_PLANNING_RADARS)).toEqual({
      radarCount: 0,
      totalPolygonAreaM2: 0,
      estimatedCoveredAreaM2: 0,
      estimatedUncoveredAreaM2: 0,
      coveragePercent: 0,
      coveredCells: [],
      uncoveredCells: [],
      blindSpotHints: [],
    });
  });

});
