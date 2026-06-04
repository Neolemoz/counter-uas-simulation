import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import {
  DEFAULT_PLANNING_COVERAGE_OPTIONS,
  EMPTY_PLANNING_POLYGON,
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
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
import { analyzePlanningCoverage, type PlanningCoverageAnalysis } from "@/cesium/planningCoverageAnalysis";
import { planningExtentById } from "@/cesium/planningWorld";
import {
  DEFAULT_PLANNING_MEASUREMENT_STATE,
  addPlanningMeasurementPoint,
  type PlanningMeasurementState,
} from "@/cesium/planningMeasurements";
import { buildPlanningMcSnapshot } from "@/layout/planningMcSnapshot";
import { buildPlanningMcPackage, type PlanningMcPackageV1 } from "@/layout/planningMcPackage";
import {
  buildEmptyPlanningResultLink,
  buildPlanningResultLinkPreview,
  type PlanningResultLinkV1,
} from "@/layout/planningMcResultLink";
import {
  buildPlanningLayoutCompareSlot,
  capturePlanningLayoutCompareSlot,
  type PlanningLayoutCompareSlotV1,
} from "@/layout/planningLayoutComparison";
import { buildPlanningLayoutComparePanelDerivation } from "@/workstation/PlanningLayoutComparePanel";
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
  coverageAnalysis = analyzePlanningCoverage(polygon, radars, undefined, { radarPresets: PLANNING_RADAR_PRESETS }),
  planningExtent = planningExtentById("planning_10km"),
  planningMeasurements = DEFAULT_PLANNING_MEASUREMENT_STATE,
  planningMcPackage = null,
  planningMcPackageStale = false,
  planningResultLink = planningMcPackage ? buildEmptyPlanningResultLink(planningMcPackage) : null,
  planningResultLinkPreview = buildPlanningResultLinkPreview(planningResultLink),
  planningResultImportText = "",
  planningResultImportError = null,
  planningLayoutCompareSlots = [] as PlanningLayoutCompareSlotV1[],
  planningLayoutCompareAnalytics = buildPlanningLayoutComparePanelDerivation(
    planningLayoutCompareSlots,
  ),
  planningLayoutCompareCaptureDisabled = planningLayoutCompareSlots.length >= 3,
  planningLayoutCompareImportSlotLabel = "A" as const,
  planningLayoutCompareImportText = "",
  planningLayoutCompareImportError = null as string | null,
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
  coverageAnalysis?: PlanningCoverageAnalysis;
  planningExtent?: ReturnType<typeof planningExtentById>;
  planningMeasurements?: PlanningMeasurementState;
  planningMcPackage?: PlanningMcPackageV1 | null;
  planningMcPackageStale?: boolean;
  planningResultLink?: PlanningResultLinkV1 | null;
  planningResultLinkPreview?: ReturnType<typeof buildPlanningResultLinkPreview>;
  planningResultImportText?: string;
  planningResultImportError?: string | null;
  planningLayoutCompareSlots?: PlanningLayoutCompareSlotV1[];
  planningLayoutCompareAnalytics?: ReturnType<typeof buildPlanningLayoutComparePanelDerivation>;
  planningLayoutCompareCaptureDisabled?: boolean;
  planningLayoutCompareImportSlotLabel?: "A" | "B" | "C";
  planningLayoutCompareImportText?: string;
  planningLayoutCompareImportError?: string | null;
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
      coverageAnalysis={coverageAnalysis}
      planningExtent={planningExtent}
      planningMeasurements={planningMeasurements}
      planningMcPackage={planningMcPackage}
      planningMcPackageStale={planningMcPackageStale}
      planningResultLink={planningResultLink}
      planningResultLinkPreview={planningResultLinkPreview}
      planningResultImportText={planningResultImportText}
      planningResultImportError={planningResultImportError}
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
      onPlanningExtentChange={vi.fn()}
      onPlanningExtentCameraFit={vi.fn()}
      onPlanningRadiusChange={vi.fn()}
      onClearPlanningMeasurements={vi.fn()}
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
      onGeneratePlanningMcPackage={vi.fn()}
      onCopyPlanningMcPackage={vi.fn()}
      onDownloadPlanningMcPackage={vi.fn()}
      onPlanningResultImportTextChange={vi.fn()}
      onImportPlanningResultMetadata={vi.fn()}
      onImportMockPlanningResultRef={vi.fn()}
      onClearPlanningResultImport={vi.fn()}
      planningLayoutCompareSlots={planningLayoutCompareSlots}
      planningLayoutCompareAnalytics={planningLayoutCompareAnalytics}
      planningLayoutCompareCaptureDisabled={planningLayoutCompareCaptureDisabled}
      planningLayoutCompareImportSlotLabel={planningLayoutCompareImportSlotLabel}
      planningLayoutCompareImportText={planningLayoutCompareImportText}
      planningLayoutCompareImportError={planningLayoutCompareImportError}
      onCapturePlanningLayoutCompare={vi.fn()}
      onRemovePlanningLayoutCompareSlot={vi.fn()}
      onClearPlanningLayoutCompareSlots={vi.fn()}
      onPlanningLayoutCompareImportSlotLabelChange={vi.fn()}
      onPlanningLayoutCompareImportTextChange={vi.fn()}
      onImportPlanningLayoutCompareSnapshot={vi.fn()}
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

  it("renders Planning overlap and redundancy analytics", () => {
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
    const coverage = estimatePlanningCoverage(polygon, radars, 4);
    const coverageAnalysis = analyzePlanningCoverage(polygon, radars, 4);

    const markup = renderPlanningPanel({ polygon, radars, coverage, coverageAnalysis });

    expect(markup).toContain('data-testid="planning-analytics-v2"');
    expect(markup).toContain("Planning heuristic only.");
    expect(markup).toContain("Coverage % 100.0%");
    expect(markup).toContain("Overlap % 100.0%");
    expect(markup).toContain("Redundancy % 100.0%");
  });



  it("renders Planning advisory recommendations", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1000, y: 0 },
        { x: 1000, y: 1000 },
        { x: 0, y: 1000 },
      ],
    };
    const coverage = estimatePlanningCoverage(polygon, EMPTY_PLANNING_RADARS, 4);
    const coverageAnalysis = analyzePlanningCoverage(
      polygon,
      EMPTY_PLANNING_RADARS,
      4,
      { radarPresets: PLANNING_RADAR_PRESETS },
    );

    const markup = renderPlanningPanel({ polygon, coverage, coverageAnalysis });

    expect(markup).toContain("data-testid=\"planning-advisory-v2\"");
    expect(markup).toContain("Advisory planning heuristic only.");
    expect(markup).toContain("Blind Spot Summary 100.0% uncovered");
    expect(markup).toContain("Suggested Radar Long Range");
    expect(markup).toContain("Suggested Position");
    expect(markup).toContain("Reason Add Long Range near NE uncovered sector.");
  });


  it("renders Planning MC package preview and identifier linkage", () => {
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
          position: { x: 150, y: 150 },
          radar_type: "Short Range",
          detection_range_m: 500,
        },
      ],
    };
    const coverage = estimatePlanningCoverage(polygon, radars, 4);
    const coverageAnalysis = analyzePlanningCoverage(polygon, radars, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    });
    const snapshot = buildPlanningMcSnapshot(polygon, radars, coverageAnalysis, {
      createdUtc: "2026-06-04T00:00:00Z",
      terrainMode: "ellipsoid",
      selectedLocationPreset: "bangkok",
      sourceLayoutId: "rt_layout_source",
      sourceGeometryId: "rt_layout:sha256:source",
    });
    const planningMcPackage = buildPlanningMcPackage(snapshot, {
      scenarioLabel: "planning-analysis",
      suggestedRunCount: 25,
      suggestedSeedBase: 7001,
    });

    const markup = renderPlanningPanel({
      polygon,
      radars,
      coverage,
      coverageAnalysis,
      planningMcPackage,
    });

    expect(markup).toContain('data-testid="planning-mc-package-preview"');
    expect(markup).toContain("Planning MC package preview");
    expect(markup).toContain("planning_snapshot_id rt_planning_snapshot:sha256:");
    expect(markup).toContain("planning_geometry_id rt_planning:sha256:");
    expect(markup).toContain("Radar count 1");
    expect(markup).toContain("Coverage summary");
    expect(markup).toContain("Overlap summary");
    expect(markup).toContain("Redundancy summary");
    expect(markup).toContain("Suggested MC settings planning-analysis · 25 runs · seed 7001");
    expect(markup).toContain('data-testid="planning-mc-result-link-preview"');
    expect(markup).toContain("Planning MC result linkage");
    expect(markup).toContain("Linkage status Unlinked");
    expect(markup).toContain("Result link planning_result_link_v1 · unlinked");
    expect(markup).toContain("Import metadata");
    expect(markup).toContain("Import mock result ref");
    expect(markup).toContain("Copy package JSON");
    expect(markup).toContain("Download package JSON");
  });

  it("renders Planning extent/runtime distinction copy", () => {
    const markup = renderPlanningPanel();

    expect(markup).toContain("10 km Planning World");
    expect(markup).toContain("Approx. area 314.2 km^2");
    expect(markup).toContain("Runtime bounds remain the ±500m sandbox");
    expect(markup).toContain("Planning extent only; not runtime bounds");
  });

  it("renders Planning measurement readouts and governance copy", () => {
    const planningMeasurements = addPlanningMeasurementPoint(
      addPlanningMeasurementPoint(DEFAULT_PLANNING_MEASUREMENT_STATE, { x: 0, y: 0 }),
      { x: 3000, y: 4000 },
    );
    const markup = renderPlanningPanel({ planningMeasurements });

    expect(markup).toContain('data-testid="planning-measurement-controls"');
    expect(markup).toContain("Distance m 5000");
    expect(markup).toContain("Distance km 5.00");
    expect(markup).toContain("Bearing NE");
    expect(markup).toContain("Start X 0m, Y 0m");
    expect(markup).toContain("End X 3000m, Y 4000m");
    expect(markup).toContain("Planning measurement tool only; not runtime authority");
  });

  it("allows Planning coordinates outside runtime bounds when inside Planning extent", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [],
      completedVertices: [
        { x: 900, y: 0 },
        { x: 1300, y: 0 },
        { x: 1300, y: 500 },
      ],
    };
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 1200, y: 250 });
    const markup = renderPlanningPanel({
      polygon,
      radars,
      coverage: estimatePlanningCoverage(polygon, radars, 4),
      coverageAnalysis: analyzePlanningCoverage(polygon, radars, 4, {
        radarPresets: PLANNING_RADAR_PRESETS,
      }),
      planningExtent: planningExtentById("planning_5km"),
    });

    expect(markup).toContain('data-testid="planning-runtime-guardrail"');
    expect(markup).toContain("Outside runtime sandbox; valid for planning only.");
    expect(markup).toContain("inside 5 km Planning World");
  });

  it("renders Planning MC package stale advisory", () => {
    const markup = renderPlanningPanel({
      planningMcPackage: {
        schema_version: "rt_planning_mc_package_v1",
        planning_snapshot_id: "rt_planning_snapshot:sha256:old",
        planning_geometry_id: "rt_planning:sha256:old",
        planning_extent: planningExtentById("planning_10km"),
        planning_summary: {
          radar_count: 0,
          coverage_summary: { coverage_percent: 0, blind_spot_summary: "none" },
          overlap_summary: { overlap_percent: 0 },
          redundancy_summary: { redundancy_percent: 0 },
        },
        mc_preparation: {
          scenario_label: "planning-analysis",
          suggested_run_count: 50,
          suggested_seed_base: 1,
        },
        metadata: { created_utc: "2026-06-04T00:00:00Z", package_version: "1" },
      },
      planningMcPackageStale: true,
    });

    expect(markup).toContain("Package may be stale. Regenerate.");
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

  it("renders Planning cognition summary with extent, polygon, radar, and measurement counts", () => {
    let polygon = addPlanningVertex(EMPTY_PLANNING_POLYGON, { x: 0, y: 0 });
    polygon = addPlanningVertex(polygon, { x: 500, y: 0 });
    polygon = addPlanningVertex(polygon, { x: 500, y: 500 });
    polygon = finishPlanningPolygon(polygon);
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 100, y: 100 });
    const planningMeasurements = addPlanningMeasurementPoint(
      addPlanningMeasurementPoint(DEFAULT_PLANNING_MEASUREMENT_STATE, { x: 0, y: 0 }),
      { x: 1000, y: 0 },
    );
    const markup = renderPlanningPanel({ polygon, radars, planningMeasurements });

    expect(markup).toContain('data-testid="planning-cognition-panel"');
    expect(markup).toContain('data-testid="planning-summary"');
    expect(markup).toContain("Extent 10 km Planning World");
    expect(markup).toContain("Polygons 1");
    expect(markup).toContain("Radar sites 1");
    expect(markup).toContain("Measurements 1");
    expect(markup).toContain("Planning cognition summary is UI-local and non-authoritative");
  });

  it("shows informational Planning warnings without blocking controls", () => {
    const markup = renderPlanningPanel({ tool: "measure_distance" });

    expect(markup).toContain('data-testid="planning-warnings"');
    expect(markup).toContain('data-testid="planning-warning-no_polygon_defined"');
    expect(markup).toContain('data-testid="planning-warning-no_radar_sites"');
    expect(markup).toContain("Finish Polygon");
    expect(markup).toContain("Place Radar Site");
  });

  it("updates Planning cognition summary when extent switches", () => {
    const markup5 = renderPlanningPanel({
      planningExtent: planningExtentById("planning_5km"),
    });
    const markup20 = renderPlanningPanel({
      planningExtent: planningExtentById("planning_20km"),
    });

    expect(markup5).toContain("Extent 5 km Planning World");
    expect(markup5).toContain("Radius 5,000m");
    expect(markup5).toContain("Compact Planning World");
    expect(markup20).toContain("Extent 20 km Planning World");
    expect(markup20).toContain("Radius 20,000m");
    expect(markup20).toContain("Wide Planning World");
  });

  it("renders Planning layout compare panel with governance and table", () => {
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
          position: { x: 150, y: 150 },
          radar_type: "Short Range",
          detection_range_m: 500,
        },
      ],
    };
    const coverageAnalysis = analyzePlanningCoverage(polygon, radars, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    });
    const snapshot = buildPlanningMcSnapshot(polygon, radars, coverageAnalysis, {
      createdUtc: "2026-06-04T00:00:00Z",
      terrainMode: "ellipsoid",
      selectedLocationPreset: "bangkok",
    });
    const snapB = buildPlanningMcSnapshot(
      polygon,
      addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 800, y: 800 }),
      analyzePlanningCoverage(
        polygon,
        addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 800, y: 800 }),
        4,
        { radarPresets: PLANNING_RADAR_PRESETS },
      ),
      {
        createdUtc: "2026-06-04T01:00:00Z",
        terrainMode: "ellipsoid",
        selectedLocationPreset: "bangkok",
      },
    );
    let planningLayoutCompareSlots = capturePlanningLayoutCompareSlot(
      [],
      snapshot,
      "2026-06-04T00:00:00Z",
    ).slots;
    planningLayoutCompareSlots = capturePlanningLayoutCompareSlot(
      planningLayoutCompareSlots,
      snapB,
      "2026-06-04T01:00:00Z",
    ).slots;

    const markup = renderPlanningPanel({
      polygon,
      radars,
      coverageAnalysis,
      planningLayoutCompareSlots,
    });

    expect(markup).toContain('data-testid="planning-layout-compare-panel"');
    expect(markup).toContain('data-testid="planning-layout-compare-governance"');
    expect(markup).toContain('data-testid="planning-layout-compare-table"');
    expect(markup).toContain('data-testid="planning-layout-compare-row-A"');
    expect(markup).toContain('data-testid="planning-layout-compare-deltas"');
    expect(markup).toContain('data-testid="planning-layout-compare-blind-spots"');
    expect(markup).toContain('data-testid="planning-layout-compare-recommendations"');
    expect(markup).toContain('data-testid="planning-layout-compare-import"');
    expect(markup).toContain("Capture Current Layout");
    expect(markup).toContain("Clear All Slots");
    expect(markup).toContain(snapshot.planning_snapshot_id);
    expect(markup).toContain("planning_10km");
  });

  it("supports capture, remove slot, and clear all slot flows via compare helpers", () => {
    const polygon: PlanningPolygonState = {
      draftVertices: [],
      completedVertices: [
        { x: 0, y: 0 },
        { x: 1000, y: 0 },
        { x: 1000, y: 1000 },
      ],
    };
    const radarsA = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 150, y: 150 });
    const radarsB = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 800, y: 800 });
    const snapA = buildPlanningMcSnapshot(
      polygon,
      radarsA,
      analyzePlanningCoverage(polygon, radarsA, 4, { radarPresets: PLANNING_RADAR_PRESETS }),
      {
        createdUtc: "2026-06-04T00:00:00Z",
        terrainMode: "ellipsoid",
        selectedLocationPreset: "bangkok",
      },
    );
    const snapB = buildPlanningMcSnapshot(
      polygon,
      radarsB,
      analyzePlanningCoverage(polygon, radarsB, 4, { radarPresets: PLANNING_RADAR_PRESETS }),
      {
        createdUtc: "2026-06-04T01:00:00Z",
        terrainMode: "ellipsoid",
        selectedLocationPreset: "bangkok",
      },
    );

    const capturedA = capturePlanningLayoutCompareSlot([], snapA).slots;
    const capturedBoth = capturePlanningLayoutCompareSlot(capturedA, snapB).slots;
    expect(capturedBoth.map((slot) => slot.slot_label)).toEqual(["A", "B"]);

    const duplicateSlotA = buildPlanningLayoutCompareSlot("A", snapA);
    const duplicateSlotB = buildPlanningLayoutCompareSlot("B", snapA, {
      captureUtc: "2026-06-04T02:00:00Z",
    });
    const duplicateWarningMarkup = renderPlanningPanel({
      planningLayoutCompareSlots: [duplicateSlotA, duplicateSlotB],
    });
    expect(duplicateWarningMarkup).toContain(
      'data-testid="planning-layout-compare-warning-duplicate_geometry"',
    );
  });

});
