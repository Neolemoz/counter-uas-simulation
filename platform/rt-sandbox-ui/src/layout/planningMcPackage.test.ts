import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import {
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
  type PlanningPolygonState,
  type PlanningRadarState,
} from "@/cesium/planningDrawing";
import { analyzePlanningCoverage } from "@/cesium/planningCoverageAnalysis";
import { planningExtentById } from "@/cesium/planningWorld";
import { buildPlanningMcSnapshot } from "./planningMcSnapshot";
import {
  buildEmptyPlanningResultLink,
  buildPlanningMcPackage,
  copyPlanningMcPackage,
  downloadPlanningMcPackage,
  exportPlanningMcPackageJson,
  suggestedPlanningMcPackageFilename,
} from "./planningMcPackage";

const POLYGON: PlanningPolygonState = {
  draftVertices: [],
  completedVertices: [
    { x: 0, y: 0 },
    { x: 1000, y: 0 },
    { x: 1000, y: 1000 },
    { x: 0, y: 1000 },
  ],
};

const RADARS: PlanningRadarState = {
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

function snapshot() {
  return buildPlanningMcSnapshot(
    POLYGON,
    RADARS,
    analyzePlanningCoverage(POLYGON, RADARS, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    }),
    {
      createdUtc: "2026-06-04T00:00:00Z",
      terrainMode: "ellipsoid",
      selectedLocationPreset: "bangkok",
      sourceLayoutId: "rt_layout_source",
      sourceGeometryId: "rt_layout:sha256:source",
    },
  );
}

describe("planningMcPackage", () => {
  beforeEach(() => {
    vi.stubGlobal(
      "URL",
      Object.assign(URL, {
        createObjectURL: vi.fn(() => "blob:test"),
        revokeObjectURL: vi.fn(),
      }),
    );
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("builds deterministic packages from stable snapshots", () => {
    const first = buildPlanningMcPackage(snapshot());
    const second = buildPlanningMcPackage(snapshot());

    expect(second).toEqual(first);
    expect(first.schema_version).toBe("rt_planning_mc_package_v1");
    expect(first.metadata).toEqual({
      created_utc: "2026-06-04T00:00:00Z",
      package_version: "1",
    });
  });

  it("preserves planning and source identifiers without replacing existing MC identifiers", () => {
    const row = buildPlanningMcPackage(snapshot());

    expect(row.planning_snapshot_id.startsWith("rt_planning_snapshot:sha256:")).toBe(true);
    expect(row.planning_geometry_id.startsWith("rt_planning:sha256:")).toBe(true);
    expect(row.planning_extent.planning_extent_id).toBe("planning_10km");
    expect(row).not.toHaveProperty("geometry_id");
    expect(row).not.toHaveProperty("layout_id");
    expect(row.source_layout_id).toBe("rt_layout_source");
    expect(row.source_geometry_id).toBe("rt_layout:sha256:source");
  });

  it("preserves Planning extent metadata from snapshot", () => {
    const source = buildPlanningMcSnapshot(
      POLYGON,
      RADARS,
      analyzePlanningCoverage(POLYGON, RADARS, 4, {
        radarPresets: PLANNING_RADAR_PRESETS,
      }),
      {
        createdUtc: "2026-06-04T00:00:00Z",
        terrainMode: "ellipsoid",
        selectedLocationPreset: "bangkok",
        planningExtent: planningExtentById("planning_5km"),
      },
    );
    const row = buildPlanningMcPackage(source);

    expect(row.planning_extent).toEqual({
      planning_extent_id: "planning_5km",
      planning_extent_radius_m: 5000,
      planning_extent_label: "5 km Planning World",
    });
  });

  it("preserves planning analytics summaries", () => {
    const source = snapshot();
    const row = buildPlanningMcPackage(source);

    expect(row.planning_summary.radar_count).toBe(1);
    expect(row.planning_summary.coverage_summary).toEqual({
      coverage_percent: source.analytics_summary.coverage_percent,
      blind_spot_summary: source.analytics_summary.blind_spot_summary,
    });
    expect(row.planning_summary.overlap_summary.overlap_percent).toBe(
      source.analytics_summary.overlap_percent,
    );
    expect(row.planning_summary.redundancy_summary.redundancy_percent).toBe(
      source.analytics_summary.redundancy_percent,
    );
  });

  it("includes deterministic MC preparation suggestions only", () => {
    const row = buildPlanningMcPackage(snapshot(), {
      scenarioLabel: "planning-single-target",
      suggestedRunCount: 25,
      suggestedSeedBase: 7001,
    });

    expect(row.mc_preparation).toEqual({
      scenario_label: "planning-single-target",
      suggested_run_count: 25,
      suggested_seed_base: 7001,
    });
  });


  it("defines an unlinked result-link foundation from package identifiers", () => {
    const row = buildPlanningMcPackage(snapshot());
    const link = buildEmptyPlanningResultLink(row);

    expect(link).toEqual({
      schema_version: "planning_result_link_v1",
      linked_package_id: `rt_planning_package:${row.planning_snapshot_id}`,
      linked_planning_snapshot_id: row.planning_snapshot_id,
      linked_planning_geometry_id: row.planning_geometry_id,
      linked_source_layout_id: "rt_layout_source",
      linked_source_geometry_id: "rt_layout:sha256:source",
      result_ref: null,
      status: "unlinked",
    });
  });

  it("exports, copies, and downloads packages without runtime, bridge, or MC execution", async () => {
    const bridgeCommand = vi.fn();
    const runtimeMutation = vi.fn();
    const monteCarloExecution = vi.fn();
    const writeText = vi.fn().mockResolvedValue(undefined);
    const click = vi.fn();
    vi.stubGlobal("navigator", { clipboard: { writeText } });
    vi.stubGlobal("document", {
      createElement: vi.fn(() => ({ href: "", download: "", click })),
    });

    const row = buildPlanningMcPackage(snapshot());
    const parsed = JSON.parse(exportPlanningMcPackageJson(row));
    const copyResult = await copyPlanningMcPackage(row);
    downloadPlanningMcPackage(row);

    expect(parsed.schema_version).toBe("rt_planning_mc_package_v1");
    expect(suggestedPlanningMcPackageFilename(row)).toContain("_package.json");
    expect(copyResult.ok).toBe(true);
    expect(writeText.mock.calls[0]![0]).toContain("rt_planning_mc_package_v1");
    expect(click).toHaveBeenCalled();
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeMutation).not.toHaveBeenCalled();
    expect(monteCarloExecution).not.toHaveBeenCalled();
  });
});
