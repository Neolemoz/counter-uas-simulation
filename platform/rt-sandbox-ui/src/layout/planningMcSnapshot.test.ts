import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import {
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
  type PlanningPolygonState,
  type PlanningRadarState,
} from "@/cesium/planningDrawing";
import { analyzePlanningCoverage } from "@/cesium/planningCoverageAnalysis";
import { planningExtentById } from "@/cesium/planningWorld";
import {
  buildPlanningMcSnapshot,
  copyPlanningMcSnapshot,
  downloadPlanningMcSnapshot,
  exportPlanningMcSnapshotJson,
  planningGeometryFingerprint,
  suggestedPlanningMcSnapshotFilename,
} from "./planningMcSnapshot";

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

function snapshot(createdUtc = "2026-06-04T00:00:00Z") {
  return buildPlanningMcSnapshot(
    POLYGON,
    RADARS,
    analyzePlanningCoverage(POLYGON, RADARS, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    }),
    {
      createdUtc,
      terrainMode: "ellipsoid",
      selectedLocationPreset: "bangkok",
      sourceLayoutId: "rt_layout_source",
      sourceGeometryId: "rt_layout:sha256:source",
    },
  );
}

describe("planningMcSnapshot", () => {
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

  it("creates a deterministic planning geometry fingerprint separate from RT layout IDs", () => {
    const first = planningGeometryFingerprint(POLYGON, RADARS);
    const second = planningGeometryFingerprint(POLYGON, {
      ...RADARS,
      sites: [{ ...RADARS.sites[0], id: "renamed-local-site" }],
    });

    expect(first).toBe(second);
    expect(first.startsWith("rt_planning:sha256:")).toBe(true);
    expect(first.startsWith("rt_layout:sha256:")).toBe(false);
  });

  it("builds a stable snapshot when created_utc is provided", () => {
    const first = snapshot();
    const second = snapshot();

    expect(second).toEqual(first);
    expect(first.schema_version).toBe("rt_planning_mc_snapshot_v1");
    expect(first.planning_snapshot_id.startsWith("rt_planning_snapshot:sha256:")).toBe(true);
    expect(first.planning_geometry_id.startsWith("rt_planning:sha256:")).toBe(true);
    expect(first.polygon.defense_area_vertices).toHaveLength(4);
    expect(first.radars.radar_sites[0]).toMatchObject({
      id: "planning-radar-1",
      radar_preset: "short",
      detection_range_m: 500,
    });
    expect(first.planning_extent).toEqual({
      planning_extent_id: "planning_10km",
      planning_extent_radius_m: 10000,
      planning_extent_label: "10 km Planning World",
    });
    expect(first.presentation).toEqual({
      terrain_mode: "ellipsoid",
      selected_location_preset: "bangkok",
    });
    expect(first.provenance.source_layout_id).toBe("rt_layout_source");
    expect(first.provenance.source_geometry_id).toBe("rt_layout:sha256:source");
  });

  it("preserves explicit Planning extent metadata", () => {
    const row = buildPlanningMcSnapshot(
      POLYGON,
      RADARS,
      analyzePlanningCoverage(POLYGON, RADARS, 4, {
        radarPresets: PLANNING_RADAR_PRESETS,
      }),
      {
        createdUtc: "2026-06-04T00:00:00Z",
        terrainMode: "ellipsoid",
        selectedLocationPreset: "bangkok",
        planningExtent: planningExtentById("planning_20km"),
      },
    );

    expect(row.planning_extent).toEqual({
      planning_extent_id: "planning_20km",
      planning_extent_radius_m: 20000,
      planning_extent_label: "20 km Planning World",
    });
  });

  it("includes analytics and advisory summaries", () => {
    const row = snapshot().analytics_summary;

    expect(row.coverage_percent).toBeGreaterThan(0);
    expect(row.overlap_percent).toBe(0);
    expect(row.redundancy_percent).toBe(0);
    expect(row.blind_spot_summary).toContain("uncovered");
    expect(row.recommendation_summary.suggested_radar).toBeTruthy();
    expect(row.recommendation_summary.suggested_position).toEqual(
      expect.objectContaining({ x: expect.any(Number), y: expect.any(Number) }),
    );
    expect(row.recommendation_summary.reason).toContain("planning area uncovered");
  });

  it("exports snapshot JSON and safe filenames", () => {
    const row = snapshot();
    const parsed = JSON.parse(exportPlanningMcSnapshotJson(row));

    expect(parsed.schema_version).toBe("rt_planning_mc_snapshot_v1");
    expect(suggestedPlanningMcSnapshotFilename(row)).toContain(
      "rt_planning_snapshot_sha256_",
    );
  });

  it("copies and downloads snapshots without runtime, bridge, or MC execution", async () => {
    const bridgeCommand = vi.fn();
    const runtimeMutation = vi.fn();
    const monteCarloExecution = vi.fn();
    const writeText = vi.fn().mockResolvedValue(undefined);
    const click = vi.fn();
    vi.stubGlobal("navigator", { clipboard: { writeText } });
    vi.stubGlobal("document", {
      createElement: vi.fn(() => ({ href: "", download: "", click })),
    });

    const row = snapshot();
    const copyResult = await copyPlanningMcSnapshot(row);
    downloadPlanningMcSnapshot(row);

    expect(copyResult.ok).toBe(true);
    expect(writeText.mock.calls[0]![0]).toContain("rt_planning_mc_snapshot_v1");
    expect(click).toHaveBeenCalled();
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeMutation).not.toHaveBeenCalled();
    expect(monteCarloExecution).not.toHaveBeenCalled();
  });
});
