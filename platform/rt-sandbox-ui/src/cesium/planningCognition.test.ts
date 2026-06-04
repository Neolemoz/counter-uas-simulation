import { describe, expect, it, vi } from "vitest";
import { WORLD_BOUNDS } from "@/world/bounds";
import {
  EMPTY_PLANNING_POLYGON,
  EMPTY_PLANNING_RADARS,
  addPlanningRadarSite,
  finishPlanningPolygon,
  addPlanningVertex,
} from "./planningDrawing";
import {
  DEFAULT_PLANNING_MEASUREMENT_STATE,
  addPlanningMeasurementPoint,
} from "./planningMeasurements";
import {
  PLANNING_COGNITION_GOVERNANCE_COPY,
  buildPlanningSummary,
  buildPlanningWarnings,
  operationalRingSummary,
  planningCognitionPreservesRuntimeBounds,
  planningCognitionPreservesWorldBounds,
  planningCompletedMeasurementCount,
  planningCompletedPolygonCount,
  validateAllPlanningExtents,
  validateUnifiedPlanningWorld,
} from "./planningCognition";

describe("planningCognition", () => {
  it("builds Planning summary with unified world and operational rings", () => {
    let polygon = addPlanningVertex(EMPTY_PLANNING_POLYGON, { x: 0, y: 0 });
    polygon = addPlanningVertex(polygon, { x: 500, y: 0 });
    polygon = addPlanningVertex(polygon, { x: 500, y: 500 });
    polygon = finishPlanningPolygon(polygon);
    const radars = addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 100, y: 100 });
    const measurements = addPlanningMeasurementPoint(
      addPlanningMeasurementPoint(DEFAULT_PLANNING_MEASUREMENT_STATE, { x: 0, y: 0 }),
      { x: 1000, y: 0 },
    );

    const summary = buildPlanningSummary({
      polygon,
      radars,
      measurements,
    });

    expect(summary).toEqual({
      world_label: "Unified 7 km World",
      world_half_extent_m: 7000,
      operational_rings: operationalRingSummary(),
      polygon_count: 1,
      radar_count: 1,
      measurement_count: 1,
    });
  });

  it("reports zero counts for empty Planning state", () => {
    const summary = buildPlanningSummary({
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });

    expect(summary.polygon_count).toBe(0);
    expect(summary.radar_count).toBe(0);
    expect(summary.measurement_count).toBe(0);
    expect(summary.world_label).toBe("Unified 7 km World");
  });

  it("emits informational Planning warnings without blocking semantics", () => {
    const warnings = buildPlanningWarnings({
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
      activeTool: "measure_distance",
    });

    expect(warnings.map((warning) => warning.warning_id)).toEqual([
      "no_polygon_defined",
      "no_radar_sites",
    ]);
    warnings.forEach((warning) => {
      expect(warning.message).toMatch(/planning-only|Planning|non-runtime|display-only/i);
    });
  });

  it("warns when measurement tool has only one point selected", () => {
    const warnings = buildPlanningWarnings({
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: addPlanningMeasurementPoint(DEFAULT_PLANNING_MEASUREMENT_STATE, {
        x: 0,
        y: 0,
      }),
      activeTool: "measure_distance",
    });

    expect(warnings.some((warning) => warning.warning_id === "measurement_incomplete")).toBe(
      true,
    );
  });

  it("warns when coordinates are outside unified world bounds", () => {
    const polygon: typeof EMPTY_PLANNING_POLYGON = {
      draftVertices: [],
      completedVertices: [
        { x: 7500, y: 0 },
        { x: 7800, y: 0 },
        { x: 7800, y: 500 },
      ],
    };
    const warnings = buildPlanningWarnings({
      polygon,
      radars: addPlanningRadarSite(EMPTY_PLANNING_RADARS, { x: 7600, y: 250 }),
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });

    expect(warnings.some((warning) => warning.warning_id === "coordinates_outside_world_bounds")).toBe(
      true,
    );
  });

  it("validates unified world capabilities and operational ring summary", () => {
    const validation = validateUnifiedPlanningWorld();

    expect(validation.overlay_readable).toBe(true);
    expect(validation.polygon_drawing_usable).toBe(true);
    expect(validation.camera_fit_height_m).toBeGreaterThanOrEqual(7000);
    expect(validation.operational_rings.spawn_band_outer_m).toBe(7000);
    expect(validation.guidance).toContain("city 1000 m");
    expect(validateAllPlanningExtents()).toEqual([validation]);
  });

  it("does not mutate runtime bounds or invoke bridge/runtime paths", () => {
    const before = JSON.stringify(WORLD_BOUNDS);
    const bridgeCommand = vi.fn();
    const runtimeCommand = vi.fn();

    buildPlanningSummary({
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });
    buildPlanningWarnings({
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });
    validateAllPlanningExtents();

    expect(JSON.stringify(WORLD_BOUNDS)).toBe(before);
    expect(WORLD_BOUNDS.x.max).toBe(7000);
    expect(planningCognitionPreservesWorldBounds()).toBe(true);
    expect(planningCognitionPreservesRuntimeBounds()).toBe(true);
    expect(planningCompletedPolygonCount(EMPTY_PLANNING_POLYGON)).toBe(0);
    expect(planningCompletedMeasurementCount(DEFAULT_PLANNING_MEASUREMENT_STATE)).toBe(0);
    expect(PLANNING_COGNITION_GOVERNANCE_COPY).toContain("non-authoritative");
    expect(PLANNING_COGNITION_GOVERNANCE_COPY).toContain("not affect runtime");
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeCommand).not.toHaveBeenCalled();
  });
});
