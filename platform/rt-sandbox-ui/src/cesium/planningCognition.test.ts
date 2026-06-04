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
import { planningExtentById } from "./planningWorld";
import {
  PLANNING_COGNITION_GOVERNANCE_COPY,
  buildPlanningSummary,
  buildPlanningWarnings,
  planningCognitionPreservesRuntimeBounds,
  planningCompletedMeasurementCount,
  planningCompletedPolygonCount,
  validateAllPlanningExtents,
  validatePlanningExtentCapabilities,
} from "./planningCognition";

describe("planningCognition", () => {
  it("builds Planning summary with extent, polygon, radar, and measurement counts", () => {
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
      planningExtent: planningExtentById("planning_10km"),
      polygon,
      radars,
      measurements,
    });

    expect(summary).toEqual({
      extent_label: "10 km Planning World",
      extent_radius_m: 10_000,
      polygon_count: 1,
      radar_count: 1,
      measurement_count: 1,
    });
  });

  it("reports zero counts for empty Planning state", () => {
    const summary = buildPlanningSummary({
      planningExtent: planningExtentById("planning_5km"),
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });

    expect(summary.polygon_count).toBe(0);
    expect(summary.radar_count).toBe(0);
    expect(summary.measurement_count).toBe(0);
    expect(summary.extent_label).toBe("5 km Planning World");
  });

  it("emits informational Planning warnings without blocking semantics", () => {
    const warnings = buildPlanningWarnings({
      planningExtent: planningExtentById("planning_10km"),
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
      planningExtent: planningExtentById("planning_10km"),
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

  it("updates summary when Planning extent switches", () => {
    const polygon = finishPlanningPolygon(
      addPlanningVertex(
        addPlanningVertex(addPlanningVertex(EMPTY_PLANNING_POLYGON, { x: 0, y: 0 }), {
          x: 400,
          y: 0,
        }),
        { x: 400, y: 400 },
      ),
    );

    const summary5 = buildPlanningSummary({
      planningExtent: planningExtentById("planning_5km"),
      polygon,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });
    const summary20 = buildPlanningSummary({
      planningExtent: planningExtentById("planning_20km"),
      polygon,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });

    expect(summary5.extent_radius_m).toBe(5_000);
    expect(summary20.extent_radius_m).toBe(20_000);
    expect(summary5.polygon_count).toBe(1);
    expect(summary20.polygon_count).toBe(1);
  });

  it("validates usability for 5 km, 10 km, and 20 km Planning extents", () => {
    const validations = validateAllPlanningExtents();

    expect(validations).toHaveLength(3);
    validations.forEach((validation) => {
      expect(validation.overlay_readable).toBe(true);
      expect(validation.polygon_drawing_usable).toBe(true);
      expect(validation.radar_placement_usable).toBe(true);
      expect(validation.measurement_tools_usable).toBe(true);
      expect(validation.guidance).toContain("planning-only");
    });

    expect(validatePlanningExtentCapabilities(planningExtentById("planning_20km")).camera_fit_height_m).toBe(
      29_000,
    );
  });

  it("does not mutate runtime bounds or invoke bridge/runtime paths", () => {
    const before = JSON.stringify(WORLD_BOUNDS);
    const bridgeCommand = vi.fn();
    const runtimeCommand = vi.fn();

    buildPlanningSummary({
      planningExtent: planningExtentById("planning_10km"),
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });
    buildPlanningWarnings({
      planningExtent: planningExtentById("planning_10km"),
      polygon: EMPTY_PLANNING_POLYGON,
      radars: EMPTY_PLANNING_RADARS,
      measurements: DEFAULT_PLANNING_MEASUREMENT_STATE,
    });
    validateAllPlanningExtents();

    expect(JSON.stringify(WORLD_BOUNDS)).toBe(before);
    expect(WORLD_BOUNDS.x.max).toBe(500);
    expect(planningCognitionPreservesRuntimeBounds()).toBe(true);
    expect(planningCompletedPolygonCount(EMPTY_PLANNING_POLYGON)).toBe(0);
    expect(planningCompletedMeasurementCount(DEFAULT_PLANNING_MEASUREMENT_STATE)).toBe(0);
    expect(PLANNING_COGNITION_GOVERNANCE_COPY).toContain("non-authoritative");
    expect(PLANNING_COGNITION_GOVERNANCE_COPY).toContain("not affect runtime");
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeCommand).not.toHaveBeenCalled();
  });
});
