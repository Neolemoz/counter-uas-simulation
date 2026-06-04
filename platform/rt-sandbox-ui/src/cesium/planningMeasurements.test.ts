import { describe, expect, it, vi } from "vitest";
import { WORLD_BOUNDS } from "@/world/bounds";
import {
  DEFAULT_PLANNING_MEASUREMENT_STATE,
  PLANNING_MEASUREMENT_GOVERNANCE_COPY,
  addPlanningMeasurementPoint,
  planningBearingCardinal,
  planningCoordinateReadout,
  planningDistanceMeters,
  planningMeasurementSummary,
  planningMeasurementsPreserveRuntimeBounds,
  planningRadiusMetadata,
  setPlanningRadiusMeters,
  syncPlanningMeasurementLayer,
} from "./planningMeasurements";

function createViewer() {
  const values: unknown[] = [];
  return {
    scene: {},
    camera: {},
    entities: {
      values,
      add(entity: unknown) {
        values.push(entity);
        return entity;
      },
      remove(entity: unknown) {
        const index = values.indexOf(entity);
        if (index >= 0) values.splice(index, 1);
      },
    },
  };
}

function entityRows(viewer: ReturnType<typeof createViewer>) {
  return viewer.entities.values as Array<{ id?: string; name?: string }>;
}

describe("planningMeasurements", () => {
  it("calculates distance in meters and kilometers", () => {
    const state = addPlanningMeasurementPoint(
      addPlanningMeasurementPoint(DEFAULT_PLANNING_MEASUREMENT_STATE, { x: 0, y: 0 }),
      { x: 3000, y: 4000 },
    );
    const summary = planningMeasurementSummary(state);

    expect(planningDistanceMeters({ x: 0, y: 0 }, { x: 3000, y: 4000 })).toBe(5000);
    expect(summary.distance_m).toBe(5000);
    expect(summary.distance_km).toBe(5);
  });

  it("calculates cardinal bearing", () => {
    expect(planningBearingCardinal({ x: 0, y: 0 }, { x: 0, y: 10 })).toBe("N");
    expect(planningBearingCardinal({ x: 0, y: 0 }, { x: 10, y: 10 })).toBe("NE");
    expect(planningBearingCardinal({ x: 0, y: 0 }, { x: 10, y: 0 })).toBe("E");
    expect(planningBearingCardinal({ x: 0, y: 0 }, { x: -10, y: -10 })).toBe("SW");
  });

  it("reports radius display metadata", () => {
    expect(planningRadiusMetadata(3_000)).toEqual({
      radius_m: 3000,
      radius_km: 3,
      label: "3 km planning radius",
    });
    expect(
      planningMeasurementSummary(
        setPlanningRadiusMeters(DEFAULT_PLANNING_MEASUREMENT_STATE, 5_000),
      ).radius_label,
    ).toBe("5 km planning radius");
  });

  it("formats coordinate readouts", () => {
    expect(planningCoordinateReadout({ x: 123.4, y: -987.6 })).toBe("X 123m, Y -988m");
  });

  it("renders display-only measurement metadata", () => {
    const viewer = createViewer();
    const state = addPlanningMeasurementPoint(
      addPlanningMeasurementPoint(DEFAULT_PLANNING_MEASUREMENT_STATE, { x: 0, y: 0 }),
      { x: 1000, y: 0 },
    );

    syncPlanningMeasurementLayer(viewer as never, state, true);

    expect(entityRows(viewer).map((row) => row.id)).toContain(
      "rt-planning-measurement-distance-line",
    );
    expect(entityRows(viewer).map((row) => row.id)).toContain(
      "rt-planning-measurement-radius-ring",
    );
  });

  it("does not mutate runtime bounds or call bridge/runtime paths", () => {
    const before = JSON.stringify(WORLD_BOUNDS);
    const bridgeCommand = vi.fn();
    const runtimeCommand = vi.fn();

    planningMeasurementSummary(DEFAULT_PLANNING_MEASUREMENT_STATE);
    syncPlanningMeasurementLayer(createViewer() as never, DEFAULT_PLANNING_MEASUREMENT_STATE, true);

    expect(JSON.stringify(WORLD_BOUNDS)).toBe(before);
    expect(planningMeasurementsPreserveRuntimeBounds()).toBe(true);
    expect(PLANNING_MEASUREMENT_GOVERNANCE_COPY).toContain("not runtime authority");
    expect(PLANNING_MEASUREMENT_GOVERNANCE_COPY).toContain("sensor truth");
    expect(PLANNING_MEASUREMENT_GOVERNANCE_COPY).toContain("MC execution authority");
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeCommand).not.toHaveBeenCalled();
  });
});
