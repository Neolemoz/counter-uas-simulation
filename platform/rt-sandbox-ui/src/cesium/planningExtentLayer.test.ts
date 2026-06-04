import { describe, expect, it, vi } from "vitest";
import { WORLD_BOUNDS } from "@/world/bounds";
import { planningExtentById } from "./planningWorld";
import {
  PLANNING_EXTENT_LAYER_COPY,
  isInsidePlanningExtent,
  isInsideRuntimeBounds,
  planningExtentAreaKm2,
  syncPlanningExtentLayer,
} from "./planningExtentLayer";

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

function entities(viewer: ReturnType<typeof createViewer>) {
  return viewer.entities.values as Array<{ id?: string; name?: string; label?: { text?: string } }>;
}

describe("planningExtentLayer", () => {
  it("renders Planning extent boundary metadata distinct from runtime bounds", () => {
    const viewer = createViewer();

    syncPlanningExtentLayer(viewer as never, planningExtentById("planning_20km"), true);

    const rows = entities(viewer);
    expect(rows.map((row) => row.id)).toContain("rt-planning-extent-ring");
    expect(rows.find((row) => row.id === "rt-planning-extent-ring")?.name).toContain(
      "20 km Planning World boundary (planning only)",
    );
    expect(rows.map((row) => row.id)).toContain("rt-planning-extent-runtime-distinction");
    expect(PLANNING_EXTENT_LAYER_COPY).toContain("not runtime bounds");
  });

  it("allows detecting coordinates outside runtime bounds but inside Planning extent", () => {
    const extent = planningExtentById("planning_5km");
    const vertex = { x: 1200, y: 0 };

    expect(isInsideRuntimeBounds(vertex)).toBe(false);
    expect(isInsidePlanningExtent(vertex, extent)).toBe(true);
  });

  it("does not mutate runtime bounds or bridge/runtime paths", () => {
    const viewer = createViewer();
    const before = JSON.stringify(WORLD_BOUNDS);
    const bridgeCommand = vi.fn();
    const runtimeCommand = vi.fn();

    syncPlanningExtentLayer(viewer as never, planningExtentById("planning_10km"), true);

    expect(JSON.stringify(WORLD_BOUNDS)).toBe(before);
    expect(WORLD_BOUNDS.x.max).toBe(500);
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeCommand).not.toHaveBeenCalled();
  });

  it("reports approximate circular Planning area", () => {
    expect(planningExtentAreaKm2(planningExtentById("planning_10km"))).toBeCloseTo(314.159, 3);
  });
});
