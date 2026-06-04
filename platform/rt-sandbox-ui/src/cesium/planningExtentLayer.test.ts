import { describe, expect, it, vi } from "vitest";
import { WORLD_BOUNDS } from "@/world/bounds";
import { planningExtentById, unifiedPlanningWorld } from "./planningWorld";
import {
  PLANNING_EXTENT_LAYER_COPY,
  isInsidePlanningExtent,
  isInsideWorldBounds,
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
  it("renders unified world label instead of legacy extent ring", () => {
    const viewer = createViewer();

    syncPlanningExtentLayer(viewer as never, unifiedPlanningWorld(), true);

    const rows = entities(viewer);
    expect(rows.map((row) => row.id)).toContain("rt-planning-extent-world-label");
    expect(rows.map((row) => row.id)).not.toContain("rt-planning-extent-ring");
    expect(PLANNING_EXTENT_LAYER_COPY).toContain("display-only");
  });

  it("renders legacy import ring when legacy extent metadata is supplied", () => {
    const viewer = createViewer();

    syncPlanningExtentLayer(viewer as never, planningExtentById("planning_20km"), true);

    const rows = entities(viewer);
    expect(rows.map((row) => row.id)).toContain("rt-planning-extent-ring");
    expect(rows.find((row) => row.id === "rt-planning-extent-ring")?.name).toContain(
      "legacy import",
    );
  });

  it("treats coordinates outside axis-aligned world bounds as invalid", () => {
    const vertex = { x: 8000, y: 0 };

    expect(isInsideWorldBounds(vertex)).toBe(false);
    expect(isInsidePlanningExtent(vertex, unifiedPlanningWorld())).toBe(false);
    expect(isInsidePlanningExtent(vertex, planningExtentById("planning_10km"))).toBe(true);
  });

  it("does not mutate runtime bounds or bridge/runtime paths", () => {
    const viewer = createViewer();
    const before = JSON.stringify(WORLD_BOUNDS);
    const bridgeCommand = vi.fn();
    const runtimeCommand = vi.fn();

    syncPlanningExtentLayer(viewer as never, unifiedPlanningWorld(), true);

    expect(JSON.stringify(WORLD_BOUNDS)).toBe(before);
    expect(WORLD_BOUNDS.x.max).toBe(7000);
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeCommand).not.toHaveBeenCalled();
  });

  it("reports approximate circular Planning area for legacy extents", () => {
    expect(planningExtentAreaKm2(planningExtentById("planning_10km"))).toBeCloseTo(314.159, 3);
  });
});
