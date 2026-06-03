import { describe, expect, it, vi } from "vitest";
import {
  EMPTY_PLANNING_RADARS,
  syncPlanningDefenseAreaLayer,
  type PlanningPolygonState,
} from "./planningDrawing";

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

function entityIds(viewer: ReturnType<typeof createViewer>): string[] {
  return viewer.entities.values.map((entity) => String((entity as { id?: string }).id));
}

const polygon: PlanningPolygonState = {
  draftVertices: [],
  completedVertices: [
    { x: 0, y: 0 },
    { x: 1000, y: 0 },
    { x: 1000, y: 1000 },
    { x: 0, y: 1000 },
  ],
};

describe("planning advisory Cesium overlays", () => {
  it("renders display-only advisory blind-spot and recommended radar entities", () => {
    const viewer = createViewer();

    syncPlanningDefenseAreaLayer(viewer as never, polygon, EMPTY_PLANNING_RADARS, {
      showCoverage: true,
      showBlindSpots: true,
    });

    const ids = entityIds(viewer);
    expect(ids.some((id) => id.includes("advisory-blind-sector-"))).toBe(true);
    expect(ids).toContain("rt-planning-defense-area-advisory-radar-marker");
    expect(ids).toContain("rt-planning-defense-area-advisory-radar-range");
    expect(ids.every((id) => !id.startsWith("rt-entity-"))).toBe(true);
  });

  it("keeps advisory overlays tied to Planning blind-spot display", () => {
    const viewer = createViewer();

    syncPlanningDefenseAreaLayer(viewer as never, polygon, EMPTY_PLANNING_RADARS, {
      showCoverage: true,
      showBlindSpots: false,
    });

    const ids = entityIds(viewer);
    expect(ids.some((id) => id.includes("advisory-blind-sector-"))).toBe(false);
    expect(ids).not.toContain("rt-planning-defense-area-advisory-radar-marker");
    expect(ids).not.toContain("rt-planning-defense-area-advisory-radar-range");
  });

  it("does not call runtime or bridge mutation paths", () => {
    const viewer = createViewer();
    const bridgeCommand = vi.fn();
    const runtimeSpawn = vi.fn();

    syncPlanningDefenseAreaLayer(viewer as never, polygon, EMPTY_PLANNING_RADARS, {
      showCoverage: true,
      showBlindSpots: true,
    });

    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeSpawn).not.toHaveBeenCalled();
  });
});
