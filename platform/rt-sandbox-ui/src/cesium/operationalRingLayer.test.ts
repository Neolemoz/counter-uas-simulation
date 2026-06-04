import { describe, expect, it } from "vitest";
import { WORLD_AXIS_HALF_EXTENT_M } from "@/world/bounds";
import {
  OPERATIONAL_RING_RADII_M,
  OPERATIONAL_RING_SPECS,
  syncOperationalRingLayer,
} from "./operationalRingLayer";

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

describe("operationalRingLayer", () => {
  it("defines unified-world operational radii", () => {
    expect(OPERATIONAL_RING_RADII_M.cityM).toBe(1000);
    expect(OPERATIONAL_RING_RADII_M.defenseM).toBe(3000);
    expect(OPERATIONAL_RING_RADII_M.warningM).toBe(5000);
    expect(OPERATIONAL_RING_RADII_M.spawnInnerM).toBe(5000);
    expect(OPERATIONAL_RING_RADII_M.spawnOuterM).toBe(WORLD_AXIS_HALF_EXTENT_M);
  });

  it("renders display-only ring entities when visible", () => {
    const viewer = createViewer();
    syncOperationalRingLayer(viewer as never, true);

    const ids = (viewer.entities.values as Array<{ id?: string }>).map((row) => row.id);
    expect(ids).toContain("rt-operational-ring-city");
    expect(ids).toContain("rt-operational-ring-spawn-inner");
    expect(ids).toContain("rt-operational-ring-spawn-band-label");
    expect(OPERATIONAL_RING_SPECS).toHaveLength(4);
  });

  it("clears ring entities when hidden", () => {
    const viewer = createViewer();
    syncOperationalRingLayer(viewer as never, true);
    syncOperationalRingLayer(viewer as never, false);
    expect(viewer.entities.values).toHaveLength(0);
  });
});
