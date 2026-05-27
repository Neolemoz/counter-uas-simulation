import { describe, expect, it } from "vitest";
import {
  ENTITY_TYPE_LIMITS,
  MAX_ENTITY_COUNT,
  WORLD_BOUNDS,
  canSpawn,
  clampPose,
  isEditingAllowed,
} from "./bounds";

describe("bounds", () => {
  it("matches governance constants", () => {
    expect(MAX_ENTITY_COUNT).toBe(32);
    expect(ENTITY_TYPE_LIMITS.drone).toBe(8);
    expect(WORLD_BOUNDS.x.min).toBe(-500);
    expect(WORLD_BOUNDS.z.max).toBe(200);
  });

  it("clamps pose to world bounds", () => {
    const p = clampPose({ x: 9999, y: -9999, z: 300 });
    expect(p.x).toBe(500);
    expect(p.y).toBe(-500);
    expect(p.z).toBe(200);
  });

  it("blocks spawn at cap", () => {
    expect(
      canSpawn({ entity_count: 32, by_type: { drone: 8 } }, "drone").ok,
    ).toBe(false);
  });

  it("allows editing in running and paused", () => {
    expect(isEditingAllowed("running")).toBe(true);
    expect(isEditingAllowed("stopped")).toBe(false);
  });
});
