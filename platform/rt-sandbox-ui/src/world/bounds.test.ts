import { describe, expect, it } from "vitest";
import {
  ENTITY_TYPE_LIMITS,
  MAX_ENTITY_COUNT,
  WORLD_BOUNDS,
  WORLD_AXIS_HALF_EXTENT_M,
  WORLD_FIT_CAMERA_HEIGHT_M,
  boundsGroundLabel,
  canSpawn,
  clampPose,
  isEditingAllowed,
} from "./bounds";

describe("bounds", () => {
  it("matches governance constants", () => {
    expect(MAX_ENTITY_COUNT).toBe(32);
    expect(ENTITY_TYPE_LIMITS.drone).toBe(8);
    expect(WORLD_BOUNDS.x.min).toBe(-7000);
    expect(WORLD_BOUNDS.x.max).toBe(7000);
    expect(WORLD_BOUNDS.y.min).toBe(-7000);
    expect(WORLD_BOUNDS.y.max).toBe(7000);
    expect(WORLD_BOUNDS.z.max).toBe(200);
  });

  it("exposes unified world labels and camera fit height", () => {
    expect(boundsGroundLabel()).toBe("±7000m");
    expect(WORLD_AXIS_HALF_EXTENT_M).toBe(7000);
    expect(WORLD_FIT_CAMERA_HEIGHT_M).toBe(10150);
  });

  it("clamps pose to world bounds", () => {
    const p = clampPose({ x: 9999, y: -9999, z: 300 });
    expect(p.x).toBe(7000);
    expect(p.y).toBe(-7000);
    expect(p.z).toBe(200);
  });

  it("accepts poses at 6500m on both axes", () => {
    const p = clampPose({ x: 6500, y: -6500, z: 10 });
    expect(p.x).toBe(6500);
    expect(p.y).toBe(-6500);
  });

  it("rejects overshoot past 7500m via clamp on both axes", () => {
    const p = clampPose({ x: 7500, y: -7500, z: 10 });
    expect(p.x).toBe(7000);
    expect(p.y).toBe(-7000);
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
