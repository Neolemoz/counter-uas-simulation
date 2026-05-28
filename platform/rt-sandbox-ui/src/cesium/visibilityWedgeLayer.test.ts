import { describe, expect, it } from "vitest";
import {
  boundsDiagonalHalfM,
  clearVisibilityWedgeLayer,
  countWedgePolylines,
  syncVisibilityWedgeLayer,
  wedgeRayEndpoints,
} from "./visibilityWedgeLayer";

describe("visibilityWedgeLayer", () => {
  it("computes bounds diagonal half", () => {
    expect(boundsDiagonalHalfM()).toBeCloseTo(707.1, 0);
  });

  it("clamps wedge polyline count to budget", () => {
    expect(countWedgePolylines(30)).toBeLessThanOrEqual(4);
    expect(countWedgePolylines(30)).toBeGreaterThanOrEqual(2);
  });

  it("produces left/right ray endpoints", () => {
    const { left, right } = wedgeRayEndpoints(0, 0, 10, 0, 30, 100);
    expect(left[0]).toBeGreaterThan(50);
    expect(right[0]).toBeGreaterThan(50);
    expect(left[2]).toBe(10);
  });

  it("noops on null viewer", () => {
    expect(() =>
      syncVisibilityWedgeLayer(null, null, true, false),
    ).not.toThrow();
    expect(() => clearVisibilityWedgeLayer(null)).not.toThrow();
  });
});
