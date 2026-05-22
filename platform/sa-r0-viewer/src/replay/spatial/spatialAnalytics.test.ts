import { describe, expect, it } from "vitest";
import { maxGridCount, normalizeGrid } from "./spatialAnalytics";

describe("spatialAnalytics", () => {
  it("normalizes grid", () => {
    const g = normalizeGrid({
      origin_enu_m: [-2500, -500],
      spacing_m: 100,
      size: [40, 30],
    });
    expect(g.size).toEqual([40, 30]);
  });

  it("maxGridCount handles empty", () => {
    expect(maxGridCount({})).toBe(1);
  });
});
