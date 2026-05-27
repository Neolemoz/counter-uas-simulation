import { describe, expect, it } from "vitest";
import { generateContourPolylines } from "./rtFictionalTerrain";
import { clearTerrainContourLayer, syncTerrainContourLayer } from "./terrainContourLayer";

describe("terrainContourLayer", () => {
  it("generates multiple contour levels", () => {
    const contours = generateContourPolylines();
    const levels = new Set(contours.map((c) => c.level_m));
    expect(levels.size).toBeGreaterThan(1);
  });

  it("noops on null viewer", () => {
    expect(() => syncTerrainContourLayer(null, true)).not.toThrow();
    expect(() => clearTerrainContourLayer(null)).not.toThrow();
  });
});
