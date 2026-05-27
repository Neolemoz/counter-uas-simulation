import { describe, expect, it } from "vitest";
import {
  applyTerrainDisplayOffset,
  contourLevelUnderM,
  generateContourPolylines,
  listElevationBandsM,
  listVegetationMarkers,
  nearestRidgeLabel,
  ridgeElevationBandColor,
  sampleTerrainHeight,
} from "./rtFictionalTerrain";

describe("rtFictionalTerrain", () => {
  it("samples height within grid", () => {
    const h = sampleTerrainHeight(0, 0);
    expect(h).toBeGreaterThanOrEqual(0);
    expect(h).toBeLessThan(80);
  });

  it("returns zero outside grid", () => {
    expect(sampleTerrainHeight(-900, 0)).toBe(0);
  });

  it("applies display offset as registry z plus terrain", () => {
    const z = applyTerrainDisplayOffset(0, 0, 10);
    expect(z).toBeGreaterThan(10);
  });

  it("resolves nearest ridge label", () => {
    expect(nearestRidgeLabel(0, 320)).toMatch(/ridge|valley|spur|saddle/i);
  });

  it("lists elevation bands from fixture", () => {
    expect(listElevationBandsM()).toEqual([10, 20, 30, 40]);
  });

  it("generates contour polylines", () => {
    const contours = generateContourPolylines();
    expect(contours.length).toBeGreaterThan(0);
    expect(contours[0].points_enu_m.length).toBeGreaterThanOrEqual(2);
  });

  it("maps ridge band colors", () => {
    expect(ridgeElevationBandColor(5)).toContain("rgba");
    expect(ridgeElevationBandColor(45)).toContain("rgba");
  });

  it("resolves contour level under point", () => {
    expect(contourLevelUnderM(0, 350)).toBeGreaterThanOrEqual(10);
  });

  it("lists vegetation markers", () => {
    expect(listVegetationMarkers().length).toBeGreaterThanOrEqual(2);
  });
});
