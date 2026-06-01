import { describe, expect, it } from "vitest";
import { sampleTerrainHeight } from "./rtFictionalTerrain";
import {
  DEFAULT_CAMERA_PITCH_DEG,
  markerDisplayZ,
  markerVisualRegistryZ,
  MARKER_SURFACE_LIFT_M,
  radarVolumeDomeCones,
  radarVolumeSphereCenterZ,
  RADAR_VOLUME_CENTER_BLEND_M,
  terrainBaseZ,
  TERRAIN_OVERVIEW_PITCH_DEG,
  ZONE_SURFACE_LIFT_M,
  zoneLabelLiftM,
} from "./terrainGrounding";

describe("terrainGrounding", () => {
  it("uses steeper default camera pitch than horizon-neutral", () => {
    expect(DEFAULT_CAMERA_PITCH_DEG).toBeLessThan(-45);
    expect(TERRAIN_OVERVIEW_PITCH_DEG).toBeLessThanOrEqual(DEFAULT_CAMERA_PITCH_DEG);
  });

  it("keeps marker visual AGL minimal above terrain", () => {
    expect(markerVisualRegistryZ(10)).toBeLessThan(2);
    expect(markerVisualRegistryZ(10)).toBeGreaterThanOrEqual(0.35);
    expect(markerVisualRegistryZ(40)).toBeLessThanOrEqual(1.15);
  });

  it("clamps markers to terrain with fixed lift when terrain display is on", () => {
    const x = 120;
    const y = -80;
    const terrain = sampleTerrainHeight(x, y);
    const withTerrain = markerDisplayZ(x, y, 10, true);
    const withoutTerrainDisplay = markerDisplayZ(x, y, 10, false);
    expect(withTerrain - terrain).toBeCloseTo(MARKER_SURFACE_LIFT_M, 5);
    expect(withTerrain).toBeLessThan(terrain + 0.2);
    expect(withoutTerrainDisplay - terrain).toBeLessThan(1.5);
  });

  it("orients radar volume dome as upper hemisphere on terrain", () => {
    const x = 40;
    const y = -120;
    const radius = 250;
    expect(radarVolumeSphereCenterZ(x, y, radius)).toBeCloseTo(
      terrainBaseZ(x, y) + RADAR_VOLUME_CENTER_BLEND_M,
      5,
    );
    const cones = radarVolumeDomeCones();
    expect(cones.minimumCone).toBe(0);
    expect(cones.maximumCone).toBeCloseTo(Math.PI / 2, 5);
  });

  it("keeps zone labels low above surface", () => {
    expect(zoneLabelLiftM("core")).toBeLessThan(4);
    expect(zoneLabelLiftM("warning")).toBeLessThan(5);
  });
});
