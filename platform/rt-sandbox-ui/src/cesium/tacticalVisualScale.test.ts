import { describe, expect, it } from "vitest";
import type { EnuPoint } from "./tacticalGeometry";
import { pathLengthM } from "./tacticalTimingLabels";
import {
  corridorHalfWidthM,
  corridorPolylineWidths,
  decimateEnuPathForRender,
  tacticalDashLengthPx,
  tacticalRankingCuePixelOffset,
  tacticalTimingLabelPixelOffset,
  tacticalTrajectoryWidthPx,
  TACTICAL_CORRIDOR_MAX_HALF_WIDTH_M,
  TACTICAL_CORRIDOR_MIN_HALF_WIDTH_M,
} from "./tacticalVisualScale";
import { WORLD_FIT_CAMERA_HEIGHT_M } from "@/world/bounds";
import { TIGHT_BOUNDS_CAMERA_HEIGHT_M } from "./visualStyle";

const path7km: EnuPoint[] = [
  { x: 0, y: 0, z: 20 },
  { x: 6000, y: 4000, z: 35 },
];

const corridor5km: EnuPoint[] = [
  { x: 0, y: 0, z: 20 },
  { x: 3000, y: 4000, z: 30 },
  { x: 5000, y: 5000, z: 35 },
];

describe("tacticalVisualScale", () => {
  it("widens corridor ribbon at world-fit camera for 7km legs", () => {
    const width = corridorHalfWidthM(WORLD_FIT_CAMERA_HEIGHT_M, path7km);
    expect(width).toBeGreaterThan(120);
    expect(width).toBeLessThanOrEqual(TACTICAL_CORRIDOR_MAX_HALF_WIDTH_M);
  });

  it("keeps corridor ribbon near base width at city-core camera", () => {
    const shortLeg: EnuPoint[] = [
      { x: 0, y: 0, z: 20 },
      { x: 120, y: 30, z: 35 },
    ];
    const width = corridorHalfWidthM(TIGHT_BOUNDS_CAMERA_HEIGHT_M, shortLeg);
    expect(width).toBeGreaterThanOrEqual(TACTICAL_CORRIDOR_MIN_HALF_WIDTH_M);
    expect(width).toBeLessThan(45);
  });

  it("scales 5km corridor width between city-core and world-fit", () => {
    const cityCore = corridorHalfWidthM(TIGHT_BOUNDS_CAMERA_HEIGHT_M, corridor5km);
    const worldFit = corridorHalfWidthM(WORLD_FIT_CAMERA_HEIGHT_M, corridor5km);
    expect(worldFit).toBeGreaterThan(cityCore);
    expect(worldFit).toBeGreaterThan(80);
  });

  it("increases trajectory width for 7km paths at world-fit camera", () => {
    const cityCore = tacticalTrajectoryWidthPx(
      "telemetry",
      TIGHT_BOUNDS_CAMERA_HEIGHT_M,
      path7km,
    );
    const worldFit = tacticalTrajectoryWidthPx(
      "telemetry",
      WORLD_FIT_CAMERA_HEIGHT_M,
      path7km,
    );
    expect(worldFit).toBeGreaterThanOrEqual(cityCore);
    expect(worldFit).toBeGreaterThanOrEqual(4);
  });

  it("uses longer dash lengths for long-range world-fit paths", () => {
    const legLengthM = pathLengthM(path7km);
    const shortDash = tacticalDashLengthPx(
      "telemetry",
      TIGHT_BOUNDS_CAMERA_HEIGHT_M,
      200,
    );
    const longDash = tacticalDashLengthPx(
      "telemetry",
      WORLD_FIT_CAMERA_HEIGHT_M,
      legLengthM,
    );
    expect(longDash).toBeGreaterThan(shortDash);
  });

  it("scales timing and ranking label offsets with camera height", () => {
    const cityCoreTiming = tacticalTimingLabelPixelOffset(
      TIGHT_BOUNDS_CAMERA_HEIGHT_M,
      "intercept_top",
    );
    const worldFitTiming = tacticalTimingLabelPixelOffset(
      WORLD_FIT_CAMERA_HEIGHT_M,
      "intercept_top",
    );
    expect(worldFitTiming.y).toBeGreaterThanOrEqual(cityCoreTiming.y);

    const cityCoreRank = tacticalRankingCuePixelOffset(
      TIGHT_BOUNDS_CAMERA_HEIGHT_M,
      2,
    );
    const worldFitRank = tacticalRankingCuePixelOffset(
      WORLD_FIT_CAMERA_HEIGHT_M,
      2,
    );
    expect(worldFitRank.y).toBeLessThanOrEqual(cityCoreRank.y);
  });

  it("boosts corridor polyline width for long legs", () => {
    const short = corridorPolylineWidths(TIGHT_BOUNDS_CAMERA_HEIGHT_M, 200);
    const long = corridorPolylineWidths(WORLD_FIT_CAMERA_HEIGHT_M, pathLengthM(path7km));
    expect(long.edgeWidth).toBeGreaterThanOrEqual(short.edgeWidth);
  });

  it("decimates dense render paths without dropping endpoints", () => {
    const dense = Array.from({ length: 120 }, (_, i) => ({
      x: i * 50,
      y: 0,
      z: 10,
    }));
    const decimated = decimateEnuPathForRender(dense, 48);
    expect(decimated.length).toBeLessThanOrEqual(48);
    expect(decimated[0]).toEqual(dense[0]);
    expect(decimated.at(-1)).toEqual(dense.at(-1));
  });
});
