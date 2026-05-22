import { describe, expect, it } from "vitest";
import {
  getBracketingSamples,
  interpolatePosition,
  splitTrailAndFuture,
} from "./trackPlayback";

const samples = [
  { t: 2, x_m: 0, y_m: 0, z_m: 10 },
  { t: 4, x_m: 100, y_m: 0, z_m: 10 },
  { t: 8, x_m: 200, y_m: 50, z_m: 20 },
];

describe("trackPlayback", () => {
  it("interpolates between sparse samples", () => {
    const pos = interpolatePosition(samples, 3);
    expect(pos).not.toBeNull();
    expect(pos!.interpolated).toBe(true);
    expect(pos!.x_m).toBeCloseTo(50, 1);
  });

  it("clamps before first sample", () => {
    const pos = interpolatePosition(samples, 0);
    expect(pos!.x_m).toBe(0);
    expect(pos!.interpolated).toBe(false);
  });

  it("splits trail and future", () => {
    const { trail, future, head } = splitTrailAndFuture(samples, 5);
    expect(trail.map((s) => s.t)).toEqual([2, 4]);
    expect(future.map((s) => s.t)).toEqual([8]);
    expect(head?.interpolated).toBe(true);
  });

  it("getBracketingSamples finds between", () => {
    const b = getBracketingSamples(samples, 3);
    expect(b.kind).toBe("between");
  });
});
