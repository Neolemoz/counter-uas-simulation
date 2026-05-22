import { describe, expect, it } from "vitest";
import { sampleFictionalHeight } from "./fictionalTerrain";
import type { ReplaySaBundle } from "@/replay/bundleSchema";

const bundle = {
  scenario: {
    terrain_model: {
      type: "fictional_heightmap" as const,
      grid_enu_m: {
        origin: [-2000, -2000],
        spacing_m: 200,
        size: 4,
        heights_m: [
          [0, 10, 20, 10],
          [10, 40, 50, 20],
          [5, 30, 60, 15],
          [0, 5, 10, 0],
        ],
      },
    },
  },
} as ReplaySaBundle;

describe("sampleFictionalHeight", () => {
  it("returns zero outside grid", () => {
    expect(sampleFictionalHeight(bundle, 0, 0)).toBe(0);
  });

  it("returns positive height on ridge proxy", () => {
    expect(sampleFictionalHeight(bundle, -1900, -1900)).toBeGreaterThan(0);
  });
});
