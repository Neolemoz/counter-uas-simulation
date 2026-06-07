import { describe, expect, it } from "vitest";
import { deriveCorridorCoveragePolylines } from "./runtimeCorridorCoverage";
import type { RuntimeRadarDisc } from "./runtimeCoverageAnalysis";

describe("deriveCorridorCoveragePolylines", () => {
  const radarDisc: RuntimeRadarDisc = {
    id: "radar-a",
    position: { x: 250, y: 0 },
    detectionRangeM: 250,
  };

  it("splits half-covered corridor into covered and uncovered polylines", () => {
    const result = deriveCorridorCoveragePolylines(
      [
        { x: 0, y: 0, z: 10 },
        { x: 1000, y: 0, z: 10 },
      ],
      [radarDisc],
    );
    expect(result.coveredPolylines.length).toBeGreaterThan(0);
    expect(result.uncoveredPolylines.length).toBeGreaterThan(0);
  });

  it("returns empty polylines for degenerate corridor input", () => {
    expect(deriveCorridorCoveragePolylines([], [radarDisc])).toEqual({
      coveredPolylines: [],
      uncoveredPolylines: [],
    });
  });
});
