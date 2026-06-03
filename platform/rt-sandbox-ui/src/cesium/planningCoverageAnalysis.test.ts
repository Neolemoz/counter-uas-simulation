import { describe, expect, it, vi } from "vitest";
import {
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
  estimatePlanningCoverage,
  type PlanningPolygonState,
  type PlanningRadarState,
} from "./planningDrawing";
import { analyzePlanningCoverage } from "./planningCoverageAnalysis";

const squarePolygon: PlanningPolygonState = {
  draftVertices: [],
  completedVertices: [
    { x: 0, y: 0 },
    { x: 1000, y: 0 },
    { x: 1000, y: 1000 },
    { x: 0, y: 1000 },
  ],
};

function radarState(rangeM: number, positions: { id: string; x: number; y: number }[]): PlanningRadarState {
  return {
    ...EMPTY_PLANNING_RADARS,
    sites: positions.map((position) => ({
      id: position.id,
      position: { x: position.x, y: position.y },
      radar_type: "Medium Radar",
      detection_range_m: rangeM,
    })),
  };
}

describe("planning coverage analysis", () => {
  it("preserves the legacy coverage estimate output", () => {
    const radars = radarState(300, [{ id: "planning-radar-1", x: 500, y: 500 }]);

    const legacyEstimate = estimatePlanningCoverage(squarePolygon, radars, 10);
    const analysis = analyzePlanningCoverage(squarePolygon, radars, 10);

    expect(analysis.estimate).toEqual(legacyEstimate);
    expect(analysis.estimate.coveragePercent).toBe(32);
    expect(analysis.estimate.estimatedCoveredAreaM2).toBe(320_000);
    expect(analysis.estimate.estimatedUncoveredAreaM2).toBe(680_000);
  });

  it("records coverageCount and coveringRadarIds for sampled cells", () => {
    const radars = radarState(900, [
      { id: "planning-radar-1", x: 450, y: 500 },
      { id: "planning-radar-2", x: 550, y: 500 },
    ]);

    const analysis = analyzePlanningCoverage(squarePolygon, radars, 4);

    expect(analysis.sampledCells).toHaveLength(16);
    expect(analysis.coveredCells).toHaveLength(16);
    expect(analysis.uncoveredCells).toHaveLength(0);
    expect(analysis.sampledCells.every((cell) => cell.coverageCount === 2)).toBe(true);
    expect(
      analysis.sampledCells.every(
        (cell) =>
          cell.coveringRadarIds.includes("planning-radar-1") &&
          cell.coveringRadarIds.includes("planning-radar-2"),
      ),
    ).toBe(true);
  });

  it("exposes overlap derivation without double-counting coverage percent", () => {
    const radars = radarState(900, [
      { id: "planning-radar-1", x: 450, y: 500 },
      { id: "planning-radar-2", x: 550, y: 500 },
    ]);

    const analysis = analyzePlanningCoverage(squarePolygon, radars, 4);

    expect(analysis.overlapCells).toHaveLength(16);
    expect(analysis.overlapCellCount).toBe(16);
    expect(analysis.overlapAreaM2).toBe(1_000_000);
    expect(analysis.overlapPercent).toBe(100);
    expect(analysis.redundancyPercent).toBe(100);
    expect(analysis.estimate.coveragePercent).toBe(100);
    expect(analysis.estimate.estimatedCoveredAreaM2).toBe(
      analysis.estimate.totalPolygonAreaM2,
    );
  });

  it("keeps empty planning coverage behavior unchanged", () => {
    const analysis = analyzePlanningCoverage(
      { draftVertices: [], completedVertices: null },
      EMPTY_PLANNING_RADARS,
    );

    expect(analysis.estimate).toEqual({
      radarCount: 0,
      totalPolygonAreaM2: 0,
      estimatedCoveredAreaM2: 0,
      estimatedUncoveredAreaM2: 0,
      coveragePercent: 0,
      coveredCells: [],
      uncoveredCells: [],
      blindSpotHints: [],
    });
    expect(analysis.sampledCells).toEqual([]);
    expect(analysis.overlapCells).toEqual([]);
    expect(analysis.overlapCellCount).toBe(0);
    expect(analysis.overlapAreaM2).toBe(0);
    expect(analysis.overlapPercent).toBe(0);
    expect(analysis.redundancyPercent).toBe(0);
  });



  it("recommends a radar for the no radar case", () => {
    const analysis = analyzePlanningCoverage(squarePolygon, EMPTY_PLANNING_RADARS, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    });

    expect(analysis.blindSpotV2.uncoveredPercent).toBe(100);
    expect(analysis.blindSpotV2.majorUncoveredSectors.length).toBeGreaterThan(0);
    expect(analysis.radarRecommendation?.recommendedPresetId).toBe("long");
    expect(analysis.radarRecommendation?.reason).toContain("Add Long Range near NE uncovered sector.");
  });

  it("does not recommend more radar when sampled cells are fully covered", () => {
    const radars = radarState(900, [
      { id: "planning-radar-1", x: 450, y: 500 },
      { id: "planning-radar-2", x: 550, y: 500 },
    ]);

    const analysis = analyzePlanningCoverage(squarePolygon, radars, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    });

    expect(analysis.blindSpotV2.summary).toBe("No uncovered planning cells.");
    expect(analysis.blindSpotV2.majorUncoveredSectors).toEqual([]);
    expect(analysis.radarRecommendation).toBeNull();
  });

  it("recommends against the largest uncovered sector", () => {
    const radars = radarState(250, [{ id: "planning-radar-1", x: 150, y: 150 }]);

    const analysis = analyzePlanningCoverage(squarePolygon, radars, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    });

    expect(analysis.blindSpotV2.majorUncoveredSectors[0].sector).toBe("NE");
    expect(analysis.radarRecommendation?.recommendedPresetId).toBe("long");
    expect(analysis.radarRecommendation?.approximatePlacement).toEqual({ x: 750, y: 750 });
    expect(analysis.radarRecommendation?.reason).toContain("near NE uncovered sector");
  });

  it("keeps recommendations deterministic", () => {
    const radars = radarState(250, [{ id: "planning-radar-1", x: 150, y: 150 }]);

    const first = analyzePlanningCoverage(squarePolygon, radars, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    });
    const second = analyzePlanningCoverage(squarePolygon, radars, 4, {
      radarPresets: PLANNING_RADAR_PRESETS,
    });

    expect(second.blindSpotV2).toEqual(first.blindSpotV2);
    expect(second.radarRecommendation).toEqual(first.radarRecommendation);
  });

  it("does not call runtime, bridge, sensor, LOS, or MC mutation paths", () => {
    const bridgeCommand = vi.fn();
    const runtimeSpawn = vi.fn();
    const sensorUpdate = vi.fn();
    const losUpdate = vi.fn();
    const monteCarloUpdate = vi.fn();
    const radars = radarState(500, [{ id: "planning-radar-1", x: 500, y: 500 }]);

    analyzePlanningCoverage(squarePolygon, radars, 8);

    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeSpawn).not.toHaveBeenCalled();
    expect(sensorUpdate).not.toHaveBeenCalled();
    expect(losUpdate).not.toHaveBeenCalled();
    expect(monteCarloUpdate).not.toHaveBeenCalled();
    expect(radars.sites[0]).not.toHaveProperty("entity_id");
    expect(radars.sites[0]).not.toHaveProperty("entity_type");
  });
});
