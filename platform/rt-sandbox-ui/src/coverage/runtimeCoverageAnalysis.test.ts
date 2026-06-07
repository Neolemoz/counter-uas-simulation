import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import type { MirrorEntity } from "@/cesium/entityMarkers";
import type { DefenseZoneConfig } from "@/cesium/defenseZoneConfig";
import type { RadarDomeConfig } from "@/cesium/sensorDomeLayer";
import {
  adaptRuntimeRadarEntities,
  analyzeRuntimeCoverage,
  buildRuntimeCoverageInputs,
  buildWarningDiscPolygon,
  deriveRuntimeRadarDiscs,
  getCoveringRadarCount,
  getNearestRadarEdgeDistanceM,
  isProtectedCenterCovered,
} from "./runtimeCoverageAnalysis";
import { analyzeCorridorCoverage } from "./runtimeCorridorCoverage";

const fixturePath = join(
  dirname(fileURLToPath(import.meta.url)),
  "../../../../fixtures/rt_sandbox/runtime_coverage_golden_v1.json",
);

type GoldenCase = {
  protected_center_entity_id?: string;
  defense_zone_config?: Partial<DefenseZoneConfig>;
  radar_dome_config?: Partial<RadarDomeConfig>;
  entities: MirrorEntity[];
  sample_steps?: number;
  corridor_points?: Array<{ x: number; y: number; z: number }>;
  expect: Record<string, unknown>;
};

type GoldenFixture = {
  schema: string;
  cases: Record<string, GoldenCase>;
};

function loadGoldenFixture(): GoldenFixture {
  return JSON.parse(readFileSync(fixturePath, "utf8")) as GoldenFixture;
}

function runGoldenCase(caseId: keyof GoldenFixture["cases"]) {
  const golden = loadGoldenFixture();
  return golden.cases[caseId as string];
}

describe("runtime coverage analysis", () => {
  it("builds a warning-disc polygon adapter for runtime inputs", () => {
    const disc = buildWarningDiscPolygon({ x: 100, y: 200 }, 50);
    expect(disc).toHaveLength(72);
    expect(disc[0].x).toBeCloseTo(150, 5);
    expect(disc[0].y).toBeCloseTo(200, 5);
  });

  it("maps runtime radar entities to planning radar sites", () => {
    const entities: MirrorEntity[] = [
      { entity_id: "r2", entity_type: "radar", pose: { x: 10, y: 20, z: 5 } },
      { entity_id: "r1", entity_type: "radar", pose: { x: 0, y: 0, z: 5 } },
      { entity_id: "d1", entity_type: "drone", pose: { x: 0, y: 0, z: 5 } },
    ];

    const sites = adaptRuntimeRadarEntities(entities, 300);
    expect(sites).toHaveLength(2);
    expect(sites.map((site) => site.id)).toEqual(["r1", "r2"]);
    expect(sites[0].detection_range_m).toBe(300);
    expect(sites[0].radar_type).toBe("Runtime Radar");
  });

  it("returns protected_center_unavailable when center is missing", () => {
    const result = buildRuntimeCoverageInputs({
      entities: [],
      protectedCenterEntityId: null,
    });
    expect(result).toEqual({ ok: false, reason: "protected_center_unavailable" });
  });

  it("returns unsupported_defense_shape for rectangle defense zones", () => {
    const result = buildRuntimeCoverageInputs({
      entities: [
        {
          entity_id: "center-1",
          entity_type: "waypoint_marker",
          pose: { x: 0, y: 0, z: 10 },
        },
      ],
      protectedCenterEntityId: "center-1",
      defenseZoneConfig: { shape: "rectangle" },
    });
    expect(result).toEqual({ ok: false, reason: "unsupported_defense_shape" });
  });

  it("golden: one radar covering center", () => {
    const golden = runGoldenCase("one_radar_covering_center");
    const result = analyzeRuntimeCoverage(
      {
        entities: golden.entities,
        protectedCenterEntityId: golden.protected_center_entity_id ?? null,
        defenseZoneConfig: golden.defense_zone_config,
        radarDomeConfig: golden.radar_dome_config,
      },
      golden.sample_steps,
    );

    expect(result.availability).toBe("ready");
    expect(result.analysis?.estimate.coveragePercent).toBeGreaterThanOrEqual(
      golden.expect.coverage_percent_min as number,
    );
    expect(result.analysis?.overlapPercent).toBeLessThanOrEqual(
      golden.expect.overlap_percent_max as number,
    );

    const discs = deriveRuntimeRadarDiscs(golden.entities, golden.radar_dome_config!.detectionM!);
    const center = result.inputs!.protectedCenterPosition;
    expect(isProtectedCenterCovered(center, discs)).toBe(golden.expect.center_covered);
    expect(getCoveringRadarCount(center, discs)).toBe(golden.expect.covering_radar_count);
    expect(getNearestRadarEdgeDistanceM(center, discs)).toBe(
      golden.expect.nearest_edge_distance_m,
    );
  });

  it("golden: two-radar overlap", () => {
    const golden = runGoldenCase("two_radar_overlap");
    const result = analyzeRuntimeCoverage(
      {
        entities: golden.entities,
        protectedCenterEntityId: golden.protected_center_entity_id ?? null,
        defenseZoneConfig: golden.defense_zone_config,
        radarDomeConfig: golden.radar_dome_config,
      },
      golden.sample_steps,
    );

    expect(result.availability).toBe("ready");
    expect(result.analysis?.estimate.coveragePercent).toBeGreaterThanOrEqual(
      golden.expect.coverage_percent_min as number,
    );
    expect(result.analysis?.overlapPercent).toBeGreaterThanOrEqual(
      golden.expect.overlap_percent_min as number,
    );
    expect(result.analysis?.redundancyPercent).toBeGreaterThanOrEqual(
      golden.expect.redundancy_percent_min as number,
    );

    const center = result.inputs!.protectedCenterPosition;
    const discs = deriveRuntimeRadarDiscs(golden.entities, golden.radar_dome_config!.detectionM!);
    expect(getCoveringRadarCount(center, discs)).toBe(golden.expect.covering_radar_count);
  });

  it("golden: blind spot sectors when radar offset from center", () => {
    const golden = runGoldenCase("blind_spot");
    const result = analyzeRuntimeCoverage(
      {
        entities: golden.entities,
        protectedCenterEntityId: golden.protected_center_entity_id ?? null,
        defenseZoneConfig: golden.defense_zone_config,
        radarDomeConfig: golden.radar_dome_config,
      },
      golden.sample_steps,
    );

    expect(result.availability).toBe("ready");
    expect(result.analysis?.estimate.coveragePercent).toBeLessThanOrEqual(
      golden.expect.coverage_percent_max as number,
    );
    expect(result.analysis?.blindSpotV2.majorUncoveredSectors.length).toBeGreaterThanOrEqual(
      golden.expect.blind_spot_sector_count_min as number,
    );

    const center = result.inputs!.protectedCenterPosition;
    const discs = deriveRuntimeRadarDiscs(golden.entities, golden.radar_dome_config!.detectionM!);
    expect(isProtectedCenterCovered(center, discs)).toBe(false);
    expect(getNearestRadarEdgeDistanceM(center, discs)).toBeGreaterThanOrEqual(
      golden.expect.nearest_edge_distance_m_min as number,
    );
  });

  it("golden: no radar yields zero coverage", () => {
    const golden = runGoldenCase("no_radar");
    const result = analyzeRuntimeCoverage(
      {
        entities: golden.entities,
        protectedCenterEntityId: golden.protected_center_entity_id ?? null,
        defenseZoneConfig: golden.defense_zone_config,
        radarDomeConfig: golden.radar_dome_config,
      },
      golden.sample_steps,
    );

    expect(result.availability).toBe("ready");
    expect(result.analysis?.estimate.coveragePercent).toBeLessThanOrEqual(
      golden.expect.coverage_percent_max as number,
    );
    expect(result.analysis?.blindSpotV2.uncoveredPercent).toBeGreaterThanOrEqual(
      golden.expect.blind_spot_uncovered_percent_min as number,
    );

    const center = result.inputs!.protectedCenterPosition;
    const discs = deriveRuntimeRadarDiscs(golden.entities, golden.radar_dome_config!.detectionM!);
    expect(getNearestRadarEdgeDistanceM(center, discs)).toBeNull();
  });

  it("computes nearest radar edge distance for uncovered center", () => {
    const discs = deriveRuntimeRadarDiscs(
      [{ entity_id: "radar-a", entity_type: "radar", pose: { x: 0, y: 0, z: 10 } }],
      300,
    );
    expect(getNearestRadarEdgeDistanceM({ x: 400, y: 0 }, discs)).toBe(100);
  });

  it("does not mutate planning coverage module behavior", () => {
    const built = buildRuntimeCoverageInputs({
      entities: runGoldenCase("one_radar_covering_center").entities,
      protectedCenterEntityId: "center-1",
    });
    expect(built.ok).toBe(true);
    if (!built.ok) return;
    expect(built.inputs.polygon.completedVertices?.length).toBe(72);
    expect(built.inputs.radars.sites[0].id).toBe("radar-a");
  });
});

describe("runtime corridor coverage", () => {
  it("golden: half-covered corridor", () => {
    const golden = runGoldenCase("half_covered_corridor");
    const discs = deriveRuntimeRadarDiscs(
      golden.entities,
      golden.radar_dome_config!.detectionM!,
    );
    const result = analyzeCorridorCoverage(golden.corridor_points!, discs);

    expect(result).not.toBeNull();
    expect(result!.totalLengthM).toBeCloseTo(golden.expect.total_length_m as number, 3);
    expect(result!.coveredPercent).toBeGreaterThanOrEqual(
      golden.expect.covered_percent_min as number,
    );
    expect(result!.coveredPercent).toBeLessThanOrEqual(
      golden.expect.covered_percent_max as number,
    );
    expect(result!.uncoveredPercent).toBeGreaterThanOrEqual(
      golden.expect.uncovered_percent_min as number,
    );
    expect(result!.uncoveredPercent).toBeLessThanOrEqual(
      golden.expect.uncovered_percent_max as number,
    );
    expect(result!.coveredLengthM + result!.uncoveredLengthM).toBeCloseTo(
      result!.totalLengthM,
      3,
    );
  });

  it("returns null for degenerate corridor input", () => {
    expect(analyzeCorridorCoverage([], [])).toBeNull();
    expect(analyzeCorridorCoverage([{ x: 0, y: 0, z: 0 }], [])).toBeNull();
  });
});
