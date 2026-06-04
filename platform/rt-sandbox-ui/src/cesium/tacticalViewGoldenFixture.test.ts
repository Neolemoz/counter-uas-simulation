import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { enableTacticalViewPreset, isTacticalViewPresetActive } from "./tacticalPreset";
import { defaultVisibilityFromRegistry } from "./visualLayerRegistry";
import { resolveTacticalRankingCues } from "./tacticalRankingCueLayer";
import {
  assertGoldenFixtureGeometry,
  deriveGoldenFixtureScalingAtCamera,
  goldenFixtureInterceptFromPathOnly,
  goldenFixturePresetMatchesRegistry,
  goldenFixtureTimingBlock,
  parseTacticalView7kmGoldenFixture,
  TACTICAL_VIEW_7KM_GOLDEN_SCHEMA,
  validateTacticalViewGoldenFixture,
} from "./tacticalViewGoldenFixture";
import { pathLengthM } from "./tacticalTimingLabels";

const FIXTURE_PATH = join(
  import.meta.dirname,
  "../../../../fixtures/rt_visualization/tactical_view_7km_golden_v1.json",
);

function loadTacticalView7kmGoldenFixture() {
  return parseTacticalView7kmGoldenFixture(
    JSON.parse(readFileSync(FIXTURE_PATH, "utf-8")),
  );
}

describe("tacticalViewGoldenFixture integration", () => {
  const fixture = loadTacticalView7kmGoldenFixture();

  it("loads golden fixture with expected schema and governance flags", () => {
    expect(fixture.schema).toBe(TACTICAL_VIEW_7KM_GOLDEN_SCHEMA);
    expect(validateTacticalViewGoldenFixture(fixture)).toEqual([]);
    expect(fixture.governance.display_only).toBe(true);
    expect(fixture.governance.no_command_authority).toBe(true);
    expect(fixture.governance.no_autonomous_engagement_authority).toBe(true);
  });

  it("enables tactical preset layers from registry defaults", () => {
    const base = defaultVisibilityFromRegistry();
    expect(base.showTacticalPredictedPath).toBe(false);
    const enabled = enableTacticalViewPreset(base);
    expect(isTacticalViewPresetActive(enabled)).toBe(true);
    expect(goldenFixturePresetMatchesRegistry(fixture)).toBe(true);
    expect(enabled.showTacticalCompareOverlay).toBe(false);
  });

  it("derives telemetry predicted path and intercept pose at 7km scale", () => {
    const { geometry } = assertGoldenFixtureGeometry(fixture);
    expect(geometry.pathMode).toBe("telemetry");
    expect(geometry.pathPoints).toHaveLength(2);
    expect(geometry.interceptPose).toEqual(fixture.expected_geometry.intercept_pose);
    expect(pathLengthM(geometry.pathPoints)).toBeCloseTo(
      fixture.expected_geometry.path_length_m,
      0,
    );
  });

  it("derives intercept point from path endpoint when last_intercept_pose is absent", () => {
    const derived = goldenFixtureInterceptFromPathOnly(fixture);
    expect(derived).toEqual(fixture.expected_geometry.intercept_pose);
  });

  it("renders direct-fallback threat corridor across spawn band to solution", () => {
    const { corridor } = assertGoldenFixtureGeometry(fixture);
    expect(corridor.mode).toBe("direct_fallback");
    expect(corridor.corridorPoints[0]).toEqual(
      fixture.expected_geometry.corridor_points[0],
    );
    expect(corridor.corridorPoints.at(-1)).toEqual(
      fixture.expected_geometry.corridor_points[1],
    );
    expect(pathLengthM(corridor.corridorPoints)).toBeCloseTo(
      fixture.expected_geometry.corridor_length_m,
      0,
    );
  });

  it("formats ETA and TTI timing block from fixture state", () => {
    expect(goldenFixtureTimingBlock(fixture)).toBe(
      fixture.expected_geometry.timing_labels.display_block,
    );
  });

  it("resolves ranked target cue from fixture ranking state", () => {
    const resolution = resolveTacticalRankingCues(fixture.tactical_state, null);
    expect(resolution.mode).toBe("ranked_list");
    expect(resolution.cues[0]?.entityId).toBe(
      fixture.expected_geometry.ranking.primary_target_id,
    );
    expect(resolution.cues[0]?.rank).toBe(fixture.expected_geometry.ranking.rank);
  });

  it("matches world-fit and city-core camera scaling expectations", () => {
    const worldFit = deriveGoldenFixtureScalingAtCamera(
      fixture,
      fixture.camera_validation.world_fit.camera_height_m,
    );
    expect(worldFit).toEqual(fixture.camera_validation.world_fit.expected_scaling);

    const cityCore = deriveGoldenFixtureScalingAtCamera(
      fixture,
      fixture.camera_validation.city_core.camera_height_m,
    );
    expect(cityCore).toEqual(fixture.camera_validation.city_core.expected_scaling);
    expect(worldFit.corridor_half_width_m).toBeGreaterThan(
      cityCore.trajectory_width_px,
    );
  });

  it("does not mutate fixture tactical state during derivation", () => {
    const snapshot = JSON.stringify(fixture.tactical_state);
    assertGoldenFixtureGeometry(fixture);
    goldenFixtureInterceptFromPathOnly(fixture);
    enableTacticalViewPreset(defaultVisibilityFromRegistry());
    expect(JSON.stringify(fixture.tactical_state)).toBe(snapshot);
  });
});
