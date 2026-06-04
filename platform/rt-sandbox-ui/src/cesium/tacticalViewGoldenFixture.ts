import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import { deriveTacticalTrajectoryGeometry } from "./tacticalTrajectoryLayer";
import { resolveThreatCorridorForRender } from "./tacticalCorridorLayer";
import {
  enableTacticalViewPreset,
  isTacticalViewPresetActive,
  TACTICAL_VIEW_PRESET_LAYER_IDS,
} from "./tacticalPreset";
import { formatTacticalTimingBlock, deriveTacticalTimingSeconds } from "./tacticalTimingLabels";
import {
  deriveDisplayInterceptPose,
  parsePredictedPathTelemetry,
} from "./tacticalGeometry";
import {
  corridorHalfWidthM,
  tacticalRankingCuePixelOffset,
  tacticalSelectionHaloPixelSize,
  tacticalTimingLabelPixelOffset,
  tacticalTrajectoryWidthPx,
} from "./tacticalVisualScale";
import { defaultVisibilityFromRegistry } from "./visualLayerRegistry";

export const TACTICAL_VIEW_7KM_GOLDEN_SCHEMA = "rt_tactical_view_7km_golden_v1";

export interface TacticalViewGoldenCameraScaling {
  corridor_half_width_m: number;
  trajectory_width_px: number;
  timing_label_offset_y: number;
  selection_halo_pixel_size: number;
  ranking_cue_offset_y_rank_1: number;
}

export interface TacticalView7kmGoldenFixture {
  schema: string;
  fixture_id: string;
  description: string;
  world: {
    bounds_half_extent_m: number;
    operational_rings_m: Record<string, number>;
  };
  governance: {
    display_only: boolean;
    no_command_authority: boolean;
    no_autonomous_engagement_authority: boolean;
    copy: string;
  };
  entities: Array<{
    entity_id: string;
    entity_type: string;
    role: string;
    pose: { x: number; y: number; z: number };
    operational_context?: string;
  }>;
  tactical_state: TacticalStatePayload & {
    ranked_candidates?: Array<{ entity_id: string; rank: number }>;
  };
  expected_geometry: {
    path_mode: string;
    path_length_m: number;
    intercept_pose: { x: number; y: number; z: number };
    path_end_pose: { x: number; y: number; z: number };
    corridor_mode: string;
    corridor_length_m: number;
    corridor_points: Array<{ x: number; y: number; z: number }>;
    timing_labels: {
      tti_s: number;
      eta_s: number;
      display_block: string;
    };
    ranking: {
      mode: string;
      primary_target_id: string;
      rank: number;
    };
  };
  camera_validation: {
    world_fit: {
      camera_height_m: number;
      camera_preset: string;
      expected_scaling: TacticalViewGoldenCameraScaling;
    };
    city_core: {
      camera_height_m: number;
      camera_preset: string;
      expected_scaling: TacticalViewGoldenCameraScaling;
    };
  };
  preset_layers: string[];
  preset_excludes: string[];
}

export const TACTICAL_VIEW_7KM_GOLDEN_FIXTURE_RELATIVE_PATH =
  "fixtures/rt_visualization/tactical_view_7km_golden_v1.json";

export function parseTacticalView7kmGoldenFixture(
  raw: unknown,
): TacticalView7kmGoldenFixture {
  return raw as TacticalView7kmGoldenFixture;
}

export function mirrorEntitiesFromGoldenFixture(
  fixture: TacticalView7kmGoldenFixture,
): MirrorEntity[] {
  return fixture.entities.map((row) => ({
    entity_id: row.entity_id,
    entity_type: row.entity_type,
    pose: row.pose,
  }));
}

export function validateTacticalViewGoldenFixture(
  fixture: TacticalView7kmGoldenFixture,
): string[] {
  const errors: string[] = [];
  if (fixture.schema !== TACTICAL_VIEW_7KM_GOLDEN_SCHEMA) {
    errors.push(`schema must be ${TACTICAL_VIEW_7KM_GOLDEN_SCHEMA}`);
  }
  if (!fixture.governance.display_only) {
    errors.push("governance.display_only must be true");
  }
  if (!fixture.governance.no_command_authority) {
    errors.push("governance.no_command_authority must be true");
  }
  if (!fixture.governance.no_autonomous_engagement_authority) {
    errors.push("governance.no_autonomous_engagement_authority must be true");
  }
  if (fixture.world.bounds_half_extent_m !== 7000) {
    errors.push("world.bounds_half_extent_m must be 7000");
  }
  const attacker = fixture.entities.find((e) => e.role === "attacker");
  if (!attacker) {
    errors.push("attacker entity required");
  } else {
    const spawnDist = Math.hypot(attacker.pose.x, attacker.pose.y);
    if (spawnDist < 5000 || spawnDist > 7000) {
      errors.push("attacker should sit in spawn band (5000–7000 m)");
    }
  }
  return errors;
}

export function deriveGoldenFixtureScalingAtCamera(
  fixture: TacticalView7kmGoldenFixture,
  cameraHeightM: number,
): TacticalViewGoldenCameraScaling {
  const entities = mirrorEntitiesFromGoldenFixture(fixture);
  const geometry = deriveTacticalTrajectoryGeometry(fixture.tactical_state, entities);
  const corridor = resolveThreatCorridorForRender(fixture.tactical_state, entities);
  const pathPoints = geometry?.pathPoints ?? [];
  const corridorPoints = corridor?.corridorPoints ?? pathPoints;
  const timingOffset = tacticalTimingLabelPixelOffset(cameraHeightM, "intercept_top");
  const rankingOffset = tacticalRankingCuePixelOffset(cameraHeightM, 1);
  return {
    corridor_half_width_m: Number(
      corridorHalfWidthM(cameraHeightM, corridorPoints).toFixed(1),
    ),
    trajectory_width_px: geometry
      ? tacticalTrajectoryWidthPx(geometry.pathMode, cameraHeightM, pathPoints)
      : 0,
    timing_label_offset_y: timingOffset.y,
    selection_halo_pixel_size: tacticalSelectionHaloPixelSize(cameraHeightM),
    ranking_cue_offset_y_rank_1: rankingOffset.y,
  };
}

export function assertGoldenFixtureGeometry(
  fixture: TacticalView7kmGoldenFixture,
): {
  geometry: NonNullable<ReturnType<typeof deriveTacticalTrajectoryGeometry>>;
  corridor: NonNullable<ReturnType<typeof resolveThreatCorridorForRender>>;
} {
  const entities = mirrorEntitiesFromGoldenFixture(fixture);
  const geometry = deriveTacticalTrajectoryGeometry(fixture.tactical_state, entities);
  if (!geometry) {
    throw new Error("golden fixture failed to derive trajectory geometry");
  }
  const corridor = resolveThreatCorridorForRender(fixture.tactical_state, entities);
  if (!corridor) {
    throw new Error("golden fixture failed to derive corridor geometry");
  }
  return { geometry, corridor };
}

export function goldenFixturePresetMatchesRegistry(
  fixture: TacticalView7kmGoldenFixture,
): boolean {
  const enabled = enableTacticalViewPreset(defaultVisibilityFromRegistry());
  return (
    isTacticalViewPresetActive(enabled) &&
    TACTICAL_VIEW_PRESET_LAYER_IDS.every((id) => fixture.preset_layers.includes(id))
  );
}

export function goldenFixtureTimingBlock(
  fixture: TacticalView7kmGoldenFixture,
): string | null {
  const timing = deriveTacticalTimingSeconds(fixture.tactical_state, null);
  return formatTacticalTimingBlock(timing);
}

export function goldenFixtureInterceptFromPathOnly(
  fixture: TacticalView7kmGoldenFixture,
): ReturnType<typeof deriveDisplayInterceptPose> {
  const path = parsePredictedPathTelemetry(fixture.tactical_state);
  return deriveDisplayInterceptPose(
    { ...fixture.tactical_state, last_intercept_pose: null },
    path,
  );
}
