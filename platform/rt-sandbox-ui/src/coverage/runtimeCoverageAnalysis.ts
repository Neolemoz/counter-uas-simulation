import type { MirrorEntity } from "@/cesium/entityMarkers";
import {
  DEFAULT_DEFENSE_ZONE_CONFIG,
  normalizedDefenseZoneConfig,
  type DefenseZoneConfig,
} from "@/cesium/defenseZoneConfig";
import { analyzePlanningCoverage, type PlanningCoverageAnalysis } from "@/cesium/planningCoverageAnalysis";
import {
  EMPTY_PLANNING_RADARS,
  type PlanningPolygonState,
  type PlanningRadarSite,
  type PlanningRadarState,
  type PlanningVertex,
} from "@/cesium/planningDrawing";
import type { RadarDomeConfig } from "@/cesium/sensorDomeLayer";
import { normalizedRadarDomeConfig } from "@/cesium/sensorDomeLayer";

export type RuntimeCoverageUnavailableReason =
  | "protected_center_unavailable"
  | "unsupported_defense_shape";

export interface RuntimeCoverageInputs {
  polygon: PlanningPolygonState;
  radars: PlanningRadarState;
  protectedCenterPosition: PlanningVertex;
  protectedCenterEntityId: string;
  warningDiscRadiusM: number;
}

export type RuntimeCoverageBuildResult =
  | { ok: true; inputs: RuntimeCoverageInputs }
  | { ok: false; reason: RuntimeCoverageUnavailableReason };

export interface RuntimeCoverageBuildParams {
  entities: readonly MirrorEntity[];
  protectedCenterEntityId: string | null;
  defenseZoneConfig?: Partial<DefenseZoneConfig>;
  radarDomeConfig?: Partial<RadarDomeConfig>;
}

export interface RuntimeRadarDisc {
  id: string;
  position: PlanningVertex;
  detectionRangeM: number;
}

export interface RuntimeCoverageResult {
  availability: "ready";
  inputs: RuntimeCoverageInputs;
  analysis: PlanningCoverageAnalysis;
}

export type RuntimeCoverageAnalysisResult =
  | RuntimeCoverageResult
  | { availability: RuntimeCoverageUnavailableReason; inputs: null; analysis: null };

const WARNING_DISC_SEGMENTS = 72;

function entityPosition(entity: MirrorEntity | undefined): PlanningVertex | null {
  if (!entity) return null;
  const x = Number(entity.pose?.x);
  const y = Number(entity.pose?.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  return { x, y };
}

function horizontalDistanceM(a: PlanningVertex, b: PlanningVertex): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

/** Approximate circle as closed vertex ring for planning polygon sampling. */
export function buildWarningDiscPolygon(
  center: PlanningVertex,
  radiusM: number,
  segments = WARNING_DISC_SEGMENTS,
): PlanningVertex[] {
  const radius = Math.max(1, radiusM);
  return Array.from({ length: segments }, (_, index) => {
    const theta = (Math.PI * 2 * index) / segments;
    return {
      x: center.x + Math.cos(theta) * radius,
      y: center.y + Math.sin(theta) * radius,
    };
  });
}

export function adaptRuntimeRadarEntities(
  entities: readonly MirrorEntity[],
  detectionRangeM: number,
): PlanningRadarSite[] {
  const rangeM = Math.max(10, detectionRangeM);
  return entities
    .filter((entity) => entity.entity_type === "radar" && entity.entity_id)
    .map((entity) => {
      const position = entityPosition(entity) ?? { x: 0, y: 0 };
      return {
        id: entity.entity_id,
        position,
        radar_type: "Runtime Radar",
        detection_range_m: rangeM,
      };
    })
    .sort((a, b) => a.id.localeCompare(b.id));
}

export function deriveRuntimeRadarDiscs(
  entities: readonly MirrorEntity[],
  detectionRangeM: number,
): RuntimeRadarDisc[] {
  return adaptRuntimeRadarEntities(entities, detectionRangeM).map((site) => ({
    id: site.id,
    position: site.position,
    detectionRangeM: site.detection_range_m,
  }));
}

export function buildRuntimeCoverageInputs(
  params: RuntimeCoverageBuildParams,
): RuntimeCoverageBuildResult {
  const defenseConfig = normalizedDefenseZoneConfig(
    params.defenseZoneConfig ?? DEFAULT_DEFENSE_ZONE_CONFIG,
  );
  if (defenseConfig.shape !== "circle") {
    return { ok: false, reason: "unsupported_defense_shape" };
  }

  const centerId = params.protectedCenterEntityId;
  if (!centerId) {
    return { ok: false, reason: "protected_center_unavailable" };
  }

  const centerEntity = params.entities.find((entity) => entity.entity_id === centerId);
  const protectedCenterPosition = entityPosition(centerEntity);
  if (!protectedCenterPosition) {
    return { ok: false, reason: "protected_center_unavailable" };
  }

  const detectionRangeM = normalizedRadarDomeConfig(params.radarDomeConfig).detectionM;
  const sites = adaptRuntimeRadarEntities(params.entities, detectionRangeM);
  const warningDiscRadiusM = defenseConfig.sizes.warningM;

  return {
    ok: true,
    inputs: {
      polygon: {
        draftVertices: [],
        completedVertices: buildWarningDiscPolygon(
          protectedCenterPosition,
          warningDiscRadiusM,
        ),
      },
      radars: {
        ...EMPTY_PLANNING_RADARS,
        sites,
      },
      protectedCenterPosition,
      protectedCenterEntityId: centerId,
      warningDiscRadiusM,
    },
  };
}

export function analyzeRuntimeCoverage(
  params: RuntimeCoverageBuildParams,
  sampleSteps?: number,
): RuntimeCoverageAnalysisResult {
  const built = buildRuntimeCoverageInputs(params);
  if (!built.ok) {
    return { availability: built.reason, inputs: null, analysis: null };
  }

  const analysis = analyzePlanningCoverage(
    built.inputs.polygon,
    built.inputs.radars,
    sampleSteps,
  );

  return {
    availability: "ready",
    inputs: built.inputs,
    analysis,
  };
}

function pointInsideRadarDisc(point: PlanningVertex, disc: RuntimeRadarDisc): boolean {
  return horizontalDistanceM(point, disc.position) <= disc.detectionRangeM;
}

export function isProtectedCenterCovered(
  center: PlanningVertex,
  radarDiscs: readonly RuntimeRadarDisc[],
): boolean {
  return getCoveringRadarCount(center, radarDiscs) > 0;
}

export function getCoveringRadarCount(
  center: PlanningVertex,
  radarDiscs: readonly RuntimeRadarDisc[],
): number {
  return radarDiscs.filter((disc) => pointInsideRadarDisc(center, disc)).length;
}

/** Distance from center to nearest radar coverage edge; 0 when inside a disc. */
export function getNearestRadarEdgeDistanceM(
  center: PlanningVertex,
  radarDiscs: readonly RuntimeRadarDisc[],
): number | null {
  if (radarDiscs.length === 0) return null;
  let nearestEdge = Number.POSITIVE_INFINITY;
  for (const disc of radarDiscs) {
    const distanceToRadar = horizontalDistanceM(center, disc.position);
    nearestEdge = Math.min(nearestEdge, Math.max(0, distanceToRadar - disc.detectionRangeM));
  }
  return nearestEdge;
}
