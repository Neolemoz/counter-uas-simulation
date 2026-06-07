import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "@/cesium/entityMarkers";
import type { DefenseZoneConfig } from "@/cesium/defenseZoneConfig";
import { deriveThreatCorridorGeometry } from "@/cesium/tacticalThreatCorridor";
import { deriveTacticalTrajectoryGeometry } from "@/cesium/tacticalTrajectoryLayer";
import type { RadarDomeConfig } from "@/cesium/sensorDomeLayer";
import { BANNER_RUNTIME_COVERAGE } from "@/governance/banners";
import { analyzeCorridorCoverage } from "./runtimeCorridorCoverage";
import {
  analyzeRuntimeCoverage,
  deriveRuntimeRadarDiscs,
  getCoveringRadarCount,
  getNearestRadarEdgeDistanceM,
  isProtectedCenterCovered,
  type RuntimeCoverageUnavailableReason,
} from "./runtimeCoverageAnalysis";

export type ProtectedCenterCoverageState = "yes" | "no" | "unavailable";

export interface RuntimeCoverageStatusModel {
  banner: string;
  availability: "ready" | RuntimeCoverageUnavailableReason;
  protectedCenterCovered: ProtectedCenterCoverageState;
  coveringRadarCount: number | null;
  coveragePercent: number | null;
  overlapPercent: number | null;
  nearestEdgeDistanceM: number | null;
  topBlindSpotSector: string | null;
  corridorUncoveredPercent: number | null;
  guidance: string | null;
}

export interface DeriveRuntimeCoverageStatusParams {
  entities: readonly MirrorEntity[];
  protectedCenterEntityId: string | null;
  defenseZoneConfig?: Partial<DefenseZoneConfig>;
  radarDomeConfig?: Partial<RadarDomeConfig>;
  tacticalState?: TacticalStatePayload | null;
}

function formatPercent(value: number | null): number | null {
  if (value == null || !Number.isFinite(value)) return null;
  return Math.round(value * 10) / 10;
}

function formatDistanceM(value: number | null): number | null {
  if (value == null || !Number.isFinite(value)) return null;
  return Math.round(value);
}

export function deriveRuntimeCoverageStatus(
  params: DeriveRuntimeCoverageStatusParams,
): RuntimeCoverageStatusModel {
  const coverage = analyzeRuntimeCoverage(params);
  const detectionM = params.radarDomeConfig?.detectionM;
  const radarDiscs = deriveRuntimeRadarDiscs(
    params.entities,
    detectionM ?? 300,
  );

  if (coverage.availability === "protected_center_unavailable") {
    return {
      banner: BANNER_RUNTIME_COVERAGE,
      availability: coverage.availability,
      protectedCenterCovered: "unavailable",
      coveringRadarCount: null,
      coveragePercent: null,
      overlapPercent: null,
      nearestEdgeDistanceM: null,
      topBlindSpotSector: null,
      corridorUncoveredPercent: null,
      guidance: "Designate a protected center to evaluate runtime coverage heuristics.",
    };
  }

  if (coverage.availability === "unsupported_defense_shape") {
    return {
      banner: BANNER_RUNTIME_COVERAGE,
      availability: coverage.availability,
      protectedCenterCovered: "unavailable",
      coveringRadarCount: null,
      coveragePercent: null,
      overlapPercent: null,
      nearestEdgeDistanceM: null,
      topBlindSpotSector: null,
      corridorUncoveredPercent: null,
      guidance: "Runtime coverage V1 supports circle defense zones only.",
    };
  }

  const center = coverage.inputs!.protectedCenterPosition;
  const centerCovered = isProtectedCenterCovered(center, radarDiscs);
  const analysis = coverage.analysis!;

  const entities = [...params.entities];
  const trajectoryGeometry = deriveTacticalTrajectoryGeometry(
    params.tacticalState,
    entities,
  );
  const corridorGeometry =
    trajectoryGeometry != null
      ? deriveThreatCorridorGeometry(
          params.tacticalState,
          trajectoryGeometry,
          entities,
        )
      : null;
  const corridorCoverage =
    corridorGeometry != null
      ? analyzeCorridorCoverage(corridorGeometry.corridorPoints, radarDiscs)
      : null;

  const topSector = analysis.blindSpotV2.majorUncoveredSectors[0]?.sector ?? null;

  return {
    banner: BANNER_RUNTIME_COVERAGE,
    availability: "ready",
    protectedCenterCovered: centerCovered ? "yes" : "no",
    coveringRadarCount: getCoveringRadarCount(center, radarDiscs),
    coveragePercent: formatPercent(analysis.estimate.coveragePercent),
    overlapPercent: formatPercent(analysis.overlapPercent),
    nearestEdgeDistanceM: formatDistanceM(getNearestRadarEdgeDistanceM(center, radarDiscs)),
    topBlindSpotSector: topSector,
    corridorUncoveredPercent: formatPercent(corridorCoverage?.uncoveredPercent ?? null),
    guidance:
      radarDiscs.length === 0
        ? "No runtime radar entities — defended-disc coverage is zero."
        : null,
  };
}
