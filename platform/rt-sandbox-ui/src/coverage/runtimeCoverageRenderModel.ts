import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { PlanningCoverageCell } from "@/cesium/planningDrawing";
import type { PlanningUncoveredSectorSummary } from "@/cesium/planningCoverageAnalysis";
import type { EnuPoint } from "@/cesium/tacticalGeometry";
import { deriveThreatCorridorGeometry } from "@/cesium/tacticalThreatCorridor";
import { deriveTacticalTrajectoryGeometry } from "@/cesium/tacticalTrajectoryLayer";
import {
  analyzeRuntimeCoverage,
  deriveRuntimeRadarDiscs,
  type RuntimeCoverageBuildParams,
} from "./runtimeCoverageAnalysis";
import { deriveCorridorCoveragePolylines } from "./runtimeCorridorCoverage";

export const RUNTIME_COVERAGE_ENTITY_PREFIX = "rt-runtime-coverage-";

export interface RuntimeCoverageRenderModel {
  ready: true;
  coveredCells: PlanningCoverageCell[];
  uncoveredCells: PlanningCoverageCell[];
  blindSpotHints: PlanningCoverageCell[];
  majorUncoveredSectors: PlanningUncoveredSectorSummary[];
  coveredCorridorPolylines: EnuPoint[][];
  uncoveredCorridorPolylines: EnuPoint[][];
}

export type RuntimeCoverageRenderResult =
  | RuntimeCoverageRenderModel
  | { ready: false; reason: "protected_center_unavailable" | "unsupported_defense_shape" };

export interface RuntimeCoverageRenderParams extends RuntimeCoverageBuildParams {
  tacticalState?: TacticalStatePayload | null;
}

export function deriveRuntimeCoverageRenderModel(
  params: RuntimeCoverageRenderParams,
): RuntimeCoverageRenderResult {
  const coverage = analyzeRuntimeCoverage(params);
  if (coverage.availability !== "ready" || !coverage.analysis) {
    return { ready: false, reason: coverage.availability };
  }

  const detectionM = params.radarDomeConfig?.detectionM ?? 300;
  const radarDiscs = deriveRuntimeRadarDiscs(params.entities, detectionM);
  const entities = [...params.entities];
  const trajectoryGeometry = deriveTacticalTrajectoryGeometry(params.tacticalState, entities);
  const corridorGeometry =
    trajectoryGeometry != null
      ? deriveThreatCorridorGeometry(params.tacticalState, trajectoryGeometry, entities)
      : null;
  const corridorPolylines =
    corridorGeometry != null
      ? deriveCorridorCoveragePolylines(corridorGeometry.corridorPoints, radarDiscs)
      : { coveredPolylines: [], uncoveredPolylines: [] };

  return {
    ready: true,
    coveredCells: coverage.analysis.estimate.coveredCells,
    uncoveredCells: coverage.analysis.estimate.uncoveredCells,
    blindSpotHints: coverage.analysis.estimate.blindSpotHints,
    majorUncoveredSectors: coverage.analysis.blindSpotV2.majorUncoveredSectors,
    coveredCorridorPolylines: corridorPolylines.coveredPolylines,
    uncoveredCorridorPolylines: corridorPolylines.uncoveredPolylines,
  };
}

export function plannedRuntimeCoverageEntityIds(
  model: RuntimeCoverageRenderModel,
): string[] {
  const prefix = RUNTIME_COVERAGE_ENTITY_PREFIX;
  return [
    ...model.coveredCells.map((_, index) => `${prefix}covered-${index}`),
    ...model.uncoveredCells.map((_, index) => `${prefix}uncovered-${index}`),
    ...model.blindSpotHints.map((_, index) => `${prefix}blind-spot-${index}`),
    ...model.majorUncoveredSectors.map(
      (_, index) => `${prefix}sector-label-${index}`,
    ),
    ...model.majorUncoveredSectors.flatMap((_, index) => [
      `${prefix}sector-${index}`,
    ]),
    ...model.coveredCorridorPolylines.map((_, index) => `${prefix}corridor-covered-${index}`),
    ...model.uncoveredCorridorPolylines.map(
      (_, index) => `${prefix}corridor-uncovered-${index}`,
    ),
  ];
}

export function runtimeCoverageLayerRegistered(
  registryLayers: readonly { layer_id: string; visibility_key?: string; default_on?: boolean }[],
): boolean {
  const layer = registryLayers.find((row) => row.layer_id === "runtime_coverage_cells");
  return (
    layer != null &&
    layer.visibility_key === "showRuntimeCoverageCells" &&
    layer.default_on === false
  );
}
