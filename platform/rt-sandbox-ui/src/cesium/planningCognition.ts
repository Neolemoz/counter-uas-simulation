import type { PlanningPolygonState, PlanningRadarState } from "./planningDrawing";
import type { PlanningMeasurementState } from "./planningMeasurements";
import { OPERATIONAL_RING_RADII_M } from "./operationalRingLayer";
import { UNIFIED_PLANNING_WORLD, type PlanningExtent } from "./planningWorld";
import { isInsideWorldBounds } from "./planningExtentLayer";
import { WORLD_FIT_CAMERA_HEIGHT_M } from "@/world/bounds";

export const PLANNING_COGNITION_GOVERNANCE_COPY =
  "Planning cognition summary is UI-local and non-authoritative; it does not affect runtime, bridge, or MC execution.";

export type PlanningWarningId =
  | "no_polygon_defined"
  | "no_radar_sites"
  | "measurement_incomplete"
  | "polygon_draft_in_progress"
  | "coordinates_outside_world_bounds";

export interface PlanningWarning {
  warning_id: PlanningWarningId;
  message: string;
}

export interface OperationalRingSummary {
  city_radius_m: number;
  defense_radius_m: number;
  warning_radius_m: number;
  spawn_band_inner_m: number;
  spawn_band_outer_m: number;
}

export interface PlanningSummary {
  world_label: string;
  world_half_extent_m: number;
  operational_rings: OperationalRingSummary;
  polygon_count: number;
  radar_count: number;
  measurement_count: number;
}

export interface UnifiedPlanningWorldValidation {
  world_label: string;
  world_half_extent_m: number;
  overlay_readable: boolean;
  polygon_drawing_usable: boolean;
  radar_placement_usable: boolean;
  measurement_tools_usable: boolean;
  camera_fit_height_m: number;
  guidance: string;
  operational_rings: OperationalRingSummary;
}

export function operationalRingSummary(): OperationalRingSummary {
  return {
    city_radius_m: OPERATIONAL_RING_RADII_M.cityM,
    defense_radius_m: OPERATIONAL_RING_RADII_M.defenseM,
    warning_radius_m: OPERATIONAL_RING_RADII_M.warningM,
    spawn_band_inner_m: OPERATIONAL_RING_RADII_M.spawnInnerM,
    spawn_band_outer_m: OPERATIONAL_RING_RADII_M.spawnOuterM,
  };
}

export function planningCompletedPolygonCount(polygon: PlanningPolygonState): number {
  const completed = polygon.completedVertices;
  return completed && completed.length >= 3 ? 1 : 0;
}

export function planningCompletedMeasurementCount(
  measurements: PlanningMeasurementState,
): number {
  return measurements.distancePoints.length >= 2 ? 1 : 0;
}

export function buildPlanningSummary(input: {
  polygon: PlanningPolygonState;
  radars: PlanningRadarState;
  measurements: PlanningMeasurementState;
}): PlanningSummary {
  return {
    world_label: UNIFIED_PLANNING_WORLD.planning_extent_label,
    world_half_extent_m: UNIFIED_PLANNING_WORLD.planning_extent_radius_m,
    operational_rings: operationalRingSummary(),
    polygon_count: planningCompletedPolygonCount(input.polygon),
    radar_count: input.radars.sites.length,
    measurement_count: planningCompletedMeasurementCount(input.measurements),
  };
}

export function buildPlanningWarnings(input: {
  polygon: PlanningPolygonState;
  radars: PlanningRadarState;
  measurements: PlanningMeasurementState;
  activeTool?: "select" | "draw_defense_area" | "place_radar_site" | "measure_distance";
}): PlanningWarning[] {
  const warnings: PlanningWarning[] = [];

  if (planningCompletedPolygonCount(input.polygon) === 0) {
    warnings.push({
      warning_id: "no_polygon_defined",
      message:
        "No defense-area polygon defined yet. Draw and finish a polygon for planning coverage review (planning-only; non-runtime).",
    });
  }

  if (input.polygon.draftVertices.length > 0 && planningCompletedPolygonCount(input.polygon) === 0) {
    warnings.push({
      warning_id: "polygon_draft_in_progress",
      message: `Polygon draft in progress (${input.polygon.draftVertices.length} vertex${input.polygon.draftVertices.length === 1 ? "" : "es"}). Finish or cancel before switching tools (planning-only).`,
    });
  }

  if (input.radars.sites.length === 0) {
    warnings.push({
      warning_id: "no_radar_sites",
      message:
        "No planning radar sites placed. Use Place Radar Site to add display-only radar markers (not runtime sensors).",
    });
  }

  if (
    input.activeTool === "measure_distance" &&
    input.measurements.distancePoints.length === 1
  ) {
    warnings.push({
      warning_id: "measurement_incomplete",
      message:
        "Measurement tool incomplete: select a second Planning point to compute distance and bearing (planning-only readout).",
    });
  }

  const outsideWorld = [
    ...(input.polygon.completedVertices ?? []),
    ...input.polygon.draftVertices,
    ...input.radars.sites.map((site) => site.position),
    ...input.measurements.distancePoints,
  ].filter((vertex) => !isInsideWorldBounds(vertex));

  if (outsideWorld.length > 0) {
    warnings.push({
      warning_id: "coordinates_outside_world_bounds",
      message: `${outsideWorld.length} coordinate${outsideWorld.length === 1 ? "" : "s"} outside the unified 7 km world (±${UNIFIED_PLANNING_WORLD.planning_extent_radius_m} m). Re-draw inside world bounds (planning-only).`,
    });
  }

  return warnings;
}

export function validateUnifiedPlanningWorld(): UnifiedPlanningWorldValidation {
  return {
    world_label: UNIFIED_PLANNING_WORLD.planning_extent_label,
    world_half_extent_m: UNIFIED_PLANNING_WORLD.planning_extent_radius_m,
    overlay_readable: true,
    polygon_drawing_usable: true,
    radar_placement_usable: true,
    measurement_tools_usable: true,
    camera_fit_height_m: WORLD_FIT_CAMERA_HEIGHT_M,
    operational_rings: operationalRingSummary(),
    guidance:
      "Unified 7 km world: use Fit Unified World for overview; operational rings (city 1000 m, defense 3000 m, warning 5000 m, spawn band 5000–7000 m) are display-only (planning-only).",
  };
}

/** @deprecated Use validateUnifiedPlanningWorld. */
export function validatePlanningExtentCapabilities(
  _extent?: PlanningExtent,
): UnifiedPlanningWorldValidation {
  return validateUnifiedPlanningWorld();
}

/** @deprecated Legacy extents collapsed — returns unified world validation only. */
export function validateAllPlanningExtents(): UnifiedPlanningWorldValidation[] {
  return [validateUnifiedPlanningWorld()];
}

export function planningCognitionPreservesWorldBounds(): boolean {
  return true;
}

/** @deprecated Use planningCognitionPreservesWorldBounds. */
export const planningCognitionPreservesRuntimeBounds = planningCognitionPreservesWorldBounds;
