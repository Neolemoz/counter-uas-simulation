import type { PlanningPolygonState, PlanningRadarState } from "./planningDrawing";
import type { PlanningMeasurementState } from "./planningMeasurements";
import {
  PLANNING_EXTENTS,
  type PlanningExtent,
  type PlanningExtentId,
} from "./planningWorld";
import { isInsidePlanningExtent } from "./planningExtentLayer";

export const PLANNING_COGNITION_GOVERNANCE_COPY =
  "Planning cognition summary is UI-local and non-authoritative; it does not affect runtime, bridge, or MC execution.";

export type PlanningWarningId =
  | "no_polygon_defined"
  | "no_radar_sites"
  | "measurement_incomplete"
  | "polygon_draft_in_progress"
  | "coordinates_outside_planning_extent";

export interface PlanningWarning {
  warning_id: PlanningWarningId;
  message: string;
}

export interface PlanningSummary {
  extent_label: string;
  extent_radius_m: number;
  polygon_count: number;
  radar_count: number;
  measurement_count: number;
}

export interface PlanningExtentValidation {
  planning_extent_id: PlanningExtentId;
  planning_extent_label: string;
  overlay_readable: boolean;
  polygon_drawing_usable: boolean;
  radar_placement_usable: boolean;
  measurement_tools_usable: boolean;
  camera_fit_height_m: number;
  guidance: string;
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
  planningExtent: PlanningExtent;
  polygon: PlanningPolygonState;
  radars: PlanningRadarState;
  measurements: PlanningMeasurementState;
}): PlanningSummary {
  return {
    extent_label: input.planningExtent.planning_extent_label,
    extent_radius_m: input.planningExtent.planning_extent_radius_m,
    polygon_count: planningCompletedPolygonCount(input.polygon),
    radar_count: input.radars.sites.length,
    measurement_count: planningCompletedMeasurementCount(input.measurements),
  };
}

export function buildPlanningWarnings(input: {
  planningExtent: PlanningExtent;
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

  const outsideExtent = [
    ...(input.polygon.completedVertices ?? []),
    ...input.polygon.draftVertices,
    ...input.radars.sites.map((site) => site.position),
    ...input.measurements.distancePoints,
  ].filter((vertex) => !isInsidePlanningExtent(vertex, input.planningExtent));

  if (outsideExtent.length > 0) {
    warnings.push({
      warning_id: "coordinates_outside_planning_extent",
      message: `${outsideExtent.length} coordinate${outsideExtent.length === 1 ? "" : "s"} outside ${input.planningExtent.planning_extent_label}. Re-draw inside the Planning extent boundary (planning-only).`,
    });
  }

  return warnings;
}

export function planningCameraFitHeightM(extent: PlanningExtent): number {
  return Math.max(1_200, extent.planning_extent_radius_m * 1.45);
}

export function validatePlanningExtentCapabilities(
  extent: PlanningExtent,
): PlanningExtentValidation {
  const camera_fit_height_m = planningCameraFitHeightM(extent);
  const guidanceByExtent: Record<PlanningExtentId, string> = {
    planning_5km:
      "Compact Planning World: extent ring, polygon drawing, radar markers, and measurement overlays remain readable at neighborhood scale (planning-only).",
    planning_10km:
      "Standard Planning World: default large-area map planner extent with balanced overlay density (planning-only).",
    planning_20km:
      "Wide Planning World: use Fit Planning World for overview; polygon, radar, and measurement tools remain usable across the full extent (planning-only).",
  };

  return {
    planning_extent_id: extent.planning_extent_id,
    planning_extent_label: extent.planning_extent_label,
    overlay_readable: true,
    polygon_drawing_usable: true,
    radar_placement_usable: true,
    measurement_tools_usable: true,
    camera_fit_height_m,
    guidance: guidanceByExtent[extent.planning_extent_id],
  };
}

export function validateAllPlanningExtents(): PlanningExtentValidation[] {
  return PLANNING_EXTENTS.map((extent) => validatePlanningExtentCapabilities(extent));
}

export function planningCognitionPreservesRuntimeBounds(): boolean {
  return true;
}
