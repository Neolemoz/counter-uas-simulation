import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  PointGraphics,
  PolylineDashMaterialProperty,
  Viewer,
} from "cesium";
import { WORLD_BOUNDS } from "@/world/bounds";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { sampleTerrainHeight } from "./rtFictionalTerrain";
import type { PlanningVertex } from "./planningDrawing";

const MEASUREMENT_PREFIX = "rt-planning-measurement-";

export const PLANNING_MEASUREMENT_GOVERNANCE_COPY =
  "Planning measurement tool only; not runtime authority, sensor truth, or MC execution authority.";

export const PLANNING_RADIUS_OPTIONS_M = [1_000, 3_000, 5_000] as const;

export type PlanningRadiusMeters = (typeof PLANNING_RADIUS_OPTIONS_M)[number];

export interface PlanningMeasurementState {
  distancePoints: PlanningVertex[];
  radiusCenter: PlanningVertex | null;
  radiusMeters: PlanningRadiusMeters;
}

export const DEFAULT_PLANNING_MEASUREMENT_STATE: PlanningMeasurementState = {
  distancePoints: [],
  radiusCenter: null,
  radiusMeters: 1_000,
};

export function addPlanningMeasurementPoint(
  state: PlanningMeasurementState,
  point: PlanningVertex,
): PlanningMeasurementState {
  const nextPoints =
    state.distancePoints.length >= 2 ? [point] : [...state.distancePoints, point];
  return {
    ...state,
    distancePoints: nextPoints,
    radiusCenter: point,
  };
}

export function setPlanningRadiusMeters(
  state: PlanningMeasurementState,
  radiusMeters: PlanningRadiusMeters,
): PlanningMeasurementState {
  return { ...state, radiusMeters };
}

export function clearPlanningMeasurements(
  state: PlanningMeasurementState,
): PlanningMeasurementState {
  return { ...state, distancePoints: [], radiusCenter: null };
}

export function planningDistanceMeters(a: PlanningVertex, b: PlanningVertex): number {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

export function planningBearingDegrees(a: PlanningVertex, b: PlanningVertex): number {
  const radians = Math.atan2(b.x - a.x, b.y - a.y);
  return (radians * 180) / Math.PI + (radians < 0 ? 360 : 0);
}

export function planningBearingCardinal(a: PlanningVertex, b: PlanningVertex): string {
  const labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
  const index = Math.round(planningBearingDegrees(a, b) / 45) % labels.length;
  return labels[index] ?? "N";
}

export function planningCoordinateReadout(vertex: PlanningVertex): string {
  return `X ${Math.round(vertex.x)}m, Y ${Math.round(vertex.y)}m`;
}

export function planningRadiusMetadata(radiusMeters: PlanningRadiusMeters): {
  radius_m: number;
  radius_km: number;
  label: string;
} {
  return {
    radius_m: radiusMeters,
    radius_km: radiusMeters / 1000,
    label: `${radiusMeters / 1000} km planning radius`,
  };
}

export function planningMeasurementSummary(state: PlanningMeasurementState): {
  distance_m: number | null;
  distance_km: number | null;
  bearing: string | null;
  start_readout: string | null;
  end_readout: string | null;
  radius_label: string;
} {
  const [start, end] = state.distancePoints;
  const distance = start && end ? planningDistanceMeters(start, end) : null;
  const radius = planningRadiusMetadata(state.radiusMeters);
  return {
    distance_m: distance,
    distance_km: distance === null ? null : distance / 1000,
    bearing: start && end ? planningBearingCardinal(start, end) : null,
    start_readout: start ? planningCoordinateReadout(start) : null,
    end_readout: end ? planningCoordinateReadout(end) : null,
    radius_label: radius.label,
  };
}

function measurementCartesian(vertex: PlanningVertex): import("cesium").Cartesian3 {
  return worldToCartesian(vertex.x, vertex.y, sampleTerrainHeight(vertex.x, vertex.y) + 2.2);
}

function circleVertices(center: PlanningVertex, radiusM: number): PlanningVertex[] {
  return Array.from({ length: 73 }, (_, index) => {
    const theta = (Math.PI * 2 * index) / 72;
    return {
      x: center.x + Math.cos(theta) * radiusM,
      y: center.y + Math.sin(theta) * radiusM,
    };
  });
}

function removeMeasurementEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((entity) => {
    if ((entity.id ?? "").startsWith(MEASUREMENT_PREFIX)) toRemove.push(entity);
  });
  for (const entity of toRemove) viewer.entities.remove(entity);
}

function addPoint(viewer: Viewer, point: PlanningVertex, index: number): void {
  viewer.entities.add(
    new Entity({
      id: `${MEASUREMENT_PREFIX}point-${index}`,
      position: measurementCartesian(point),
      point: new PointGraphics({
        pixelSize: 10,
        color: Color.fromCssColorString("rgba(251, 191, 36, 0.96)"),
        outlineColor: Color.fromCssColorString("rgba(120, 53, 15, 0.98)"),
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      }),
      label: {
        text: planningCoordinateReadout(point),
        font: "bold 10px sans-serif",
        fillColor: Color.WHITE,
        outlineColor: Color.BLACK,
        outlineWidth: 2,
        style: LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cartesian2(0, -18),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
  );
}

export function syncPlanningMeasurementLayer(
  viewer: Viewer | null | undefined,
  state: PlanningMeasurementState | null | undefined,
  visible: boolean,
): void {
  if (!isViewerUsable(viewer)) return;
  removeMeasurementEntities(viewer);
  if (!visible || !state) return;

  state.distancePoints.forEach((point, index) => addPoint(viewer, point, index));

  const [start, end] = state.distancePoints;
  if (start && end) {
    viewer.entities.add(
      new Entity({
        id: `${MEASUREMENT_PREFIX}distance-line`,
        name: "Planning distance measurement",
        polyline: {
          positions: [measurementCartesian(start), measurementCartesian(end)],
          width: 2.5,
          material: Color.fromCssColorString("rgba(251, 191, 36, 0.86)"),
        },
      }),
    );
    const distance = planningDistanceMeters(start, end);
    viewer.entities.add(
      new Entity({
        id: `${MEASUREMENT_PREFIX}distance-label`,
        position: measurementCartesian({
          x: (start.x + end.x) / 2,
          y: (start.y + end.y) / 2,
        }),
        label: {
          text: `${Math.round(distance)}m / ${(distance / 1000).toFixed(2)}km ${planningBearingCardinal(start, end)}`,
          font: "bold 11px sans-serif",
          fillColor: Color.WHITE,
          outlineColor: Color.BLACK,
          outlineWidth: 2,
          style: LabelStyle.FILL_AND_OUTLINE,
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(120, 53, 15, 0.88)"),
          pixelOffset: new Cartesian2(0, -20),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      }),
    );
  }

  if (state.radiusCenter) {
    viewer.entities.add(
      new Entity({
        id: `${MEASUREMENT_PREFIX}radius-ring`,
        name: planningRadiusMetadata(state.radiusMeters).label,
        polyline: {
          positions: circleVertices(state.radiusCenter, state.radiusMeters).map(measurementCartesian),
          width: 2,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString("rgba(250, 204, 21, 0.68)"),
            dashLength: 14,
          }),
        },
      }),
    );
  }
}

export function planningMeasurementsPreserveRuntimeBounds(): boolean {
  return WORLD_BOUNDS.x.max === 500 && WORLD_BOUNDS.y.max === 500;
}
