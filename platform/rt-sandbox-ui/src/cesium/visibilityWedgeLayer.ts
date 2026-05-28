import { Color, Entity, PolylineDashMaterialProperty, Viewer } from "cesium";
import { WORLD_BOUNDS } from "@/world/bounds";
import type { MirrorEntity } from "./entityMarkers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import {
  DEFAULT_VISIBILITY_WEDGE_AZIMUTH_DEG,
  VISIBILITY_WEDGE_COLOR,
} from "./visualStyle";
import { CANONICAL_VISUAL_LAYER_REGISTRY } from "./visualLayerRegistry";

const WEDGE_PREFIX = "rt-visibility-wedge-";

export function boundsDiagonalHalfM(): number {
  const { x, y } = WORLD_BOUNDS;
  const dx = x.max - x.min;
  const dy = y.max - y.min;
  return Math.hypot(dx, dy) / 2;
}

function removeWedgeEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(WEDGE_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

export function wedgeRayEndpoints(
  originX: number,
  originY: number,
  originZ: number,
  yawDeg: number,
  halfAzimuthDeg: number,
  lengthM: number,
): { left: [number, number, number]; right: [number, number, number] } {
  const yawRad = (yawDeg * Math.PI) / 180;
  const leftAngle = yawRad - (halfAzimuthDeg * Math.PI) / 180;
  const rightAngle = yawRad + (halfAzimuthDeg * Math.PI) / 180;
  return {
    left: [
      originX + Math.cos(leftAngle) * lengthM,
      originY + Math.sin(leftAngle) * lengthM,
      originZ,
    ],
    right: [
      originX + Math.cos(rightAngle) * lengthM,
      originY + Math.sin(rightAngle) * lengthM,
      originZ,
    ],
  };
}

export function countWedgePolylines(halfAzimuthDeg: number): number {
  return Math.min(
    3,
    CANONICAL_VISUAL_LAYER_REGISTRY.performance_budget.max_wedge_polylines_per_session,
    Math.max(2, Math.ceil((halfAzimuthDeg * 2) / 30)),
  );
}

export function syncVisibilityWedgeLayer(
  viewer: Viewer | null | undefined,
  selected: MirrorEntity | null,
  show: boolean,
  applyTerrainDisplay: boolean,
  options?: { halfAzimuthDeg?: number },
): void {
  if (!isViewerUsable(viewer)) return;
  removeWedgeEntities(viewer);
  if (!show || !selected) return;

  const ax = Number(selected.pose.x ?? 0);
  const ay = Number(selected.pose.y ?? 0);
  const az = Number(selected.pose.z ?? 0);
  const z = applyTerrainDisplay ? applyTerrainDisplayOffset(ax, ay, az) : az;
  const yawDeg = Number(selected.pose.yaw_deg ?? 0);
  const halfAz = options?.halfAzimuthDeg ?? DEFAULT_VISIBILITY_WEDGE_AZIMUTH_DEG;
  const lengthM = boundsDiagonalHalfM();

  const { left, right } = wedgeRayEndpoints(ax, ay, z, yawDeg, halfAz, lengthM);
  const origin: [number, number, number] = [ax, ay, z];
  const material = new PolylineDashMaterialProperty({
    color: Color.fromCssColorString(VISIBILITY_WEDGE_COLOR),
    dashLength: 10,
  });

  const rays: [number, number, number][][] = [
    [origin, left],
    [origin, right],
  ];
  const midAngle = (yawDeg * Math.PI) / 180;
  const midEnd: [number, number, number] = [
    ax + Math.cos(midAngle) * lengthM,
    ay + Math.sin(midAngle) * lengthM,
    z,
  ];
  if (countWedgePolylines(halfAz) >= 3) {
    rays.push([origin, midEnd]);
  }

  rays.forEach((positions, i) => {
    viewer.entities.add(
      new Entity({
        id: `${WEDGE_PREFIX}ray-${i}`,
        polyline: {
          positions: positions.map(([x, y, h]) => worldToCartesian(x, y, h)),
          width: 2,
          material,
        },
      }),
    );
  });
}

export function clearVisibilityWedgeLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeWedgeEntities(viewer);
}
