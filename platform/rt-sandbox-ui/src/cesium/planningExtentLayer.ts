import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  PolylineDashMaterialProperty,
  Viewer,
} from "cesium";
import { WORLD_BOUNDS } from "@/world/bounds";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { sampleTerrainHeight } from "./rtFictionalTerrain";
import type { PlanningExtent } from "./planningWorld";

const PLANNING_EXTENT_PREFIX = "rt-planning-extent-";
const PLANNING_EXTENT_RING_ID = `${PLANNING_EXTENT_PREFIX}ring`;
const PLANNING_EXTENT_LABEL_ID = `${PLANNING_EXTENT_PREFIX}label`;
const PLANNING_EXTENT_RUNTIME_LABEL_ID = `${PLANNING_EXTENT_PREFIX}runtime-distinction`;

export const PLANNING_EXTENT_LAYER_COPY =
  "Planning extent only; not runtime bounds, bridge authority, or MC execution authority.";

export function planningExtentAreaKm2(extent: PlanningExtent): number {
  return Math.PI * (extent.planning_extent_radius_m / 1000) ** 2;
}

export function isInsideRuntimeBounds(vertex: { x: number; y: number }): boolean {
  return (
    vertex.x >= WORLD_BOUNDS.x.min &&
    vertex.x <= WORLD_BOUNDS.x.max &&
    vertex.y >= WORLD_BOUNDS.y.min &&
    vertex.y <= WORLD_BOUNDS.y.max
  );
}

export function isInsidePlanningExtent(
  vertex: { x: number; y: number },
  extent: PlanningExtent,
): boolean {
  return Math.hypot(vertex.x, vertex.y) <= extent.planning_extent_radius_m;
}

function circlePositions(extent: PlanningExtent): import("cesium").Cartesian3[] {
  return Array.from({ length: 145 }, (_, index) => {
    const theta = (Math.PI * 2 * index) / 144;
    const x = Math.cos(theta) * extent.planning_extent_radius_m;
    const y = Math.sin(theta) * extent.planning_extent_radius_m;
    return worldToCartesian(x, y, sampleTerrainHeight(x, y) + 1.2);
  });
}

function removePlanningExtentEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((entity) => {
    if ((entity.id ?? "").startsWith(PLANNING_EXTENT_PREFIX)) toRemove.push(entity);
  });
  for (const entity of toRemove) viewer.entities.remove(entity);
}

export function syncPlanningExtentLayer(
  viewer: Viewer | null | undefined,
  extent: PlanningExtent | null | undefined,
  visible: boolean,
): void {
  if (!isViewerUsable(viewer)) return;
  removePlanningExtentEntities(viewer);
  if (!visible || !extent) return;

  const dashLength = Math.max(18, Math.round(extent.planning_extent_radius_m / 550));

  viewer.entities.add(
    new Entity({
      id: PLANNING_EXTENT_RING_ID,
      name: `${extent.planning_extent_label} boundary (planning only)`,
      polyline: {
        positions: circlePositions(extent),
        width: extent.planning_extent_radius_m >= 20_000 ? 3 : 2.4,
        material: new PolylineDashMaterialProperty({
          color: Color.fromCssColorString("rgba(34, 211, 238, 0.88)"),
          dashLength,
        }),
        clampToGround: false,
      },
    }),
  );

  const labelStyle = {
    font: "bold 11px sans-serif",
    fillColor: Color.fromCssColorString("rgba(207, 250, 254, 0.98)"),
    outlineColor: Color.BLACK,
    outlineWidth: 2,
    style: LabelStyle.FILL_AND_OUTLINE,
    showBackground: true,
    backgroundColor: Color.fromCssColorString("rgba(8, 47, 73, 0.88)"),
    pixelOffset: new Cartesian2(0, -20),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  };
  viewer.entities.add(
    new Entity({
      id: PLANNING_EXTENT_LABEL_ID,
      position: worldToCartesian(
        0,
        extent.planning_extent_radius_m,
        sampleTerrainHeight(0, extent.planning_extent_radius_m) + 8,
      ),
      label: {
        text: `${extent.planning_extent_label} · planning-only boundary · ${(extent.planning_extent_radius_m / 1000).toFixed(0)} km radius`,
        ...labelStyle,
      },
    }),
  );
  viewer.entities.add(
    new Entity({
      id: PLANNING_EXTENT_RUNTIME_LABEL_ID,
      position: worldToCartesian(WORLD_BOUNDS.x.max, WORLD_BOUNDS.y.max, WORLD_BOUNDS.z.max + 18),
      label: {
        text: "Runtime bounds remain +/-500m",
        ...labelStyle,
        backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.9)"),
      },
    }),
  );
}
