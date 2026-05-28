import { Color, Entity, PolylineDashMaterialProperty, Viewer } from "cesium";
import { WORLD_BOUNDS } from "@/world/bounds";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { VISIBILITY_HORIZON_COLOR } from "./visualStyle";

const HORIZON_PREFIX = "rt-visibility-horizon-";

function groundRingPositions(): import("cesium").Cartesian3[] {
  const { x, y, z } = WORLD_BOUNDS;
  const zGround = z.min;
  const corners: [number, number][] = [
    [x.min, y.min],
    [x.max, y.min],
    [x.max, y.max],
    [x.min, y.max],
    [x.min, y.min],
  ];
  return corners.map(([cx, cy]) => worldToCartesian(cx, cy, zGround));
}

function removeHorizonEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(HORIZON_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

export function syncHorizonHintLayer(
  viewer: Viewer | null | undefined,
  show: boolean,
): void {
  if (!isViewerUsable(viewer)) return;
  removeHorizonEntities(viewer);
  if (!show) return;

  viewer.entities.add(
    new Entity({
      id: `${HORIZON_PREFIX}ring`,
      polyline: {
        positions: groundRingPositions(),
        width: 2,
        material: new PolylineDashMaterialProperty({
          color: Color.fromCssColorString(VISIBILITY_HORIZON_COLOR),
          dashLength: 14,
          gapColor: Color.TRANSPARENT,
        }),
      },
    }),
  );
}

export function clearHorizonHintLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeHorizonEntities(viewer);
}

export function horizonRingVertexCount(): number {
  return 5;
}
