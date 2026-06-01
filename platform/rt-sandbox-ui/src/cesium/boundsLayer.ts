import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  PolylineDashMaterialProperty,
  Viewer,
} from "cesium";
import { WORLD_BOUNDS } from "@/world/bounds";
import { BOUNDS_LAYER_ID } from "./constants";
import {
  BOUNDS_GROUND_COLOR,
  BOUNDS_LABEL_GROUND,
  BOUNDS_LABEL_Z,
  BOUNDS_LINE_WIDTH,
  BOUNDS_TOP_COLOR,
  BOUNDS_VERTICAL_COLOR,
  BOUNDS_VERTICAL_WIDTH,
} from "./visualStyle";
import { worldToCartesian } from "./coordinates";
import { isViewerUsable } from "./cesiumEditing";
import { sampleTerrainHeight } from "./rtFictionalTerrain";

const BOUNDS_TOP_ID = "rt-world-bounds-top";
const BOUNDS_VERT_PREFIX = "rt-world-bounds-vert-";
const BOUNDS_LABEL_GROUND_ID = "rt-world-bounds-label-ground";
const BOUNDS_LABEL_Z_ID = "rt-world-bounds-label-z";

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

function topRingPositions(): import("cesium").Cartesian3[] {
  const { x, y, z } = WORLD_BOUNDS;
  const zTop = z.max;
  const corners: [number, number][] = [
    [x.min, y.min],
    [x.max, y.min],
    [x.max, y.max],
    [x.min, y.max],
    [x.min, y.min],
  ];
  return corners.map(([cx, cy]) => worldToCartesian(cx, cy, zTop));
}

function verticalEdgePositions(cx: number, cy: number): import("cesium").Cartesian3[] {
  const { z } = WORLD_BOUNDS;
  return [
    worldToCartesian(cx, cy, z.min),
    worldToCartesian(cx, cy, z.max),
  ];
}

function removeBoundsEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    const id = e.id ?? "";
    if (
      id === BOUNDS_LAYER_ID ||
      id === BOUNDS_TOP_ID ||
      id.startsWith(BOUNDS_VERT_PREFIX) ||
      id === BOUNDS_LABEL_GROUND_ID ||
      id === BOUNDS_LABEL_Z_ID
    ) {
      toRemove.push(e);
    }
  });
  for (const e of toRemove) {
    viewer.entities.remove(e);
  }
}

export function syncBoundsLayer(
  viewer: Viewer | null | undefined,
  visible: boolean,
  options?: { showVertical?: boolean; showCornerLabels?: boolean },
): void {
  if (!isViewerUsable(viewer)) return;
  const showVertical = options?.showVertical ?? true;
  const showCornerLabels = options?.showCornerLabels ?? visible;

  if (!visible) {
    removeBoundsEntities(viewer);
    return;
  }

  removeBoundsEntities(viewer);

  const groundMaterial = new PolylineDashMaterialProperty({
    color: Color.fromCssColorString(BOUNDS_GROUND_COLOR),
    dashLength: 14,
  });
  const verticalMaterial = new PolylineDashMaterialProperty({
    color: Color.fromCssColorString(BOUNDS_VERTICAL_COLOR),
    dashLength: 10,
  });
  const topMaterial = new PolylineDashMaterialProperty({
    color: Color.fromCssColorString(BOUNDS_TOP_COLOR),
    dashLength: 12,
  });

  viewer.entities.add(
    new Entity({
      id: BOUNDS_LAYER_ID,
      name: "RT world bounds (ground)",
      polyline: {
        positions: groundRingPositions(),
        width: BOUNDS_LINE_WIDTH,
        material: groundMaterial,
        clampToGround: false,
      },
    }),
  );

  viewer.entities.add(
    new Entity({
      id: BOUNDS_TOP_ID,
      name: "RT world bounds (top)",
      polyline: {
        positions: topRingPositions(),
        width: BOUNDS_VERTICAL_WIDTH,
        material: topMaterial,
        clampToGround: false,
      },
    }),
  );

  if (showVertical) {
    const { x, y } = WORLD_BOUNDS;
    const corners: [number, number][] = [
      [x.min, y.min],
      [x.max, y.min],
      [x.max, y.max],
      [x.min, y.max],
    ];
    corners.forEach(([cx, cy], i) => {
      viewer.entities.add(
        new Entity({
          id: `${BOUNDS_VERT_PREFIX}${i}`,
          polyline: {
            positions: verticalEdgePositions(cx, cy),
            width: BOUNDS_VERTICAL_WIDTH,
            material: verticalMaterial,
            clampToGround: false,
          },
        }),
      );
    });
  }

  if (showCornerLabels) {
    const { x, y, z } = WORLD_BOUNDS;
    const labelStyle = {
      font: "11px sans-serif",
      fillColor: Color.fromCssColorString("rgba(226, 232, 240, 0.95)"),
      outlineColor: Color.BLACK,
      outlineWidth: 2,
      style: LabelStyle.FILL_AND_OUTLINE,
      showBackground: true,
      backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.85)"),
      pixelOffset: new Cartesian2(0, -20),
    };
    viewer.entities.add(
      new Entity({
        id: BOUNDS_LABEL_GROUND_ID,
        position: worldToCartesian(x.min, y.min, z.min),
        label: { text: BOUNDS_LABEL_GROUND, ...labelStyle },
      }),
    );
    viewer.entities.add(
      new Entity({
        id: BOUNDS_LABEL_Z_ID,
        position: worldToCartesian(x.max, y.max, z.max),
        label: { text: BOUNDS_LABEL_Z, ...labelStyle },
      }),
    );
  }
}

export function boundsCenterCartesian(): import("cesium").Cartesian3 {
  const { x, y } = WORLD_BOUNDS;
  const cx = (x.min + x.max) / 2;
  const cy = (y.min + y.max) / 2;
  const cz = sampleTerrainHeight(cx, cy) + 18;
  return worldToCartesian(cx, cy, cz);
}
