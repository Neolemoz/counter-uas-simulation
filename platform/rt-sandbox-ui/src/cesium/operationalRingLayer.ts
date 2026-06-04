import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  PolylineDashMaterialProperty,
  Viewer,
} from "cesium";
import { WORLD_AXIS_HALF_EXTENT_M, WORLD_BOUNDS } from "@/world/bounds";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { sampleTerrainHeight } from "./rtFictionalTerrain";

const RING_PREFIX = "rt-operational-ring-";

export const OPERATIONAL_RING_GOVERNANCE_COPY =
  "Operational rings are display-only; not runtime authority, bridge validation, or MC semantics.";

export const OPERATIONAL_RING_RADII_M = {
  cityM: 1000,
  defenseM: 3000,
  warningM: 5000,
  spawnInnerM: 5000,
  spawnOuterM: WORLD_AXIS_HALF_EXTENT_M,
} as const;

export interface OperationalRingSpec {
  ringId: string;
  radiusM: number;
  label: string;
  color: string;
  dashLength: number;
  width: number;
}

export const OPERATIONAL_RING_SPECS: OperationalRingSpec[] = [
  {
    ringId: "city",
    radiusM: OPERATIONAL_RING_RADII_M.cityM,
    label: "City radius 1000 m (display only)",
    color: "rgba(52, 211, 153, 0.82)",
    dashLength: 10,
    width: 2,
  },
  {
    ringId: "defense",
    radiusM: OPERATIONAL_RING_RADII_M.defenseM,
    label: "Defense radius 3000 m (display only)",
    color: "rgba(251, 191, 36, 0.78)",
    dashLength: 12,
    width: 2.2,
  },
  {
    ringId: "warning",
    radiusM: OPERATIONAL_RING_RADII_M.warningM,
    label: "Warning radius 5000 m (display only)",
    color: "rgba(251, 146, 60, 0.78)",
    dashLength: 14,
    width: 2.4,
  },
  {
    ringId: "spawn-outer",
    radiusM: OPERATIONAL_RING_RADII_M.spawnOuterM,
    label: "Spawn band outer 7000 m (display only)",
    color: "rgba(248, 113, 113, 0.72)",
    dashLength: 16,
    width: 2.6,
  },
];

function circlePositions(radiusM: number): import("cesium").Cartesian3[] {
  return Array.from({ length: 97 }, (_, index) => {
    const theta = (Math.PI * 2 * index) / 96;
    const x = Math.cos(theta) * radiusM;
    const y = Math.sin(theta) * radiusM;
    return worldToCartesian(x, y, sampleTerrainHeight(x, y) + 1.4);
  });
}

function spawnBandInnerPositions(): import("cesium").Cartesian3[] {
  return circlePositions(OPERATIONAL_RING_RADII_M.spawnInnerM);
}

function removeOperationalRingEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((entity) => {
    if ((entity.id ?? "").startsWith(RING_PREFIX)) toRemove.push(entity);
  });
  for (const entity of toRemove) viewer.entities.remove(entity);
}

export function syncOperationalRingLayer(
  viewer: Viewer | null | undefined,
  visible: boolean,
): void {
  if (!isViewerUsable(viewer)) return;
  removeOperationalRingEntities(viewer);
  if (!visible) return;

  const labelStyle = {
    font: "10px sans-serif",
    fillColor: Color.fromCssColorString("rgba(254, 243, 199, 0.96)"),
    outlineColor: Color.BLACK,
    outlineWidth: 2,
    style: LabelStyle.FILL_AND_OUTLINE,
    showBackground: true,
    backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.88)"),
    pixelOffset: new Cartesian2(0, -16),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  };

  for (const spec of OPERATIONAL_RING_SPECS) {
    viewer.entities.add(
      new Entity({
        id: `${RING_PREFIX}${spec.ringId}`,
        name: spec.label,
        polyline: {
          positions: circlePositions(spec.radiusM),
          width: spec.width,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString(spec.color),
            dashLength: spec.dashLength,
          }),
          clampToGround: false,
        },
      }),
    );
    viewer.entities.add(
      new Entity({
        id: `${RING_PREFIX}${spec.ringId}-label`,
        position: worldToCartesian(
          0,
          spec.radiusM,
          sampleTerrainHeight(0, spec.radiusM) + 6,
        ),
        label: { text: spec.label, ...labelStyle },
      }),
    );
  }

  viewer.entities.add(
    new Entity({
      id: `${RING_PREFIX}spawn-inner`,
      name: "Spawn band inner 5000 m (display only)",
      polyline: {
        positions: spawnBandInnerPositions(),
        width: 2.2,
        material: new PolylineDashMaterialProperty({
          color: Color.fromCssColorString("rgba(248, 113, 113, 0.55)"),
          dashLength: 10,
        }),
        clampToGround: false,
      },
    }),
  );

  viewer.entities.add(
    new Entity({
      id: `${RING_PREFIX}spawn-band-label`,
      position: worldToCartesian(
        OPERATIONAL_RING_RADII_M.spawnInnerM * 0.72,
        OPERATIONAL_RING_RADII_M.spawnInnerM * 0.72,
        sampleTerrainHeight(
          OPERATIONAL_RING_RADII_M.spawnInnerM * 0.72,
          OPERATIONAL_RING_RADII_M.spawnInnerM * 0.72,
        ) + 8,
      ),
      label: {
        text: "Attacker spawn band 5000–7000 m (display only)",
        ...labelStyle,
      },
    }),
  );

  viewer.entities.add(
    new Entity({
      id: `${RING_PREFIX}governance`,
      position: worldToCartesian(
        WORLD_BOUNDS.x.min,
        WORLD_BOUNDS.y.max,
        WORLD_BOUNDS.z.max + 24,
      ),
      label: {
        text: OPERATIONAL_RING_GOVERNANCE_COPY,
        ...labelStyle,
        font: "9px sans-serif",
      },
    }),
  );
}
