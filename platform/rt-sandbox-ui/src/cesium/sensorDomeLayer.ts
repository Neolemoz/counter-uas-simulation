import { Cartesian3, Color, Entity, LabelStyle, Viewer } from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import {
  applyTerrainDisplayOffset,
  NOMINAL_SENSOR_DOME_RADIUS_M,
  sampleTerrainHeight,
} from "./rtFictionalTerrain";

const DOME_PREFIX = "rt-terrain-dome-";

export function horizontalDistanceM(
  ax: number,
  ay: number,
  bx: number,
  by: number,
): number {
  const dx = ax - bx;
  const dy = ay - by;
  return Math.sqrt(dx * dx + dy * dy);
}

/** Count entities within horizontal nominal dome range (explanatory geometry). */
export function countEntitiesInNominalDome(
  radarX: number,
  radarY: number,
  entities: MirrorEntity[],
  radiusM = NOMINAL_SENSOR_DOME_RADIUS_M,
): number {
  let count = 0;
  for (const ent of entities) {
    if (ent.entity_type === "radar") continue;
    const x = Number(ent.pose.x ?? 0);
    const y = Number(ent.pose.y ?? 0);
    if (horizontalDistanceM(radarX, radarY, x, y) <= radiusM) count += 1;
  }
  return count;
}

function removeDomeEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(DOME_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

export function syncSensorDomeLayer(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[],
  show: boolean,
  applyTerrainDisplay: boolean,
): void {
  if (!isViewerUsable(viewer)) return;
  removeDomeEntities(viewer);
  if (!show) return;

  for (const ent of entities) {
    if (ent.entity_type !== "radar" || !ent.entity_id) continue;
    const x = Number(ent.pose.x ?? 0);
    const y = Number(ent.pose.y ?? 0);
    const zReg = Number(ent.pose.z ?? 10);
    const z = applyTerrainDisplay
      ? applyTerrainDisplayOffset(x, y, zReg)
      : zReg + sampleTerrainHeight(x, y);
    const r = NOMINAL_SENSOR_DOME_RADIUS_M;
    const groundZ = applyTerrainDisplay ? sampleTerrainHeight(x, y) : 0;

    viewer.entities.add(
      new Entity({
        id: `${DOME_PREFIX}${ent.entity_id}`,
        position: worldToCartesian(x, y, z),
        ellipsoid: {
          radii: new Cartesian3(r, r, r * 0.4),
          material: Color.fromCssColorString("rgba(56, 189, 248, 0.1)"),
          outline: true,
          outlineColor: Color.fromCssColorString("rgba(56, 189, 248, 0.5)"),
          outlineWidth: 1,
        },
        label: {
          text: `nominal dome ${r}m (explanatory)`,
          font: "10px sans-serif",
          fillColor: Color.fromCssColorString("rgba(186, 230, 253, 0.95)"),
          outlineColor: Color.BLACK,
          outlineWidth: 2,
          style: LabelStyle.FILL_AND_OUTLINE,
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.88)"),
        },
      }),
    );

    const ring: Cartesian3[] = [];
    for (let i = 0; i <= 16; i++) {
      const a = (i / 16) * Math.PI * 2;
      ring.push(worldToCartesian(x + Math.cos(a) * r, y + Math.sin(a) * r, groundZ + 1));
    }
    viewer.entities.add(
      new Entity({
        id: `${DOME_PREFIX}ring-${ent.entity_id}`,
        polyline: {
          positions: ring,
          width: 2,
          material: Color.fromCssColorString("rgba(56, 189, 248, 0.35)"),
        },
      }),
    );
  }
}

export function clearSensorDomeLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeDomeEntities(viewer);
}
