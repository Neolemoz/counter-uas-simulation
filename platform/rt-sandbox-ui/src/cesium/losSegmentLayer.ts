import { Color, Entity, PolylineDashMaterialProperty, Viewer } from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { nearestOcclusionTarget } from "./terrainCognition";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import { VISIBILITY_STACKED_LOS_COLOR } from "./visualStyle";

const LOS_PREFIX = "rt-terrain-los-";

function removeLosEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(LOS_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

export function syncLosSegmentLayer(
  viewer: Viewer | null | undefined,
  selected: MirrorEntity | null,
  entities: MirrorEntity[],
  show: boolean,
  applyTerrainDisplay: boolean,
  stackedMode = false,
): void {
  if (!isViewerUsable(viewer)) return;
  removeLosEntities(viewer);
  if (!show || !selected) return;

  const occ = nearestOcclusionTarget(selected, entities);
  if (!occ) return;

  const ax = Number(selected.pose.x ?? 0);
  const ay = Number(selected.pose.y ?? 0);
  const az = Number(selected.pose.z ?? 0);
  const bx = Number(occ.target.pose.x ?? 0);
  const by = Number(occ.target.pose.y ?? 0);
  const bz = Number(occ.target.pose.z ?? 0);
  const zA = applyTerrainDisplay ? applyTerrainDisplayOffset(ax, ay, az) : az;
  const zB = applyTerrainDisplay ? applyTerrainDisplayOffset(bx, by, bz) : bz;

  const color = stackedMode
    ? VISIBILITY_STACKED_LOS_COLOR
    : occ.status === "clear"
      ? "rgba(148, 163, 184, 0.5)"
      : occ.status === "terrain_blocked"
        ? "rgba(251, 146, 60, 0.65)"
        : "rgba(74, 222, 128, 0.55)";

  viewer.entities.add(
    new Entity({
      id: `${LOS_PREFIX}segment`,
      polyline: {
        positions: [worldToCartesian(ax, ay, zA), worldToCartesian(bx, by, zB)],
        width: 2,
        material: new PolylineDashMaterialProperty({
          color: Color.fromCssColorString(color),
          dashLength: 8,
        }),
      },
    }),
  );
}

export function clearLosSegmentLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeLosEntities(viewer);
}
