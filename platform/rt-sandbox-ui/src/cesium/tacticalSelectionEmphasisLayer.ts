import { Cartesian2, Color, Entity, LabelStyle, VerticalOrigin, Viewer } from "cesium";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { cameraHeightM } from "./cameraHelpers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import {
  tacticalLabelFontCss,
  tacticalSelectionHaloPixelSize,
  tacticalSelectionLabelOffset,
  tacticalSelectionOutlineWidth,
} from "./tacticalVisualScale";
import { shortEntityId } from "./visualStyle";

const TACTICAL_SELECTION_PREFIX = "rt-tactical-selection-";

function removeTacticalSelectionEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(TACTICAL_SELECTION_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function poseFromEntity(
  entity: MirrorEntity | undefined,
): { x: number; y: number; z: number } | null {
  if (!entity) return null;
  const x = Number(entity.pose.x);
  const y = Number(entity.pose.y);
  const z = Number(entity.pose.z);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return null;
  }
  return { x, y, z };
}

export function targetIdFromTacticalState(
  state: TacticalStatePayload | null | undefined,
): string | null {
  return state?.assigned_target_id ?? state?.selected_target_id ?? null;
}

export function syncTacticalSelectionEmphasisLayer(
  viewer: Viewer | null | undefined,
  options: {
    enabled: boolean;
    tacticalState: TacticalStatePayload | null | undefined;
    entities: MirrorEntity[];
    selectedEntityId: string | null;
    applyTerrainDisplay: boolean;
    stale?: boolean;
  },
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalSelectionEntities(viewer);
  if (!options.enabled) return;

  const targetId = targetIdFromTacticalState(options.tacticalState);
  if (!targetId || targetId === options.selectedEntityId) return;

  const target = options.entities.find((e) => e.entity_id === targetId);
  const pose = poseFromEntity(target);
  if (!pose) return;

  const displayZ = options.applyTerrainDisplay
    ? applyTerrainDisplayOffset(pose.x, pose.y, pose.z)
    : pose.z;
  const alpha = options.stale ? 0.48 : 1;
  const cameraHeight = cameraHeightM(viewer);
  const haloSize = tacticalSelectionHaloPixelSize(cameraHeight);
  const outlineWidth = tacticalSelectionOutlineWidth(cameraHeight);
  const labelOffset = tacticalSelectionLabelOffset(cameraHeight);
  const position = worldToCartesian(pose.x, pose.y, displayZ);

  viewer.entities.add(
    new Entity({
      id: `${TACTICAL_SELECTION_PREFIX}target-halo`,
      position,
      point: {
        pixelSize: haloSize,
        color: Color.fromCssColorString("rgba(248, 113, 113, 0.22)").withAlpha(
          0.22 * alpha,
        ),
        outlineColor: Color.fromCssColorString("rgba(254, 226, 226, 0.95)").withAlpha(
          0.95 * alpha,
        ),
        outlineWidth,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: `tactical target ${shortEntityId(targetId)}`,
        font: tacticalLabelFontCss(cameraHeight, true),
        fillColor: Color.fromCssColorString("rgba(254, 243, 199, 0.98)").withAlpha(
          0.98 * alpha,
        ),
        outlineColor: Color.BLACK,
        outlineWidth: 2,
        style: LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: VerticalOrigin.BOTTOM,
        pixelOffset: new Cartesian2(labelOffset.x, labelOffset.y),
        showBackground: true,
        backgroundColor: Color.fromCssColorString("rgba(69, 10, 10, 0.82)").withAlpha(
          0.82 * alpha,
        ),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
  );
}

export function clearTacticalSelectionEmphasisLayer(
  viewer: Viewer | null | undefined,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalSelectionEntities(viewer);
}
