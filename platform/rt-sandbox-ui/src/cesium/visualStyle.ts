/** Cesium visual tokens (PLAT-RT-V1). */

import {
  ENTITY_GLYPHS,
  ENTITY_LABELS,
  type EntityType,
  isEntityType,
} from "@/world/entityCatalog";

export const MARKER_PIXEL_SIZE = 12;
export const MARKER_SELECTED_PIXEL_SIZE = 16;
export const GHOST_PIXEL_SIZE = 10;

export const LABEL_FONT = "12px sans-serif";
export const LABEL_OFFSET_Y = -18;
export const LABEL_BACKGROUND = "rgba(15, 23, 42, 0.88)";

export const BOUNDS_LINE_WIDTH = 3;
export const BOUNDS_VERTICAL_WIDTH = 2;
export const BOUNDS_GROUND_COLOR = "rgba(56, 189, 248, 0.95)";
export const BOUNDS_VERTICAL_COLOR = "rgba(56, 189, 248, 0.55)";
export const BOUNDS_TOP_COLOR = "rgba(56, 189, 248, 0.45)";

export const TIGHT_BOUNDS_CAMERA_HEIGHT_M = 1200;
export const TERRAIN_OVERVIEW_CAMERA_HEIGHT_M = 2400;

export const SELECTION_RING_PIXEL_SIZE = 22;
export const SELECTION_RING_COLOR = "rgba(251, 191, 36, 0.35)";

export const BOUNDS_LABEL_GROUND = "±500m";
export const BOUNDS_LABEL_Z = "z 0–200m";

/** PLAT-RT-V3 P1 — visibility overlay tokens */
export const VISIBILITY_WEDGE_COLOR = "rgba(167, 139, 250, 0.55)";
export const VISIBILITY_HORIZON_COLOR = "rgba(148, 163, 184, 0.45)";
export const VISIBILITY_STACKED_LOS_COLOR = "rgba(129, 140, 248, 0.7)";
export const DEFAULT_VISIBILITY_WEDGE_AZIMUTH_DEG = 30;

export function shortEntityId(entityId: string): string {
  if (entityId.length <= 8) return entityId;
  return entityId.slice(0, 8);
}

export function labelText(entityType: string, entityId: string): string {
  const type = isEntityType(entityType) ? entityType : "drone";
  const glyph = ENTITY_GLYPHS[type as EntityType] ?? "?";
  const label = ENTITY_LABELS[type as EntityType] ?? entityType;
  return `${glyph} ${label} · ${shortEntityId(entityId)}`;
}

export function labelTextWithAgl(
  entityType: string,
  entityId: string,
  displayAglM: number,
): string {
  return `${labelText(entityType, entityId)} · AGL ${displayAglM.toFixed(0)}m`;
}

export function markerPixelSize(selected: boolean, distanceScale = 1): number {
  const base = selected ? MARKER_SELECTED_PIXEL_SIZE : MARKER_PIXEL_SIZE;
  const scaled = base * distanceScale;
  return Math.max(8, Math.min(20, scaled));
}

/** Scale markers slightly when camera is far from bounds center. */
export function distanceScaleFromHeight(cameraHeightM: number): number {
  if (cameraHeightM > 2500) return 1.25;
  if (cameraHeightM > 1500) return 1.1;
  if (cameraHeightM < 600) return 0.95;
  return 1;
}

export function sessionAccentCss(index: number): string {
  const accents = [
    "rgb(251, 191, 36)",
    "rgb(56, 189, 248)",
    "rgb(167, 139, 250)",
  ];
  return accents[index % accents.length] ?? accents[0];
}
