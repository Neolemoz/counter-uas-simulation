import type { EntityType } from "@/world/entityCatalog";
import { ENTITY_LABELS, isEntityType } from "@/world/entityCatalog";

/** Invisible grab target extends beyond cell for easier drag (pass: usability). */
export const MARKER_HIT_RADIUS_FACTOR = 1.12;
export const SPAWN_PREVIEW_RADIUS_FACTOR = 0.92;
export const SELECTED_RING_RADIUS_FACTOR = 0.98;
export const HOVER_RING_RADIUS_FACTOR = 0.88;

const TYPE_FILL: Record<EntityType, string> = {
  radar: "#0e7490",
  interceptor: "#047857",
  drone: "#b91c1c",
  waypoint_marker: "#b45309",
};

const TYPE_STROKE: Record<EntityType, string> = {
  radar: "#22d3ee",
  interceptor: "#34d399",
  drone: "#f87171",
  waypoint_marker: "#fbbf24",
};

export interface MarkerVisualStyle {
  fill: string;
  stroke: string;
  strokeWidth: number;
  glyphSize: number;
  hitRadiusFactor: number;
  outerRingStroke: string;
  outerRingWidth: number;
  outerRingRadiusFactor: number;
  outerRingOpacity: number;
  innerRingStroke: string | null;
}

export function resolveEntityType(entityType: string): EntityType {
  return isEntityType(entityType) ? entityType : "drone";
}

export function markerVisualStyle(
  entityType: string,
  selected: boolean,
  hovered: boolean,
  dragging: boolean,
): MarkerVisualStyle {
  const type = resolveEntityType(entityType);
  const fill = selected ? "#0891b2" : TYPE_FILL[type];
  const stroke = selected ? "#e0f2fe" : hovered || dragging ? "#67e8f9" : TYPE_STROKE[type];

  return {
    fill,
    stroke,
    strokeWidth: selected ? 2.8 : hovered || dragging ? 2.2 : 1.4,
    glyphSize: selected ? 17 : hovered || dragging ? 16.5 : 16,
    hitRadiusFactor: MARKER_HIT_RADIUS_FACTOR,
    outerRingStroke: selected ? "#fde68a" : hovered || dragging ? "#67e8f9" : "transparent",
    outerRingWidth: selected ? 2.4 : hovered || dragging ? 1.8 : 0,
    outerRingRadiusFactor: selected
      ? SELECTED_RING_RADIUS_FACTOR
      : hovered || dragging
        ? HOVER_RING_RADIUS_FACTOR
        : 0,
    outerRingOpacity: selected ? 1 : 0.85,
    innerRingStroke: selected ? "#f0f9ff" : null,
  };
}

export function markerCellOffset(
  indexInCell: number,
  countInCell: number,
  cellSize: number,
): { dx: number; dy: number } {
  if (countInCell <= 1) return { dx: 0, dy: 0 };
  const angle = (2 * Math.PI * indexInCell) / countInCell - Math.PI / 2;
  const radius = cellSize * 0.24;
  return {
    dx: Math.cos(angle) * radius,
    dy: Math.sin(angle) * radius,
  };
}

export function spawnTypeLabel(entityType: EntityType): string {
  return ENTITY_LABELS[entityType] ?? entityType;
}
