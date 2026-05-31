import { LABEL_OFFSET_Y } from "./visualStyle";

export interface MarkerLabelLayoutInput {
  entityId: string;
  entityType: string;
  x: number;
  y: number;
  selected: boolean;
  hovered: boolean;
  /** Tactical assigned/selected target — fan bias to reduce overlay overlap. */
  tacticalTarget?: boolean;
}

export interface MarkerLabelLayout {
  offsetX: number;
  offsetY: number;
  /** Icon-only label when clustered at distance — reduces overlap. */
  glyphOnly: boolean;
}

const TYPE_BIAS_X: Record<string, number> = {
  radar: -14,
  interceptor: -4,
  drone: 4,
  waypoint_marker: 14,
};

export function clusterRadiusMForCamera(cameraHeightM: number): number {
  if (cameraHeightM > 4000) return 72;
  if (cameraHeightM > 2500) return 48;
  if (cameraHeightM > 1200) return 28;
  if (cameraHeightM > 600) return 16;
  return 10;
}

function hashEntityId(entityId: string): number {
  let hash = 0;
  for (let i = 0; i < entityId.length; i += 1) {
    hash = (hash * 31 + entityId.charCodeAt(i)) | 0;
  }
  return hash;
}

function singleMarkerOffset(
  entityType: string,
  entityId: string,
  selected: boolean,
  tacticalTarget = false,
): { offsetX: number; offsetY: number } {
  const hash = hashEntityId(entityId);
  const jitter = ((hash % 5) - 2) * 3;
  let x = (TYPE_BIAS_X[entityType] ?? 0) + jitter;
  let y = LABEL_OFFSET_Y + Math.max(-5, Math.min(5, Math.round(jitter / 2)));
  if (entityType === "radar" && selected) {
    y -= 18;
  }
  if (tacticalTarget) {
    x += 18;
    y -= 10;
  }
  return { offsetX: x, offsetY: y };
}

function clusterFanOffset(
  index: number,
  count: number,
  selected: boolean,
  hovered: boolean,
): { offsetX: number; offsetY: number } {
  if (selected) return { offsetX: 0, offsetY: -22 };
  if (hovered) return { offsetX: 0, offsetY: -18 };
  const angle = (2 * Math.PI * index) / count - Math.PI / 2;
  const radius = Math.min(32, 10 + count * 5);
  return {
    offsetX: Math.round(Math.cos(angle) * radius),
    offsetY: Math.round(Math.sin(angle) * radius) + LABEL_OFFSET_Y,
  };
}

function buildClusters(
  markers: MarkerLabelLayoutInput[],
  radiusM: number,
): MarkerLabelLayoutInput[][] {
  const remaining = [...markers];
  const clusters: MarkerLabelLayoutInput[][] = [];

  while (remaining.length > 0) {
    const seed = remaining.shift();
    if (!seed) break;
    const cluster = [seed];
    for (let i = remaining.length - 1; i >= 0; i -= 1) {
      const other = remaining[i];
      const near = cluster.some(
        (member) => Math.hypot(member.x - other.x, member.y - other.y) <= radiusM,
      );
      if (near) {
        cluster.push(other);
        remaining.splice(i, 1);
      }
    }
    clusters.push(cluster);
  }

  return clusters;
}

export function computeMarkerLabelLayouts(
  markers: MarkerLabelLayoutInput[],
  cameraHeightM: number,
): Map<string, MarkerLabelLayout> {
  const radiusM = clusterRadiusMForCamera(cameraHeightM);
  const clusters = buildClusters(markers, radiusM);
  const layouts = new Map<string, MarkerLabelLayout>();

  for (const cluster of clusters) {
    const sorted = [...cluster].sort((a, b) => {
      if (a.tacticalTarget !== b.tacticalTarget) return a.tacticalTarget ? -1 : 1;
      if (a.selected !== b.selected) return a.selected ? -1 : 1;
      if (a.hovered !== b.hovered) return a.hovered ? -1 : 1;
      return a.entityId.localeCompare(b.entityId);
    });
    const count = sorted.length;
    const denseCluster = count > 1;
    const glyphOnlyAtDistance =
      cameraHeightM > 2200 && count > 2;

    sorted.forEach((marker, index) => {
      const offset = denseCluster
        ? clusterFanOffset(index, count, marker.selected, marker.hovered)
        : singleMarkerOffset(
            marker.entityType,
            marker.entityId,
            marker.selected,
            marker.tacticalTarget,
          );
      const glyphOnly =
        glyphOnlyAtDistance &&
        !marker.selected &&
        !marker.hovered &&
        !marker.tacticalTarget;
      layouts.set(marker.entityId, {
        offsetX: offset.offsetX,
        offsetY: offset.offsetY,
        glyphOnly,
      });
    });
  }

  return layouts;
}
