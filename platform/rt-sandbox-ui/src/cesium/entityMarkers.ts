import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  VerticalOrigin,
  Viewer,
} from "cesium";
import { markerStyleForHealth } from "./cognition";
import { cameraHeightM } from "./cameraHelpers";
import { worldToCartesian } from "./coordinates";
import { applyTerrainDisplayOffset, displayAglM, sampleTerrainHeight } from "./rtFictionalTerrain";
import { toCesiumEntityId } from "./entityId";
import { isViewerUsable } from "./cesiumEditing";
import {
  distanceScaleFromHeight,
  GHOST_PIXEL_SIZE,
  LABEL_BACKGROUND,
  LABEL_FONT,
  LABEL_OFFSET_Y,
  labelText,
  labelTextWithAgl,
  markerPixelSize,
  SELECTION_RING_COLOR,
  SELECTION_RING_PIXEL_SIZE,
} from "./visualStyle";

const GHOST_ENTITY_SUFFIX = "-cmd-ghost";
const GROUND_TICK_SUFFIX = "-ground-tick";

export interface MirrorEntity {
  entity_id: string;
  entity_type: string;
  pose: Record<string, unknown>;
}

function poseFromRecord(pose: Record<string, unknown>): {
  x: number;
  y: number;
  z: number;
} {
  return {
    x: Number(pose.x ?? 0),
    y: Number(pose.y ?? 0),
    z: Number(pose.z ?? 0),
  };
}

function colorForType(
  entityType: string,
  selected: boolean,
  health: "ok" | "stale" | "warn",
  accentOutline?: string,
) {
  if (health === "stale") {
    return Color.fromCssColorString(
      selected ? "rgba(251, 191, 36, 0.98)" : "rgba(251, 191, 36, 0.75)",
    );
  }
  if (health === "warn") {
    return Color.fromCssColorString(
      selected ? "rgba(248, 113, 113, 0.98)" : "rgba(248, 113, 113, 0.8)",
    );
  }
  const base: Record<string, string> = {
    radar: "#38bdf8",
    interceptor: "#a78bfa",
    drone: "#34d399",
    waypoint_marker: "#fbbf24",
  };
  const hex = base[entityType] ?? "#94a3b8";
  if (selected && accentOutline) {
    return Color.fromCssColorString(hex);
  }
  return Color.fromCssColorString(selected ? hex : `${hex}99`);
}

function outlineColorForMarker(
  selected: boolean,
  health: "ok" | "stale" | "warn",
  sessionAccentCss?: string,
): Color {
  if (selected && sessionAccentCss) {
    return Color.fromCssColorString(sessionAccentCss);
  }
  if (health === "stale") {
    return Color.fromCssColorString("rgba(251, 191, 36, 0.9)");
  }
  if (health === "warn") {
    return Color.fromCssColorString("rgba(248, 113, 113, 0.9)");
  }
  return Color.WHITE;
}

export function syncEntityMarkers(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[],
  options: {
    selectedEntityId: string | null;
    showLabels: boolean;
    syncHealth?: string;
    telemetryHealth?: string;
    perEntityDriftM?: Record<string, number>;
    commandGhost?: { entityId: string; pose: { x: number; y: number; z: number } } | null;
    dragOverride?: { entityId: string; pose: { x: number; y: number; z: number } } | null;
    sessionAccentCss?: string;
    applyTerrainDisplay?: boolean;
  },
): void {
  if (!isViewerUsable(viewer)) return;
  const keep = new Set<string>();
  const distScale = distanceScaleFromHeight(cameraHeightM(viewer));

  for (const ent of entities) {
    if (!ent.entity_id) continue;
    const id = toCesiumEntityId(ent.entity_id);
    const ringId = `${id}-selection-ring`;
    keep.add(id);
    let { x, y, z } = poseFromRecord(ent.pose);
    if (
      options.dragOverride &&
      options.dragOverride.entityId === ent.entity_id
    ) {
      x = options.dragOverride.pose.x;
      y = options.dragOverride.pose.y;
      z = options.dragOverride.pose.z;
    }
    const displayZ = options.applyTerrainDisplay
      ? applyTerrainDisplayOffset(x, y, z)
      : z;
    const position = worldToCartesian(x, y, displayZ);
    const groundTickId = `${id}${GROUND_TICK_SUFFIX}`;
    const selected = ent.entity_id === options.selectedEntityId;
    const drift = options.perEntityDriftM?.[ent.entity_id];
    const healthStyle = markerStyleForHealth(
      options.syncHealth,
      options.telemetryHealth,
      drift,
    );
    const color = colorForType(
      ent.entity_type,
      selected,
      healthStyle,
      options.sessionAccentCss,
    );
    const pixelSize = markerPixelSize(selected, distScale);
    const outline = outlineColorForMarker(
      selected,
      healthStyle,
      options.sessionAccentCss,
    );

    const existing = viewer.entities.getById(id);
    if (existing) viewer.entities.remove(existing);
    const existingRing = viewer.entities.getById(ringId);
    if (existingRing) viewer.entities.remove(existingRing);
    const existingTick = viewer.entities.getById(groundTickId);
    if (existingTick) viewer.entities.remove(existingTick);

    if (options.applyTerrainDisplay) {
      keep.add(groundTickId);
      const terrainZ = sampleTerrainHeight(x, y);
      viewer.entities.add(
        new Entity({
          id: groundTickId,
          polyline: {
            positions: [
              worldToCartesian(x, y, terrainZ),
              worldToCartesian(x, y, displayZ),
            ],
            width: 1,
            material: Color.fromCssColorString("rgba(148, 163, 184, 0.65)"),
          },
        }),
      );
    }

    if (selected) {
      keep.add(ringId);
      viewer.entities.add(
        new Entity({
          id: ringId,
          position,
          point: {
            pixelSize: SELECTION_RING_PIXEL_SIZE,
            color: Color.fromCssColorString(SELECTION_RING_COLOR),
            outlineWidth: 0,
          },
        }),
      );
    }

    viewer.entities.add(
      new Entity({
        id,
        name: ent.entity_id,
        position,
        point: {
          pixelSize,
          color,
          outlineColor: outline,
          outlineWidth: selected ? 3 : 2,
        },
        label: options.showLabels
          ? {
              text: options.applyTerrainDisplay
                ? labelTextWithAgl(
                    ent.entity_type,
                    ent.entity_id,
                    displayAglM(x, y, z),
                  )
                : labelText(ent.entity_type, ent.entity_id),
              font: LABEL_FONT,
              fillColor: Color.WHITE,
              outlineColor: Color.BLACK,
              outlineWidth: 3,
              style: LabelStyle.FILL_AND_OUTLINE,
              verticalOrigin: VerticalOrigin.BOTTOM,
              pixelOffset: new Cartesian2(0, LABEL_OFFSET_Y),
              showBackground: true,
              backgroundColor: Color.fromCssColorString(LABEL_BACKGROUND),
            }
          : undefined,
      }),
    );
  }

  if (options.commandGhost) {
    const ghostId = `${toCesiumEntityId(options.commandGhost.entityId)}${GHOST_ENTITY_SUFFIX}`;
    keep.add(ghostId);
    const g = options.commandGhost.pose;
    const ghostPos = worldToCartesian(g.x, g.y, g.z);
    const existingGhost = viewer.entities.getById(ghostId);
    if (existingGhost) viewer.entities.remove(existingGhost);
    viewer.entities.add(
      new Entity({
        id: ghostId,
        name: `${options.commandGhost.entityId} (command)`,
        position: ghostPos,
        point: {
          pixelSize: GHOST_PIXEL_SIZE,
          color: Color.fromCssColorString("rgba(148, 163, 184, 0.7)"),
          outlineColor: Color.fromCssColorString("rgba(226, 232, 240, 0.9)"),
          outlineWidth: 2,
        },
        label: {
          text: "cmd",
          font: "10px monospace",
          fillColor: Color.fromCssColorString("rgba(226, 232, 240, 0.95)"),
          outlineColor: Color.BLACK,
          outlineWidth: 2,
          style: LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: VerticalOrigin.BOTTOM,
          pixelOffset: new Cartesian2(0, -12),
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(30, 41, 59, 0.85)"),
        },
      }),
    );
  }

  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    const eid = e.id ?? "";
    if (
      (eid.startsWith("rt-entity-") ||
        eid.includes("-selection-ring") ||
        eid.includes(GROUND_TICK_SUFFIX)) &&
      !keep.has(eid)
    ) {
      toRemove.push(e);
    }
  });
  for (const e of toRemove) {
    viewer.entities.remove(e);
  }
}

export function clearEntityMarkers(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    const eid = e.id ?? "";
    if (
      eid.startsWith("rt-entity-") ||
      eid.includes("-selection-ring") ||
      eid.includes(GROUND_TICK_SUFFIX)
    ) {
      toRemove.push(e);
    }
  });
  for (const e of toRemove) {
    viewer.entities.remove(e);
  }
}
