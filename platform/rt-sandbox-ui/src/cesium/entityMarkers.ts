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
import {
  computeMarkerLabelLayouts,
  type MarkerLabelLayoutInput,
} from "./markerLabelLayout";
import {
  compactSelectedLabelSuffix,
  type EntityRuntimeTelemetry,
} from "@/telemetry/entityMirrorFields";
import {
  groundedSurfaceZ,
  markerDisplayZ,
  MARKER_SURFACE_LIFT_M,
} from "./terrainGrounding";
import { toCesiumEntityId } from "./entityId";
import { isViewerUsable } from "./cesiumEditing";
import {
  distanceScaleFromHeight,
  GHOST_PIXEL_SIZE,
  LABEL_BACKGROUND,
  labelFontCss,
  LABEL_OFFSET_Y,
  shortEntityId,
  markerPixelSize,
  SELECTION_RING_PIXEL_SIZE,
} from "./visualStyle";

const GHOST_ENTITY_SUFFIX = "-cmd-ghost";
const GROUND_TICK_SUFFIX = "-ground-tick";
const TACTICAL_TARGET_HALO_SUFFIX = "-tactical-target-halo";
const MUTED_MARKER_ALPHA_SCALE = 0.55;
const TACTICAL_TARGET_HALO_COLOR = "rgba(248, 113, 113, 0.42)";
const TACTICAL_TARGET_LABEL_FILL = "rgba(254, 243, 199, 0.98)";

export type MarkerEmphasis = "full" | "muted";

function applyAlphaScale(color: Color, scale: number): Color {
  const c = color.clone();
  c.alpha *= scale;
  return c;
}

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
    radar: "#22d3ee",
    interceptor: "#34d399",
    drone: "#f87171",
    waypoint_marker: "#fbbf24",
  };
  const hex = base[entityType] ?? "#94a3b8";
  if (selected && accentOutline) {
    return Color.fromCssColorString(hex);
  }
  return Color.fromCssColorString(selected ? hex : `${hex}99`);
}

function iconForType(entityType: string): string {
  switch (entityType) {
    case "radar":
      return "◉";
    case "interceptor":
      return "▲";
    case "drone":
      return "✈";
    case "waypoint_marker":
      return "◆";
    default:
      return "●";
  }
}

function shortTypeLabel(entityType: string): string {
  switch (entityType) {
    case "radar":
      return "rad";
    case "interceptor":
      return "int";
    case "drone":
      return "drn";
    case "waypoint_marker":
      return "wp";
    default:
      return entityType.slice(0, 3) || "ent";
  }
}

function compactMarkerLabelText(entityType: string, entityId: string): string {
  return `${iconForType(entityType)} ${shortEntityId(entityId)}`;
}

function fullMarkerLabelText(
  entityType: string,
  entityId: string,
  poseZ: number,
): string {
  return `${iconForType(entityType)} ${shortTypeLabel(entityType)} ${shortEntityId(entityId)} · z ${Math.round(poseZ)}m`;
}

function markerLabelText(
  entityType: string,
  entityId: string,
  poseZ: number,
  hovered: boolean,
  selected: boolean,
  glyphOnly: boolean,
  selectedRuntimeSuffix = "",
): string {
  if (glyphOnly) return iconForType(entityType);
  if (entityType === "radar" && selected) {
    return `${compactMarkerLabelText(entityType, entityId)}${selectedRuntimeSuffix}`;
  }
  if (hovered || selected) {
    return `${fullMarkerLabelText(entityType, entityId, poseZ)}${selected ? selectedRuntimeSuffix : ""}`;
  }
  return compactMarkerLabelText(entityType, entityId);
}

function labelStyleForMarker(
  selected: boolean,
  hovered: boolean,
  cameraHeight: number,
) {
  if (selected || hovered) {
    return {
      font: labelFontCss(cameraHeight, selected, hovered),
      outlineWidth: selected ? 4 : 3,
      showBackground: true,
    };
  }
  return {
    font: labelFontCss(cameraHeight, false, false),
    outlineWidth: 2,
    showBackground: false,
  };
}

function outlineColorForMarker(
  selected: boolean,
  hovered: boolean,
  health: "ok" | "stale" | "warn",
  sessionAccentCss?: string,
): Color {
  if (selected && sessionAccentCss) {
    return Color.fromCssColorString(sessionAccentCss);
  }
  if (hovered) {
    return Color.fromCssColorString("rgba(186, 230, 253, 0.98)");
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
    hoveredEntityId?: string | null;
    showLabels: boolean;
    syncHealth?: string;
    telemetryHealth?: string;
    perEntityDriftM?: Record<string, number>;
    commandGhost?: { entityId: string; pose: { x: number; y: number; z: number } } | null;
    dragOverride?: { entityId: string; pose: { x: number; y: number; z: number } } | null;
    sessionAccentCss?: string;
    applyTerrainDisplay?: boolean;
    markerEmphasis?: MarkerEmphasis;
    /** Assigned/selected tactical target — display-only emphasis. */
    tacticalTargetEntityId?: string | null;
    runtimeTelemetryByEntityId?: Map<string, EntityRuntimeTelemetry>;
  },
): void {
  const mutedUnselected = options.markerEmphasis === "muted";
  if (!isViewerUsable(viewer)) return;
  const keep = new Set<string>();
  const cameraHeight = cameraHeightM(viewer);
  const distScale = distanceScaleFromHeight(cameraHeight);
  const showLabels = options.showLabels;
  const hoveredEntityId = options.hoveredEntityId ?? null;

  const layoutInputs: MarkerLabelLayoutInput[] = [];
  for (const ent of entities) {
    if (!ent.entity_id) continue;
    let { x, y } = poseFromRecord(ent.pose);
    if (
      options.dragOverride &&
      options.dragOverride.entityId === ent.entity_id
    ) {
      x = options.dragOverride.pose.x;
      y = options.dragOverride.pose.y;
    }
    const tacticalTarget =
      options.tacticalTargetEntityId != null &&
      ent.entity_id === options.tacticalTargetEntityId;
    layoutInputs.push({
      entityId: ent.entity_id,
      entityType: ent.entity_type,
      x,
      y,
      selected: ent.entity_id === options.selectedEntityId,
      hovered: ent.entity_id === hoveredEntityId,
      tacticalTarget,
    });
  }
  const labelLayouts = computeMarkerLabelLayouts(layoutInputs, cameraHeight);

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
    const displayZ = markerDisplayZ(x, y, z, options.applyTerrainDisplay === true);
    const position = worldToCartesian(x, y, displayZ);
    const groundTickId = `${id}${GROUND_TICK_SUFFIX}`;
    const selected = ent.entity_id === options.selectedEntityId;
    const hovered = ent.entity_id === hoveredEntityId;
    const tacticalTarget =
      options.tacticalTargetEntityId != null &&
      ent.entity_id === options.tacticalTargetEntityId;
    const drift = options.perEntityDriftM?.[ent.entity_id];
    const healthStyle = markerStyleForHealth(
      options.syncHealth,
      options.telemetryHealth,
      drift,
    );
    let color = colorForType(
      ent.entity_type,
      selected,
      healthStyle,
      options.sessionAccentCss,
    );
    let outline = outlineColorForMarker(
      selected,
      hovered,
      healthStyle,
      options.sessionAccentCss,
    );
    if (mutedUnselected && !selected && !hovered && !tacticalTarget) {
      color = applyAlphaScale(color, MUTED_MARKER_ALPHA_SCALE);
      outline = applyAlphaScale(outline, MUTED_MARKER_ALPHA_SCALE);
    }
    if (tacticalTarget) {
      const base: Record<string, string> = {
        radar: "#22d3ee",
        interceptor: "#34d399",
        drone: "#f87171",
        waypoint_marker: "#fbbf24",
      };
      const hex = base[ent.entity_type] ?? "#94a3b8";
      color = Color.fromCssColorString(hex);
      outline = Color.fromCssColorString("rgba(254, 226, 226, 0.98)");
    }
    const pixelSize = markerPixelSize(
      selected || hovered || tacticalTarget,
      distScale,
    );

    const existing = viewer.entities.getById(id);
    if (existing) viewer.entities.remove(existing);
    const existingRing = viewer.entities.getById(ringId);
    if (existingRing) viewer.entities.remove(existingRing);
    const existingTick = viewer.entities.getById(groundTickId);
    if (existingTick) viewer.entities.remove(existingTick);

    if (options.applyTerrainDisplay) {
      keep.add(groundTickId);
      const surfaceZ = groundedSurfaceZ(x, y, MARKER_SURFACE_LIFT_M);
      viewer.entities.add(
        new Entity({
          id: groundTickId,
          polyline: {
            positions: [
              worldToCartesian(x, y, surfaceZ),
              worldToCartesian(x, y, displayZ),
            ],
            width: selected || hovered ? 1.5 : 1,
            material: Color.fromCssColorString(
              selected || hovered
                ? "rgba(148, 163, 184, 0.85)"
                : "rgba(148, 163, 184, 0.55)",
            ),
          },
        }),
      );
    }

    const tacticalHaloId = `${id}${TACTICAL_TARGET_HALO_SUFFIX}`;
    if (tacticalTarget) {
      keep.add(tacticalHaloId);
      const existingHalo = viewer.entities.getById(tacticalHaloId);
      if (existingHalo) viewer.entities.remove(existingHalo);
      viewer.entities.add(
        new Entity({
          id: tacticalHaloId,
          position,
          point: {
            pixelSize: SELECTION_RING_PIXEL_SIZE + 2,
            color: Color.fromCssColorString(TACTICAL_TARGET_HALO_COLOR),
            outlineWidth: 0,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
        }),
      );
    }

    if (selected || hovered) {
      keep.add(ringId);
      viewer.entities.add(
        new Entity({
          id: ringId,
          position,
          point: {
            pixelSize: hovered ? SELECTION_RING_PIXEL_SIZE - 2 : SELECTION_RING_PIXEL_SIZE,
            color: Color.fromCssColorString(
              selected
                ? "rgba(251, 191, 36, 0.45)"
                : "rgba(186, 230, 253, 0.36)",
            ),
            outlineWidth: 0,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
        }),
      );
    }

    const labelLayout = labelLayouts.get(ent.entity_id);
    const labelPixelOffset = new Cartesian2(
      labelLayout?.offsetX ?? 0,
      labelLayout?.offsetY ?? LABEL_OFFSET_Y,
    );
    const glyphOnly = labelLayout?.glyphOnly ?? false;
    const labelStyle = labelStyleForMarker(
      selected || tacticalTarget,
      hovered,
      cameraHeight,
    );
    const markerEmphasized = selected || hovered || tacticalTarget;
    const selectedRuntimeSuffix = selected
      ? compactSelectedLabelSuffix(
          options.runtimeTelemetryByEntityId?.get(ent.entity_id),
        )
      : "";

    viewer.entities.add(
      new Entity({
        id,
        name: ent.entity_id,
        position,
        point: {
          pixelSize,
          color,
          outlineColor: outline,
          outlineWidth: tacticalTarget ? 5 : selected ? 4 : hovered ? 3 : 2,
          disableDepthTestDistance: markerEmphasized
            ? Number.POSITIVE_INFINITY
            : 0,
        },
        label: {
          text: showLabels
            ? markerLabelText(
                ent.entity_type,
                ent.entity_id,
                displayZ,
                hovered,
                selected || tacticalTarget,
                glyphOnly,
                selectedRuntimeSuffix,
              )
            : "",
          font: labelStyle.font,
          fillColor: tacticalTarget
            ? Color.fromCssColorString(TACTICAL_TARGET_LABEL_FILL)
            : selected || hovered
              ? Color.WHITE
              : mutedUnselected
                ? applyAlphaScale(Color.WHITE, MUTED_MARKER_ALPHA_SCALE)
                : Color.WHITE,
          outlineColor: Color.BLACK,
          outlineWidth: labelStyle.outlineWidth,
          style: LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: VerticalOrigin.BOTTOM,
          pixelOffset: labelPixelOffset,
          showBackground: labelStyle.showBackground || tacticalTarget,
          backgroundColor:
            mutedUnselected && !markerEmphasized
              ? applyAlphaScale(
                  Color.fromCssColorString(LABEL_BACKGROUND),
                  MUTED_MARKER_ALPHA_SCALE,
                )
              : Color.fromCssColorString(LABEL_BACKGROUND),
          disableDepthTestDistance: markerEmphasized
            ? Number.POSITIVE_INFINITY
            : 0,
        },
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
        eid.includes(TACTICAL_TARGET_HALO_SUFFIX) ||
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
      eid.includes(TACTICAL_TARGET_HALO_SUFFIX) ||
      eid.includes(GROUND_TICK_SUFFIX)
    ) {
      toRemove.push(e);
    }
  });
  for (const e of toRemove) {
    viewer.entities.remove(e);
  }
}
