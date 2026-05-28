import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  PolylineDashMaterialProperty,
  Viewer,
} from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import type { TerrainLayerVisibility } from "./terrainLayers";
import { anyTerrainLayerEnabled } from "./terrainLayers";
import {
  entityTerrainRelation,
  nearestOcclusionTarget,
  terrainContextLine,
} from "./terrainCognition";
import type { VisualLayerVisibility } from "./visualLayerRegistry";
import {
  DEFAULT_VISIBILITY_WEDGE_AZIMUTH_DEG,
  LABEL_BACKGROUND,
  LABEL_FONT,
  VISIBILITY_STACKED_LOS_COLOR,
  VISIBILITY_WEDGE_COLOR,
} from "./visualStyle";
import { boundsDiagonalHalfM, wedgeRayEndpoints } from "./visibilityWedgeLayer";

const V4_PREFIX = "rt-v4-visibility-";
const CORRIDOR_LENGTH_SCALE = 0.42;
const OCCLUSION_BAND_SCALE = 0.34;

export type VisibilityOverlayV4Kind =
  | "visibility_corridor"
  | "occlusion_band"
  | "terrain_relation"
  | "compare_emphasis";

export interface VisibilityOverlayV4Hint {
  kind: VisibilityOverlayV4Kind;
  label: string;
  tone: "info" | "warn" | "muted";
  explanatory: true;
}

function selectedPose(selected: MirrorEntity): { x: number; y: number; z: number; yawDeg: number } {
  return {
    x: Number(selected.pose.x ?? 0),
    y: Number(selected.pose.y ?? 0),
    z: Number(selected.pose.z ?? 0),
    yawDeg: Number(selected.pose.yaw_deg ?? 0),
  };
}

function visibilityOverlayV4On(visibility: VisualLayerVisibility): boolean {
  return (
    visibility.showVisibilityCorridorV4 ||
    visibility.showOcclusionBandsV4 ||
    visibility.showTerrainRelationLabelsV4 ||
    visibility.showCompareEmphasisV4
  );
}

export function deriveVisibilityOverlayV4Hints({
  visibility,
  selected,
  entities,
  terrainLayers,
}: {
  visibility: VisualLayerVisibility;
  selected: MirrorEntity | null;
  entities: readonly MirrorEntity[];
  terrainLayers: TerrainLayerVisibility;
}): VisibilityOverlayV4Hint[] {
  const hints: VisibilityOverlayV4Hint[] = [];
  if (!visibilityOverlayV4On(visibility)) return hints;

  if (visibility.showVisibilityCorridorV4) {
    hints.push({
      kind: "visibility_corridor",
      label: selected
        ? "Visibility corridor uses selected heading as a heuristic emphasis cue"
        : "Visibility corridor waiting for selected entity",
      tone: selected ? "info" : "muted",
      explanatory: true,
    });
  }

  if (visibility.showOcclusionBandsV4) {
    const occ = selected ? nearestOcclusionTarget(selected, [...entities]) : null;
    hints.push({
      kind: "occlusion_band",
      label: occ
        ? `Occlusion band nearest peer: ${occ.status}`
        : "Occlusion bands require selected and peer entities",
      tone: occ?.status === "clear" ? "info" : occ ? "warn" : "muted",
      explanatory: true,
    });
  }

  if (visibility.showTerrainRelationLabelsV4) {
    const relation = selected
      ? entityTerrainRelation(
          Number(selected.pose.x ?? 0),
          Number(selected.pose.y ?? 0),
          Number(selected.pose.z ?? 0),
        )
      : null;
    hints.push({
      kind: "terrain_relation",
      label: relation
        ? `Terrain relation: ${terrainContextLine(relation)}`
        : "Terrain relation labels waiting for selected entity",
      tone: relation ? "info" : "muted",
      explanatory: true,
    });
  }

  if (visibility.showCompareEmphasisV4) {
    hints.push({
      kind: "compare_emphasis",
      label: "Compare emphasis dims secondary/background sessions visually only",
      tone: "muted",
      explanatory: true,
    });
  }

  if (hints.length > 0 && !anyTerrainLayerEnabled(terrainLayers)) {
    hints.push({
      kind: "terrain_relation",
      label: "Terrain visibility cues use fictional terrain only when terrain layers are enabled",
      tone: "muted",
      explanatory: true,
    });
  }

  return hints;
}

export function visibilityOverlayV4SummaryLine(
  hints: readonly VisibilityOverlayV4Hint[],
): string {
  if (hints.length === 0) {
    return "V4 visibility overlays off - corridor, occlusion bands, and terrain labels are available.";
  }
  const active = hints.map((h) => h.kind.replace(/_/g, " ")).join(", ");
  return `V4 visibility overlays: ${active} - heuristic and explanatory only.`;
}

function removeV4Entities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(V4_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function displayZ(x: number, y: number, z: number, terrainLayers: TerrainLayerVisibility): number {
  return terrainLayers.showTerrainMesh ? applyTerrainDisplayOffset(x, y, z) : z;
}

function addCorridor(viewer: Viewer, selected: MirrorEntity, terrainLayers: TerrainLayerVisibility): void {
  const { x, y, z, yawDeg } = selectedPose(selected);
  const dz = displayZ(x, y, z, terrainLayers);
  const lengthM = boundsDiagonalHalfM() * CORRIDOR_LENGTH_SCALE;
  const endpoints = wedgeRayEndpoints(
    x,
    y,
    dz,
    yawDeg,
    DEFAULT_VISIBILITY_WEDGE_AZIMUTH_DEG * 0.75,
    lengthM,
  );
  const origin: [number, number, number] = [x, y, dz];
  const material = new PolylineDashMaterialProperty({
    color: Color.fromCssColorString(VISIBILITY_WEDGE_COLOR).withAlpha(0.42),
    dashLength: 18,
    gapColor: Color.TRANSPARENT,
  });
  [endpoints.left, endpoints.right].forEach((end, i) => {
    viewer.entities.add(
      new Entity({
        id: `${V4_PREFIX}corridor-${i}`,
        polyline: {
          positions: [origin, end].map(([px, py, pz]) => worldToCartesian(px, py, pz)),
          width: 3,
          material,
        },
      }),
    );
  });
}

function addOcclusionBand(viewer: Viewer, selected: MirrorEntity, terrainLayers: TerrainLayerVisibility): void {
  const { x, y, z, yawDeg } = selectedPose(selected);
  const dz = displayZ(x, y, z, terrainLayers);
  const lengthM = boundsDiagonalHalfM() * OCCLUSION_BAND_SCALE;
  const yawRad = (yawDeg * Math.PI) / 180;
  const rightYaw = yawRad + Math.PI / 2;
  const leftYaw = yawRad - Math.PI / 2;
  const centerX = x + Math.cos(yawRad) * lengthM;
  const centerY = y + Math.sin(yawRad) * lengthM;
  const halfWidth = lengthM * 0.22;
  const left: [number, number, number] = [
    centerX + Math.cos(leftYaw) * halfWidth,
    centerY + Math.sin(leftYaw) * halfWidth,
    dz,
  ];
  const right: [number, number, number] = [
    centerX + Math.cos(rightYaw) * halfWidth,
    centerY + Math.sin(rightYaw) * halfWidth,
    dz,
  ];
  viewer.entities.add(
    new Entity({
      id: `${V4_PREFIX}occlusion-band`,
      polyline: {
        positions: [left, right].map(([px, py, pz]) => worldToCartesian(px, py, pz)),
        width: 4,
        material: new PolylineDashMaterialProperty({
          color: Color.fromCssColorString(VISIBILITY_STACKED_LOS_COLOR).withAlpha(0.5),
          dashLength: 10,
          gapColor: Color.TRANSPARENT,
        }),
      },
    }),
  );
}

function addTerrainLabel(viewer: Viewer, selected: MirrorEntity, terrainLayers: TerrainLayerVisibility): void {
  const { x, y, z } = selectedPose(selected);
  const dz = displayZ(x, y, z, terrainLayers) + 18;
  const relation = entityTerrainRelation(x, y, z);
  viewer.entities.add(
    new Entity({
      id: `${V4_PREFIX}terrain-label`,
      position: worldToCartesian(x, y, dz),
      label: {
        text: `terrain ${relation.terrain_m.toFixed(0)}m / AGL ${relation.display_agl_m.toFixed(0)}m`,
        font: LABEL_FONT,
        style: LabelStyle.FILL_AND_OUTLINE,
        fillColor: Color.WHITE,
        outlineColor: Color.BLACK,
        outlineWidth: 2,
        showBackground: true,
        backgroundColor: Color.fromCssColorString(LABEL_BACKGROUND),
        pixelOffset: new Cartesian2(0, -34),
      },
    }),
  );
}

export function syncVisibilityOverlayV4(
  viewer: Viewer | null | undefined,
  selected: MirrorEntity | null,
  entities: readonly MirrorEntity[],
  visibility: VisualLayerVisibility,
  terrainLayers: TerrainLayerVisibility,
): void {
  if (!isViewerUsable(viewer)) return;
  removeV4Entities(viewer);
  if (!selected) return;

  const hints = deriveVisibilityOverlayV4Hints({
    visibility,
    selected,
    entities,
    terrainLayers,
  });
  if (hints.length === 0) return;

  if (visibility.showVisibilityCorridorV4) addCorridor(viewer, selected, terrainLayers);
  if (visibility.showOcclusionBandsV4) addOcclusionBand(viewer, selected, terrainLayers);
  if (visibility.showTerrainRelationLabelsV4) addTerrainLabel(viewer, selected, terrainLayers);
}

export function clearVisibilityOverlayV4(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeV4Entities(viewer);
}
