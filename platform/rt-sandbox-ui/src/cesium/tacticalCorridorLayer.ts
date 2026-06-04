import {
  Color,
  Entity,
  LabelStyle,
  PolygonHierarchy,
  PolylineDashMaterialProperty,
  VerticalOrigin,
  Viewer,
  Cartesian2,
} from "cesium";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { cameraHeightM } from "./cameraHelpers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import { pathLengthM } from "./tacticalTimingLabels";
import {
  buildCorridorRibbonPolygon,
  deriveThreatCorridorGeometry,
  type ThreatCorridorGeometry,
} from "./tacticalThreatCorridor";
import {
  deriveTacticalTrajectoryGeometry,
  type EnuPoint,
} from "./tacticalTrajectoryLayer";
import {
  corridorHalfWidthM,
  corridorPolylineWidths,
  decimateEnuPathForRender,
  tacticalLabelFontCss,
  tacticalTimingLabelPixelOffset,
} from "./tacticalVisualScale";
import {
  TACTICAL_THREAT_CORRIDOR_CENTER,
  TACTICAL_THREAT_CORRIDOR_EDGE,
  TACTICAL_THREAT_CORRIDOR_FILL,
} from "./visualStyle";

const TACTICAL_CORRIDOR_PREFIX = "rt-tactical-corridor-";

/** Caps ribbon/polyline complexity for Cesium decor entity budget. */
export const MAX_THREAT_CORRIDOR_POLYLINE_POINTS = 48;

function removeTacticalCorridorEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(TACTICAL_CORRIDOR_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function displayZ(
  x: number,
  y: number,
  z: number,
  applyTerrainDisplay: boolean,
): number {
  return applyTerrainDisplay ? applyTerrainDisplayOffset(x, y, z) : z;
}

function toCartesian(point: EnuPoint, applyTerrainDisplay: boolean) {
  return worldToCartesian(
    point.x,
    point.y,
    displayZ(point.x, point.y, point.z, applyTerrainDisplay),
  );
}

export function clampCorridorPoints(
  points: EnuPoint[],
  maxPoints: number = MAX_THREAT_CORRIDOR_POLYLINE_POINTS,
): EnuPoint[] {
  return decimateEnuPathForRender(points, maxPoints);
}

export function resolveThreatCorridorForRender(
  tacticalState: TacticalStatePayload | null | undefined,
  entities: MirrorEntity[],
): ThreatCorridorGeometry | null {
  const geometry = deriveTacticalTrajectoryGeometry(tacticalState, entities);
  if (!geometry) return null;
  const threat = deriveThreatCorridorGeometry(tacticalState, geometry, entities);
  if (!threat || threat.corridorPoints.length < 2) return null;
  return {
    ...threat,
    corridorPoints: clampCorridorPoints(threat.corridorPoints),
  };
}

export interface TacticalCorridorSyncOptions {
  enabled: boolean;
  tacticalState: TacticalStatePayload | null | undefined;
  entities: MirrorEntity[];
  applyTerrainDisplay: boolean;
  stale?: boolean;
}

export function syncTacticalCorridorLayer(
  viewer: Viewer | null | undefined,
  options: TacticalCorridorSyncOptions,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalCorridorEntities(viewer);
  if (!options.enabled) return;

  const threat = resolveThreatCorridorForRender(
    options.tacticalState,
    options.entities,
  );
  if (!threat) return;

  const alphaScale = options.stale ? 0.45 : 1;
  const cameraHeight = cameraHeightM(viewer);
  const legLengthM = pathLengthM(threat.corridorPoints);
  const halfWidthM = corridorHalfWidthM(cameraHeight, threat.corridorPoints);
  const { edgeWidth, centerWidth } = corridorPolylineWidths(
    cameraHeight,
    legLengthM,
  );
  const hintOffset = tacticalTimingLabelPixelOffset(cameraHeight, "path_bottom");
  const corridorPositions = threat.corridorPoints.map((p) =>
    toCartesian(p, options.applyTerrainDisplay),
  );
  const ribbon = buildCorridorRibbonPolygon(
    threat.corridorPoints,
    halfWidthM,
  );
  if (ribbon.length >= 3) {
    viewer.entities.add(
      new Entity({
        id: `${TACTICAL_CORRIDOR_PREFIX}fill`,
        polygon: {
          hierarchy: new PolygonHierarchy(
            ribbon.map((p) => toCartesian(p, options.applyTerrainDisplay)),
          ),
          material: Color.fromCssColorString(TACTICAL_THREAT_CORRIDOR_FILL).withAlpha(
            0.28 * alphaScale,
          ),
          outline: false,
          perPositionHeight: true,
        },
      }),
    );
  }
  viewer.entities.add(
    new Entity({
      id: `${TACTICAL_CORRIDOR_PREFIX}edge`,
      polyline: {
        positions: corridorPositions,
        width: edgeWidth,
        material: new PolylineDashMaterialProperty({
          color: Color.fromCssColorString(TACTICAL_THREAT_CORRIDOR_EDGE).withAlpha(
            0.42 * alphaScale,
          ),
          dashLength: 10,
        }),
      },
    }),
  );
  viewer.entities.add(
    new Entity({
      id: `${TACTICAL_CORRIDOR_PREFIX}center`,
      polyline: {
        positions: corridorPositions,
        width: centerWidth,
        material: Color.fromCssColorString(TACTICAL_THREAT_CORRIDOR_CENTER).withAlpha(
          0.34 * alphaScale,
        ),
      },
    }),
  );

  const mid = threat.corridorPoints[Math.floor(threat.corridorPoints.length / 2)];
  viewer.entities.add(
    new Entity({
      id: `${TACTICAL_CORRIDOR_PREFIX}hint`,
      position: toCartesian(mid, options.applyTerrainDisplay),
      label: {
        text:
          threat.mode === "telemetry_path"
            ? "threat corridor · telemetry path · display only"
            : "threat corridor · heuristic · display only",
        font: tacticalLabelFontCss(cameraHeight),
        fillColor: Color.fromCssColorString("rgba(254, 215, 170, 0.92)").withAlpha(
          0.92 * alphaScale,
        ),
        outlineColor: Color.BLACK,
        outlineWidth: 1,
        style: LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: VerticalOrigin.BOTTOM,
        pixelOffset: new Cartesian2(hintOffset.x, hintOffset.y),
        showBackground: true,
        backgroundColor: Color.fromCssColorString("rgba(69, 10, 10, 0.78)").withAlpha(
          0.78 * alphaScale,
        ),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
  );
}

export function clearTacticalCorridorLayer(
  viewer: Viewer | null | undefined,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalCorridorEntities(viewer);
}
