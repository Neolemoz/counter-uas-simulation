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
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
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
  TACTICAL_THREAT_CORRIDOR_CENTER,
  TACTICAL_THREAT_CORRIDOR_EDGE,
  TACTICAL_THREAT_CORRIDOR_FILL,
  TACTICAL_TIMING_FONT,
} from "./visualStyle";

const TACTICAL_CORRIDOR_PREFIX = "rt-tactical-corridor-";

/** Caps ribbon/polyline complexity for Cesium decor entity budget. */
export const MAX_THREAT_CORRIDOR_POLYLINE_POINTS = 48;

const THREAT_CORRIDOR_HALF_WIDTH_M = 14;

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
  if (points.length <= maxPoints) return points;
  if (maxPoints < 2) return points.slice(0, maxPoints);
  const step = (points.length - 1) / (maxPoints - 1);
  const sampled: EnuPoint[] = [];
  for (let i = 0; i < maxPoints; i += 1) {
    const idx = Math.min(points.length - 1, Math.round(i * step));
    const point = points[idx];
    const prev = sampled[sampled.length - 1];
    if (
      prev &&
      prev.x === point.x &&
      prev.y === point.y &&
      prev.z === point.z
    ) {
      continue;
    }
    sampled.push(point);
  }
  return sampled.length >= 2 ? sampled : points.slice(0, 2);
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
  const corridorPositions = threat.corridorPoints.map((p) =>
    toCartesian(p, options.applyTerrainDisplay),
  );
  const ribbon = buildCorridorRibbonPolygon(
    threat.corridorPoints,
    THREAT_CORRIDOR_HALF_WIDTH_M,
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
        width: 7,
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
        width: 2,
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
        font: TACTICAL_TIMING_FONT,
        fillColor: Color.fromCssColorString("rgba(254, 215, 170, 0.92)").withAlpha(
          0.92 * alphaScale,
        ),
        outlineColor: Color.BLACK,
        outlineWidth: 1,
        style: LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: VerticalOrigin.BOTTOM,
        pixelOffset: new Cartesian2(0, -8),
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
