import {
  Color,
  Entity,
  LabelStyle,
  PolylineDashMaterialProperty,
  VerticalOrigin,
  Viewer,
  Cartesian2,
} from "cesium";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { isViewerUsable } from "./cesiumEditing";
import { deriveTacticalCompareDeltaLabels } from "./tacticalCompareDelta";
import {
  deriveTacticalTrajectoryGeometry,
  type EnuPoint,
  resolveTacticalRoleIds,
} from "./tacticalTrajectoryLayer";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import { worldToCartesian } from "./coordinates";
import {
  TACTICAL_INTERCEPT_POINT_COLOR,
  TACTICAL_TIMING_FONT,
  TACTICAL_TIMING_LABEL_BG,
} from "./visualStyle";

const COMPARE_PREFIX = "rt-tactical-compare-";

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

function compareLabelOptions(alphaScale: number) {
  return {
    font: TACTICAL_TIMING_FONT,
    fillColor: Color.fromCssColorString("rgba(203, 213, 225, 0.92)").withAlpha(
      0.9 * alphaScale,
    ),
    outlineColor: Color.BLACK,
    outlineWidth: 1,
    style: LabelStyle.FILL_AND_OUTLINE,
    showBackground: true,
    backgroundColor: Color.fromCssColorString(TACTICAL_TIMING_LABEL_BG).withAlpha(
      0.82 * alphaScale,
    ),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  };
}

export interface TacticalCompareOverlaySyncOptions {
  enabled: boolean;
  currentState: TacticalStatePayload | null | undefined;
  compareState: TacticalStatePayload | null | undefined;
  entities: MirrorEntity[];
  applyTerrainDisplay: boolean;
  stale?: boolean;
}

export function syncTacticalCompareOverlay(
  viewer: Viewer | null | undefined,
  options: TacticalCompareOverlaySyncOptions,
): void {
  if (!isViewerUsable(viewer)) return;

  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(COMPARE_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);

  if (!options.enabled || !options.compareState) return;

  const geometry = deriveTacticalTrajectoryGeometry(
    options.compareState,
    options.entities,
  );
  if (!geometry || geometry.pathPoints.length < 2) return;

  const alphaScale = (options.stale ? 0.45 : 1) * 0.55;
  const deltas = deriveTacticalCompareDeltaLabels(
    options.currentState,
    options.compareState,
  );

  const positions = geometry.pathPoints.map((p) =>
    toCartesian(p, options.applyTerrainDisplay),
  );

  viewer.entities.add(
    new Entity({
      id: `${COMPARE_PREFIX}path`,
      polyline: {
        positions,
        width: 2,
        material: new PolylineDashMaterialProperty({
          color: Color.fromCssColorString("rgba(248, 113, 113, 0.42)").withAlpha(
            0.42 * alphaScale,
          ),
          dashLength: 6,
          gapColor: Color.TRANSPARENT,
        }),
      },
    }),
  );

  const solution =
    geometry.interceptPose ??
    (geometry.pathEndPose ? geometry.pathEndPose : null);
  if (solution) {
    viewer.entities.add(
      new Entity({
        id: `${COMPARE_PREFIX}solution-point`,
        position: toCartesian(solution, options.applyTerrainDisplay),
        point: {
          pixelSize: 8,
          color: Color.fromCssColorString(TACTICAL_INTERCEPT_POINT_COLOR).withAlpha(
            0.5 * alphaScale,
          ),
          outlineColor: Color.fromCssColorString("rgba(148, 163, 184, 0.75)"),
          outlineWidth: 1,
        },
      }),
    );
  }

  if (deltas.block) {
    const mid = geometry.pathPoints[Math.floor(geometry.pathPoints.length / 2)];
    viewer.entities.add(
      new Entity({
        id: `${COMPARE_PREFIX}delta`,
        position: toCartesian(mid, options.applyTerrainDisplay),
        label: {
          text: deltas.block,
          ...compareLabelOptions(alphaScale),
          verticalOrigin: VerticalOrigin.CENTER,
          pixelOffset: new Cartesian2(-58, 14),
        },
      }),
    );
  }

  const { targetId: currentTarget } = resolveTacticalRoleIds(options.currentState);
  const { targetId: compareTarget } = resolveTacticalRoleIds(options.compareState);
  if (
    deltas.targetLine &&
    compareTarget &&
    compareTarget !== currentTarget
  ) {
    const targetEnt = options.entities.find((e) => e.entity_id === compareTarget);
    const pose = targetEnt?.pose;
    if (pose) {
      const x = Number(pose.x);
      const y = Number(pose.y);
      const z = Number(pose.z);
      if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
        viewer.entities.add(
          new Entity({
            id: `${COMPARE_PREFIX}target-hint`,
            position: toCartesian({ x, y, z }, options.applyTerrainDisplay),
            label: {
              text: "compare target",
              font: "9px sans-serif",
              fillColor: Color.fromCssColorString("rgba(148, 163, 184, 0.88)"),
              outlineColor: Color.BLACK,
              outlineWidth: 1,
              style: LabelStyle.FILL_AND_OUTLINE,
              verticalOrigin: VerticalOrigin.BOTTOM,
              pixelOffset: new Cartesian2(0, -20),
              showBackground: true,
              backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.72)"),
            },
          }),
        );
      }
    }
  }
}

export function clearTacticalCompareOverlay(
  viewer: Viewer | null | undefined,
): void {
  syncTacticalCompareOverlay(viewer, {
    enabled: false,
    currentState: null,
    compareState: null,
    entities: [],
    applyTerrainDisplay: false,
  });
}
