import {
  Color,
  Entity,
  LabelStyle,
  PolylineDashMaterialProperty,
  VerticalOrigin,
  Viewer,
  Cartesian2,
} from "cesium";
import type {
  TacticalRecommendationPayload,
  TacticalStatePayload,
} from "@/bridge/tacticalCommands";
import { cameraHeightM } from "./cameraHelpers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import {
  deriveDisplayInterceptPose,
  parseEnuPoseRecord,
  parsePredictedPathTelemetry,
  type EnuPoint,
} from "./tacticalGeometry";
import {
  decimateEnuPathForRender,
  tacticalDashLengthPx,
  tacticalLabelFontCss,
  tacticalTrajectoryWidthPx,
} from "./tacticalVisualScale";
import {
  deriveTacticalTimingSeconds,
  formatTacticalTimingBlock,
  pathLengthM,
  tacticalTimingLabelPixelOffset,
} from "./tacticalTimingLabels";
import {
  distanceScaleFromHeight,
  TACTICAL_INTERCEPT_POINT_COLOR,
  TACTICAL_PATH_COLOR,
  TACTICAL_PATH_HEURISTIC_COLOR,
  TACTICAL_TIMING_LABEL_BG,
  TACTICAL_TIMING_LABEL_FILL,
} from "./visualStyle";

export type { EnuPoint };

const TACTICAL_TRAJ_PREFIX = "rt-tactical-traj-";

function tacticalTimingLabelOptions(alphaScale: number, cameraHeight: number) {
  return {
    font: tacticalLabelFontCss(cameraHeight),
    fillColor: Color.fromCssColorString(TACTICAL_TIMING_LABEL_FILL).withAlpha(
      0.96 * alphaScale,
    ),
    outlineColor: Color.BLACK,
    outlineWidth: 1,
    style: LabelStyle.FILL_AND_OUTLINE,
    showBackground: true,
    backgroundColor: Color.fromCssColorString(TACTICAL_TIMING_LABEL_BG).withAlpha(
      0.9 * alphaScale,
    ),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  };
}

function addTacticalTimingLabel(
  viewer: Viewer,
  id: string,
  position: ReturnType<typeof toCartesian>,
  text: string,
  verticalOrigin: VerticalOrigin,
  pixelOffset: Cartesian2,
  alphaScale: number,
  cameraHeight: number,
): void {
  viewer.entities.add(
    new Entity({
      id,
      position,
      label: {
        text,
        ...tacticalTimingLabelOptions(alphaScale, cameraHeight),
        verticalOrigin,
        pixelOffset,
      },
    }),
  );
}

export type TacticalPathMode = "telemetry" | "heuristic_intercept" | "heuristic_target";

export interface TacticalTrajectoryGeometry {
  interceptorPose: EnuPoint;
  pathEndPose: EnuPoint;
  interceptPose: EnuPoint | null;
  pathPoints: EnuPoint[];
  pathMode: TacticalPathMode;
}

function removeTacticalEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(TACTICAL_TRAJ_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function entityPose(entity: MirrorEntity | undefined): EnuPoint | null {
  return parseEnuPoseRecord(entity?.pose);
}

export function resolveTacticalRoleIds(
  state: TacticalStatePayload | null | undefined,
): { interceptorId: string | null; targetId: string | null } {
  if (!state) return { interceptorId: null, targetId: null };
  return {
    interceptorId:
      state.assigned_interceptor_id ??
      state.selected_interceptor_id ??
      null,
    targetId:
      state.assigned_target_id ?? state.selected_target_id ?? null,
  };
}

export function deriveTacticalTrajectoryGeometry(
  state: TacticalStatePayload | null | undefined,
  entities: MirrorEntity[],
): TacticalTrajectoryGeometry | null {
  const { interceptorId, targetId } = resolveTacticalRoleIds(state);
  if (!interceptorId) return null;

  const interceptor = entities.find((e) => e.entity_id === interceptorId);
  const target = targetId
    ? entities.find((e) => e.entity_id === targetId)
    : undefined;
  const interceptorPose = entityPose(interceptor);
  if (!interceptorPose) return null;

  const targetPose = entityPose(target);
  const telemetryPath = parsePredictedPathTelemetry(state);
  const interceptPose = deriveDisplayInterceptPose(state, telemetryPath);

  if (telemetryPath) {
    return {
      interceptorPose,
      pathEndPose: telemetryPath[telemetryPath.length - 1],
      interceptPose,
      pathPoints: telemetryPath,
      pathMode: "telemetry",
    };
  }

  if (interceptPose) {
    return {
      interceptorPose,
      pathEndPose: interceptPose,
      interceptPose,
      pathPoints: [interceptorPose, interceptPose],
      pathMode: "heuristic_intercept",
    };
  }

  if (targetPose) {
    return {
      interceptorPose,
      pathEndPose: targetPose,
      interceptPose: null,
      pathPoints: [interceptorPose, targetPose],
      pathMode: "heuristic_target",
    };
  }

  return null;
}

function displayZ(
  x: number,
  y: number,
  z: number,
  applyTerrainDisplay: boolean,
): number {
  return applyTerrainDisplay ? applyTerrainDisplayOffset(x, y, z) : z;
}

function toCartesian(
  point: EnuPoint,
  applyTerrainDisplay: boolean,
) {
  return worldToCartesian(
    point.x,
    point.y,
    displayZ(point.x, point.y, point.z, applyTerrainDisplay),
  );
}

export interface TacticalTrajectorySyncOptions {
  showPath: boolean;
  showInterceptPoint: boolean;
  showTimingLabels?: boolean;
  tacticalState: TacticalStatePayload | null | undefined;
  tacticalRecommendation?: TacticalRecommendationPayload | null | undefined;
  entities: MirrorEntity[];
  applyTerrainDisplay: boolean;
  stale?: boolean;
}

export function syncTacticalTrajectoryLayer(
  viewer: Viewer | null | undefined,
  options: TacticalTrajectorySyncOptions,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalEntities(viewer);

  const geometry = deriveTacticalTrajectoryGeometry(
    options.tacticalState,
    options.entities,
  );
  if (!geometry) return;

  const alphaScale = options.stale ? 0.45 : 1;
  const cameraHeight = cameraHeightM(viewer);
  const renderPathPoints = decimateEnuPathForRender(geometry.pathPoints);
  const legLengthM = pathLengthM(renderPathPoints);
  const timing = deriveTacticalTimingSeconds(
    options.tacticalState,
    options.tacticalRecommendation,
  );
  const timingText = formatTacticalTimingBlock(timing);

  if (options.showPath && renderPathPoints.length >= 2) {
    const pathColor =
      geometry.pathMode === "telemetry"
        ? TACTICAL_PATH_COLOR
        : TACTICAL_PATH_HEURISTIC_COLOR;
    const pathWidth = tacticalTrajectoryWidthPx(
      geometry.pathMode,
      cameraHeight,
      renderPathPoints,
    );
    const dashLength = tacticalDashLengthPx(
      geometry.pathMode,
      cameraHeight,
      legLengthM,
    );
    const positions = renderPathPoints.map((p) =>
      toCartesian(p, options.applyTerrainDisplay),
    );
    viewer.entities.add(
      new Entity({
        id: `${TACTICAL_TRAJ_PREFIX}path`,
        polyline: {
          positions,
          width: pathWidth,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString(pathColor).withAlpha(
              (geometry.pathMode === "telemetry" ? 0.85 : 0.65) * alphaScale,
            ),
            dashLength,
          }),
        },
      }),
    );

    if (geometry.pathMode !== "telemetry") {
      const mid = renderPathPoints[Math.floor(renderPathPoints.length / 2)];
      const hintOffset = tacticalTimingLabelPixelOffset(cameraHeight, "path_bottom");
      viewer.entities.add(
        new Entity({
          id: `${TACTICAL_TRAJ_PREFIX}path-hint`,
          position: toCartesian(mid, options.applyTerrainDisplay),
          label: {
            text:
              geometry.pathMode === "heuristic_intercept"
                ? "heuristic path · display only"
                : "provisional leg · display only",
            font: tacticalLabelFontCss(cameraHeight),
            fillColor: Color.fromCssColorString("rgba(251, 191, 36, 0.9)"),
            outlineColor: Color.BLACK,
            outlineWidth: 1,
            style: LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: VerticalOrigin.BOTTOM,
            pixelOffset: new Cartesian2(-52, hintOffset.y + 30),
            showBackground: true,
            backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.72)"),
          },
        }),
      );
    }
  }

  if (options.showInterceptPoint && geometry.interceptPose) {
    const ip = geometry.interceptPose;
    viewer.entities.add(
      new Entity({
        id: `${TACTICAL_TRAJ_PREFIX}solution-point`,
        position: toCartesian(ip, options.applyTerrainDisplay),
        point: {
          pixelSize: options.tacticalState?.assigned_target_id ||
            options.tacticalState?.selected_target_id
            ? 13
            : 11,
          color: Color.fromCssColorString(TACTICAL_INTERCEPT_POINT_COLOR).withAlpha(
            0.92 * alphaScale,
          ),
          outlineColor: Color.fromCssColorString("rgba(254, 243, 199, 0.95)"),
          outlineWidth:
            options.tacticalState?.assigned_target_id ||
            options.tacticalState?.selected_target_id
              ? 3
              : 2,
        },
        label: {
          text: "solution point · display only",
          font: tacticalLabelFontCss(cameraHeight),
          fillColor: Color.fromCssColorString("rgba(254, 226, 226, 0.95)"),
          outlineColor: Color.BLACK,
          outlineWidth: 1,
          style: LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: VerticalOrigin.TOP,
          pixelOffset: new Cartesian2(
            0,
            Math.round(10 * distanceScaleFromHeight(cameraHeight)),
          ),
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.78)"),
        },
      }),
    );
  }

  if (options.showTimingLabels && timingText) {
    const anchor = geometry.interceptPose ?? geometry.pathEndPose;
    const placement = geometry.interceptPose ? "intercept_top" : "path_bottom";
    const timingOffset = tacticalTimingLabelPixelOffset(cameraHeight, placement);
    addTacticalTimingLabel(
      viewer,
      `${TACTICAL_TRAJ_PREFIX}timing`,
      toCartesian(anchor, options.applyTerrainDisplay),
      timingText,
      geometry.interceptPose ? VerticalOrigin.TOP : VerticalOrigin.BOTTOM,
      new Cartesian2(timingOffset.x, timingOffset.y),
      alphaScale,
      cameraHeight,
    );
  }

}

export function clearTacticalTrajectoryLayer(
  viewer: Viewer | null | undefined,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalEntities(viewer);
}
