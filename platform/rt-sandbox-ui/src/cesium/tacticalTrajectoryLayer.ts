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
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import {
  deriveTacticalTimingSeconds,
  formatTacticalTimingBlock,
} from "./tacticalTimingLabels";
import {
  TACTICAL_INTERCEPT_POINT_COLOR,
  TACTICAL_PATH_COLOR,
  TACTICAL_PATH_HEURISTIC_COLOR,
  TACTICAL_TIMING_FONT,
  TACTICAL_TIMING_LABEL_BG,
  TACTICAL_TIMING_LABEL_FILL,
} from "./visualStyle";

const TACTICAL_TRAJ_PREFIX = "rt-tactical-traj-";

function tacticalTimingLabelOptions(alphaScale: number) {
  return {
    font: TACTICAL_TIMING_FONT,
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
): void {
  viewer.entities.add(
    new Entity({
      id,
      position,
      label: {
        text,
        ...tacticalTimingLabelOptions(alphaScale),
        verticalOrigin,
        pixelOffset,
      },
    }),
  );
}

export type EnuPoint = { x: number; y: number; z: number };

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

function poseFromRecord(
  pose: Record<string, unknown> | undefined,
): EnuPoint | null {
  if (!pose) return null;
  const x = Number(pose.x);
  const y = Number(pose.y);
  const z = Number(pose.z);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return null;
  }
  return { x, y, z };
}

function entityPose(entity: MirrorEntity | undefined): EnuPoint | null {
  return poseFromRecord(entity?.pose);
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

function parsePredictedPathTelemetry(
  state: TacticalStatePayload | null | undefined,
): EnuPoint[] | null {
  if (!state) return null;
  const raw = (state as Record<string, unknown>).predicted_path_enu_m;
  if (!Array.isArray(raw) || raw.length < 2) return null;
  const points: EnuPoint[] = [];
  for (const item of raw) {
    if (!Array.isArray(item) || item.length < 3) return null;
    const x = Number(item[0]);
    const y = Number(item[1]);
    const z = Number(item[2]);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      return null;
    }
    points.push({ x, y, z });
  }
  return points.length >= 2 ? points : null;
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

  const interceptPose = poseFromRecord(
    state?.last_intercept_pose ?? undefined,
  );
  const targetPose = entityPose(target);
  const telemetryPath = parsePredictedPathTelemetry(state);

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
  const timing = deriveTacticalTimingSeconds(
    options.tacticalState,
    options.tacticalRecommendation,
  );
  const timingText = formatTacticalTimingBlock(timing);

  if (options.showPath && geometry.pathPoints.length >= 2) {
    const pathColor =
      geometry.pathMode === "telemetry"
        ? TACTICAL_PATH_COLOR
        : TACTICAL_PATH_HEURISTIC_COLOR;
    const positions = geometry.pathPoints.map((p) =>
      toCartesian(p, options.applyTerrainDisplay),
    );
    viewer.entities.add(
      new Entity({
        id: `${TACTICAL_TRAJ_PREFIX}path`,
        polyline: {
          positions,
          width: geometry.pathMode === "telemetry" ? 3 : 2,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString(pathColor).withAlpha(
              (geometry.pathMode === "telemetry" ? 0.85 : 0.65) * alphaScale,
            ),
            dashLength: geometry.pathMode === "telemetry" ? 12 : 8,
          }),
        },
      }),
    );

    if (geometry.pathMode !== "telemetry") {
      const mid = geometry.pathPoints[Math.floor(geometry.pathPoints.length / 2)];
      viewer.entities.add(
        new Entity({
          id: `${TACTICAL_TRAJ_PREFIX}path-hint`,
          position: toCartesian(mid, options.applyTerrainDisplay),
          label: {
            text:
              geometry.pathMode === "heuristic_intercept"
                ? "heuristic path · display only"
                : "provisional leg · display only",
            font: TACTICAL_TIMING_FONT,
            fillColor: Color.fromCssColorString("rgba(251, 191, 36, 0.9)"),
            outlineColor: Color.BLACK,
            outlineWidth: 1,
            style: LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: VerticalOrigin.BOTTOM,
            pixelOffset: new Cartesian2(-52, -6),
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
          font: TACTICAL_TIMING_FONT,
          fillColor: Color.fromCssColorString("rgba(254, 226, 226, 0.95)"),
          outlineColor: Color.BLACK,
          outlineWidth: 1,
          style: LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: VerticalOrigin.TOP,
          pixelOffset: new Cartesian2(0, 10),
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.78)"),
        },
      }),
    );
  }

  if (options.showTimingLabels && timingText) {
    const anchor = geometry.interceptPose ?? geometry.pathEndPose;
    addTacticalTimingLabel(
      viewer,
      `${TACTICAL_TRAJ_PREFIX}timing`,
      toCartesian(anchor, options.applyTerrainDisplay),
      timingText,
      geometry.interceptPose ? VerticalOrigin.TOP : VerticalOrigin.BOTTOM,
      geometry.interceptPose ? new Cartesian2(0, 42) : new Cartesian2(0, -36),
      alphaScale,
    );
  }

}

export function clearTacticalTrajectoryLayer(
  viewer: Viewer | null | undefined,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalEntities(viewer);
}
