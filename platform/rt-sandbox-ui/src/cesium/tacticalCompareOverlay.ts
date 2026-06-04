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
import { cameraHeightM } from "./cameraHelpers";
import { isViewerUsable } from "./cesiumEditing";
import {
  deriveTacticalCompareDeltaLabels,
  type TacticalCompareDeltaLabels,
} from "./tacticalCompareDelta";
import {
  deriveTacticalTrajectoryGeometry,
  type EnuPoint,
  resolveTacticalRoleIds,
} from "./tacticalTrajectoryLayer";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import { worldToCartesian } from "./coordinates";
import type { TacticalCompareSource } from "@/workstation/tacticalCompareContext";
import { pathLengthM } from "./tacticalTimingLabels";
import {
  decimateEnuPathForRender,
  tacticalCompareHintLabelOffset,
  tacticalComparePathWidthPx,
  tacticalCompareSolutionPixelSize,
  tacticalDashLengthPx,
  tacticalLabelFontCss,
} from "./tacticalVisualScale";
import {
  TACTICAL_COMPARE_PATH_COLOR,
  TACTICAL_COMPARE_SOLUTION_COLOR,
  TACTICAL_TIMING_LABEL_BG,
} from "./visualStyle";

const COMPARE_PREFIX = "rt-tactical-compare-";

function removeCompareEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(COMPARE_PREFIX)) toRemove.push(e);
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

function compareLabelOptions(alphaScale: number, cameraHeight: number) {
  return {
    font: tacticalLabelFontCss(cameraHeight),
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

function compareDeltaAnchor(
  geometry: ReturnType<typeof deriveTacticalTrajectoryGeometry>,
): EnuPoint | null {
  if (!geometry) return null;
  if (geometry.interceptPose) return geometry.interceptPose;
  if (geometry.pathPoints.length > 0) {
    return geometry.pathPoints[Math.floor(geometry.pathPoints.length / 2)];
  }
  return null;
}

export interface TacticalCompareOverlaySyncOptions {
  enabled: boolean;
  currentState: TacticalStatePayload | null | undefined;
  compareState: TacticalStatePayload | null | undefined;
  entities: MirrorEntity[];
  applyTerrainDisplay: boolean;
  stale?: boolean;
  compareSource?: TacticalCompareSource;
}

export function hasTacticalCompareGeometry(
  compareState: TacticalStatePayload | null | undefined,
  entities: MirrorEntity[],
): boolean {
  const geometry = deriveTacticalTrajectoryGeometry(compareState, entities);
  if (!geometry) return false;
  if (geometry.pathPoints.length >= 2) return true;
  return Boolean(geometry.interceptPose ?? geometry.pathEndPose);
}

export function syncTacticalCompareOverlay(
  viewer: Viewer | null | undefined,
  options: TacticalCompareOverlaySyncOptions,
): void {
  if (!isViewerUsable(viewer)) return;
  removeCompareEntities(viewer);

  if (!options.enabled || !options.compareState) return;

  const geometry = deriveTacticalTrajectoryGeometry(
    options.compareState,
    options.entities,
  );
  const deltas: TacticalCompareDeltaLabels = deriveTacticalCompareDeltaLabels(
    options.currentState,
    options.compareState,
  );
  const alphaScale = (options.stale ? 0.45 : 1) * 0.55;
  const cameraHeight = cameraHeightM(viewer);
  const sourceTag =
    options.compareSource === "session"
      ? "background session"
      : options.compareSource === "embedded"
        ? "embedded compare"
        : options.compareSource === "previous"
          ? "prior snapshot"
          : "compare";

  if (geometry && geometry.pathPoints.length >= 2) {
    const renderPathPoints = decimateEnuPathForRender(geometry.pathPoints);
    const legLengthM = pathLengthM(renderPathPoints);
    const pathWidth = tacticalComparePathWidthPx(cameraHeight, renderPathPoints);
    const dashLength = tacticalDashLengthPx("telemetry", cameraHeight, legLengthM);
    const pathHintOffset = tacticalCompareHintLabelOffset(cameraHeight, "path");
    const positions = renderPathPoints.map((p) =>
      toCartesian(p, options.applyTerrainDisplay),
    );
    viewer.entities.add(
      new Entity({
        id: `${COMPARE_PREFIX}path`,
        polyline: {
          positions,
          width: pathWidth,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString(TACTICAL_COMPARE_PATH_COLOR).withAlpha(
              0.5 * alphaScale,
            ),
            dashLength: Math.max(4, Math.round(dashLength * 0.45)),
            gapColor: Color.TRANSPARENT,
          }),
        },
      }),
    );

    const pathMid = renderPathPoints[Math.floor(renderPathPoints.length / 2)];
    viewer.entities.add(
      new Entity({
        id: `${COMPARE_PREFIX}path-hint`,
        position: toCartesian(pathMid, options.applyTerrainDisplay),
        label: {
          text: `compare path · ${sourceTag} · display only`,
          font: tacticalLabelFontCss(cameraHeight),
          fillColor: Color.fromCssColorString("rgba(148, 163, 184, 0.9)").withAlpha(
            0.9 * alphaScale,
          ),
          outlineColor: Color.BLACK,
          outlineWidth: 1,
          style: LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: VerticalOrigin.BOTTOM,
          pixelOffset: new Cartesian2(pathHintOffset.x, pathHintOffset.y),
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.75)"),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      }),
    );
  }

  const solution =
    geometry?.interceptPose ??
    (geometry?.pathEndPose ? geometry.pathEndPose : null);
  if (solution) {
    const legLengthM = geometry ? pathLengthM(geometry.pathPoints) : 0;
    const solutionSize = tacticalCompareSolutionPixelSize(cameraHeight, legLengthM);
    const solutionOffset = tacticalCompareHintLabelOffset(cameraHeight, "solution");
    viewer.entities.add(
      new Entity({
        id: `${COMPARE_PREFIX}solution-point`,
        position: toCartesian(solution, options.applyTerrainDisplay),
        point: {
          pixelSize: solutionSize,
          color: Color.fromCssColorString(TACTICAL_COMPARE_SOLUTION_COLOR).withAlpha(
            0.55 * alphaScale,
          ),
          outlineColor: Color.fromCssColorString("rgba(100, 116, 139, 0.8)"),
          outlineWidth: Math.max(1, Math.round(solutionSize / 6)),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
        label: {
          text: "compare solution · display only",
          font: tacticalLabelFontCss(cameraHeight),
          fillColor: Color.fromCssColorString("rgba(203, 213, 225, 0.9)").withAlpha(
            0.9 * alphaScale,
          ),
          outlineColor: Color.BLACK,
          outlineWidth: 1,
          style: LabelStyle.FILL_AND_OUTLINE,
          verticalOrigin: VerticalOrigin.TOP,
          pixelOffset: new Cartesian2(solutionOffset.x, solutionOffset.y),
          showBackground: true,
          backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.72)"),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      }),
    );
  }

  const deltaAnchor = compareDeltaAnchor(geometry);
  if (deltas.block && deltaAnchor) {
    const deltaOffset = tacticalCompareHintLabelOffset(cameraHeight, "delta");
    viewer.entities.add(
      new Entity({
        id: `${COMPARE_PREFIX}delta`,
        position: toCartesian(deltaAnchor, options.applyTerrainDisplay),
        label: {
          text: `${deltas.block}\ncompare summary · display only`,
          ...compareLabelOptions(alphaScale, cameraHeight),
          verticalOrigin: VerticalOrigin.CENTER,
          pixelOffset: new Cartesian2(deltaOffset.x, deltaOffset.y),
        },
      }),
    );
  }

  const { targetId: currentTarget } = resolveTacticalRoleIds(options.currentState);
  const { targetId: compareTarget } = resolveTacticalRoleIds(options.compareState);
  if (deltas.targetLine && compareTarget && compareTarget !== currentTarget) {
    const targetEnt = options.entities.find((e) => e.entity_id === compareTarget);
    const pose = targetEnt?.pose;
    if (pose) {
      const x = Number(pose.x);
      const y = Number(pose.y);
      const z = Number(pose.z);
      if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
        const targetOffset = tacticalCompareHintLabelOffset(cameraHeight, "target");
        viewer.entities.add(
          new Entity({
            id: `${COMPARE_PREFIX}target-hint`,
            position: toCartesian({ x, y, z }, options.applyTerrainDisplay),
            label: {
              text: "compare target · display only",
              font: tacticalLabelFontCss(cameraHeight),
              fillColor: Color.fromCssColorString("rgba(148, 163, 184, 0.88)"),
              outlineColor: Color.BLACK,
              outlineWidth: 1,
              style: LabelStyle.FILL_AND_OUTLINE,
              verticalOrigin: VerticalOrigin.BOTTOM,
              pixelOffset: new Cartesian2(targetOffset.x, targetOffset.y),
              showBackground: true,
              backgroundColor: Color.fromCssColorString("rgba(15, 23, 42, 0.72)"),
              disableDepthTestDistance: Number.POSITIVE_INFINITY,
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
  if (!isViewerUsable(viewer)) return;
  removeCompareEntities(viewer);
}
