import type { EnuPoint } from "./tacticalGeometry";
import { pathLengthM } from "./tacticalTimingLabels";
import { WORLD_FIT_CAMERA_HEIGHT_M } from "@/world/bounds";
import {
  distanceScaleFromHeight,
  TIGHT_BOUNDS_CAMERA_HEIGHT_M,
} from "./visualStyle";

export const TACTICAL_CORRIDOR_BASE_HALF_WIDTH_M = 14;
export const TACTICAL_CORRIDOR_MIN_HALF_WIDTH_M = 12;
export const TACTICAL_CORRIDOR_MAX_HALF_WIDTH_M = 420;
export const MAX_TACTICAL_TRAJECTORY_RENDER_POINTS = 48;

/** Scale corridor ribbon half-width from camera height and corridor span (m). */
export function corridorHalfWidthM(
  cameraHeightM: number,
  corridorPoints: EnuPoint[],
): number {
  const legLengthM = pathLengthM(corridorPoints);
  const heightScale = distanceScaleFromHeight(cameraHeightM);
  const legScale =
    legLengthM > 250
      ? 1 + Math.min((legLengthM - 250) / 4500, 2.4)
      : 1;
  const cameraBoost =
    cameraHeightM > WORLD_FIT_CAMERA_HEIGHT_M * 0.6
      ? 1 + (cameraHeightM - WORLD_FIT_CAMERA_HEIGHT_M * 0.6) / 6000
      : cameraHeightM > 4000
        ? 1 + (cameraHeightM - 4000) / 12000
        : 1;
  const width =
    TACTICAL_CORRIDOR_BASE_HALF_WIDTH_M * heightScale * legScale * cameraBoost +
    legLengthM * 0.022 * heightScale;
  return Math.min(
    TACTICAL_CORRIDOR_MAX_HALF_WIDTH_M,
    Math.max(TACTICAL_CORRIDOR_MIN_HALF_WIDTH_M, width),
  );
}

export function corridorPolylineWidths(
  cameraHeightM: number,
  legLengthM: number,
): { edgeWidth: number; centerWidth: number } {
  const scale = distanceScaleFromHeight(cameraHeightM);
  const legBoost =
    legLengthM > 1500
      ? 1 + Math.min((legLengthM - 1500) / 6000, 0.75)
      : 1;
  return {
    edgeWidth: Math.min(12, Math.max(4, Math.round(7 * scale * legBoost))),
    centerWidth: Math.min(4, Math.max(2, Math.round(2 * scale))),
  };
}

export function tacticalTrajectoryWidthPx(
  pathMode: "telemetry" | "heuristic_intercept" | "heuristic_target",
  cameraHeightM: number,
  pathPoints: EnuPoint[],
): number {
  const legLengthM = pathLengthM(pathPoints);
  const base = pathMode === "telemetry" ? 3 : 2;
  const scale = distanceScaleFromHeight(cameraHeightM);
  const legBoost =
    legLengthM > 800
      ? 1 + Math.min((legLengthM - 800) / 5000, 1.2)
      : legLengthM < 300
        ? 0.92
        : 1;
  return Math.min(8, Math.max(2, Math.round(base * scale * legBoost)));
}

export function tacticalDashLengthPx(
  pathMode: "telemetry" | "heuristic_intercept" | "heuristic_target",
  cameraHeightM: number,
  legLengthM: number,
): number {
  const base = pathMode === "telemetry" ? 12 : 8;
  if (cameraHeightM > 8000 || legLengthM > 5000) return base * 2;
  if (cameraHeightM > 4000 || legLengthM > 2500) return Math.round(base * 1.5);
  return base;
}

export type TacticalTimingLabelPlacement = "intercept_top" | "path_bottom";

export function tacticalTimingLabelPixelOffset(
  cameraHeightM: number,
  placement: TacticalTimingLabelPlacement,
): { x: number; y: number } {
  const scale = distanceScaleFromHeight(cameraHeightM);
  if (placement === "intercept_top") {
    return {
      x: 0,
      y: Math.round(Math.min(56, Math.max(28, 42 * scale))),
    };
  }
  return {
    x: 0,
    y: Math.round(-Math.min(48, Math.max(24, 36 * scale))),
  };
}

export function tacticalRankingCuePixelOffset(
  cameraHeightM: number,
  rank: number,
): { x: number; y: number } {
  const scale = distanceScaleFromHeight(cameraHeightM);
  const stackGap = Math.round(5 * scale);
  const baseY = -34 - rank * stackGap;
  return { x: 0, y: Math.max(-72, baseY) };
}

export function tacticalLabelFontPx(
  cameraHeightM: number,
  emphasis = false,
): number {
  if (cameraHeightM > WORLD_FIT_CAMERA_HEIGHT_M * 0.75) {
    return emphasis ? 11 : 9;
  }
  if (cameraHeightM > 5000) return emphasis ? 12 : 10;
  if (cameraHeightM > TIGHT_BOUNDS_CAMERA_HEIGHT_M) return emphasis ? 11 : 10;
  if (cameraHeightM < 700) return emphasis ? 12 : 11;
  return emphasis ? 11 : 10;
}

export function tacticalLabelFontCss(
  cameraHeightM: number,
  emphasis = false,
): string {
  return `${tacticalLabelFontPx(cameraHeightM, emphasis)}px sans-serif`;
}

export function decimateEnuPathForRender(
  points: EnuPoint[],
  maxPoints: number = MAX_TACTICAL_TRAJECTORY_RENDER_POINTS,
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

export function tacticalComparePathWidthPx(
  cameraHeightM: number,
  pathPoints: EnuPoint[],
): number {
  const legLengthM = pathLengthM(pathPoints);
  const scale = distanceScaleFromHeight(cameraHeightM);
  const legBoost =
    legLengthM > 800
      ? 1 + Math.min((legLengthM - 800) / 5000, 0.9)
      : 1;
  return Math.min(6, Math.max(2, Math.round(2 * scale * legBoost)));
}

export function tacticalCompareSolutionPixelSize(
  cameraHeightM: number,
  legLengthM: number,
): number {
  const scale = distanceScaleFromHeight(cameraHeightM);
  const legBoost =
    legLengthM > 1500
      ? 1 + Math.min((legLengthM - 1500) / 6000, 0.5)
      : 1;
  return Math.min(14, Math.max(7, Math.round(8 * scale * legBoost)));
}

export function tacticalCompareHintLabelOffset(
  cameraHeightM: number,
  kind: "path" | "solution" | "delta" | "target",
): { x: number; y: number } {
  const scale = distanceScaleFromHeight(cameraHeightM);
  switch (kind) {
    case "path":
      return {
        x: Math.round(-64 * Math.min(1.2, scale)),
        y: Math.round(-Math.min(12, Math.max(6, 6 * scale))),
      };
    case "solution":
      return {
        x: 0,
        y: Math.round(Math.min(14, Math.max(8, 8 * scale))),
      };
    case "delta":
      return {
        x: Math.round(Math.min(72, 58 * scale)),
        y: Math.round(Math.min(24, 18 * scale)),
      };
    case "target":
      return {
        x: 0,
        y: Math.round(-Math.min(28, Math.max(16, 20 * scale))),
      };
    default:
      return { x: 0, y: 0 };
  }
}

export function tacticalSelectionHaloPixelSize(cameraHeightM: number): number {
  const scale = distanceScaleFromHeight(cameraHeightM);
  return Math.min(40, Math.max(26, Math.round(32 * scale)));
}

export function tacticalSelectionOutlineWidth(cameraHeightM: number): number {
  const scale = distanceScaleFromHeight(cameraHeightM);
  return Math.min(4, Math.max(2, Math.round(3 * scale)));
}

export function tacticalSelectionLabelOffset(
  cameraHeightM: number,
): { x: number; y: number } {
  const scale = distanceScaleFromHeight(cameraHeightM);
  return {
    x: 0,
    y: Math.round(-Math.min(36, Math.max(24, 30 * scale))),
  };
}
