import { Cartesian3 } from "cesium";
import { worldToCartesian } from "./coordinates";
import { sampleTerrainHeight } from "./rtFictionalTerrain";
import { GRID_SCALE, GRID_Y_SCALE, worldToCell } from "@/world/gridCoords";
import type { DefenseZoneShape } from "./defenseZoneConfig";

export const DEFENSE_LABEL_MAX_ANCHOR_DISTANCE_M = 420;

/** Meters → SVG pixels (world editor grid). */
export function metersToSvg(sizeM: number, cellSize: number): number {
  return sizeM * GRID_SCALE * cellSize;
}

export function entitySvgCenter(
  x: number,
  y: number,
  cellSize: number,
): { cx: number; cy: number } {
  const cell = worldToCell(x, y);
  return {
    cx: cell.col * cellSize + cellSize / 2,
    cy: cell.row * cellSize + cellSize / 2,
  };
}

/** Closed boundary positions clamped to sampled terrain (pass #1). */
export function zoneBoundaryPositionsGrounded(
  shape: DefenseZoneShape,
  x: number,
  y: number,
  sizeM: number,
  liftM: number,
  segments = 96,
): Cartesian3[] {
  if (shape === "rectangle") {
    const corners: [number, number][] = [
      [x - sizeM, y - sizeM],
      [x + sizeM, y - sizeM],
      [x + sizeM, y + sizeM],
      [x - sizeM, y + sizeM],
    ];
    const positions = corners.map(([wx, wy]) =>
      worldToCartesian(wx, wy, sampleTerrainHeight(wx, wy) + liftM),
    );
    positions.push(positions[0]);
    return positions;
  }

  const ring: Cartesian3[] = [];
  for (let i = 0; i <= segments; i++) {
    const a = (i / segments) * Math.PI * 2;
    const wx = x + Math.cos(a) * sizeM;
    const wy = y + Math.sin(a) * sizeM;
    ring.push(worldToCartesian(wx, wy, sampleTerrainHeight(wx, wy) + liftM));
  }
  return ring;
}

export function labelAnchorWorld(
  shape: DefenseZoneShape,
  x: number,
  y: number,
  sizeM: number,
  azimuthDeg: number,
): { wx: number; wy: number } {
  const labelDistance = Math.min(
    sizeM * 1.1,
    DEFENSE_LABEL_MAX_ANCHOR_DISTANCE_M,
  );
  if (shape === "rectangle") {
    if (azimuthDeg >= 60 && azimuthDeg < 150) {
      return { wx: x, wy: y + labelDistance };
    }
    if (azimuthDeg >= 150 && azimuthDeg < 270) {
      return {
        wx: x - labelDistance,
        wy: y - Math.min(sizeM * 0.15, labelDistance * 0.25),
      };
    }
    return {
      wx: x + labelDistance,
      wy: y - Math.min(sizeM * 0.15, labelDistance * 0.25),
    };
  }
  const az = (azimuthDeg * Math.PI) / 180;
  return {
    wx: x + Math.cos(az) * labelDistance,
    wy: y + Math.sin(az) * labelDistance,
  };
}

export function labelSvgAnchor(
  shape: DefenseZoneShape,
  cx: number,
  cy: number,
  sizePx: number,
  azimuthDeg: number,
): { lx: number; ly: number } {
  const labelOffset = 1.12;
  if (shape === "rectangle") {
    if (azimuthDeg >= 60 && azimuthDeg < 150) {
      return { lx: cx, ly: cy - sizePx * labelOffset };
    }
    if (azimuthDeg >= 150 && azimuthDeg < 270) {
      return { lx: cx - sizePx * labelOffset, ly: cy + sizePx * 0.15 };
    }
    return { lx: cx + sizePx * labelOffset, ly: cy + sizePx * 0.15 };
  }
  const az = (azimuthDeg * Math.PI) / 180;
  return {
    lx: cx + Math.cos(az) * sizePx * labelOffset,
    ly: cy - Math.sin(az) * sizePx * labelOffset,
  };
}

/** Stagger defense labels outside rings — wider bearings reduce overlap. */
export function labelAzimuthDegForDefenseZone(
  suffix: "core" | "mid" | "warning",
): number {
  if (suffix === "core") return 86;
  if (suffix === "mid") return 208;
  return 328;
}

/** Screen-space nudge per ring label (Cesium pixel offset). */
export function defenseLabelPixelOffset(
  suffix: "core" | "mid" | "warning",
): { x: number; y: number } {
  if (suffix === "core") return { x: 0, y: -10 };
  if (suffix === "mid") return { x: -14, y: 5 };
  return { x: 14, y: 3 };
}

export function labelHeightLiftM(_suffix: "core" | "mid" | "warning"): number {
  return 0;
}

/** World meters per SVG pixel along each axis (for rectangle sizing). */
export function svgAxisScales(cellSize: number): { x: number; y: number } {
  return {
    x: 1 / (GRID_SCALE * cellSize),
    y: 1 / (GRID_Y_SCALE * cellSize),
  };
}
