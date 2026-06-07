import type { EnuPoint } from "@/cesium/tacticalGeometry";
import type { RuntimeRadarDisc } from "./runtimeCoverageAnalysis";

export interface CorridorCoverageResult {
  totalLengthM: number;
  coveredLengthM: number;
  uncoveredLengthM: number;
  coveredPercent: number;
  uncoveredPercent: number;
}

const CORRIDOR_SAMPLE_SPACING_M = 25;

function horizontalDistanceM(a: EnuPoint, b: EnuPoint): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function pointCoveredByRadarDiscs(
  point: EnuPoint,
  radarDiscs: readonly RuntimeRadarDisc[],
): boolean {
  return radarDiscs.some(
    (disc) =>
      Math.hypot(point.x - disc.position.x, point.y - disc.position.y) <=
      disc.detectionRangeM,
  );
}

function interpolatePoint(a: EnuPoint, b: EnuPoint, t: number): EnuPoint {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
    z: a.z + (b.z - a.z) * t,
  };
}

export interface CorridorCoveragePolylines {
  coveredPolylines: EnuPoint[][];
  uncoveredPolylines: EnuPoint[][];
}

/** Merge consecutive corridor legs with the same coverage state. */
export function deriveCorridorCoveragePolylines(
  corridorPoints: readonly EnuPoint[],
  radarDiscs: readonly RuntimeRadarDisc[],
): CorridorCoveragePolylines {
  if (corridorPoints.length < 2) {
    return { coveredPolylines: [], uncoveredPolylines: [] };
  }

  const coveredPolylines: EnuPoint[][] = [];
  const uncoveredPolylines: EnuPoint[][] = [];
  let activeCovered: boolean | null = null;
  let activePolyline: EnuPoint[] = [];

  const flush = () => {
    if (activePolyline.length < 2 || activeCovered == null) {
      activePolyline = [];
      activeCovered = null;
      return;
    }
    if (activeCovered) coveredPolylines.push([...activePolyline]);
    else uncoveredPolylines.push([...activePolyline]);
    activePolyline = [];
    activeCovered = null;
  };

  const appendPoint = (point: EnuPoint, covered: boolean) => {
    if (activeCovered == null) {
      activeCovered = covered;
      activePolyline = [point];
      return;
    }
    if (activeCovered !== covered) {
      activePolyline.push(point);
      flush();
      activeCovered = covered;
      activePolyline = [point];
      return;
    }
    activePolyline.push(point);
  };

  for (let index = 0; index < corridorPoints.length - 1; index += 1) {
    const start = corridorPoints[index];
    const end = corridorPoints[index + 1];
    const segmentLengthM = horizontalDistanceM(start, end);
    if (segmentLengthM <= 0) continue;

    const sampleCount = Math.max(1, Math.ceil(segmentLengthM / CORRIDOR_SAMPLE_SPACING_M));
    appendPoint(start, pointCoveredByRadarDiscs(start, radarDiscs));

    for (let sampleIndex = 0; sampleIndex < sampleCount; sampleIndex += 1) {
      const t0 = sampleIndex / sampleCount;
      const t1 = (sampleIndex + 1) / sampleCount;
      const midpoint = interpolatePoint(start, end, (t0 + t1) / 2);
      const sliceEnd = interpolatePoint(start, end, t1);
      appendPoint(midpoint, pointCoveredByRadarDiscs(midpoint, radarDiscs));
      appendPoint(sliceEnd, pointCoveredByRadarDiscs(sliceEnd, radarDiscs));
    }
  }

  flush();
  return { coveredPolylines, uncoveredPolylines };
}

/** Length-weighted corridor coverage using midpoint sampling per segment. */
export function analyzeCorridorCoverage(
  corridorPoints: readonly EnuPoint[],
  radarDiscs: readonly RuntimeRadarDisc[],
): CorridorCoverageResult | null {
  if (corridorPoints.length < 2) return null;

  let totalLengthM = 0;
  let coveredLengthM = 0;

  for (let index = 0; index < corridorPoints.length - 1; index += 1) {
    const start = corridorPoints[index];
    const end = corridorPoints[index + 1];
    const segmentLengthM = horizontalDistanceM(start, end);
    if (segmentLengthM <= 0) continue;

    totalLengthM += segmentLengthM;
    const sampleCount = Math.max(1, Math.ceil(segmentLengthM / CORRIDOR_SAMPLE_SPACING_M));
    let segmentCoveredLengthM = 0;

    for (let sampleIndex = 0; sampleIndex < sampleCount; sampleIndex += 1) {
      const t0 = sampleIndex / sampleCount;
      const t1 = (sampleIndex + 1) / sampleCount;
      const midpoint = interpolatePoint(start, end, (t0 + t1) / 2);
      const sliceLengthM = segmentLengthM / sampleCount;
      if (pointCoveredByRadarDiscs(midpoint, radarDiscs)) {
        segmentCoveredLengthM += sliceLengthM;
      }
    }

    coveredLengthM += segmentCoveredLengthM;
  }

  if (totalLengthM <= 0) return null;

  const uncoveredLengthM = Math.max(0, totalLengthM - coveredLengthM);
  const coveredPercent = (coveredLengthM / totalLengthM) * 100;
  const uncoveredPercent = (uncoveredLengthM / totalLengthM) * 100;

  return {
    totalLengthM,
    coveredLengthM,
    uncoveredLengthM,
    coveredPercent,
    uncoveredPercent,
  };
}
