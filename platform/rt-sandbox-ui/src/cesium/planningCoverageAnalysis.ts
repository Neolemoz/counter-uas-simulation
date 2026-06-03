import type {
  PlanningCoverageCell,
  PlanningCoverageEstimate,
  PlanningPolygonState,
  PlanningRadarSite,
  PlanningRadarState,
  PlanningRadarPreset,
  PlanningVertex,
} from "./planningDrawing";

export interface PlanningCoverageAnalyzedCell extends PlanningCoverageCell {
  coverageCount: number;
  coveringRadarIds: string[];
}

export type PlanningUncoveredSector = "NE" | "NW" | "SE" | "SW";

export interface PlanningUncoveredSectorSummary {
  sector: PlanningUncoveredSector;
  cellCount: number;
  areaM2: number;
  centroid: PlanningVertex;
  farthestHint: PlanningCoverageCell;
}

export interface PlanningBlindSpotV2 {
  uncoveredAreaM2: number;
  uncoveredPercent: number;
  majorUncoveredSectors: PlanningUncoveredSectorSummary[];
  farthestUncoveredClusterHints: PlanningCoverageCell[];
  summary: string;
}

export interface PlanningRadarRecommendation {
  recommendedPresetId: string;
  recommendedPresetLabel: string;
  approximatePlacement: PlanningVertex;
  reason: string;
}

export interface PlanningCoverageAnalysis {
  estimate: PlanningCoverageEstimate;
  sampledCells: PlanningCoverageAnalyzedCell[];
  coveredCells: PlanningCoverageAnalyzedCell[];
  uncoveredCells: PlanningCoverageAnalyzedCell[];
  overlapCells: PlanningCoverageAnalyzedCell[];
  overlapCellCount: number;
  overlapAreaM2: number;
  overlapPercent: number;
  redundancyPercent: number;
  blindSpotV2: PlanningBlindSpotV2;
  radarRecommendation: PlanningRadarRecommendation | null;
}

const COVERAGE_SAMPLE_STEPS = 28;
const MAX_RENDERED_COVERAGE_CELLS = 320;
const MAX_BLIND_SPOT_HINTS = 6;
const SECTOR_ORDER: PlanningUncoveredSector[] = ["NE", "NW", "SE", "SW"];

export interface PlanningCoverageAnalysisOptions {
  radarPresets?: readonly PlanningRadarPreset[];
}

function emptyBlindSpotV2(): PlanningBlindSpotV2 {
  return {
    uncoveredAreaM2: 0,
    uncoveredPercent: 0,
    majorUncoveredSectors: [],
    farthestUncoveredClusterHints: [],
    summary: "No uncovered planning cells.",
  };
}

function sectorForPoint(
  point: PlanningVertex,
  center: PlanningVertex,
): PlanningUncoveredSector {
  if (point.x >= center.x && point.y >= center.y) return "NE";
  if (point.x < center.x && point.y >= center.y) return "NW";
  if (point.x >= center.x && point.y < center.y) return "SE";
  return "SW";
}

function distanceFromRadarsOrCenter(
  point: PlanningVertex,
  radars: PlanningRadarSite[],
  center: PlanningVertex,
): number {
  if (radars.length === 0) return distanceM(point, center);
  return Math.min(...radars.map((site) => distanceM(point, site.position)));
}

function sectorLabel(sector: PlanningUncoveredSector): string {
  return `${sector} uncovered sector`;
}

function deriveBlindSpotV2(
  uncoveredCells: PlanningCoverageAnalyzedCell[],
  radars: PlanningRadarSite[],
  boundsCenter: PlanningVertex,
  totalPolygonAreaM2: number,
): PlanningBlindSpotV2 {
  if (uncoveredCells.length === 0 || totalPolygonAreaM2 <= 0) return emptyBlindSpotV2();

  const sectorRows = SECTOR_ORDER.map((sector) => {
    const cells = uncoveredCells.filter((cell) => sectorForPoint(cell.center, boundsCenter) === sector);
    const areaM2 = cells.reduce((sum, cell) => sum + cell.areaM2, 0);
    const centroid = cells.length
      ? {
          x: cells.reduce((sum, cell) => sum + cell.center.x, 0) / cells.length,
          y: cells.reduce((sum, cell) => sum + cell.center.y, 0) / cells.length,
        }
      : boundsCenter;
    const farthest = [...cells].sort((a, b) => {
      const distanceDelta =
        distanceFromRadarsOrCenter(b.center, radars, boundsCenter) -
        distanceFromRadarsOrCenter(a.center, radars, boundsCenter);
      if (distanceDelta !== 0) return distanceDelta;
      if (a.center.y !== b.center.y) return b.center.y - a.center.y;
      return b.center.x - a.center.x;
    })[0];
    return { sector, cells, areaM2, centroid, farthest };
  })
    .filter((row) => row.cells.length > 0)
    .sort((a, b) => {
      if (b.areaM2 !== a.areaM2) return b.areaM2 - a.areaM2;
      return SECTOR_ORDER.indexOf(a.sector) - SECTOR_ORDER.indexOf(b.sector);
    });

  const majorUncoveredSectors = sectorRows.slice(0, 3).map((row) => ({
    sector: row.sector,
    cellCount: row.cells.length,
    areaM2: row.areaM2,
    centroid: row.centroid,
    farthestHint: toCoverageCell(row.farthest),
  }));
  const farthestUncoveredClusterHints = sectorRows
    .map((row) => toCoverageCell(row.farthest))
    .slice(0, MAX_BLIND_SPOT_HINTS);
  const uncoveredAreaM2 = uncoveredCells.reduce((sum, cell) => sum + cell.areaM2, 0);
  const uncoveredPercent = (uncoveredAreaM2 / totalPolygonAreaM2) * 100;
  const topSector = majorUncoveredSectors[0]?.sector;

  return {
    uncoveredAreaM2,
    uncoveredPercent,
    majorUncoveredSectors,
    farthestUncoveredClusterHints,
    summary: topSector
      ? `${uncoveredPercent.toFixed(1)}% uncovered, largest gap in ${sectorLabel(topSector)}.`
      : `${uncoveredPercent.toFixed(1)}% uncovered.`,
  };
}

function chooseRecommendationPreset(
  uncoveredPercent: number,
  presets: readonly PlanningRadarPreset[],
): PlanningRadarPreset | null {
  if (presets.length === 0) return null;
  const sorted = [...presets].sort((a, b) => a.detection_range_m - b.detection_range_m);
  if (uncoveredPercent >= 45) return sorted[sorted.length - 1];
  if (uncoveredPercent >= 15) return sorted[Math.min(1, sorted.length - 1)];
  return sorted[0];
}

function deriveRadarRecommendation(
  blindSpotV2: PlanningBlindSpotV2,
  radars: PlanningRadarState,
  presets: readonly PlanningRadarPreset[],
): PlanningRadarRecommendation | null {
  const largestSector = blindSpotV2.majorUncoveredSectors[0];
  if (!largestSector || blindSpotV2.uncoveredAreaM2 <= 0) return null;
  const preset = chooseRecommendationPreset(blindSpotV2.uncoveredPercent, presets);
  if (!preset) return null;
  const radarContext = radars.sites.length === 0 ? "No planning radar sites cover" : "Existing planning radars leave";
  return {
    recommendedPresetId: preset.id,
    recommendedPresetLabel: preset.label,
    approximatePlacement: largestSector.centroid,
    reason: `Add ${preset.label} near ${sectorLabel(largestSector.sector)}. ${radarContext} ${blindSpotV2.uncoveredPercent.toFixed(1)}% of sampled planning area uncovered.`,
  };
}

function polygonArea(vertices: PlanningVertex[]): number {
  if (vertices.length < 3) return 0;
  let sum = 0;
  for (let index = 0; index < vertices.length; index += 1) {
    const current = vertices[index];
    const next = vertices[(index + 1) % vertices.length];
    sum += current.x * next.y - next.x * current.y;
  }
  return Math.abs(sum) / 2;
}

function pointInPolygon(point: PlanningVertex, vertices: PlanningVertex[]): boolean {
  if (vertices.length < 3) return false;
  let inside = false;
  for (let index = 0, previous = vertices.length - 1; index < vertices.length; previous = index++) {
    const currentVertex = vertices[index];
    const previousVertex = vertices[previous];
    const crosses =
      currentVertex.y > point.y !== previousVertex.y > point.y &&
      point.x <
        ((previousVertex.x - currentVertex.x) * (point.y - currentVertex.y)) /
          (previousVertex.y - currentVertex.y) +
          currentVertex.x;
    if (crosses) inside = !inside;
  }
  return inside;
}

function distanceM(a: PlanningVertex, b: PlanningVertex): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function coveringRadarIdsForPoint(
  point: PlanningVertex,
  radars: PlanningRadarSite[],
): string[] {
  return radars
    .filter((site) => distanceM(point, site.position) <= site.detection_range_m)
    .map((site) => site.id);
}

function boundsForPolygon(vertices: PlanningVertex[]) {
  return vertices.reduce(
    (bounds, vertex) => ({
      minX: Math.min(bounds.minX, vertex.x),
      maxX: Math.max(bounds.maxX, vertex.x),
      minY: Math.min(bounds.minY, vertex.y),
      maxY: Math.max(bounds.maxY, vertex.y),
    }),
    {
      minX: Number.POSITIVE_INFINITY,
      maxX: Number.NEGATIVE_INFINITY,
      minY: Number.POSITIVE_INFINITY,
      maxY: Number.NEGATIVE_INFINITY,
    },
  );
}

function limitCoverageCells<T extends PlanningCoverageCell>(cells: T[]): T[] {
  if (cells.length <= MAX_RENDERED_COVERAGE_CELLS) return cells;
  const stride = Math.ceil(cells.length / MAX_RENDERED_COVERAGE_CELLS);
  return cells.filter((_, index) => index % stride === 0).slice(0, MAX_RENDERED_COVERAGE_CELLS);
}

function rankBlindSpotHints(
  uncoveredCells: PlanningCoverageAnalyzedCell[],
  radars: PlanningRadarSite[],
): PlanningCoverageAnalyzedCell[] {
  return [...uncoveredCells]
    .sort((a, b) => {
      const nearestA = radars.length
        ? Math.min(...radars.map((site) => distanceM(a.center, site.position)))
        : Number.POSITIVE_INFINITY;
      const nearestB = radars.length
        ? Math.min(...radars.map((site) => distanceM(b.center, site.position)))
        : Number.POSITIVE_INFINITY;
      return nearestB - nearestA;
    })
    .slice(0, MAX_BLIND_SPOT_HINTS);
}

function toCoverageCell(cell: PlanningCoverageAnalyzedCell): PlanningCoverageCell {
  return {
    center: cell.center,
    sizeM: cell.sizeM,
    areaM2: cell.areaM2,
  };
}

function emptyEstimate(radarCount: number, totalPolygonAreaM2 = 0): PlanningCoverageEstimate {
  return {
    radarCount,
    totalPolygonAreaM2,
    estimatedCoveredAreaM2: 0,
    estimatedUncoveredAreaM2: totalPolygonAreaM2,
    coveragePercent: 0,
    coveredCells: [],
    uncoveredCells: [],
    blindSpotHints: [],
  };
}

export function analyzePlanningCoverage(
  polygon: PlanningPolygonState,
  radars: PlanningRadarState,
  sampleSteps = COVERAGE_SAMPLE_STEPS,
  options: PlanningCoverageAnalysisOptions = {},
): PlanningCoverageAnalysis {
  const vertices = polygon.completedVertices ?? [];
  const totalPolygonAreaM2 = polygonArea(vertices);
  if (vertices.length < 3 || totalPolygonAreaM2 <= 0) {
    return {
      estimate: emptyEstimate(radars.sites.length, 0),
      sampledCells: [],
      coveredCells: [],
      uncoveredCells: [],
      overlapCells: [],
      overlapCellCount: 0,
      overlapAreaM2: 0,
      overlapPercent: 0,
      redundancyPercent: 0,
      blindSpotV2: emptyBlindSpotV2(),
      radarRecommendation: null,
    };
  }

  const bounds = boundsForPolygon(vertices);
  const stepCount = Math.max(4, sampleSteps);
  const cellWidth = (bounds.maxX - bounds.minX) / stepCount;
  const cellHeight = (bounds.maxY - bounds.minY) / stepCount;
  if (cellWidth <= 0 || cellHeight <= 0) {
    return {
      estimate: emptyEstimate(radars.sites.length, totalPolygonAreaM2),
      sampledCells: [],
      coveredCells: [],
      uncoveredCells: [],
      overlapCells: [],
      overlapCellCount: 0,
      overlapAreaM2: 0,
      overlapPercent: 0,
      redundancyPercent: 0,
      blindSpotV2: emptyBlindSpotV2(),
      radarRecommendation: null,
    };
  }

  const sampledCells: PlanningCoverageAnalyzedCell[] = [];
  for (let xIndex = 0; xIndex < stepCount; xIndex += 1) {
    for (let yIndex = 0; yIndex < stepCount; yIndex += 1) {
      const center = {
        x: bounds.minX + cellWidth * (xIndex + 0.5),
        y: bounds.minY + cellHeight * (yIndex + 0.5),
      };
      if (!pointInPolygon(center, vertices)) continue;
      const coveringRadarIds = coveringRadarIdsForPoint(center, radars.sites);
      sampledCells.push({
        center,
        sizeM: Math.min(cellWidth, cellHeight),
        areaM2: 0,
        coverageCount: coveringRadarIds.length,
        coveringRadarIds,
      });
    }
  }

  const cellAreaM2 = sampledCells.length > 0 ? totalPolygonAreaM2 / sampledCells.length : 0;
  const withArea = (cell: PlanningCoverageAnalyzedCell): PlanningCoverageAnalyzedCell => ({
    ...cell,
    areaM2: cellAreaM2,
  });
  const cellsWithArea = sampledCells.map(withArea);
  const coveredWithArea = cellsWithArea.filter((cell) => cell.coverageCount > 0);
  const uncoveredWithArea = cellsWithArea.filter((cell) => cell.coverageCount === 0);
  const overlapCells = cellsWithArea.filter((cell) => cell.coverageCount > 1);
  const overlapCellCount = overlapCells.length;
  const overlapAreaM2 = overlapCellCount * cellAreaM2;
  const estimatedCoveredAreaM2 = coveredWithArea.length * cellAreaM2;
  const estimatedUncoveredAreaM2 = Math.max(0, totalPolygonAreaM2 - estimatedCoveredAreaM2);
  const overlapPercent =
    totalPolygonAreaM2 > 0 ? (overlapAreaM2 / totalPolygonAreaM2) * 100 : 0;
  const redundancyPercent =
    estimatedCoveredAreaM2 > 0 ? (overlapAreaM2 / estimatedCoveredAreaM2) * 100 : 0;
  const boundsCenter = {
    x: (bounds.minX + bounds.maxX) / 2,
    y: (bounds.minY + bounds.maxY) / 2,
  };
  const blindSpotV2 = deriveBlindSpotV2(
    uncoveredWithArea,
    radars.sites,
    boundsCenter,
    totalPolygonAreaM2,
  );
  const radarRecommendation = deriveRadarRecommendation(
    blindSpotV2,
    radars,
    options.radarPresets ?? [],
  );

  return {
    estimate: {
      radarCount: radars.sites.length,
      totalPolygonAreaM2,
      estimatedCoveredAreaM2,
      estimatedUncoveredAreaM2,
      coveragePercent:
        totalPolygonAreaM2 > 0 ? (estimatedCoveredAreaM2 / totalPolygonAreaM2) * 100 : 0,
      coveredCells: limitCoverageCells(coveredWithArea).map(toCoverageCell),
      uncoveredCells: limitCoverageCells(uncoveredWithArea).map(toCoverageCell),
      blindSpotHints: rankBlindSpotHints(uncoveredWithArea, radars.sites).map(toCoverageCell),
    },
    sampledCells: cellsWithArea,
    coveredCells: coveredWithArea,
    uncoveredCells: uncoveredWithArea,
    overlapCells,
    overlapCellCount,
    overlapAreaM2,
    overlapPercent,
    redundancyPercent,
    blindSpotV2,
    radarRecommendation,
  };
}

export function estimatePlanningCoverageFromAnalysis(
  polygon: PlanningPolygonState,
  radars: PlanningRadarState,
  sampleSteps = COVERAGE_SAMPLE_STEPS,
): PlanningCoverageEstimate {
  return analyzePlanningCoverage(polygon, radars, sampleSteps).estimate;
}
