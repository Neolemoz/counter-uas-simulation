import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  PointGraphics,
  PolygonHierarchy,
  VerticalOrigin,
  Viewer,
} from "cesium";
import { isViewerUsable } from "@/cesium/cesiumEditing";
import { worldToCartesian } from "@/cesium/coordinates";
import type { PlanningCoverageCell, PlanningVertex } from "@/cesium/planningDrawing";
import { sampleTerrainHeight } from "@/cesium/rtFictionalTerrain";
import type { EnuPoint } from "@/cesium/tacticalGeometry";
import { ZONE_SURFACE_LIFT_M } from "@/cesium/terrainGrounding";
import {
  deriveRuntimeCoverageRenderModel,
  RUNTIME_COVERAGE_ENTITY_PREFIX,
  type RuntimeCoverageRenderModel,
  type RuntimeCoverageRenderParams,
} from "./runtimeCoverageRenderModel";

export { RUNTIME_COVERAGE_ENTITY_PREFIX } from "./runtimeCoverageRenderModel";

export const RUNTIME_COVERAGE_COLORS = {
  coveredFill: "rgba(34, 197, 94, 0.2)",
  uncoveredFill: "rgba(239, 68, 68, 0.16)",
  blindSpotPoint: "rgba(248, 113, 113, 0.88)",
  blindSpotOutline: "rgba(127, 29, 29, 0.98)",
  sectorFill: "rgba(245, 158, 11, 0.28)",
  sectorOutline: "rgba(251, 191, 36, 0.74)",
  corridorCovered: "rgba(34, 197, 94, 0.72)",
  corridorUncovered: "rgba(239, 68, 68, 0.78)",
} as const;

function vertexToCartesian(vertex: PlanningVertex): ReturnType<typeof worldToCartesian> {
  const z = sampleTerrainHeight(vertex.x, vertex.y) + ZONE_SURFACE_LIFT_M;
  return worldToCartesian(vertex.x, vertex.y, z);
}

function enuToCartesian(point: EnuPoint): ReturnType<typeof worldToCartesian> {
  const z = sampleTerrainHeight(point.x, point.y) + ZONE_SURFACE_LIFT_M + (point.z ?? 0) * 0.02;
  return worldToCartesian(point.x, point.y, z);
}

function cellToPolygon(cell: PlanningCoverageCell): PlanningVertex[] {
  const half = cell.sizeM / 2;
  return [
    { x: cell.center.x - half, y: cell.center.y - half },
    { x: cell.center.x + half, y: cell.center.y - half },
    { x: cell.center.x + half, y: cell.center.y + half },
    { x: cell.center.x - half, y: cell.center.y + half },
  ];
}

function removeRuntimeCoverageEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((entity) => {
    if ((entity.id ?? "").startsWith(RUNTIME_COVERAGE_ENTITY_PREFIX)) {
      toRemove.push(entity);
    }
  });
  for (const entity of toRemove) viewer.entities.remove(entity);
}

function addCoverageCells(
  viewer: Viewer,
  cells: PlanningCoverageCell[],
  suffix: string,
  material: Color,
): void {
  cells.forEach((cell, index) => {
    viewer.entities.add(
      new Entity({
        id: `${RUNTIME_COVERAGE_ENTITY_PREFIX}${suffix}-${index}`,
        polygon: {
          hierarchy: new PolygonHierarchy(cellToPolygon(cell).map(vertexToCartesian)),
          perPositionHeight: true,
          material,
          outline: false,
        },
      }),
    );
  });
}

function addBlindSpotHints(viewer: Viewer, hints: PlanningCoverageCell[]): void {
  hints.forEach((cell, index) => {
    viewer.entities.add(
      new Entity({
        id: `${RUNTIME_COVERAGE_ENTITY_PREFIX}blind-spot-${index}`,
        position: vertexToCartesian(cell.center),
        point: new PointGraphics({
          pixelSize: 12,
          color: Color.fromCssColorString(RUNTIME_COVERAGE_COLORS.blindSpotPoint),
          outlineColor: Color.fromCssColorString(RUNTIME_COVERAGE_COLORS.blindSpotOutline),
          outlineWidth: 2,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        }),
        label: {
          text: "runtime blind spot hint",
          font: "bold 10px sans-serif",
          fillColor: Color.WHITE,
          outlineColor: Color.BLACK,
          outlineWidth: 2,
          style: LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new Cartesian2(0, -20),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      }),
    );
  });
}

function addSectorHints(
  viewer: Viewer,
  model: RuntimeCoverageRenderModel,
): void {
  model.majorUncoveredSectors.forEach((sector, index) => {
    const hintCell = sector.farthestHint;
    viewer.entities.add(
      new Entity({
        id: `${RUNTIME_COVERAGE_ENTITY_PREFIX}sector-${index}`,
        polygon: {
          hierarchy: new PolygonHierarchy(cellToPolygon(hintCell).map(vertexToCartesian)),
          perPositionHeight: true,
          material: Color.fromCssColorString(RUNTIME_COVERAGE_COLORS.sectorFill),
          outline: true,
          outlineColor: Color.fromCssColorString(RUNTIME_COVERAGE_COLORS.sectorOutline),
        },
      }),
    );
    viewer.entities.add(
      new Entity({
        id: `${RUNTIME_COVERAGE_ENTITY_PREFIX}sector-label-${index}`,
        position: vertexToCartesian(sector.centroid),
        point: new PointGraphics({
          pixelSize: 9,
          color: Color.fromCssColorString("rgba(245, 158, 11, 0.9)"),
          outlineColor: Color.fromCssColorString("rgba(120, 53, 15, 0.98)"),
          outlineWidth: 2,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        }),
        label: {
          text: `${sector.sector} uncovered sector`,
          font: "bold 10px sans-serif",
          fillColor: Color.WHITE,
          outlineColor: Color.BLACK,
          outlineWidth: 2,
          style: LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new Cartesian2(0, -20),
          verticalOrigin: VerticalOrigin.BOTTOM,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      }),
    );
  });
}

function addCorridorPolylines(
  viewer: Viewer,
  polylines: EnuPoint[][],
  suffix: "corridor-covered" | "corridor-uncovered",
  color: string,
): void {
  polylines.forEach((points, index) => {
    if (points.length < 2) return;
    viewer.entities.add(
      new Entity({
        id: `${RUNTIME_COVERAGE_ENTITY_PREFIX}${suffix}-${index}`,
        polyline: {
          positions: points.map(enuToCartesian),
          width: suffix === "corridor-covered" ? 4 : 5,
          material: Color.fromCssColorString(color),
        },
      }),
    );
  });
}

function renderRuntimeCoverageModel(
  viewer: Viewer,
  model: RuntimeCoverageRenderModel,
): void {
  addCoverageCells(
    viewer,
    model.coveredCells,
    "covered",
    Color.fromCssColorString(RUNTIME_COVERAGE_COLORS.coveredFill),
  );
  addCoverageCells(
    viewer,
    model.uncoveredCells,
    "uncovered",
    Color.fromCssColorString(RUNTIME_COVERAGE_COLORS.uncoveredFill),
  );
  addBlindSpotHints(viewer, model.blindSpotHints);
  addSectorHints(viewer, model);
  addCorridorPolylines(
    viewer,
    model.coveredCorridorPolylines,
    "corridor-covered",
    RUNTIME_COVERAGE_COLORS.corridorCovered,
  );
  addCorridorPolylines(
    viewer,
    model.uncoveredCorridorPolylines,
    "corridor-uncovered",
    RUNTIME_COVERAGE_COLORS.corridorUncovered,
  );
}

export interface RuntimeCoverageLayerOptions {
  enabled: boolean;
  params: RuntimeCoverageRenderParams;
}

export function syncRuntimeCoverageLayer(
  viewer: Viewer | null | undefined,
  options: RuntimeCoverageLayerOptions,
): void {
  if (!isViewerUsable(viewer)) return;
  removeRuntimeCoverageEntities(viewer);
  if (!options.enabled) return;

  const model = deriveRuntimeCoverageRenderModel(options.params);
  if (!model.ready) return;
  renderRuntimeCoverageModel(viewer, model);
}

export function clearRuntimeCoverageLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeRuntimeCoverageEntities(viewer);
}
