import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  PointGraphics,
  PolygonHierarchy,
  ScreenSpaceEventHandler,
  ScreenSpaceEventType,
  Viewer,
} from "cesium";
import { cartographicToWorld, worldToCartesian } from "./coordinates";
import { isViewerUsable } from "./cesiumEditing";
import { sampleTerrainHeight } from "./rtFictionalTerrain";
import { ZONE_SURFACE_LIFT_M } from "./terrainGrounding";

export type PlanningTool = "select" | "draw_defense_area" | "place_radar_site";

export interface PlanningVertex {
  x: number;
  y: number;
}

export interface PlanningPolygonState {
  draftVertices: PlanningVertex[];
  completedVertices: PlanningVertex[] | null;
}

export const EMPTY_PLANNING_POLYGON: PlanningPolygonState = {
  draftVertices: [],
  completedVertices: null,
};

export type PlanningRadarPresetId = "short" | "medium" | "long";

export interface PlanningRadarPreset {
  id: PlanningRadarPresetId;
  label: string;
  radar_type: string;
  detection_range_m: number;
}

export const PLANNING_RADAR_PRESETS: PlanningRadarPreset[] = [
  {
    id: "short",
    label: "Short Range",
    radar_type: "Short Range",
    detection_range_m: 500,
  },
  {
    id: "medium",
    label: "Medium Range",
    radar_type: "Medium Radar",
    detection_range_m: 1500,
  },
  {
    id: "long",
    label: "Long Range",
    radar_type: "Long Range",
    detection_range_m: 3000,
  },
];

export const DEFAULT_PLANNING_RADAR_PRESET_ID: PlanningRadarPresetId = "medium";

export interface PlanningRadarSite {
  id: string;
  position: PlanningVertex;
  radar_type: string;
  detection_range_m: number;
}

export interface PlanningRadarState {
  sites: PlanningRadarSite[];
  selectedSiteId: string | null;
  nextSiteId: number;
}

export const EMPTY_PLANNING_RADARS: PlanningRadarState = {
  sites: [],
  selectedSiteId: null,
  nextSiteId: 1,
};

export interface PlanningCoverageCell {
  center: PlanningVertex;
  sizeM: number;
  areaM2: number;
}

export interface PlanningCoverageEstimate {
  radarCount: number;
  totalPolygonAreaM2: number;
  estimatedCoveredAreaM2: number;
  estimatedUncoveredAreaM2: number;
  coveragePercent: number;
  coveredCells: PlanningCoverageCell[];
  uncoveredCells: PlanningCoverageCell[];
  blindSpotHints: PlanningCoverageCell[];
}

export interface PlanningCoverageLayerOptions {
  showCoverage: boolean;
  showBlindSpots: boolean;
}

export const DEFAULT_PLANNING_COVERAGE_OPTIONS: PlanningCoverageLayerOptions = {
  showCoverage: true,
  showBlindSpots: true,
};


const COVERAGE_SAMPLE_STEPS = 28;
const MAX_RENDERED_COVERAGE_CELLS = 320;
const MAX_BLIND_SPOT_HINTS = 6;

const PLANNING_PREFIX = "rt-planning-defense-area-";

export function getPlanningRadarPreset(
  presetId: PlanningRadarPresetId,
): PlanningRadarPreset {
  return (
    PLANNING_RADAR_PRESETS.find((preset) => preset.id === presetId) ??
    PLANNING_RADAR_PRESETS[1]
  );
}

export function planningToolAllowsDrawing(
  planningModeActive: boolean,
  tool: PlanningTool,
): boolean {
  return planningModeActive && tool === "draw_defense_area";
}

export function planningToolAllowsRadarPlacement(
  planningModeActive: boolean,
  tool: PlanningTool,
): boolean {
  return planningModeActive && tool === "place_radar_site";
}

export function planningToolUsesCesiumClick(
  planningModeActive: boolean,
  tool: PlanningTool,
): boolean {
  return (
    planningToolAllowsDrawing(planningModeActive, tool) ||
    planningToolAllowsRadarPlacement(planningModeActive, tool)
  );
}

export function addPlanningVertex(
  state: PlanningPolygonState,
  vertex: PlanningVertex,
): PlanningPolygonState {
  return {
    draftVertices: [...state.draftVertices, vertex],
    completedVertices: state.completedVertices,
  };
}

export function canFinishPlanningPolygon(state: PlanningPolygonState): boolean {
  return state.draftVertices.length >= 3;
}

export function finishPlanningPolygon(state: PlanningPolygonState): PlanningPolygonState {
  if (!canFinishPlanningPolygon(state)) return state;
  return {
    draftVertices: [],
    completedVertices: [...state.draftVertices],
  };
}

export function cancelPlanningDrawing(state: PlanningPolygonState): PlanningPolygonState {
  return {
    ...state,
    draftVertices: [],
  };
}

export function clearPlanningPolygon(): PlanningPolygonState {
  return EMPTY_PLANNING_POLYGON;
}

export function addPlanningRadarSite(
  state: PlanningRadarState,
  position: PlanningVertex,
): PlanningRadarState {
  const preset = getPlanningRadarPreset(DEFAULT_PLANNING_RADAR_PRESET_ID);
  const site: PlanningRadarSite = {
    id: `planning-radar-${state.nextSiteId}`,
    position,
    radar_type: preset.radar_type,
    detection_range_m: preset.detection_range_m,
  };
  return {
    sites: [...state.sites, site],
    selectedSiteId: site.id,
    nextSiteId: state.nextSiteId + 1,
  };
}

export function selectPlanningRadarSite(
  state: PlanningRadarState,
  siteId: string | null,
): PlanningRadarState {
  if (siteId === null) return { ...state, selectedSiteId: null };
  if (!state.sites.some((site) => site.id === siteId)) return state;
  return { ...state, selectedSiteId: siteId };
}

export function updatePlanningRadarPreset(
  state: PlanningRadarState,
  siteId: string,
  presetId: PlanningRadarPresetId,
): PlanningRadarState {
  const preset = getPlanningRadarPreset(presetId);
  return {
    ...state,
    sites: state.sites.map((site) =>
      site.id === siteId
        ? {
            ...site,
            radar_type: preset.radar_type,
            detection_range_m: preset.detection_range_m,
          }
        : site,
    ),
  };
}

export function deletePlanningRadarSite(
  state: PlanningRadarState,
  siteId: string,
): PlanningRadarState {
  const sites = state.sites.filter((site) => site.id !== siteId);
  return {
    ...state,
    sites,
    selectedSiteId: state.selectedSiteId === siteId ? null : state.selectedSiteId,
  };
}

export function clearPlanningRadarSites(state: PlanningRadarState): PlanningRadarState {
  return {
    ...state,
    sites: [],
    selectedSiteId: null,
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

function isCoveredByRadar(point: PlanningVertex, radars: PlanningRadarSite[]): boolean {
  return radars.some((site) => distanceM(point, site.position) <= site.detection_range_m);
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

function limitCoverageCells(cells: PlanningCoverageCell[]): PlanningCoverageCell[] {
  if (cells.length <= MAX_RENDERED_COVERAGE_CELLS) return cells;
  const stride = Math.ceil(cells.length / MAX_RENDERED_COVERAGE_CELLS);
  return cells.filter((_, index) => index % stride === 0).slice(0, MAX_RENDERED_COVERAGE_CELLS);
}

function rankBlindSpotHints(
  uncoveredCells: PlanningCoverageCell[],
  radars: PlanningRadarSite[],
): PlanningCoverageCell[] {
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

export function estimatePlanningCoverage(
  polygon: PlanningPolygonState,
  radars: PlanningRadarState,
  sampleSteps = COVERAGE_SAMPLE_STEPS,
): PlanningCoverageEstimate {
  const vertices = polygon.completedVertices ?? [];
  const totalPolygonAreaM2 = polygonArea(vertices);
  if (vertices.length < 3 || totalPolygonAreaM2 <= 0) {
    return {
      radarCount: radars.sites.length,
      totalPolygonAreaM2: 0,
      estimatedCoveredAreaM2: 0,
      estimatedUncoveredAreaM2: 0,
      coveragePercent: 0,
      coveredCells: [],
      uncoveredCells: [],
      blindSpotHints: [],
    };
  }

  const bounds = boundsForPolygon(vertices);
  const stepCount = Math.max(4, sampleSteps);
  const cellWidth = (bounds.maxX - bounds.minX) / stepCount;
  const cellHeight = (bounds.maxY - bounds.minY) / stepCount;
  if (cellWidth <= 0 || cellHeight <= 0) {
    return {
      radarCount: radars.sites.length,
      totalPolygonAreaM2,
      estimatedCoveredAreaM2: 0,
      estimatedUncoveredAreaM2: totalPolygonAreaM2,
      coveragePercent: 0,
      coveredCells: [],
      uncoveredCells: [],
      blindSpotHints: [],
    };
  }

  const coveredCells: PlanningCoverageCell[] = [];
  const uncoveredCells: PlanningCoverageCell[] = [];
  for (let xIndex = 0; xIndex < stepCount; xIndex += 1) {
    for (let yIndex = 0; yIndex < stepCount; yIndex += 1) {
      const center = {
        x: bounds.minX + cellWidth * (xIndex + 0.5),
        y: bounds.minY + cellHeight * (yIndex + 0.5),
      };
      if (!pointInPolygon(center, vertices)) continue;
      const cell: PlanningCoverageCell = {
        center,
        sizeM: Math.min(cellWidth, cellHeight),
        areaM2: 0,
      };
      if (isCoveredByRadar(center, radars.sites)) coveredCells.push(cell);
      else uncoveredCells.push(cell);
    }
  }

  const sampledInsideCount = coveredCells.length + uncoveredCells.length;
  const cellAreaM2 = sampledInsideCount > 0 ? totalPolygonAreaM2 / sampledInsideCount : 0;
  const estimatedCoveredAreaM2 = coveredCells.length * cellAreaM2;
  const estimatedUncoveredAreaM2 = Math.max(0, totalPolygonAreaM2 - estimatedCoveredAreaM2);
  const withArea = (cell: PlanningCoverageCell): PlanningCoverageCell => ({
    ...cell,
    areaM2: cellAreaM2,
  });
  const coveredWithArea = coveredCells.map(withArea);
  const uncoveredWithArea = uncoveredCells.map(withArea);

  return {
    radarCount: radars.sites.length,
    totalPolygonAreaM2,
    estimatedCoveredAreaM2,
    estimatedUncoveredAreaM2,
    coveragePercent:
      totalPolygonAreaM2 > 0 ? (estimatedCoveredAreaM2 / totalPolygonAreaM2) * 100 : 0,
    coveredCells: limitCoverageCells(coveredWithArea),
    uncoveredCells: limitCoverageCells(uncoveredWithArea),
    blindSpotHints: rankBlindSpotHints(uncoveredWithArea, radars.sites),
  };
}

function removePlanningEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((entity) => {
    if ((entity.id ?? "").startsWith(PLANNING_PREFIX)) toRemove.push(entity);
  });
  for (const entity of toRemove) viewer.entities.remove(entity);
}

function vertexToCartesian(vertex: PlanningVertex) {
  return worldToCartesian(
    vertex.x,
    vertex.y,
    sampleTerrainHeight(vertex.x, vertex.y) + ZONE_SURFACE_LIFT_M + 0.08,
  );
}

function addVertexMarker(viewer: Viewer, vertex: PlanningVertex, index: number): void {
  viewer.entities.add(
    new Entity({
      id: `${PLANNING_PREFIX}vertex-${index}`,
      position: vertexToCartesian(vertex),
      point: new PointGraphics({
        pixelSize: 8,
        color: Color.fromCssColorString("rgba(125, 211, 252, 0.95)"),
        outlineColor: Color.fromCssColorString("rgba(8, 47, 73, 0.98)"),
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      }),
      label: {
        text: String(index + 1),
        font: "bold 10px sans-serif",
        fillColor: Color.WHITE,
        outlineColor: Color.BLACK,
        outlineWidth: 2,
        style: LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cartesian2(0, -16),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
  );
}

function addPlanningPolyline(
  viewer: Viewer,
  id: string,
  vertices: PlanningVertex[],
  close: boolean,
): void {
  if (vertices.length < 2) return;
  const renderVertices = close ? [...vertices, vertices[0]] : vertices;
  viewer.entities.add(
    new Entity({
      id,
      polyline: {
        positions: renderVertices.map(vertexToCartesian),
        width: close ? 2.6 : 2.1,
        material: Color.fromCssColorString(
          close ? "rgba(56, 189, 248, 0.84)" : "rgba(125, 211, 252, 0.72)",
        ),
      },
    }),
  );
}

function circleVertices(center: PlanningVertex, radiusM: number): PlanningVertex[] {
  return Array.from({ length: 72 }, (_, index) => {
    const theta = (Math.PI * 2 * index) / 72;
    return {
      x: center.x + Math.cos(theta) * radiusM,
      y: center.y + Math.sin(theta) * radiusM,
    };
  });
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

function addCoverageCells(
  viewer: Viewer,
  cells: PlanningCoverageCell[],
  prefix: string,
  material: Color,
): void {
  cells.forEach((cell, index) => {
    viewer.entities.add(
      new Entity({
        id: `${PLANNING_PREFIX}${prefix}-${index}`,
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
        id: `${PLANNING_PREFIX}blind-spot-${index}`,
        position: vertexToCartesian(cell.center),
        point: new PointGraphics({
          pixelSize: 12,
          color: Color.fromCssColorString("rgba(248, 113, 113, 0.88)"),
          outlineColor: Color.fromCssColorString("rgba(127, 29, 29, 0.98)"),
          outlineWidth: 2,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        }),
        label: {
          text: "visual blind spot hint",
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

function addCoverageLayer(
  viewer: Viewer,
  coverage: PlanningCoverageEstimate,
  options: PlanningCoverageLayerOptions,
): void {
  if (options.showCoverage) {
    addCoverageCells(
      viewer,
      coverage.coveredCells,
      "coverage-covered",
      Color.fromCssColorString("rgba(34, 197, 94, 0.2)"),
    );
    addCoverageCells(
      viewer,
      coverage.uncoveredCells,
      "coverage-uncovered",
      Color.fromCssColorString("rgba(239, 68, 68, 0.16)"),
    );
  }
  if (options.showBlindSpots) addBlindSpotHints(viewer, coverage.blindSpotHints);
}

function addCompletedPolygon(viewer: Viewer, vertices: PlanningVertex[]): void {
  if (vertices.length < 3) return;
  viewer.entities.add(
    new Entity({
      id: `${PLANNING_PREFIX}complete-fill`,
      polygon: {
        hierarchy: new PolygonHierarchy(vertices.map(vertexToCartesian)),
        perPositionHeight: true,
        material: Color.fromCssColorString("rgba(14, 165, 233, 0.12)"),
        outline: false,
      },
    }),
  );
  addPlanningPolyline(viewer, `${PLANNING_PREFIX}complete-edge`, vertices, true);
}

function addRadarSite(viewer: Viewer, site: PlanningRadarSite, selected: boolean): void {
  const radiusVertices = circleVertices(site.position, site.detection_range_m);
  addPlanningPolyline(viewer, `${PLANNING_PREFIX}radar-${site.id}-range`, radiusVertices, true);
  viewer.entities.add(
    new Entity({
      id: `${PLANNING_PREFIX}radar-${site.id}-marker`,
      position: vertexToCartesian(site.position),
      point: new PointGraphics({
        pixelSize: selected ? 13 : 10,
        color: Color.fromCssColorString(
          selected ? "rgba(250, 204, 21, 0.96)" : "rgba(34, 197, 94, 0.92)",
        ),
        outlineColor: Color.fromCssColorString("rgba(2, 6, 23, 0.98)"),
        outlineWidth: selected ? 3 : 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      }),
      label: {
        text: `${site.radar_type} ${site.detection_range_m}m`,
        font: "bold 11px sans-serif",
        fillColor: Color.WHITE,
        outlineColor: Color.BLACK,
        outlineWidth: 2,
        style: LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cartesian2(0, -24),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
  );
}

export function syncPlanningDefenseAreaLayer(
  viewer: Viewer | null | undefined,
  state: PlanningPolygonState | null | undefined,
  radars?: PlanningRadarState | null,
  coverageOptions?: PlanningCoverageLayerOptions | null,
): void {
  if (!isViewerUsable(viewer)) return;
  removePlanningEntities(viewer);
  if (!state && !radars) return;

  if (state?.completedVertices && state.completedVertices.length >= 3) {
    addCompletedPolygon(viewer, state.completedVertices);
    state.completedVertices.forEach((vertex, index) => addVertexMarker(viewer, vertex, index));
  }

  state?.draftVertices.forEach((vertex, index) => addVertexMarker(viewer, vertex, index));
  addPlanningPolyline(viewer, `${PLANNING_PREFIX}draft-edge`, state?.draftVertices ?? [], false);

  if (state && radars && coverageOptions) {
    addCoverageLayer(viewer, estimatePlanningCoverage(state, radars), coverageOptions);
  }

  radars?.sites.forEach((site) => {
    addRadarSite(viewer, site, site.id === radars.selectedSiteId);
  });
}

function pickPlanningVertex(
  viewer: Viewer | null | undefined,
  position: Cartesian2,
): PlanningVertex | null {
  if (!isViewerUsable(viewer)) return null;
  if (!viewer.scene.globe?.ellipsoid) return null;
  const ray = viewer.camera.getPickRay(position);
  if (!ray) return null;
  const cartesian =
    viewer.scene.globe.pick(ray, viewer.scene) ??
    viewer.camera.pickEllipsoid(position, viewer.scene.globe.ellipsoid);
  if (!cartesian) return null;
  const carto = viewer.scene.globe.ellipsoid.cartesianToCartographic(cartesian);
  const world = cartographicToWorld({
    longitude: carto.longitude,
    latitude: carto.latitude,
    height: carto.height,
  });
  return { x: world.x, y: world.y };
}

export function attachPlanningDrawingHandlers(
  viewer: Viewer | null | undefined,
  options: {
    enabled: boolean;
    onAddVertex: (vertex: PlanningVertex) => void;
  },
): () => void {
  if (!isViewerUsable(viewer) || !viewer.scene.canvas) return () => undefined;
  const handler = new ScreenSpaceEventHandler(viewer.scene.canvas);

  handler.setInputAction((movement: { position: Cartesian2 }) => {
    if (!options.enabled) return;
    const vertex = pickPlanningVertex(viewer, movement.position);
    if (!vertex) return;
    options.onAddVertex(vertex);
  }, ScreenSpaceEventType.LEFT_CLICK);

  return () => {
    if (!handler.isDestroyed()) handler.destroy();
  };
}

export { pickPlanningVertex };
