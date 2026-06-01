import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Crosshair, LocateFixed, Map as MapIcon, Maximize2, Minus, Plus, Trash2 } from "lucide-react";
import {
  RadarDomePreviewPanel,
  type RadarDomePreviewControlHandlers,
  type RadarDomePreviewControlState,
} from "./RadarDomePreviewControls";
import { PanelShell } from "./GovernanceChrome";
import { BoundsGridOverlay, BoundsIndicator } from "./BoundsIndicator";
import { DefenseZoneSvgOverlay } from "./DefenseZoneSvgOverlay";
import { TerrainSvgOverlay } from "./TerrainSvgOverlay";
import {
  markerVisualStyle,
  markerCellOffset,
  SPAWN_PREVIEW_RADIUS_FACTOR,
  spawnTypeLabel,
} from "./worldEditorVisuals";
import { ENTITY_GLYPHS, ENTITY_LABELS, type EntityType } from "@/world/entityCatalog";
import {
  GRID_HEIGHT,
  GRID_WIDTH,
  cellCenterWorld,
  entityCell,
} from "@/world/gridCoords";
import { clampPose, type Pose } from "@/world/bounds";
import type { UiEntity } from "@/editing/localEntityMirror";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { entitiesFromSnapshot } from "@/telemetry/channelIndex";
import {
  findEntityRuntimeTelemetry,
} from "@/telemetry/entityMirrorFields";
import { SelectedEntityRuntimeStrip } from "@/components/SelectedEntityRuntimeStrip";
import {
  DEFAULT_DEFENSE_ZONE_CONFIG,
  type DefenseZoneConfig,
  type DefenseZoneShape,
  type DefenseZoneSizes,
} from "@/cesium/defenseZoneConfig";
import {
  DEFAULT_RADAR_DOME_CONFIG,
  type RadarDomeConfig,
} from "@/cesium/sensorDomeLayer";
import {
  DEFAULT_SENSOR_DOME_ZONE_MODE,
  type SensorDomeZoneMode,
} from "@/cesium/terrainLayers";
import { shouldShowDefenseZones } from "@/cesium/sensorDomeZoneMode";
import {
  formatWorldEditorCell,
  formatWorldEditorCoord,
  WORLD_EDITOR_CELL_SIZE,
  WORLD_EDITOR_COORD_BAR_HEIGHT_CLASS,
  worldEditorStageClassName,
} from "./worldEditorLayout";
import {
  clientToGridCell,
  clientToGridCellFromSvg,
  clientToSvgPoint,
  panOriginForGrab,
} from "./worldEditorPointer";
import {
  cellsEqual,
  entityIdAtCell,
  shouldShowSpawnPreview,
  type GridCell,
} from "./worldEditorHover";
import {
  WorldEditorApplyStatus,
  type ApplyRuntimeStatus,
} from "./WorldEditorApplyStatus";

const CELL_SIZE = WORLD_EDITOR_CELL_SIZE;
const MIN_ZOOM = 1;
const MAX_ZOOM = 2.8;
const PAN_CLICK_TOLERANCE_PX = 4;

function clampViewOffset(value: number, visibleSize: number, fullSize: number): number {
  return Math.max(0, Math.min(fullSize - visibleSize, value));
}

function entityGlyph(entityType: string | undefined): string {
  return (
    ENTITY_GLYPHS[entityType as EntityType] ??
    String(entityType ?? "?").slice(0, 1).toUpperCase()
  );
}

export function WorldEditingGrid({
  entities,
  selectedEntityId,
  selectedType,
  editingEnabled,
  onSelectEntity,
  onSpawn,
  onMove,
  onDelete,
  onApplyToRuntime,
  applyToRuntimeDisabled = true,
  applyRuntimeStatus = { phase: "idle" },
  worldSummary,
  showTerrainContour = true,
  showContourLines = false,
  radarDomeConfig = DEFAULT_RADAR_DOME_CONFIG,
  defenseZoneConfig = DEFAULT_DEFENSE_ZONE_CONFIG,
  radarDomeSelectedOnly = false,
  defenseZoneSelectedOnly = false,
  radarDomeVisible = true,
  radarVolumeVisible = true,
  defenseZoneVisible = true,
  radarDomeLabelsVisible = true,
  sensorDomeZoneMode = DEFAULT_SENSOR_DOME_ZONE_MODE,
  sensorDomeLayerEnabled = false,
  onRadarDomeConfigChange = () => undefined,
  onDefenseZoneConfigChange = () => undefined,
  onRadarDomeSelectedOnlyChange = () => undefined,
  onDefenseZoneSelectedOnlyChange = () => undefined,
  onRadarDomeVisibleChange = () => undefined,
  onRadarVolumeVisibleChange = () => undefined,
  onDefenseZoneVisibleChange = () => undefined,
  onRadarDomeLabelsVisibleChange = () => undefined,
  onSensorDomeZoneModeChange = () => undefined,
  mirrorSnapshot,
  captureActive = false,
}: {
  entities: UiEntity[];
  selectedEntityId: string | null;
  selectedType: EntityType;
  editingEnabled: boolean;
  worldSummary: Record<string, unknown> | undefined;
  mirrorSnapshot?: ChannelSnapshot;
  captureActive?: boolean;
  showTerrainContour?: boolean;
  showContourLines?: boolean;
  radarDomeConfig?: RadarDomeConfig;
  defenseZoneConfig?: DefenseZoneConfig;
  radarDomeSelectedOnly?: boolean;
  defenseZoneSelectedOnly?: boolean;
  radarDomeVisible?: boolean;
  radarVolumeVisible?: boolean;
  defenseZoneVisible?: boolean;
  radarDomeLabelsVisible?: boolean;
  sensorDomeZoneMode?: SensorDomeZoneMode;
  sensorDomeLayerEnabled?: boolean;
  onRadarDomeConfigChange?: (config: RadarDomeConfig) => void;
  onDefenseZoneConfigChange?: (config: DefenseZoneConfig) => void;
  onRadarDomeSelectedOnlyChange?: (selectedOnly: boolean) => void;
  onDefenseZoneSelectedOnlyChange?: (selectedOnly: boolean) => void;
  onRadarDomeVisibleChange?: (visible: boolean) => void;
  onRadarVolumeVisibleChange?: (visible: boolean) => void;
  onDefenseZoneVisibleChange?: (visible: boolean) => void;
  onRadarDomeLabelsVisibleChange?: (visible: boolean) => void;
  onSensorDomeZoneModeChange?: (mode: SensorDomeZoneMode) => void;
  onSelectEntity: (id: string | null) => void;
  onSpawn: (pose: Pose, entityType: EntityType) => void;
  onMove: (entityId: string, pose: Pose) => void;
  onDelete: (entityId: string) => void;
  onApplyToRuntime?: () => void;
  applyToRuntimeDisabled?: boolean;
  applyRuntimeStatus?: ApplyRuntimeStatus;
}) {
  const svgRef = useRef<SVGSVGElement>(null);
  const suppressClickRef = useRef(false);
  const [dragEntityId, setDragEntityId] = useState<string | null>(null);
  const [dragCell, setDragCell] = useState<{ col: number; row: number } | null>(null);
  const [hoverCell, setHoverCell] = useState<GridCell | null>(null);
  const [spawnSettleCell, setSpawnSettleCell] = useState<GridCell | null>(null);
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [panStart, setPanStart] = useState<null | {
    clientX: number;
    clientY: number;
    panX: number;
    panY: number;
    grabSvgX: number;
    grabSvgY: number;
    pointerId: number;
    moved: boolean;
  }>(null);

  const svgWidth = GRID_WIDTH * CELL_SIZE;
  const svgHeight = GRID_HEIGHT * CELL_SIZE;
  const viewWidth = svgWidth / zoom;
  const viewHeight = svgHeight / zoom;
  const selectedEntity = entities.find((ent) => ent.entity_id === selectedEntityId) ?? null;
  const mirrorEntities = useMemo(
    () => entitiesFromSnapshot(mirrorSnapshot),
    [mirrorSnapshot],
  );
  const selectedRuntimeTelemetry = useMemo(
    () => findEntityRuntimeTelemetry(mirrorEntities, selectedEntityId),
    [mirrorEntities, selectedEntityId],
  );

  const normalizedPan = useMemo(
    () => ({
      x: clampViewOffset(pan.x, viewWidth, svgWidth),
      y: clampViewOffset(pan.y, viewHeight, svgHeight),
    }),
    [pan.x, pan.y, svgWidth, svgHeight, viewWidth, viewHeight],
  );

  const setZoomAroundCenter = (nextZoom: number) => {
    const clampedZoom = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, nextZoom));
    const centerX = normalizedPan.x + viewWidth / 2;
    const centerY = normalizedPan.y + viewHeight / 2;
    const nextViewWidth = svgWidth / clampedZoom;
    const nextViewHeight = svgHeight / clampedZoom;
    setZoom(clampedZoom);
    setPan({
      x: clampViewOffset(centerX - nextViewWidth / 2, nextViewWidth, svgWidth),
      y: clampViewOffset(centerY - nextViewHeight / 2, nextViewHeight, svgHeight),
    });
  };

  const resetView = () => {
    setZoom(1);
    setPan({ x: 0, y: 0 });
  };

  const fitAll = () => {
    if (entities.length === 0) {
      resetView();
      return;
    }
    const cells = entities.map((ent) => entityCell(ent.pose));
    const minCol = Math.max(0, Math.min(...cells.map((cell) => cell.col)) - 4);
    const maxCol = Math.min(GRID_WIDTH - 1, Math.max(...cells.map((cell) => cell.col)) + 4);
    const minRow = Math.max(0, Math.min(...cells.map((cell) => cell.row)) - 4);
    const maxRow = Math.min(GRID_HEIGHT - 1, Math.max(...cells.map((cell) => cell.row)) + 4);
    const neededWidth = Math.max((maxCol - minCol + 1) * CELL_SIZE, CELL_SIZE * 12);
    const neededHeight = Math.max((maxRow - minRow + 1) * CELL_SIZE, CELL_SIZE * 12);
    const nextZoom = Math.max(
      MIN_ZOOM,
      Math.min(MAX_ZOOM, Math.min(svgWidth / neededWidth, svgHeight / neededHeight)),
    );
    const nextViewWidth = svgWidth / nextZoom;
    const nextViewHeight = svgHeight / nextZoom;
    const centerX = ((minCol + maxCol + 1) * CELL_SIZE) / 2;
    const centerY = ((minRow + maxRow + 1) * CELL_SIZE) / 2;
    setZoom(nextZoom);
    setPan({
      x: clampViewOffset(centerX - nextViewWidth / 2, nextViewWidth, svgWidth),
      y: clampViewOffset(centerY - nextViewHeight / 2, nextViewHeight, svgHeight),
    });
  };

  const svgView = useMemo(
    () => ({
      viewBoxX: normalizedPan.x,
      viewBoxY: normalizedPan.y,
      viewBoxWidth: viewWidth,
      viewBoxHeight: viewHeight,
    }),
    [normalizedPan.x, normalizedPan.y, viewHeight, viewWidth],
  );

  const clientToCell = useCallback(
    (clientX: number, clientY: number) => {
      const svg = svgRef.current;
      if (!svg) return null;
      const fromCtm = clientToGridCellFromSvg(svg, clientX, clientY, CELL_SIZE);
      if (fromCtm) return fromCtm;
      const rect = svg.getBoundingClientRect();
      return clientToGridCell(clientX, clientY, rect, svgView, CELL_SIZE);
    },
    [svgView],
  );

  const updateHoverCell = useCallback((cell: GridCell | null) => {
    setHoverCell((prev) => (cellsEqual(prev, cell) ? prev : cell));
  }, []);

  const handleBackgroundClick = (e: React.MouseEvent<SVGSVGElement>) => {
    if (suppressClickRef.current) {
      suppressClickRef.current = false;
      return;
    }
    if (!editingEnabled || dragEntityId) return;
    const target = e.target as Element;
    if (target.closest('[data-entity-marker="true"]')) return;
    const cell = clientToCell(e.clientX, e.clientY);
    if (!cell) return;
    setSpawnSettleCell(cell);
    const world = cellCenterWorld(cell.col, cell.row);
    onSpawn(clampPose({ x: world.x, y: world.y, z: 10, yaw_deg: 0 }), selectedType);
    onSelectEntity(null);
  };

  const handleBackgroundPointerDown = (e: React.PointerEvent<SVGSVGElement>) => {
    const target = e.target as Element;
    if (target.closest('[data-entity-marker="true"]')) return;
    const svg = svgRef.current;
    if (!svg) return;
    const rect = svg.getBoundingClientRect();
    const grab =
      clientToSvgPoint(e.clientX, e.clientY, rect, svgView) ??
      { x: normalizedPan.x, y: normalizedPan.y };
    setPanStart({
      clientX: e.clientX,
      clientY: e.clientY,
      panX: normalizedPan.x,
      panY: normalizedPan.y,
      grabSvgX: grab.x,
      grabSvgY: grab.y,
      pointerId: e.pointerId,
      moved: false,
    });
    e.currentTarget.setPointerCapture(e.pointerId);
  };

  const handleMarkerPointerDown = (
    e: React.PointerEvent,
    entityId: string,
  ) => {
    if (!editingEnabled) return;
    e.stopPropagation();
    (e.currentTarget as Element).setPointerCapture(e.pointerId);
    onSelectEntity(entityId);
    setDragEntityId(entityId);
    const ent = entities.find((x) => x.entity_id === entityId);
    setDragCell(ent ? entityCell(ent.pose) : clientToCell(e.clientX, e.clientY));
  };

  const handlePointerMove = (e: React.PointerEvent<SVGSVGElement>) => {
    const cell = clientToCell(e.clientX, e.clientY);
    updateHoverCell(cell);
    if (dragEntityId) {
      if (cell) setDragCell(cell);
      return;
    }
    if (!panStart) return;
    const dx = e.clientX - panStart.clientX;
    const dy = e.clientY - panStart.clientY;
    const moved = panStart.moved || Math.hypot(dx, dy) > PAN_CLICK_TOLERANCE_PX;
    if (moved) suppressClickRef.current = true;
    setPanStart({ ...panStart, moved });
    const rect = e.currentTarget.getBoundingClientRect();
    const nextPan = panOriginForGrab(
      panStart.grabSvgX,
      panStart.grabSvgY,
      e.clientX,
      e.clientY,
      rect,
      viewWidth,
      viewHeight,
    );
    setPan({
      x: clampViewOffset(nextPan.x, viewWidth, svgWidth),
      y: clampViewOffset(nextPan.y, viewHeight, svgHeight),
    });
  };

  const handlePointerUp = (e: React.PointerEvent<SVGSVGElement>) => {
    if (panStart?.pointerId === e.pointerId) {
      setPanStart(null);
      e.currentTarget.releasePointerCapture?.(e.pointerId);
    }
    if (!dragEntityId || !dragCell) {
      setDragEntityId(null);
      setDragCell(null);
      return;
    }
    const world = cellCenterWorld(dragCell.col, dragCell.row);
    const ent = entities.find((x) => x.entity_id === dragEntityId);
    const z = Number((ent?.pose as Record<string, unknown>)?.z ?? 10);
    onMove(
      dragEntityId,
      clampPose({ x: world.x, y: world.y, z, yaw_deg: 0 }),
    );
    setDragEntityId(null);
    setDragCell(null);
    (e.target as Element).releasePointerCapture?.(e.pointerId);
  };

  const selectedRadar = entities.find(
    (ent) => ent.entity_id === selectedEntityId && ent.entity_type === "radar",
  );
  const selectedProtected = entities.find(
    (ent) => ent.entity_id === selectedEntityId && ent.entity_type === "waypoint_marker",
  );

  const updateRadarDetection = (value: number) => {
    onRadarDomeConfigChange({
      detectionM: Math.max(10, value || 10),
    });
  };

  const updateDefenseSize = (key: keyof DefenseZoneSizes, value: number) => {
    const nextSizes = {
      ...defenseZoneConfig.sizes,
      [key]: Math.max(10, value || 10),
    };
    if (nextSizes.engageM <= nextSizes.coreM) nextSizes.engageM = nextSizes.coreM + 10;
    if (nextSizes.warningM <= nextSizes.engageM) {
      nextSizes.warningM = nextSizes.engageM + 10;
    }
    onDefenseZoneConfigChange({ ...defenseZoneConfig, sizes: nextSizes });
  };

  const updateDefenseShape = (shape: DefenseZoneShape) => {
    onDefenseZoneConfigChange({ ...defenseZoneConfig, shape });
  };

  const displayEntities = useMemo(
    () =>
      entities.map((ent) => {
        if (ent.entity_id === dragEntityId && dragCell) {
          const world = cellCenterWorld(dragCell.col, dragCell.row);
          return {
            ...ent,
            pose: { ...ent.pose, x: world.x, y: world.y },
          };
        }
        return ent;
      }),
    [entities, dragEntityId, dragCell],
  );

  const cellOccupancy = useMemo(() => {
    const byCell = new Map<string, string[]>();
    for (const ent of displayEntities) {
      const cell = entityCell(ent.pose);
      const key = `${cell.col},${cell.row}`;
      const list = byCell.get(key) ?? [];
      list.push(ent.entity_id);
      byCell.set(key, list);
    }
    const indexByEntity = new Map<string, { index: number; count: number }>();
    for (const ids of byCell.values()) {
      const sorted = [...ids].sort();
      sorted.forEach((id, index) => {
        indexByEntity.set(id, { index, count: sorted.length });
      });
    }
    return indexByEntity;
  }, [displayEntities]);

  const hoverEntityId = useMemo(
    () => entityIdAtCell(displayEntities, hoverCell, selectedEntityId),
    [displayEntities, hoverCell, selectedEntityId],
  );

  const hoverWorld = hoverCell ? cellCenterWorld(hoverCell.col, hoverCell.row) : null;
  const canShowSpawnPreview = shouldShowSpawnPreview({
    editingEnabled,
    dragEntityId,
    panMoved: panStart?.moved === true,
    hoverCell,
    hoverEntityId,
    spawnSettleCell,
  });
  const spawnPreviewCell = canShowSpawnPreview ? hoverCell : null;
  const previewGlyph = entityGlyph(selectedType);
  const previewLabel = spawnTypeLabel(selectedType);
  const selectedWorld = selectedEntity
    ? {
        x: Number(selectedEntity.pose.x ?? 0),
        y: Number(selectedEntity.pose.y ?? 0),
      }
    : null;
  const radarPreviewState: RadarDomePreviewControlState = {
    layerEnabled: sensorDomeLayerEnabled,
    showVolume: radarVolumeVisible,
    showRing: radarDomeVisible,
    selectedOnly: radarDomeSelectedOnly,
    showLabels: radarDomeLabelsVisible,
  };
  const radarPreviewHandlers: RadarDomePreviewControlHandlers = {
    onShowVolumeChange: onRadarVolumeVisibleChange,
    onShowRingChange: onRadarDomeVisibleChange,
    onSelectedOnlyChange: onRadarDomeSelectedOnlyChange,
    onShowLabelsChange: onRadarDomeLabelsVisibleChange,
  };

  useEffect(() => {
    if (!spawnSettleCell) return;
    const settled = entities.some((ent) => {
      const c = entityCell(ent.pose);
      return c.col === spawnSettleCell.col && c.row === spawnSettleCell.row;
    });
    if (settled) setSpawnSettleCell(null);
  }, [entities, spawnSettleCell]);

  useEffect(() => {
    if (!spawnSettleCell || !hoverCell) return;
    if (!cellsEqual(hoverCell, spawnSettleCell)) {
      setSpawnSettleCell(null);
    }
  }, [hoverCell, spawnSettleCell]);

  useEffect(() => {
    if (!editingEnabled || !selectedEntityId) return;
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target;
      if (
        target instanceof HTMLInputElement ||
        target instanceof HTMLTextAreaElement ||
        target instanceof HTMLSelectElement
      ) {
        return;
      }
      if (event.key === "Delete" || event.key === "Backspace") {
        event.preventDefault();
        onDelete(selectedEntityId);
      }
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [editingEnabled, onDelete, selectedEntityId]);

  const handleDeleteSelected = () => {
    if (selectedEntityId) onDelete(selectedEntityId);
  };

  return (
    <PanelShell
      title="World editor"
      icon={MapIcon}
      className="col-span-full !p-3 [&>h2]:mb-2"
    >
      <BoundsIndicator worldSummary={worldSummary} />
      {onApplyToRuntime && (
        <div
          className="mt-2 flex flex-wrap items-center justify-between gap-2 rounded border border-slate-800 bg-slate-950/60 px-3 py-2"
          data-testid="world-editor-apply-runtime-bar"
        >
          <p className="text-xs text-slate-500">
            Resets runtime world and spawns current layout.
          </p>
          <div className="flex flex-wrap items-center gap-2">
            <WorldEditorApplyStatus status={applyRuntimeStatus} />
            <button
              type="button"
              disabled={applyToRuntimeDisabled}
              onClick={onApplyToRuntime}
              data-testid="world-editor-apply-runtime"
              className="rounded border border-emerald-700 bg-emerald-950/50 px-3 py-1.5 text-xs font-medium text-emerald-100 hover:bg-emerald-900/50 disabled:cursor-not-allowed disabled:opacity-40"
            >
              Apply to runtime
            </button>
          </div>
        </div>
      )}
      <div className="mt-2 space-y-3">
        <div
          className={`grid ${WORLD_EDITOR_COORD_BAR_HEIGHT_CLASS} grid-rows-2 gap-1 rounded border border-slate-800 bg-slate-950/60 px-3 py-2`}
          data-testid="world-editor-coordinate-bar"
        >
          <div className="flex min-h-0 items-center justify-between gap-3 border-b border-slate-800/70 pb-1">
            <div className="flex min-w-0 flex-nowrap items-center gap-3 font-mono text-[11px] tabular-nums">
              <span
                className="inline-block w-[5.25rem] shrink-0 text-cyan-100"
                data-testid="world-editor-coord-x"
              >
                x {formatWorldEditorCoord(hoverWorld?.x ?? null)}
              </span>
              <span
                className="inline-block w-[5.25rem] shrink-0 text-cyan-100"
                data-testid="world-editor-coord-y"
              >
                y {formatWorldEditorCoord(hoverWorld?.y ?? null)}
              </span>
              <span
                className="inline-block w-[4.5rem] shrink-0 text-slate-500"
                data-testid="world-editor-coord-cell"
              >
                cell {formatWorldEditorCell(hoverCell?.col ?? null, hoverCell?.row ?? null)}
              </span>
            </div>
            <div className="flex shrink-0 flex-nowrap items-center gap-2">
              <span className="inline-block w-[4.5rem] text-right font-mono text-[11px] tabular-nums text-slate-500">
                zoom {zoom.toFixed(1)}×
              </span>
              <div className="flex items-center gap-1">
                <button
                  type="button"
                  onClick={() => setZoomAroundCenter(zoom * 1.25)}
                  className="rounded border border-slate-700 bg-slate-950/70 p-1.5 text-slate-200 hover:border-cyan-700 hover:text-cyan-100"
                  title="Zoom in"
                >
                  <Plus className="h-3.5 w-3.5" />
                </button>
                <button
                  type="button"
                  onClick={() => setZoomAroundCenter(zoom / 1.25)}
                  className="rounded border border-slate-700 bg-slate-950/70 p-1.5 text-slate-200 hover:border-cyan-700 hover:text-cyan-100"
                  title="Zoom out"
                >
                  <Minus className="h-3.5 w-3.5" />
                </button>
                <button
                  type="button"
                  onClick={fitAll}
                  className="rounded border border-slate-700 bg-slate-950/70 p-1.5 text-slate-200 hover:border-cyan-700 hover:text-cyan-100"
                  title="Fit all entities"
                >
                  <Maximize2 className="h-3.5 w-3.5" />
                </button>
                <button
                  type="button"
                  onClick={resetView}
                  className="rounded border border-slate-700 bg-slate-950/70 p-1.5 text-slate-200 hover:border-cyan-700 hover:text-cyan-100"
                  title="Reset grid view"
                >
                  <LocateFixed className="h-3.5 w-3.5" />
                </button>
              </div>
            </div>
          </div>
          <div className="flex min-h-0 items-center justify-between gap-3 pt-0.5 text-[10px]">
            <span
              className="min-w-0 truncate text-slate-500"
              data-testid="world-editor-helper-text"
            >
              <Crosshair className="mr-1 inline h-3 w-3 text-cyan-300" />
              click spawn · drag · Del · pan
            </span>
            <span
              className={`inline-block min-w-[9.5rem] shrink-0 rounded px-1.5 py-0.5 text-right font-mono ${
                canShowSpawnPreview
                  ? "bg-cyan-950/70 text-cyan-200"
                  : "invisible bg-cyan-950/70 text-cyan-200"
              }`}
              data-testid="world-editor-spawn-hint"
              aria-hidden={!canShowSpawnPreview}
            >
              click → spawn {previewLabel}
            </span>
          </div>
        </div>
        {!editingEnabled && (
          <p className="text-xs text-amber-400">
            Editing disabled — session must be running or paused.
          </p>
        )}
        {selectedEntity && editingEnabled && (
          <div className="flex flex-wrap items-center justify-between gap-3 rounded border border-amber-500/50 bg-amber-950/25 px-4 py-3">
            <div className="min-w-0">
              <div className="text-[10px] font-semibold uppercase tracking-wide text-amber-100">
                Selected
              </div>
              <div className="mt-0.5 truncate font-mono text-xs text-slate-200">
                {entityGlyph(selectedEntity.entity_type)}{" "}
                {ENTITY_LABELS[selectedEntity.entity_type as EntityType] ??
                  selectedEntity.entity_type}{" "}
                · {selectedEntity.entity_id.slice(0, 10)}
                {selectedWorld && (
                  <span className="text-slate-400">
                    {" "}
                    · x {selectedWorld.x.toFixed(0)} / y {selectedWorld.y.toFixed(0)}
                  </span>
                )}
              </div>
              {selectedRuntimeTelemetry && (
                <SelectedEntityRuntimeStrip
                  telemetry={selectedRuntimeTelemetry}
                  captureActive={captureActive}
                />
              )}
            </div>
            <div className="flex shrink-0 gap-2">
              <button
                type="button"
                onClick={handleDeleteSelected}
                className="inline-flex items-center gap-1.5 rounded border border-red-600 bg-red-900/90 px-3 py-2 text-xs font-semibold text-red-50 shadow-sm hover:bg-red-800"
                title="Delete selected (Del)"
              >
                <Trash2 className="h-4 w-4" />
                Delete
              </button>
              <button
                type="button"
                onClick={() => onSelectEntity(null)}
                className="rounded border border-slate-600 bg-slate-900 px-3 py-2 text-xs text-slate-200 hover:bg-slate-800"
              >
                Clear
              </button>
            </div>
          </div>
        )}
      </div>
      <div
        className="mt-3 w-full shrink-0"
        data-testid="world-editor-grid-section"
      >
        <div
          className={worldEditorStageClassName()}
          data-testid="world-editor-grid-stage"
        >
        <svg
          ref={svgRef}
          viewBox={`${normalizedPan.x} ${normalizedPan.y} ${viewWidth} ${viewHeight}`}
          preserveAspectRatio="xMidYMid meet"
          className={`absolute inset-0 h-full w-full touch-none ${editingEnabled ? "cursor-crosshair" : "cursor-not-allowed"} ${panStart?.moved ? "cursor-grabbing" : ""}`}
          onClick={handleBackgroundClick}
          onPointerDown={handleBackgroundPointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerLeave={(e) => {
            updateHoverCell(null);
            handlePointerUp(e);
          }}
        >
          <defs>
            <filter id="world-editor-selected-glow" x="-80%" y="-80%" width="260%" height="260%">
              <feGaussianBlur stdDeviation="2.5" result="blur" />
              <feMerge>
                <feMergeNode in="blur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>
          </defs>
          <BoundsGridOverlay cellSize={CELL_SIZE} />
          {showTerrainContour && (
            <TerrainSvgOverlay cellSize={CELL_SIZE} showContours={showContourLines} />
          )}
          <DefenseZoneSvgOverlay
            entities={displayEntities}
            cellSize={CELL_SIZE}
            selectedEntityId={selectedEntityId}
            config={defenseZoneConfig}
            visible={
              defenseZoneVisible &&
              sensorDomeLayerEnabled &&
              shouldShowDefenseZones(sensorDomeZoneMode)
            }
          />
          {Array.from({ length: GRID_HEIGHT }, (_, row) =>
            Array.from({ length: GRID_WIDTH }, (_, col) => (
              <rect
                key={`${col}-${row}`}
                x={col * CELL_SIZE}
                y={row * CELL_SIZE}
                width={CELL_SIZE}
                height={CELL_SIZE}
                fill={(col + row) % 2 === 0 ? "#0f172a" : "#1e293b"}
                stroke="#334155"
                strokeWidth={0.25}
              />
            )),
          )}
          {hoverCell && editingEnabled && !dragEntityId && (
            <rect
              x={hoverCell.col * CELL_SIZE}
              y={hoverCell.row * CELL_SIZE}
              width={CELL_SIZE}
              height={CELL_SIZE}
              fill={hoverEntityId ? "rgba(56, 189, 248, 0.12)" : "rgba(56, 189, 248, 0.2)"}
              stroke={canShowSpawnPreview ? "#67e8f9" : "#475569"}
              strokeWidth={canShowSpawnPreview ? 1.2 : 0.6}
              pointerEvents="none"
            />
          )}
          {spawnPreviewCell && (
            <g pointerEvents="none" data-testid="spawn-preview">
              <circle
                cx={spawnPreviewCell.col * CELL_SIZE + CELL_SIZE / 2}
                cy={spawnPreviewCell.row * CELL_SIZE + CELL_SIZE / 2}
                r={CELL_SIZE * SPAWN_PREVIEW_RADIUS_FACTOR}
                fill="rgba(8, 145, 178, 0.28)"
                stroke="#67e8f9"
                strokeDasharray="3 2"
                strokeWidth={1.8}
              />
              <line
                x1={spawnPreviewCell.col * CELL_SIZE + CELL_SIZE / 2 - CELL_SIZE * 0.55}
                x2={spawnPreviewCell.col * CELL_SIZE + CELL_SIZE / 2 + CELL_SIZE * 0.55}
                y1={spawnPreviewCell.row * CELL_SIZE + CELL_SIZE / 2}
                y2={spawnPreviewCell.row * CELL_SIZE + CELL_SIZE / 2}
                stroke="#a5f3fc"
                strokeWidth={0.8}
                opacity={0.9}
              />
              <line
                x1={spawnPreviewCell.col * CELL_SIZE + CELL_SIZE / 2}
                x2={spawnPreviewCell.col * CELL_SIZE + CELL_SIZE / 2}
                y1={spawnPreviewCell.row * CELL_SIZE + CELL_SIZE / 2 - CELL_SIZE * 0.55}
                y2={spawnPreviewCell.row * CELL_SIZE + CELL_SIZE / 2 + CELL_SIZE * 0.55}
                stroke="#a5f3fc"
                strokeWidth={0.8}
                opacity={0.9}
              />
              <text
                x={spawnPreviewCell.col * CELL_SIZE + CELL_SIZE / 2}
                y={spawnPreviewCell.row * CELL_SIZE + CELL_SIZE / 2 + CELL_SIZE * 0.22}
                textAnchor="middle"
                fontSize={CELL_SIZE * 0.65}
                fontWeight={700}
                fontFamily="monospace"
                fill="#ecfeff"
              >
                {previewGlyph}
              </text>
              <text
                x={spawnPreviewCell.col * CELL_SIZE + CELL_SIZE / 2}
                y={spawnPreviewCell.row * CELL_SIZE + CELL_SIZE / 2 - CELL_SIZE * 0.42}
                textAnchor="middle"
                fontSize={CELL_SIZE * 0.4}
                fontFamily="sans-serif"
                fill="#a5f3fc"
              >
                {previewLabel}
              </text>
            </g>
          )}
          {displayEntities.map((ent) => {
            const cell = entityCell(ent.pose);
            const selected = ent.entity_id === selectedEntityId;
            const hovered = ent.entity_id === hoverEntityId;
            const dragging = dragEntityId === ent.entity_id;
            const glyph = entityGlyph(ent.entity_type);
            const occupancy = cellOccupancy.get(ent.entity_id);
            const stack = markerCellOffset(
              occupancy?.index ?? 0,
              occupancy?.count ?? 1,
              CELL_SIZE,
            );
            const cx = cell.col * CELL_SIZE + CELL_SIZE / 2 + stack.dx;
            const cy = cell.row * CELL_SIZE + CELL_SIZE / 2 + stack.dy;
            const style = markerVisualStyle(ent.entity_type, selected, hovered, dragging);
            return (
              <g
                key={ent.entity_id}
                data-entity-marker="true"
                onPointerDown={(e) => handleMarkerPointerDown(e, ent.entity_id)}
                style={{
                  cursor: editingEnabled ? (dragging ? "grabbing" : "grab") : "default",
                  opacity: dragging ? 0.92 : 1,
                  transition: "opacity 120ms ease",
                }}
                filter={selected ? "url(#world-editor-selected-glow)" : undefined}
              >
                <circle
                  cx={cx}
                  cy={cy}
                  r={CELL_SIZE * style.hitRadiusFactor}
                  fill="transparent"
                  stroke="transparent"
                  pointerEvents="all"
                />
                {style.outerRingRadiusFactor > 0 && (
                  <circle
                    cx={cx}
                    cy={cy}
                    r={CELL_SIZE * style.outerRingRadiusFactor}
                    fill="none"
                    stroke={style.outerRingStroke}
                    strokeWidth={style.outerRingWidth}
                    opacity={style.outerRingOpacity}
                    pointerEvents="none"
                  />
                )}
                <rect
                  x={cx - (CELL_SIZE - 3.2) / 2}
                  y={cy - (CELL_SIZE - 3.2) / 2}
                  width={CELL_SIZE - 3.2}
                  height={CELL_SIZE - 3.2}
                  rx={4}
                  fill={style.fill}
                  stroke={style.stroke}
                  strokeWidth={style.strokeWidth}
                  pointerEvents="none"
                  style={{ transition: "stroke 120ms ease, fill 120ms ease" }}
                />
                {style.innerRingStroke && (
                  <circle
                    cx={cx}
                    cy={cy}
                    r={CELL_SIZE * 0.52}
                    fill="none"
                    stroke={style.innerRingStroke}
                    strokeWidth={1}
                    opacity={0.85}
                    pointerEvents="none"
                  />
                )}
                <text
                  x={cx}
                  y={cy + CELL_SIZE * 0.22}
                  textAnchor="middle"
                  fontSize={style.glyphSize}
                  fontWeight={selected ? 700 : 600}
                  fontFamily="monospace"
                  fill="#f8fafc"
                  pointerEvents="none"
                >
                  {glyph}
                </text>
              </g>
            );
          })}
        </svg>
        </div>
      </div>

      {(selectedRadar || selectedProtected) && (
        <div className="mt-4 space-y-4">
          {selectedRadar && (
            <div className="rounded border border-cyan-900/50 bg-slate-950/45 p-3">
              <div className="mb-3 flex items-center justify-between gap-2">
                <div>
                  <h3 className="text-xs font-semibold uppercase tracking-wide text-cyan-100">
                    Radar detection
                  </h3>
                  <p className="mt-1 font-mono text-[10px] text-slate-500">
                    {selectedRadar.entity_id.slice(0, 8)} · single cyan ring on globe
                  </p>
                </div>
                {!sensorDomeLayerEnabled && (
                  <span className="rounded border border-amber-700/50 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-200">
                    dome layer off
                  </span>
                )}
              </div>
              <label className="flex items-center justify-between gap-3 text-xs text-slate-300">
                <span>Detection radius (m)</span>
                <input
                  type="number"
                  min={10}
                  step={10}
                  value={radarDomeConfig.detectionM}
                  onChange={(e) => updateRadarDetection(Number(e.target.value))}
                  className="w-20 rounded border border-slate-700 bg-slate-950 px-2 py-1 text-right font-mono text-cyan-100"
                />
              </label>
              <RadarDomePreviewPanel
                state={radarPreviewState}
                handlers={radarPreviewHandlers}
              />
            </div>
          )}

          {selectedProtected && (
            <div className="rounded border border-rose-900/40 bg-slate-950/45 p-3">
              <div className="mb-3">
                <h3 className="text-xs font-semibold uppercase tracking-wide text-rose-100">
                  Defense zone
                </h3>
                <p className="mt-1 font-mono text-[10px] text-slate-500">
                  {selectedProtected.entity_id.slice(0, 8)} · core / engagement / warning
                </p>
              </div>
              <label className="mb-3 flex items-center justify-between gap-3 text-xs text-slate-300">
                <span>Shape</span>
                <select
                  value={defenseZoneConfig.shape}
                  onChange={(e) => updateDefenseShape(e.target.value as DefenseZoneShape)}
                  className="rounded border border-slate-700 bg-slate-950 px-2 py-1 font-mono text-slate-100"
                >
                  <option value="circle">Circle</option>
                  <option value="rectangle">Rectangle</option>
                </select>
              </label>
              <div className="grid gap-2 text-xs">
                {(
                  [
                    ["coreM", "Protected core", "text-rose-200"],
                    ["engageM", "Engagement zone", "text-amber-200"],
                    ["warningM", "Warning zone", "text-sky-200"],
                  ] as const
                ).map(([key, label, tone]) => (
                  <label
                    key={key}
                    className="flex items-center justify-between gap-3 text-slate-300"
                  >
                    <span className={tone}>{label}</span>
                    <input
                      type="number"
                      min={10}
                      step={10}
                      value={defenseZoneConfig.sizes[key]}
                      onChange={(e) =>
                        updateDefenseSize(key, Number(e.target.value))
                      }
                      className="w-20 rounded border border-slate-700 bg-slate-950 px-2 py-1 text-right font-mono text-slate-100"
                    />
                  </label>
                ))}
              </div>
              <div className="mt-3 grid gap-2 text-xs text-slate-300">
                <label className="flex items-center gap-2">
                  <input
                    type="checkbox"
                    checked={defenseZoneVisible}
                    onChange={(e) => onDefenseZoneVisibleChange(e.target.checked)}
                  />
                  Show defense zones on globe
                </label>
                <label className="flex items-center gap-2">
                  <input
                    type="checkbox"
                    checked={defenseZoneSelectedOnly}
                    onChange={(e) => onDefenseZoneSelectedOnlyChange(e.target.checked)}
                  />
                  Selected asset only
                </label>
                <label className="flex items-center gap-2">
                  <input
                    type="checkbox"
                    checked={defenseZoneConfig.showLabels}
                    onChange={(e) =>
                      onDefenseZoneConfigChange({
                        ...defenseZoneConfig,
                        showLabels: e.target.checked,
                      })
                    }
                  />
                  Show labels
                </label>
              </div>
            </div>
          )}

          {!sensorDomeLayerEnabled && (selectedRadar || selectedProtected) && (
            <p className="rounded border border-amber-700/50 bg-amber-950/35 px-3 py-2 text-[10px] text-amber-200">
              Sensor dome layer off — enable domes in visualization layers to show zones on globe.
            </p>
          )}

          {(selectedRadar || selectedProtected) && (
            <div className="rounded border border-slate-800 bg-slate-950/35 p-3">
              <div className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-300">
                Globe zone layers
              </div>
              <div className="flex flex-wrap gap-2 text-xs">
                {(
                  [
                    ["both", "Both"],
                    ["radar", "Radar only"],
                    ["defense", "Defense only"],
                  ] as const
                ).map(([mode, label]) => (
                  <label
                    key={mode}
                    className={`flex cursor-pointer items-center gap-1.5 rounded border px-2.5 py-1.5 ${
                      sensorDomeZoneMode === mode
                        ? "border-cyan-700/70 bg-cyan-950/40 text-cyan-100"
                        : "border-slate-700 bg-slate-950 text-slate-300 hover:border-slate-600"
                    }`}
                  >
                    <input
                      type="radio"
                      name="sensor-dome-zone-mode"
                      value={mode}
                      checked={sensorDomeZoneMode === mode}
                      onChange={() => onSensorDomeZoneModeChange(mode)}
                      className="sr-only"
                    />
                    {label}
                  </label>
                ))}
              </div>
            </div>
          )}
        </div>
      )}
    </PanelShell>
  );
}
