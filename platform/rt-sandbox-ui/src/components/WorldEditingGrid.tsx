import { useCallback, useRef, useState } from "react";
import { PanelShell } from "./GovernanceChrome";
import { BoundsGridOverlay, BoundsIndicator } from "./BoundsIndicator";
import { TerrainSvgOverlay } from "./TerrainSvgOverlay";
import { ENTITY_GLYPHS } from "@/world/entityCatalog";
import type { EntityType } from "@/world/entityCatalog";
import {
  GRID_HEIGHT,
  GRID_WIDTH,
  cellCenterWorld,
  entityCell,
  svgPointToCell,
} from "@/world/gridCoords";
import { clampPose, type Pose } from "@/world/bounds";
import type { UiEntity } from "@/editing/localEntityMirror";

const CELL_SIZE = 14;

export function WorldEditingGrid({
  entities,
  selectedEntityId,
  selectedType,
  editingEnabled,
  onSelectEntity,
  onSpawn,
  onMove,
  onDelete,
  worldSummary,
  showTerrainContour = true,
  showContourLines = false,
}: {
  entities: UiEntity[];
  selectedEntityId: string | null;
  selectedType: EntityType;
  editingEnabled: boolean;
  worldSummary: Record<string, unknown> | undefined;
  showTerrainContour?: boolean;
  showContourLines?: boolean;
  onSelectEntity: (id: string | null) => void;
  onSpawn: (pose: Pose, entityType: EntityType) => void;
  onMove: (entityId: string, pose: Pose) => void;
  onDelete: (entityId: string) => void;
}) {
  const svgRef = useRef<SVGSVGElement>(null);
  const [dragEntityId, setDragEntityId] = useState<string | null>(null);
  const [dragCell, setDragCell] = useState<{ col: number; row: number } | null>(
    null,
  );

  const svgWidth = GRID_WIDTH * CELL_SIZE;
  const svgHeight = GRID_HEIGHT * CELL_SIZE;

  const clientToCell = useCallback((clientX: number, clientY: number) => {
    const svg = svgRef.current;
    if (!svg) return null;
    const rect = svg.getBoundingClientRect();
    const x = (clientX - rect.left) * (svgWidth / rect.width);
    const y = (clientY - rect.top) * (svgHeight / rect.height);
    return svgPointToCell(x, y, CELL_SIZE);
  }, [svgHeight, svgWidth]);

  const handleBackgroundClick = (e: React.MouseEvent<SVGSVGElement>) => {
    if (!editingEnabled || dragEntityId) return;
    const target = e.target as Element;
    if (target.getAttribute("data-entity-marker") === "true") return;
    const cell = clientToCell(e.clientX, e.clientY);
    if (!cell) return;
    const world = cellCenterWorld(cell.col, cell.row);
    onSpawn(clampPose({ x: world.x, y: world.y, z: 10, yaw_deg: 0 }), selectedType);
    onSelectEntity(null);
  };

  const handleMarkerPointerDown = (
    e: React.PointerEvent,
    entityId: string,
  ) => {
    if (!editingEnabled) return;
    e.stopPropagation();
    (e.target as Element).setPointerCapture(e.pointerId);
    onSelectEntity(entityId);
    setDragEntityId(entityId);
    const cell = clientToCell(e.clientX, e.clientY);
    setDragCell(cell);
  };

  const handlePointerMove = (e: React.PointerEvent<SVGSVGElement>) => {
    if (!dragEntityId) return;
    const cell = clientToCell(e.clientX, e.clientY);
    if (cell) setDragCell(cell);
  };

  const handlePointerUp = (e: React.PointerEvent<SVGSVGElement>) => {
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

  const displayEntities = entities.map((ent) => {
    if (ent.entity_id === dragEntityId && dragCell) {
      const world = cellCenterWorld(dragCell.col, dragCell.row);
      return {
        ...ent,
        pose: { ...ent.pose, x: world.x, y: world.y },
      };
    }
    return ent;
  });

  return (
    <PanelShell title="World editor" className="col-span-full">
      <BoundsIndicator worldSummary={worldSummary} />
      <p className="my-2 text-xs text-slate-500">
        Click empty cell to spawn · drag marker to reposition · select + Delete to
        remove. Registry commands only — mirror updates via pull.
      </p>
      {!editingEnabled && (
        <p className="mb-2 text-xs text-amber-400">
          Editing disabled — session must be running or paused.
        </p>
      )}
      <div className="aspect-square w-full max-w-[560px] overflow-hidden">
        <svg
          ref={svgRef}
          width={svgWidth}
          height={svgHeight}
          viewBox={`0 0 ${svgWidth} ${svgHeight}`}
          className="h-full w-full rounded border border-slate-700 bg-slate-950 touch-none"
          onClick={handleBackgroundClick}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerLeave={handlePointerUp}
        >
          <BoundsGridOverlay cellSize={CELL_SIZE} />
          {showTerrainContour && (
            <TerrainSvgOverlay cellSize={CELL_SIZE} showContours={showContourLines} />
          )}
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
          {displayEntities.map((ent) => {
            const cell = entityCell(ent.pose);
            const selected = ent.entity_id === selectedEntityId;
            const glyph =
              ENTITY_GLYPHS[ent.entity_type as EntityType] ??
              String(ent.entity_type ?? "?").slice(0, 1).toUpperCase();
            return (
              <g
                key={ent.entity_id}
                data-entity-marker="true"
                onPointerDown={(e) => handleMarkerPointerDown(e, ent.entity_id)}
                style={{ cursor: editingEnabled ? "grab" : "default" }}
              >
                <rect
                  x={cell.col * CELL_SIZE + 1}
                  y={cell.row * CELL_SIZE + 1}
                  width={CELL_SIZE - 2}
                  height={CELL_SIZE - 2}
                  rx={2}
                  fill={selected ? "#059669" : "#047857"}
                  stroke={selected ? "#6ee7b7" : "#10b981"}
                  strokeWidth={selected ? 2 : 1}
                />
                <text
                  x={cell.col * CELL_SIZE + CELL_SIZE / 2}
                  y={cell.row * CELL_SIZE + CELL_SIZE / 2 + 4}
                  textAnchor="middle"
                  fontSize={10}
                  fontFamily="monospace"
                  fill="#ecfdf5"
                  pointerEvents="none"
                >
                  {glyph}
                </text>
              </g>
            );
          })}
        </svg>
      </div>
      {selectedEntityId && editingEnabled && (
        <div className="mt-2 flex gap-2">
          <button
            type="button"
            onClick={() => onDelete(selectedEntityId)}
            className="rounded bg-red-900/80 px-3 py-1 text-sm text-red-100 hover:bg-red-800"
          >
            Delete selected entity
          </button>
          <button
            type="button"
            onClick={() => onSelectEntity(null)}
            className="rounded bg-slate-700 px-3 py-1 text-sm text-slate-200"
          >
            Clear selection
          </button>
        </div>
      )}
    </PanelShell>
  );
}
