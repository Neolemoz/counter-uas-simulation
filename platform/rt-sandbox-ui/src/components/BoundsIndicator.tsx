import { WORLD_BOUNDS } from "@/world/bounds";
import { editSafetySummary } from "@/editing/cognition";
import { worldToCell } from "@/world/gridCoords";

export function BoundsIndicator({
  worldSummary,
}: {
  worldSummary: Record<string, unknown> | undefined;
}) {
  const safety = editSafetySummary(worldSummary);
  return (
    <div className="rounded border border-slate-700/80 bg-slate-950/50 px-3 py-2 text-xs text-slate-400">
      <span className="font-semibold text-slate-300">Bounds & caps: </span>
      entities {safety.entityCount} · {safety.bounds}
    </div>
  );
}

export function BoundsGridOverlay({
  cellSize,
}: {
  cellSize: number;
}) {
  const topLeft = worldToCell(WORLD_BOUNDS.x.min, WORLD_BOUNDS.y.max);
  const bottomRight = worldToCell(WORLD_BOUNDS.x.max, WORLD_BOUNDS.y.min);
  return (
    <rect
      x={topLeft.col * cellSize}
      y={topLeft.row * cellSize}
      width={(bottomRight.col - topLeft.col + 1) * cellSize}
      height={(bottomRight.row - topLeft.row + 1) * cellSize}
      fill="none"
      stroke="rgb(100 116 139)"
      strokeWidth={1}
      strokeDasharray="4 2"
    />
  );
}
