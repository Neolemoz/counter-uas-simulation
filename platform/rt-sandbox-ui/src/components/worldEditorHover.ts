import { entityCell } from "@/world/gridCoords";

export interface GridCell {
  col: number;
  row: number;
}

export function cellsEqual(a: GridCell | null, b: GridCell | null): boolean {
  if (a === b) return true;
  if (!a || !b) return false;
  return a.col === b.col && a.row === b.row;
}

export function entityIdAtCell(
  entities: Array<{ entity_id: string; pose: { x?: number; y?: number } }>,
  cell: GridCell | null,
  selectedEntityId: string | null = null,
): string | null {
  if (!cell) return null;
  const matches = entities.filter((ent) => {
    const c = entityCell(ent.pose);
    return c.col === cell.col && c.row === cell.row;
  });
  if (matches.length === 0) return null;
  const selected = matches.find((ent) => ent.entity_id === selectedEntityId);
  if (selected) return selected.entity_id;
  return [...matches].sort((a, b) => a.entity_id.localeCompare(b.entity_id))[0]
    .entity_id;
}

export function shouldShowSpawnPreview(input: {
  editingEnabled: boolean;
  dragEntityId: string | null;
  panMoved: boolean;
  hoverCell: GridCell | null;
  hoverEntityId: string | null;
  spawnSettleCell: GridCell | null;
}): boolean {
  if (!input.editingEnabled || input.dragEntityId || input.panMoved || !input.hoverCell) {
    return false;
  }
  if (input.hoverEntityId) return false;
  if (cellsEqual(input.hoverCell, input.spawnSettleCell)) return false;
  return true;
}
