/** Grid projection for the square RT world-editing surface. */

export const GRID_WIDTH = 40;
export const GRID_HEIGHT = 40;
export const GRID_SCALE = GRID_WIDTH / 1000;
export const GRID_Y_SCALE = GRID_HEIGHT / 1000;
export const GRID_OFFSET = 500;

export interface Cell {
  col: number;
  row: number;
}

export function worldToCell(x: number, y: number): Cell {
  return {
    col: Math.max(0, Math.min(GRID_WIDTH - 1, Math.floor((x + GRID_OFFSET) * GRID_SCALE))),
    row: Math.max(0, Math.min(GRID_HEIGHT - 1, Math.floor((GRID_OFFSET - y) * GRID_Y_SCALE))),
  };
}

export function cellToWorld(col: number, row: number): { x: number; y: number } {
  return {
    x: col / GRID_SCALE - GRID_OFFSET,
    y: GRID_OFFSET - row / GRID_Y_SCALE,
  };
}

export function cellCenterWorld(col: number, row: number): { x: number; y: number } {
  const corner = cellToWorld(col, row);
  const xStep = 1 / GRID_SCALE;
  const yStep = 1 / GRID_Y_SCALE;
  return { x: corner.x + xStep / 2, y: corner.y - yStep / 2 };
}

export function svgPointToCell(
  svgX: number,
  svgY: number,
  cellSize: number,
): Cell {
  const col = Math.floor(svgX / cellSize);
  const row = Math.floor(svgY / cellSize);
  return {
    col: Math.max(0, Math.min(GRID_WIDTH - 1, col)),
    row: Math.max(0, Math.min(GRID_HEIGHT - 1, row)),
  };
}

export function entityCell(pose: { x?: number; y?: number }): Cell {
  return worldToCell(Number(pose.x ?? 0), Number(pose.y ?? 0));
}

export function renderAsciiGrid(
  entities: Array<{ entity_type?: string; pose?: Record<string, unknown> }>,
  marks: Record<string, string>,
): string[] {
  const lines: string[] = [];
  for (let row = 0; row < GRID_HEIGHT; row++) {
    let rowChars = "";
    for (let col = 0; col < GRID_WIDTH; col++) {
      let ch = ".";
      for (const ent of entities) {
        const pose = ent.pose ?? {};
        const cell = entityCell(pose);
        if (cell.col === col && cell.row === row) {
          ch = marks[String(ent.entity_type ?? "")] ?? "?";
        }
      }
      rowChars += ch;
    }
    lines.push(rowChars);
  }
  return lines;
}
