/** Shared layout tokens for the square world-editing SVG surface. */
export const WORLD_EDITOR_CELL_SIZE = 30;
export const WORLD_EDITOR_STAGE_MAX_PX = 960;
export const WORLD_EDITOR_STAGE_MIN_PX = 300;

/** Fixed-height coordinate bar (two rows — prevents hover compression). */
export const WORLD_EDITOR_COORD_BAR_HEIGHT_CLASS = "h-[4.75rem]";

/** Fixed-width world coordinate slot (prevents hover bar layout shift). */
export function formatWorldEditorCoord(value: number | null | undefined): string {
  if (value == null || Number.isNaN(value)) return "———";
  return Math.round(value).toString().padStart(5, " ");
}

/** Fixed-width grid cell label for the hover bar. */
export function formatWorldEditorCell(col: number | null, row: number | null): string {
  if (col == null || row == null) return "—,—";
  return `${col},${row}`;
}

export function worldEditorStageClassName(): string {
  return `relative aspect-square w-full max-w-[${WORLD_EDITOR_STAGE_MAX_PX}px] min-h-[${WORLD_EDITOR_STAGE_MIN_PX}px] shrink-0 overflow-hidden rounded border border-slate-800 bg-slate-950/70`;
}
