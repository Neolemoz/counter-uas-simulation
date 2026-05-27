export type EditCommandType = "spawn_entity" | "move_entity" | "delete_entity";

export interface EditHistoryEntry {
  id: string;
  timestampUtc: string;
  commandType: EditCommandType;
  entityId?: string;
  entityType?: string;
  pose?: { x: number; y: number; z: number };
  ok: boolean;
  errorCode?: string;
  message?: string;
}

const MAX_HISTORY = 32;

export function createEditHistoryEntry(
  partial: Omit<EditHistoryEntry, "id" | "timestampUtc">,
): EditHistoryEntry {
  return {
    id: crypto.randomUUID(),
    timestampUtc: new Date().toISOString(),
    ...partial,
  };
}

export function appendEditHistory(
  history: EditHistoryEntry[],
  entry: EditHistoryEntry,
): EditHistoryEntry[] {
  return [entry, ...history].slice(0, MAX_HISTORY);
}

export function clearEditHistory(): EditHistoryEntry[] {
  return [];
}
