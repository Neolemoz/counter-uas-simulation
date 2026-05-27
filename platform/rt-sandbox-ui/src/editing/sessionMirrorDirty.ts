import type { LocalEntityMap } from "@/editing/localEntityMirror";

export type SessionMirrorEditState = {
  localEntities: LocalEntityMap;
  locallyDeletedIds: Set<string>;
};

export function hasUnsyncedLocalMirror(
  edit: SessionMirrorEditState,
  pendingReconcile: boolean,
): boolean {
  if (pendingReconcile) return true;
  if (edit.locallyDeletedIds.size > 0) return true;
  return Object.keys(edit.localEntities).length > 0;
}
