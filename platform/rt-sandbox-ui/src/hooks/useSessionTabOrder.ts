import { useCallback, useMemo, useSyncExternalStore } from "react";
import {
  mergeTabOrder,
  removeSessionFromTabOrder,
  validateTabOrder,
  writeTabOrder,
} from "@/workstation/sessionTabOrderStore";

let version = 0;
const listeners = new Set<() => void>();

function subscribe(listener: () => void): () => void {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

function bump(): void {
  version += 1;
  for (const listener of listeners) {
    listener();
  }
}

function getSnapshot(): number {
  return version;
}

export function useSessionTabOrder(connectedSessionIds: readonly string[]) {
  useSyncExternalStore(subscribe, getSnapshot, getSnapshot);

  const orderedSessionIds = useMemo(() => {
    void version;
    return mergeTabOrder(connectedSessionIds);
  }, [connectedSessionIds]);

  const reorderSessions = useCallback(
    (nextOrder: string[]) => {
      if (!validateTabOrder(nextOrder, connectedSessionIds)) return;
      writeTabOrder(nextOrder);
      bump();
    },
    [connectedSessionIds],
  );

  const moveSession = useCallback(
    (sessionId: string, direction: "left" | "right") => {
      const current = mergeTabOrder(connectedSessionIds);
      const idx = current.indexOf(sessionId);
      if (idx < 0) return;
      const target = direction === "left" ? idx - 1 : idx + 1;
      if (target < 0 || target >= current.length) return;
      const next = [...current];
      const [removed] = next.splice(idx, 1);
      next.splice(target, 0, removed);
      reorderSessions(next);
    },
    [connectedSessionIds, reorderSessions],
  );

  const pruneSession = useCallback((sessionId: string) => {
    removeSessionFromTabOrder(sessionId);
    bump();
  }, []);

  return {
    orderedSessionIds,
    reorderSessions,
    moveSession,
    pruneSession,
  };
}
