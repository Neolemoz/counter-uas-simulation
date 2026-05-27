import { useCallback, useSyncExternalStore } from "react";
import {
  clearSessionDisplayName,
  getSessionDisplayLabel,
  readSessionDisplayNameMap,
  setSessionDisplayName,
} from "@/workstation/sessionDisplayNameStore";

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

export function useSessionDisplayNames() {
  useSyncExternalStore(subscribe, getSnapshot, getSnapshot);

  const labelFor = useCallback((sessionId: string) => {
    void version;
    return getSessionDisplayLabel(sessionId);
  }, []);

  const renameSession = useCallback((sessionId: string, promptDefault?: string) => {
    const current = readSessionDisplayNameMap()[sessionId] ?? "";
    const next = window.prompt(
      "Optional display name (RT UI only; session_id remains authoritative). Leave empty to clear.",
      promptDefault ?? current,
    );
    if (next === null) return;
    if (!next.trim()) {
      clearSessionDisplayName(sessionId);
    } else {
      setSessionDisplayName(sessionId, next);
    }
    bump();
  }, []);

  return { labelFor, renameSession };
}
