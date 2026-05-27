import { useCallback, useEffect, useRef, useState } from "react";
import { listCaptureHandoffStatus } from "@/bridge/client";
import type { CaptureHandoffRow } from "@/bridge/types";

const HANDOFF_POLL_MS = 1000;

export function useCaptureHandoffMirror(sessionIds: readonly string[]) {
  const [bySession, setBySession] = useState<Map<string, CaptureHandoffRow[]>>(
    () => new Map(),
  );
  const [lastError, setLastError] = useState<string | null>(null);
  const idsRef = useRef(sessionIds);
  idsRef.current = sessionIds;

  const refresh = useCallback(async () => {
    const ids = idsRef.current;
    if (ids.length === 0) {
      setBySession(new Map());
      return;
    }
    const next = new Map<string, CaptureHandoffRow[]>();
    let err: string | null = null;
    await Promise.all(
      ids.map(async (sessionId) => {
        try {
          const resp = await listCaptureHandoffStatus(sessionId);
          if (resp.ok && Array.isArray(resp.captures)) {
            next.set(sessionId, resp.captures);
          } else {
            err = resp.message ?? resp.error_code ?? "handoff mirror failed";
            next.set(sessionId, []);
          }
        } catch (e) {
          err = e instanceof Error ? e.message : "handoff mirror error";
          next.set(sessionId, []);
        }
      }),
    );
    setBySession(next);
    setLastError(err);
  }, []);

  useEffect(() => {
    if (sessionIds.length === 0) {
      setBySession(new Map());
      setLastError(null);
      return;
    }
    void refresh();
    const id = window.setInterval(() => void refresh(), HANDOFF_POLL_MS);
    return () => window.clearInterval(id);
  }, [sessionIds.join(","), refresh]);

  return { bySession, lastError, refresh };
}
