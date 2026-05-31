import { useCallback, useEffect, useRef, useState } from "react";
import {
  captureStatus,
  startCapture,
  stopCapture,
} from "@/bridge/runtimeCommands";
import type { CaptureStatusResponse } from "@/bridge/types";
import {
  INACTIVE_CAPTURE_SUMMARY,
  parseCaptureStatusFromResponse,
  type LiveCaptureSummary,
} from "@/telemetry/captureSummary";

const CAPTURE_POLL_MS = 1000;

export type { LiveCaptureSummary } from "@/telemetry/captureSummary";
export { parseCaptureStatusFromResponse } from "@/telemetry/captureSummary";

export function useCaptureControls({
  sessionId,
  editingEnabled,
  doPull,
  setLastError,
}: {
  sessionId: string | null;
  editingEnabled: boolean;
  doPull: () => Promise<void>;
  setLastError: (message: string | null) => void;
}) {
  const [busy, setBusy] = useState(false);
  const [status, setStatus] = useState<LiveCaptureSummary>(INACTIVE_CAPTURE_SUMMARY);
  const sessionIdRef = useRef(sessionId);
  sessionIdRef.current = sessionId;

  const refreshStatus = useCallback(async (): Promise<LiveCaptureSummary | null> => {
    const sid = sessionIdRef.current;
    if (!sid || !editingEnabled) return null;
    try {
      const resp = await captureStatus(sid);
      if (!resp.ok) {
        setLastError(resp.message ?? resp.error_code ?? "capture status failed");
        return null;
      }
      const next = parseCaptureStatusFromResponse(resp);
      setStatus(next);
      setLastError(null);
      return next;
    } catch (err) {
      setLastError(err instanceof Error ? err.message : "capture status error");
      return null;
    }
  }, [editingEnabled, setLastError]);

  useEffect(() => {
    if (!sessionId || !editingEnabled) {
      setStatus(INACTIVE_CAPTURE_SUMMARY);
      return;
    }
    void refreshStatus();
  }, [sessionId, editingEnabled, refreshStatus]);

  useEffect(() => {
    if (!sessionId || !editingEnabled || !status.captureActive) return;
    const id = window.setInterval(() => void refreshStatus(), CAPTURE_POLL_MS);
    return () => window.clearInterval(id);
  }, [sessionId, editingEnabled, status.captureActive, refreshStatus]);

  const runCaptureCommand = useCallback(
    async (run: () => Promise<CaptureStatusResponse>) => {
      if (!sessionId || !editingEnabled) return;
      setBusy(true);
      try {
        const resp = await run();
        if (!resp.ok) {
          setLastError(resp.message ?? resp.error_code ?? "capture command failed");
          return;
        }
        setStatus(parseCaptureStatusFromResponse(resp));
        setLastError(null);
        await doPull();
        await refreshStatus();
      } catch (err) {
        setLastError(err instanceof Error ? err.message : "capture command error");
      } finally {
        setBusy(false);
      }
    },
    [sessionId, editingEnabled, doPull, setLastError, refreshStatus],
  );

  const startCaptureCommand = useCallback(async () => {
    if (!sessionId) return;
    await runCaptureCommand(() => startCapture(sessionId));
  }, [sessionId, runCaptureCommand]);

  const stopCaptureCommand = useCallback(async () => {
    if (!sessionId) return;
    await runCaptureCommand(() => stopCapture(sessionId));
  }, [sessionId, runCaptureCommand]);

  return {
    busy,
    summary: status,
    captureActive: status.captureActive,
    captureStatus: status.captureStatus,
    captureId: status.captureId,
    framesCount: status.framesCount,
    entitiesCount: status.entitiesCount,
    startedUtc: status.startedUtc,
    startCapture: startCaptureCommand,
    stopCapture: stopCaptureCommand,
    refreshCaptureStatus: refreshStatus,
  };
}
