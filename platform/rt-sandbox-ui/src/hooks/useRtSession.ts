import { useCallback, useEffect, useState } from "react";
import {
  pullTelemetry,
  startSession,
  stopSession,
  subscribeTelemetry,
  unsubscribeTelemetry,
} from "@/bridge/client";
import {
  mergeChannelSnapshots,
  type ChannelSnapshot,
} from "@/telemetry/channelIndex";
import type { TelemetryChannel } from "@/telemetry/constants";
import { DEFAULT_PULL_HZ } from "@/telemetry/constants";

export function useRtSession() {
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [subscriptionId, setSubscriptionId] = useState<string | null>(null);
  const [connected, setConnected] = useState(false);
  const [busy, setBusy] = useState(false);
  const [pulling, setPulling] = useState(false);
  const [autoRefresh, setAutoRefresh] = useState(true);
  const [pullHz, setPullHz] = useState(DEFAULT_PULL_HZ);
  const [snapshots, setSnapshots] = useState<
    Partial<Record<TelemetryChannel, ChannelSnapshot>>
  >({});
  const [lastPullUtc, setLastPullUtc] = useState<string | null>(null);
  const [drainedCount, setDrainedCount] = useState(0);
  const [lastError, setLastError] = useState<string | null>(null);

  const doPull = useCallback(async () => {
    if (!sessionId || !subscriptionId) return;
    setPulling(true);
    try {
      const result = await pullTelemetry({ sessionId, subscriptionId });
      if (!result.ok) {
        setLastError(result.error_code ?? "PULL_FAILED");
        return;
      }
      setLastError(null);
      setLastPullUtc(new Date().toISOString());
      setDrainedCount(result.drained_count ?? 0);
      const events = (result.events ?? []).map((ev) => ({
        channel: ev.channel,
        session_id: ev.session_id,
        timestamp_utc: ev.timestamp_utc,
        payload: ev.payload ?? {},
        governance_banner: ev.governance_banner,
      }));
      setSnapshots((prev) => mergeChannelSnapshots(prev, events));
    } catch (err) {
      setLastError(err instanceof Error ? err.message : "pull error");
    } finally {
      setPulling(false);
    }
  }, [sessionId, subscriptionId]);

  const handleConnect = useCallback(async () => {
    setBusy(true);
    setLastError(null);
    try {
      const started = await startSession();
      if (!started.ok || !started.session_id) {
        setLastError(started.error_code ?? started.message ?? "start failed");
        return;
      }
      const sid = started.session_id;
      const sub = await subscribeTelemetry(sid);
      if (!sub.ok || !sub.subscription_id) {
        await stopSession(sid);
        setLastError(sub.error_code ?? sub.message ?? "subscribe failed");
        return;
      }
      setSessionId(sid);
      setSubscriptionId(sub.subscription_id);
      setConnected(true);
      if (sub.initial_events) {
        const events = (sub.initial_events as Array<Record<string, unknown>>).map(
          (ev) => ({
            channel: String(ev.channel ?? ""),
            session_id: String(ev.session_id ?? sid),
            timestamp_utc: String(ev.timestamp_utc ?? ""),
            payload: (ev.payload as Record<string, unknown>) ?? {},
            governance_banner:
              typeof ev.governance_banner === "string"
                ? ev.governance_banner
                : undefined,
          }),
        );
        setSnapshots(mergeChannelSnapshots({}, events));
      }
    } catch (err) {
      setLastError(err instanceof Error ? err.message : "connect error");
    } finally {
      setBusy(false);
    }
  }, []);

  const handleDisconnect = useCallback(async () => {
    if (!sessionId) return;
    setBusy(true);
    try {
      if (subscriptionId) {
        await unsubscribeTelemetry(sessionId);
      }
      await stopSession(sessionId);
    } catch {
      /* best-effort */
    } finally {
      setSessionId(null);
      setSubscriptionId(null);
      setConnected(false);
      setSnapshots({});
      setBusy(false);
    }
  }, [sessionId, subscriptionId]);

  const resetSnapshots = useCallback(() => {
    setSnapshots({});
  }, []);

  useEffect(() => {
    if (!connected || !autoRefresh || !sessionId || !subscriptionId) return;
    const intervalMs = Math.max(100, Math.floor(1000 / pullHz));
    const id = window.setInterval(() => {
      void doPull();
    }, intervalMs);
    return () => window.clearInterval(id);
  }, [connected, autoRefresh, sessionId, subscriptionId, pullHz, doPull]);

  useEffect(() => {
    return () => {
      if (sessionId) {
        void unsubscribeTelemetry(sessionId).catch(() => undefined);
        void stopSession(sessionId).catch(() => undefined);
      }
    };
  }, [sessionId]);

  return {
    sessionId,
    subscriptionId,
    connected,
    busy,
    pulling,
    autoRefresh,
    setAutoRefresh,
    pullHz,
    setPullHz,
    snapshots,
    setSnapshots,
    lastPullUtc,
    drainedCount,
    lastError,
    setLastError,
    doPull,
    handleConnect,
    handleDisconnect,
    resetSnapshots,
  };
}
