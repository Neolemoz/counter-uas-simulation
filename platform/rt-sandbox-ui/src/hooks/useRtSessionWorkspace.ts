import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  discardSession,
  listSessions,
  pullTelemetry,
  setEditingSession,
  startSession,
  subscribeTelemetry,
  unsubscribeTelemetry,
} from "@/bridge/client";
import {
  mergeChannelSnapshots,
  sessionStateFromSnapshots,
  type ChannelSnapshot,
} from "@/telemetry/channelIndex";
import {
  BACKGROUND_PULL_HZ,
  DEFAULT_PULL_HZ,
  DIAGNOSTIC_TELEMETRY_CHANNELS,
  MAX_PULL_HZ,
  TELEMETRY_CHANNELS,
  type TelemetryChannel,
} from "@/telemetry/constants";

export type SessionSlotRole = "active" | "background";

export type SessionSlot = {
  sessionId: string;
  subscriptionId: string | null;
  role: SessionSlotRole;
  snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>;
  lastError: string | null;
  connected: boolean;
  lastPullUtc: string | null;
  drainedCount: number;
  pulling: boolean;
};

const MAX_SESSIONS = 3;

export type UseRtSessionWorkspaceOptions = {
  pauseBackgroundPoll?: boolean;
};

/** Whether auto-refresh should pull this slot (testable poll-policy helper). */
export function shouldPullSlotInAutoRefresh(
  role: SessionSlotRole,
  pauseBackgroundPoll: boolean,
): boolean {
  if (role === "background" && pauseBackgroundPoll) return false;
  return true;
}

function emptySlot(sessionId: string, role: SessionSlotRole): SessionSlot {
  return {
    sessionId,
    subscriptionId: null,
    role,
    snapshots: {},
    lastError: null,
    connected: false,
    lastPullUtc: null,
    drainedCount: 0,
    pulling: false,
  };
}

export function useRtSessionWorkspace(options: UseRtSessionWorkspaceOptions = {}) {
  const { pauseBackgroundPoll = false } = options;
  const [slots, setSlots] = useState<Map<string, SessionSlot>>(() => new Map());
  const [selectedSessionId, setSelectedSessionId] = useState<string | null>(null);
  const [editingSessionId, setEditingSessionId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [autoRefresh, setAutoRefresh] = useState(true);
  const [pullHz, setPullHz] = useState(DEFAULT_PULL_HZ);
  const [lastError, setLastError] = useState<string | null>(null);
  const slotsRef = useRef(slots);
  slotsRef.current = slots;

  const connectedCount = useMemo(
    () => [...slots.values()].filter((s) => s.connected).length,
    [slots],
  );
  const connected = connectedCount > 0;
  const atCapacity = connectedCount >= MAX_SESSIONS;

  const activeSlot = selectedSessionId ? slots.get(selectedSessionId) : undefined;
  const sessionId = selectedSessionId;
  const subscriptionId = activeSlot?.subscriptionId ?? null;
  const snapshots = activeSlot?.snapshots ?? {};

  const updateSlot = useCallback(
    (sessionId: string, patch: Partial<SessionSlot>) => {
      setSlots((prev) => {
        const next = new Map(prev);
        const current = next.get(sessionId) ?? emptySlot(sessionId, "background");
        next.set(sessionId, { ...current, ...patch });
        return next;
      });
    },
    [],
  );

  const pullSlot = useCallback(
    async (sessionId: string) => {
      const slot = slotsRef.current.get(sessionId);
      if (!slot?.subscriptionId) return;
      updateSlot(sessionId, { pulling: true });
      try {
        const result = await pullTelemetry({
          sessionId,
          subscriptionId: slot.subscriptionId,
        });
        if (!result.ok) {
          updateSlot(sessionId, {
            lastError: result.error_code ?? "PULL_FAILED",
          });
          return;
        }
        const events = (result.events ?? []).map((ev) => ({
          channel: ev.channel,
          session_id: ev.session_id,
          timestamp_utc: ev.timestamp_utc,
          payload: ev.payload ?? {},
          governance_banner: ev.governance_banner,
        }));
        setSlots((prev) => {
          const next = new Map(prev);
          const current = next.get(sessionId) ?? emptySlot(sessionId, "background");
          next.set(sessionId, {
            ...current,
            lastError: null,
            lastPullUtc: new Date().toISOString(),
            drainedCount: result.drained_count ?? 0,
            snapshots: mergeChannelSnapshots(current.snapshots, events),
          });
          return next;
        });
      } catch (err) {
        updateSlot(sessionId, {
          lastError: err instanceof Error ? err.message : "pull error",
        });
      } finally {
        updateSlot(sessionId, { pulling: false });
      }
    },
    [updateSlot],
  );

  const connectNewSession = useCallback(async () => {
    if (atCapacity) {
      setLastError("SESSION_CAPACITY_EXCEEDED");
      return;
    }
    setBusy(true);
    setLastError(null);
    try {
      const started = await startSession();
      if (!started.ok || !started.session_id) {
        setLastError(started.error_code ?? started.message ?? "start failed");
        return;
      }
      const sid = started.session_id;
      const isFirst = slotsRef.current.size === 0;
      const role: SessionSlotRole = isFirst ? "active" : "background";
      const channels = isFirst ? TELEMETRY_CHANNELS : DIAGNOSTIC_TELEMETRY_CHANNELS;
      const sub = await subscribeTelemetry(sid, channels);
      if (!sub.ok || !sub.subscription_id) {
        await discardSession(sid);
        setLastError(sub.error_code ?? sub.message ?? "subscribe failed");
        return;
      }
      let initialSnapshots: SessionSlot["snapshots"] = {};
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
        initialSnapshots = mergeChannelSnapshots({}, events);
      }
      setSlots((prev) => {
        const next = new Map(prev);
        for (const [id, slot] of next) {
          if (slot.role === "active" && !isFirst) {
            next.set(id, { ...slot, role: "background" });
          }
        }
        next.set(sid, {
          sessionId: sid,
          subscriptionId: sub.subscription_id ?? null,
          role,
          snapshots: initialSnapshots,
          lastError: null,
          connected: true,
          lastPullUtc: null,
          drainedCount: 0,
          pulling: false,
        });
        return next;
      });
      setSelectedSessionId(sid);
      const editResult = await setEditingSession(sid);
      if (editResult.ok && editResult.editing_session_id) {
        setEditingSessionId(String(editResult.editing_session_id));
      } else {
        setEditingSessionId(sid);
      }
    } catch (err) {
      setLastError(err instanceof Error ? err.message : "connect error");
    } finally {
      setBusy(false);
    }
  }, [atCapacity]);

  const disconnectSession = useCallback(
    async (targetId: string) => {
      setBusy(true);
      try {
        await unsubscribeTelemetry(targetId).catch(() => undefined);
        await discardSession(targetId).catch(() => undefined);
      } finally {
        setSlots((prev) => {
          const next = new Map(prev);
          next.delete(targetId);
          return next;
        });
        if (selectedSessionId === targetId) {
          const remaining = [...slotsRef.current.keys()].filter((id) => id !== targetId);
          setSelectedSessionId(remaining[0] ?? null);
        }
        if (editingSessionId === targetId) {
          const remaining = [...slotsRef.current.keys()].filter((id) => id !== targetId);
          setEditingSessionId(remaining[0] ?? null);
        }
        setBusy(false);
      }
    },
    [selectedSessionId, editingSessionId],
  );

  const selectTab = useCallback(
    async (targetId: string) => {
      if (targetId === selectedSessionId) return;
      setBusy(true);
      try {
        const editResult = await setEditingSession(targetId);
        if (editResult.ok) {
          setEditingSessionId(targetId);
        }
        const activeSub = await subscribeTelemetry(targetId, TELEMETRY_CHANNELS);
        for (const [id, slot] of slotsRef.current) {
          if (id === targetId || !slot.connected) continue;
          await subscribeTelemetry(id, DIAGNOSTIC_TELEMETRY_CHANNELS);
        }
        setSlots((prev) => {
          const next = new Map(prev);
          for (const [id, slot] of next) {
            const role: SessionSlotRole = id === targetId ? "active" : "background";
            const patch: Partial<SessionSlot> = { role };
            if (id === targetId && activeSub.subscription_id) {
              patch.subscriptionId = activeSub.subscription_id;
            }
            next.set(id, { ...slot, ...patch });
          }
          return next;
        });
        setSelectedSessionId(targetId);
      } catch (err) {
        setLastError(err instanceof Error ? err.message : "tab switch error");
      } finally {
        setBusy(false);
      }
    },
    [selectedSessionId],
  );

  const doPull = useCallback(async () => {
    if (selectedSessionId) {
      await pullSlot(selectedSessionId);
    }
  }, [pullSlot, selectedSessionId]);

  const resetSnapshots = useCallback(() => {
    if (!selectedSessionId) return;
    updateSlot(selectedSessionId, { snapshots: {} });
  }, [selectedSessionId, updateSlot]);

  useEffect(() => {
    if (!autoRefresh) return;
    const intervalMs = Math.max(100, Math.floor(1000 / Math.min(pullHz, MAX_PULL_HZ)));
    const id = window.setInterval(() => {
      for (const slot of slotsRef.current.values()) {
        if (!slot.connected || !slot.subscriptionId) continue;
        if (!shouldPullSlotInAutoRefresh(slot.role, pauseBackgroundPoll)) continue;
        const hz =
          slot.role === "active"
            ? Math.min(pullHz, MAX_PULL_HZ)
            : BACKGROUND_PULL_HZ;
        const slotInterval = Math.max(100, Math.floor(1000 / hz));
        const last = slot.lastPullUtc ? Date.parse(slot.lastPullUtc) : 0;
        if (Date.now() - last >= slotInterval - 50) {
          void pullSlot(slot.sessionId);
        }
      }
    }, intervalMs);
    return () => window.clearInterval(id);
  }, [autoRefresh, pullHz, pullSlot, slots.size, pauseBackgroundPoll]);

  useEffect(() => {
    return () => {
      for (const slot of slotsRef.current.values()) {
        void unsubscribeTelemetry(slot.sessionId).catch(() => undefined);
        void discardSession(slot.sessionId).catch(() => undefined);
      }
    };
  }, []);

  const backgroundSlots = useMemo(
    () =>
      [...slots.values()].filter(
        (s) => s.connected && s.sessionId !== selectedSessionId,
      ),
    [slots, selectedSessionId],
  );

  const refreshRegistry = useCallback(async () => {
    const listed = await listSessions();
    if (listed.ok && typeof listed.non_terminal_count === "number") {
      return listed;
    }
    return listed;
  }, []);

  return {
    slots,
    sessionId,
    subscriptionId,
    selectedSessionId,
    editingSessionId,
    connected,
    connectedCount,
    atCapacity,
    busy,
    pulling: activeSlot?.pulling ?? false,
    autoRefresh,
    setAutoRefresh,
    pullHz,
    setPullHz,
    snapshots,
    lastPullUtc: activeSlot?.lastPullUtc ?? null,
    drainedCount: activeSlot?.drainedCount ?? 0,
    lastError: lastError ?? activeSlot?.lastError ?? null,
    setLastError,
    doPull,
    connectNewSession,
    disconnectSession,
    selectTab,
    resetSnapshots,
    backgroundSlots,
    refreshRegistry,
    sessionStateFromSnapshots: (sid: string) =>
      sessionStateFromSnapshots(slots.get(sid)?.snapshots ?? {}),
  };
}
