import { useCallback, useMemo, useState } from "react";
import {
  approveRecommendation,
  assignCandidate,
  clearAssignment,
  getTacticalState,
  parseTacticalRecommendation,
  parseTacticalState,
  pauseAutonomousLoop,
  rejectRecommendation,
  requestRecommendation,
  resumeAutonomousLoop,
  selectCandidate,
  setTacticalMode,
  type TacticalMode,
  type TacticalRecommendationPayload,
  type TacticalStatePayload,
} from "@/bridge/tacticalCommands";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";

export function tacticalStateFromSnapshot(
  snapshot: ChannelSnapshot | undefined,
): TacticalStatePayload | null {
  if (!snapshot?.payload) return null;
  if (snapshot.payload.schema === "rt_tactical_state_v1") {
    return snapshot.payload as TacticalStatePayload;
  }
  return null;
}

export function tacticalRecommendationFromSnapshot(
  snapshot: ChannelSnapshot | undefined,
): TacticalRecommendationPayload | null {
  if (!snapshot?.payload) return null;
  if (snapshot.payload.schema === "rt_tactical_recommendation_v1") {
    return snapshot.payload as TacticalRecommendationPayload;
  }
  return null;
}

function resolveMode(state: TacticalStatePayload | null): TacticalMode {
  const m = state?.tactical_mode;
  if (m === "assisted" || m === "autonomous") return m;
  return "manual";
}

export function useTacticalState(
  sessionId: string | null,
  telemetryStateSnapshot: ChannelSnapshot | undefined,
  telemetryRecommendationSnapshot: ChannelSnapshot | undefined,
  editingEnabled: boolean,
) {
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [localState, setLocalState] = useState<TacticalStatePayload | null>(null);
  const [localRecommendation, setLocalRecommendation] =
    useState<TacticalRecommendationPayload | null>(null);
  const [targetPickActive, setTargetPickActive] = useState(false);

  const merged = useMemo(
    () => localState ?? tacticalStateFromSnapshot(telemetryStateSnapshot),
    [localState, telemetryStateSnapshot],
  );

  const recommendation = useMemo(
    () =>
      localRecommendation ??
      tacticalRecommendationFromSnapshot(telemetryRecommendationSnapshot),
    [localRecommendation, telemetryRecommendationSnapshot],
  );

  const mode = useMemo(() => resolveMode(merged), [merged]);

  const applyResponse = useCallback((resp: { ok?: boolean; message?: string }) => {
    if (!resp.ok) {
      setError(resp.message ?? "tactical command failed");
      return false;
    }
    setError(null);
    return true;
  }, []);

  const applyTacticalResponse = useCallback(
    (resp: Parameters<typeof parseTacticalState>[0]) => {
      if (!applyResponse(resp)) return false;
      const state = parseTacticalState(resp);
      if (state) setLocalState(state);
      const rec = parseTacticalRecommendation(resp);
      if (rec) setLocalRecommendation(rec);
      return true;
    },
    [applyResponse],
  );

  const refresh = useCallback(async () => {
    if (!sessionId) return;
    setBusy(true);
    try {
      const resp = await getTacticalState(sessionId);
      if (resp.ok) {
        setLocalState(parseTacticalState(resp));
        setLocalRecommendation(parseTacticalRecommendation(resp));
        setError(null);
      } else {
        setError(resp.message ?? "get_tactical_state failed");
      }
    } finally {
      setBusy(false);
    }
  }, [sessionId]);

  const setMode = useCallback(
    async (next: TacticalMode) => {
      if (!sessionId || !editingEnabled) return;
      if (next === "autonomous" && mode === "assisted") {
        setError("Switch to Manual before Autonomous mode");
        return;
      }
      setBusy(true);
      try {
        const resp = await setTacticalMode(sessionId, next);
        applyTacticalResponse(resp);
      } finally {
        setBusy(false);
      }
    },
    [sessionId, editingEnabled, mode, applyTacticalResponse],
  );

  const selectRole = useCallback(
    async (role: "interceptor" | "target", entityId: string) => {
      if (!sessionId || !editingEnabled || mode === "autonomous") return;
      setBusy(true);
      try {
        const resp = await selectCandidate(sessionId, role, entityId);
        applyTacticalResponse(resp);
        if (role === "target") {
          setTargetPickActive(false);
        }
      } finally {
        setBusy(false);
      }
    },
    [sessionId, editingEnabled, mode, applyTacticalResponse],
  );

  const assign = useCallback(async () => {
    if (!sessionId || !editingEnabled || mode !== "manual") return;
    setBusy(true);
    try {
      const resp = await assignCandidate(sessionId);
      applyTacticalResponse(resp);
    } finally {
      setBusy(false);
    }
  }, [sessionId, editingEnabled, mode, applyTacticalResponse]);

  const clear = useCallback(async () => {
    if (!sessionId || !editingEnabled || mode === "autonomous") return;
    setBusy(true);
    try {
      const resp = await clearAssignment(sessionId);
      applyTacticalResponse(resp);
    } finally {
      setBusy(false);
    }
  }, [sessionId, editingEnabled, mode, applyTacticalResponse]);

  const requestRec = useCallback(async () => {
    if (!sessionId || !editingEnabled || mode !== "assisted") return;
    setBusy(true);
    try {
      const resp = await requestRecommendation(sessionId, {
        interceptor_id: merged?.selected_interceptor_id ?? undefined,
        target_id: merged?.selected_target_id ?? undefined,
      });
      applyTacticalResponse(resp);
    } finally {
      setBusy(false);
    }
  }, [sessionId, editingEnabled, mode, merged, applyTacticalResponse]);

  const approveRec = useCallback(async () => {
    const recId = recommendation?.recommendation_id;
    if (!sessionId || !editingEnabled || !recId) return;
    setBusy(true);
    try {
      const resp = await approveRecommendation(sessionId, recId);
      applyTacticalResponse(resp);
    } finally {
      setBusy(false);
    }
  }, [sessionId, editingEnabled, recommendation, applyTacticalResponse]);

  const rejectRec = useCallback(async () => {
    const recId = recommendation?.recommendation_id;
    if (!sessionId || !editingEnabled) return;
    setBusy(true);
    try {
      const resp = await rejectRecommendation(sessionId, recId ?? undefined);
      applyTacticalResponse(resp);
    } finally {
      setBusy(false);
    }
  }, [sessionId, editingEnabled, recommendation, applyTacticalResponse]);

  const pauseLoop = useCallback(async () => {
    if (!sessionId || !editingEnabled || mode !== "autonomous") return;
    setBusy(true);
    try {
      const resp = await pauseAutonomousLoop(sessionId);
      applyTacticalResponse(resp);
    } finally {
      setBusy(false);
    }
  }, [sessionId, editingEnabled, mode, applyTacticalResponse]);

  const resumeLoop = useCallback(async () => {
    if (!sessionId || !editingEnabled || mode !== "autonomous") return;
    setBusy(true);
    try {
      const resp = await resumeAutonomousLoop(sessionId);
      applyTacticalResponse(resp);
    } finally {
      setBusy(false);
    }
  }, [sessionId, editingEnabled, mode, applyTacticalResponse]);

  const returnToManual = useCallback(async () => {
    await setMode("manual");
  }, [setMode]);

  return {
    state: merged,
    recommendation,
    mode,
    busy,
    error,
    targetPickActive,
    setTargetPickActive,
    setMode,
    selectRole,
    assign,
    clear,
    requestRec,
    approveRec,
    rejectRec,
    pauseLoop,
    resumeLoop,
    returnToManual,
    refresh,
  };
}
