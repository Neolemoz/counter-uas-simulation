import { useCallback, useState } from "react";
import { stopSession } from "@/bridge/client";
import { resetSession } from "@/bridge/lifecycleCommands";
import {
  assignTarget,
  cancelAssignment,
  pauseSim,
  resumeSim,
  spawnDefender,
} from "@/bridge/runtimeCommands";
import type { UiEntity } from "@/editing/localEntityMirror";
import type { Pose } from "@/world/bounds";

export function resolveSelectedDefenderId(
  selectedEntityId: string | null,
  entities: UiEntity[],
  tacticalInterceptorId: string | null | undefined,
): string | null {
  if (selectedEntityId) {
    const selected = entities.find((entity) => entity.entity_id === selectedEntityId);
    if (selected?.entity_type === "interceptor") return selectedEntityId;
  }
  return tacticalInterceptorId ?? null;
}

export function resolveSelectedTargetId(
  selectedEntityId: string | null,
  entities: UiEntity[],
  tacticalTargetId: string | null | undefined,
): string | null {
  if (tacticalTargetId) return tacticalTargetId;
  if (!selectedEntityId) return null;
  const selected = entities.find((entity) => entity.entity_id === selectedEntityId);
  if (selected?.entity_type === "drone") return selectedEntityId;
  return null;
}

export function useRuntimeControls({
  sessionId,
  editingEnabled,
  selectedDefenderId,
  selectedTargetId,
  doPull,
  setLastError,
}: {
  sessionId: string | null;
  editingEnabled: boolean;
  selectedDefenderId: string | null;
  selectedTargetId: string | null;
  doPull: () => Promise<void>;
  setLastError: (message: string | null) => void;
}) {
  const [busy, setBusy] = useState(false);

  const runRuntimeCommand = useCallback(
    async (run: () => Promise<{ ok?: boolean; message?: string; error_code?: string }>) => {
      if (!sessionId || !editingEnabled) return;
      setBusy(true);
      try {
        const resp = await run();
        if (!resp.ok) {
          setLastError(resp.message ?? resp.error_code ?? "runtime command failed");
          return;
        }
        setLastError(null);
        await doPull();
      } catch (err) {
        setLastError(err instanceof Error ? err.message : "runtime command error");
      } finally {
        setBusy(false);
      }
    },
    [sessionId, editingEnabled, doPull, setLastError],
  );

  const pauseSimCommand = useCallback(async () => {
    if (!sessionId) return;
    await runRuntimeCommand(() => pauseSim(sessionId));
  }, [sessionId, runRuntimeCommand]);

  const resumeSimCommand = useCallback(async () => {
    if (!sessionId) return;
    await runRuntimeCommand(() => resumeSim(sessionId));
  }, [sessionId, runRuntimeCommand]);

  const spawnDefenderCommand = useCallback(
    async (pose?: Pose) => {
      if (!sessionId) return;
      await runRuntimeCommand(() =>
        spawnDefender(sessionId, pose ? { pose } : {}),
      );
    },
    [sessionId, runRuntimeCommand],
  );

  const assignTargetCommand = useCallback(async () => {
    if (!sessionId || !selectedDefenderId || !selectedTargetId) return;
    await runRuntimeCommand(() =>
      assignTarget(sessionId, {
        defender_id: selectedDefenderId,
        target_id: selectedTargetId,
      }),
    );
  }, [sessionId, selectedDefenderId, selectedTargetId, runRuntimeCommand]);

  const cancelAssignmentCommand = useCallback(async () => {
    if (!sessionId || !selectedDefenderId) return;
    await runRuntimeCommand(() =>
      cancelAssignment(sessionId, { defender_id: selectedDefenderId }),
    );
  }, [sessionId, selectedDefenderId, runRuntimeCommand]);

  const resetSessionCommand = useCallback(async () => {
    if (!sessionId) return;
    await runRuntimeCommand(() => resetSession(sessionId));
  }, [sessionId, runRuntimeCommand]);

  const stopSessionCommand = useCallback(async () => {
    if (!sessionId) return;
    await runRuntimeCommand(() => stopSession(sessionId));
  }, [sessionId, runRuntimeCommand]);

  return {
    busy,
    pauseSim: pauseSimCommand,
    resumeSim: resumeSimCommand,
    resetSession: resetSessionCommand,
    stopSession: stopSessionCommand,
    spawnDefender: spawnDefenderCommand,
    assignTarget: assignTargetCommand,
    cancelAssignment: cancelAssignmentCommand,
  };
}
