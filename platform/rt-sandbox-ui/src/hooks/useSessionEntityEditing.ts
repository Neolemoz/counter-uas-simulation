import { useCallback, useEffect, useRef, useState } from "react";
import {
  deleteEntity,
  moveEntity,
  spawnEntity,
} from "@/bridge/entityCommands";
import {
  applyLocalEntityCommand,
  countEntitiesByType,
  mergeTelemetryAndLocalEntities,
  pruneLocalEntities,
  type LocalEntityMap,
  type UiEntity,
} from "@/editing/localEntityMirror";
import {
  appendEditHistory,
  createEditHistoryEntry,
  type EditCommandType,
  type EditHistoryEntry,
} from "@/editing/editHistory";
import { entitiesFromSnapshot } from "@/telemetry/channelIndex";
import {
  canSpawn,
  clampPose,
  COMMAND_BURST_INTERVAL_MS,
  isEditingAllowed,
  type Pose,
} from "@/world/bounds";
import { defaultPose, type EntityType } from "@/world/entityCatalog";
import { shouldClearPendingReconcile } from "@/sync/cognition";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";

export type SessionEditState = {
  editHistory: EditHistoryEntry[];
  localEntities: LocalEntityMap;
  locallyDeletedIds: Set<string>;
  selectedEntityId: string | null;
};

export function emptyEditState(): SessionEditState {
  return {
    editHistory: [],
    localEntities: {},
    locallyDeletedIds: new Set(),
    selectedEntityId: null,
  };
}

export type UseSessionEntityEditingParams = {
  sessionId: string | null;
  selectedSessionId: string | null;
  editingSessionId: string | null;
  connected: boolean;
  sessionState: string;
  entityPoseMirror: ChannelSnapshot | undefined;
  worldSummary: Record<string, unknown> | undefined;
  lastPullUtc: string | null;
  workspaceSessionIds: string[];
  selectedType: EntityType;
  doPull: () => Promise<void>;
  setLastError: (message: string | null) => void;
};

export function useSessionEntityEditing({
  sessionId,
  selectedSessionId,
  editingSessionId,
  connected,
  sessionState,
  entityPoseMirror,
  worldSummary,
  lastPullUtc,
  workspaceSessionIds,
  selectedType,
  doPull,
  setLastError,
}: UseSessionEntityEditingParams) {
  const [editBySession, setEditBySession] = useState<
    Record<string, SessionEditState>
  >({});
  const [lastCommand, setLastCommand] = useState<{
    type: EditCommandType;
    ok: boolean;
    errorCode?: string;
    message?: string;
  }>();
  const [pendingReconcile, setPendingReconcile] = useState(false);
  const [commandBusy, setCommandBusy] = useState(false);
  const lastCommandMs = useRef(0);

  const sessionEdit = sessionId
    ? (editBySession[sessionId] ?? emptyEditState())
    : emptyEditState();
  const {
    editHistory,
    localEntities,
    locallyDeletedIds,
    selectedEntityId,
  } = sessionEdit;

  const patchSessionEdit = useCallback(
    (sid: string, patch: Partial<SessionEditState>) => {
      setEditBySession((prev) => ({
        ...prev,
        [sid]: { ...(prev[sid] ?? emptyEditState()), ...patch },
      }));
    },
    [],
  );

  const telemetryEntities: UiEntity[] = entitiesFromSnapshot(entityPoseMirror).map(
    (e) => ({
      entity_id: String(e.entity_id ?? ""),
      entity_type: String(e.entity_type ?? ""),
      pose: (e.pose as Record<string, unknown>) ?? {},
    }),
  );
  const entities = mergeTelemetryAndLocalEntities(
    telemetryEntities,
    localEntities,
    locallyDeletedIds,
  );
  const mergedEntityCounts = countEntitiesByType(entities);
  const mergedWorldSummary = {
    ...(worldSummary ?? {}),
    entity_count: entities.length,
    by_type: mergedEntityCounts,
  };

  const editingEnabled =
    connected &&
    sessionId === editingSessionId &&
    sessionId === selectedSessionId &&
    isEditingAllowed(sessionState) &&
    !commandBusy;

  const runEntityCommand = useCallback(
    async (
      commandType: EditCommandType,
      run: () => Promise<
        Record<string, unknown> & {
          ok: boolean;
          error_code?: string;
          message?: string;
          entity_id?: string;
        }
      >,
      meta: { entityId?: string; entityType?: string; pose?: Pose },
    ) => {
      if (!sessionId) return;
      const now = Date.now();
      if (now - lastCommandMs.current < COMMAND_BURST_INTERVAL_MS) {
        setLastError("Command rate limited — wait before next edit");
        return;
      }
      lastCommandMs.current = now;
      setCommandBusy(true);
      setPendingReconcile(true);
      try {
        const result = await run();
        setLastCommand({
          type: commandType,
          ok: result.ok,
          errorCode: result.error_code,
          message: result.message,
        });
        const entry = createEditHistoryEntry({
          commandType,
          ok: result.ok,
          errorCode: result.error_code,
          message: result.message,
          entityId: meta.entityId ?? result.entity_id,
          entityType: meta.entityType,
          pose: meta.pose,
        });
        const current = editBySession[sessionId] ?? emptyEditState();
        patchSessionEdit(sessionId, {
          editHistory: appendEditHistory(current.editHistory, entry),
        });
        if (!result.ok) {
          setLastError(result.error_code ?? result.message ?? "command failed");
          setPendingReconcile(false);
          return;
        }
        setLastError(null);
        patchSessionEdit(sessionId, {
          localEntities: applyLocalEntityCommand(
            current.localEntities,
            commandType,
            result,
            meta,
          ),
        });
        if (commandType === "delete_entity" && meta.entityId) {
          const deleted = new Set(current.locallyDeletedIds).add(meta.entityId);
          patchSessionEdit(sessionId, {
            locallyDeletedIds: deleted,
            selectedEntityId: null,
          });
        } else if (commandType === "spawn_entity" && result.entity_id) {
          const entityId = String(result.entity_id);
          const deleted = new Set(current.locallyDeletedIds);
          deleted.delete(entityId);
          patchSessionEdit(sessionId, {
            locallyDeletedIds: deleted,
            selectedEntityId: entityId,
          });
        }
        await doPull();
        setPendingReconcile(false);
      } catch (err) {
        setLastError(err instanceof Error ? err.message : "command error");
        setPendingReconcile(false);
      } finally {
        setCommandBusy(false);
      }
    },
    [sessionId, doPull, setLastError, editBySession, patchSessionEdit],
  );

  const handleSpawn = useCallback(
    (pose: Pose, entityType = selectedType) => {
      if (!sessionId) return;
      const check = canSpawn(mergedWorldSummary, entityType);
      if (!check.ok) {
        setLastError(check.reason ?? "spawn blocked");
        return;
      }
      const clamped = clampPose(defaultPose(entityType, pose.x, pose.y));
      void runEntityCommand(
        "spawn_entity",
        () => spawnEntity(sessionId, { entity_type: entityType, pose: clamped }),
        { entityType, pose: clamped },
      );
    },
    [sessionId, selectedType, mergedWorldSummary, setLastError, runEntityCommand],
  );

  const handleMove = useCallback(
    (entityId: string, pose: Pose) => {
      if (!sessionId) return;
      const clamped = clampPose(pose);
      void runEntityCommand(
        "move_entity",
        () => moveEntity(sessionId, { entity_id: entityId, pose: clamped }),
        { entityId, pose: clamped },
      );
    },
    [sessionId, runEntityCommand],
  );

  const handleDelete = useCallback(
    (entityId: string) => {
      if (!sessionId) return;
      void runEntityCommand(
        "delete_entity",
        () => deleteEntity(sessionId, entityId),
        { entityId },
      );
    },
    [sessionId, runEntityCommand],
  );

  useEffect(() => {
    if (!sessionId) return;
    const current = editBySession[sessionId] ?? emptyEditState();
    setEditBySession((prev) => ({
      ...prev,
      [sessionId]: {
        ...current,
        localEntities: pruneLocalEntities(current.localEntities, telemetryEntities),
        locallyDeletedIds: (() => {
          if (current.locallyDeletedIds.size === 0) return current.locallyDeletedIds;
          const telemetryIds = new Set(
            telemetryEntities.map((entity) => entity.entity_id),
          );
          const next = new Set<string>();
          for (const entityId of current.locallyDeletedIds) {
            if (telemetryIds.has(entityId)) next.add(entityId);
          }
          return next.size === current.locallyDeletedIds.size
            ? current.locallyDeletedIds
            : next;
        })(),
      },
    }));
  }, [entityPoseMirror, sessionId]);

  useEffect(() => {
    if (!pendingReconcile) return;
    if (
      shouldClearPendingReconcile(
        pendingReconcile,
        worldSummary,
        worldSummary?.last_command_utc as string | undefined,
      )
    ) {
      setPendingReconcile(false);
    }
  }, [pendingReconcile, worldSummary, lastPullUtc]);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (
        e.key === "Delete" &&
        selectedEntityId &&
        sessionId &&
        connected &&
        editingEnabled
      ) {
        handleDelete(selectedEntityId);
      }
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [selectedEntityId, sessionId, connected, editingEnabled, handleDelete]);

  useEffect(() => {
    const activeIds = new Set(workspaceSessionIds);
    setEditBySession((prev) => {
      let changed = false;
      const next = { ...prev };
      for (const sid of Object.keys(next)) {
        if (!activeIds.has(sid)) {
          delete next[sid];
          changed = true;
        }
      }
      return changed ? next : prev;
    });
  }, [workspaceSessionIds]);

  return {
    editBySession,
    editHistory,
    selectedEntityId,
    entities,
    mergedWorldSummary,
    mergedEntityCounts,
    editingEnabled,
    lastCommand,
    pendingReconcile,
    commandBusy,
    handleSpawn,
    handleMove,
    handleDelete,
    patchSessionEdit,
  };
}
