import { useCallback, useEffect, useRef, useState } from "react";
import {
  deleteEntity,
  moveEntity,
  spawnEntity,
} from "@/bridge/entityCommands";
import { CaptureHandoffWorkflowPanel } from "@/components/CaptureHandoffWorkflowPanel";
import { TacticalAssistedPanel } from "@/components/TacticalAssistedPanel";
import { TacticalAutonomousPanel } from "@/components/TacticalAutonomousPanel";
import { TacticalManualPanel } from "@/components/TacticalManualPanel";
import { GovernanceChrome } from "@/components/GovernanceChrome";
import { EditHistoryPanel } from "@/components/EditHistoryPanel";
import { EditingCognitionStrip } from "@/components/EditingCognitionStrip";
import { EntityPalette } from "@/components/EntityPalette";
import { EntityPoseMirrorPanel } from "@/components/EntityPoseMirrorPanel";
import {
  BridgeConnectionBar,
  CollapsibleUiDiagnostics,
  RefreshControls,
} from "@/components/RefreshControls";
import {
  ClockMirrorPanel,
  SessionHealthPanel,
  SessionLifecyclePanel,
  WorldSummaryPanel,
} from "@/components/TelemetryPanels";
import {
  anyTerrainLayerEnabled,
  DEFAULT_TERRAIN_LAYERS,
} from "@/cesium/terrainLayers";
import { CesiumRuntimePanel } from "@/components/CesiumRuntimePanel";
import { WorldEditingGrid } from "@/components/WorldEditingGrid";
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
import { hasUnsyncedLocalMirror } from "@/editing/sessionMirrorDirty";
import { useCaptureHandoffMirror } from "@/hooks/useCaptureHandoffMirror";
import { useRtSessionWorkspace } from "@/hooks/useRtSessionWorkspace";
import { useSessionDisplayNames } from "@/hooks/useSessionDisplayNames";
import { useSessionTabOrder } from "@/hooks/useSessionTabOrder";
import { useTacticalState } from "@/hooks/useTacticalState";
import {
  entitiesFromSnapshot,
  sessionStateFromSnapshots,
} from "@/telemetry/channelIndex";
import {
  canSpawn,
  clampPose,
  COMMAND_BURST_INTERVAL_MS,
  isEditingAllowed,
  type Pose,
} from "@/world/bounds";
import { defaultPose, type EntityType } from "@/world/entityCatalog";
import { shouldClearPendingReconcile } from "@/sync/cognition";
import { BackgroundDiagnostics } from "@/workstation/BackgroundDiagnostics";
import { ConnectPlaceholder } from "@/workstation/ConnectPlaceholder";
import { MirrorsIdleCard } from "@/workstation/MirrorsIdleCard";
import { RuntimeCognitionHub } from "@/workstation/RuntimeCognitionHub";
import { ExperimentWorkbenchPanel } from "@/experiment/ExperimentWorkbenchPanel";
import { RuntimeWorkstationShell } from "@/workstation/RuntimeWorkstationShell";
import { SessionTabBar } from "@/workstation/SessionTabBar";
import { SessionWorkflowStrip } from "@/workstation/SessionWorkflowStrip";

type SessionEditState = {
  editHistory: EditHistoryEntry[];
  localEntities: LocalEntityMap;
  locallyDeletedIds: Set<string>;
  selectedEntityId: string | null;
};

function emptyEditState(): SessionEditState {
  return {
    editHistory: [],
    localEntities: {},
    locallyDeletedIds: new Set(),
    selectedEntityId: null,
  };
}

export default function App() {
  const [backgroundDiagOpen, setBackgroundDiagOpen] = useState(false);
  const { labelFor, renameSession } = useSessionDisplayNames();
  const {
    slots,
    sessionId,
    subscriptionId,
    selectedSessionId,
    editingSessionId,
    connected,
    connectedCount,
    atCapacity,
    busy,
    pulling,
    autoRefresh,
    setAutoRefresh,
    pullHz,
    setPullHz,
    snapshots,
    lastPullUtc,
    drainedCount,
    lastError,
    setLastError,
    doPull,
    connectNewSession,
    disconnectSession,
    selectTab,
    backgroundSlots,
  } = useRtSessionWorkspace({ pauseBackgroundPoll: !backgroundDiagOpen });

  const [editBySession, setEditBySession] = useState<
    Record<string, SessionEditState>
  >({});
  const [selectedType, setSelectedType] = useState<EntityType>("drone");
  const [lastCommand, setLastCommand] = useState<{
    type: EditCommandType;
    ok: boolean;
    errorCode?: string;
    message?: string;
  }>();
  const [pendingReconcile, setPendingReconcile] = useState(false);
  const [commandBusy, setCommandBusy] = useState(false);
  const [terrainLayers, setTerrainLayers] = useState(() => ({
    ...DEFAULT_TERRAIN_LAYERS,
  }));
  const terrainLayersOn = anyTerrainLayerEnabled(terrainLayers);
  const [experimentCompareActive, setExperimentCompareActive] = useState(false);
  const [experimentAnalyticsActive, setExperimentAnalyticsActive] = useState(false);
  const [experimentContinuityReviewActive, setExperimentContinuityReviewActive] =
    useState(false);
  const [experimentF5Active, setExperimentF5Active] = useState(false);
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

  const handleSpawn = (pose: Pose, entityType = selectedType) => {
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
  };

  const handleMove = (entityId: string, pose: Pose) => {
    if (!sessionId) return;
    const clamped = clampPose(pose);
    void runEntityCommand(
      "move_entity",
      () => moveEntity(sessionId, { entity_id: entityId, pose: clamped }),
      { entityId, pose: clamped },
    );
  };

  const handleDelete = (entityId: string) => {
    if (!sessionId) return;
    void runEntityCommand(
      "delete_entity",
      () => deleteEntity(sessionId, entityId),
      { entityId },
    );
  };

  const sessionState = sessionStateFromSnapshots(snapshots);
  const editingEnabled =
    connected &&
    sessionId === editingSessionId &&
    sessionId === selectedSessionId &&
    isEditingAllowed(sessionState) &&
    !commandBusy;
  const simPaused = snapshots.clock_mirror?.payload?.paused === true;
  const telemetryEntities: UiEntity[] = entitiesFromSnapshot(
    snapshots.entity_pose_mirror,
  ).map((e) => ({
    entity_id: String(e.entity_id ?? ""),
    entity_type: String(e.entity_type ?? ""),
    pose: (e.pose as Record<string, unknown>) ?? {},
  }));
  const entities = mergeTelemetryAndLocalEntities(
    telemetryEntities,
    localEntities,
    locallyDeletedIds,
  );
  const worldSummary = snapshots.world_summary?.payload;
  const mergedEntityCounts = countEntitiesByType(entities);
  const mergedWorldSummary = {
    ...(worldSummary ?? {}),
    entity_count: entities.length,
    by_type: mergedEntityCounts,
  };

  const tactical = useTacticalState(
    sessionId,
    snapshots.tactical_state,
    snapshots.tactical_recommendation,
    editingEnabled,
  );

  const handleSelectEntity = useCallback(
    (id: string | null) => {
      if (sessionId) {
        patchSessionEdit(sessionId, { selectedEntityId: id });
      }
      if (!id || !sessionId || !tactical.targetPickActive) return;
      const ent = entities.find((e) => e.entity_id === id);
      if (
        ent &&
        (ent.entity_type === "drone" || ent.entity_type === "waypoint_marker")
      ) {
        void tactical.selectRole("target", id);
      }
    },
    [sessionId, patchSessionEdit, tactical, entities],
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
  }, [snapshots.entity_pose_mirror, sessionId]);

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

  const hidePanelCognition = connected;

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
  }, [selectedEntityId, sessionId, connected, editingEnabled]);

  const slotList = [...slots.values()].filter((s) => s.connected);
  const connectedSessionIds = slotList.map((s) => s.sessionId);
  const { orderedSessionIds: workspaceSessionIds, reorderSessions, pruneSession } =
    useSessionTabOrder(connectedSessionIds);

  const handleDisconnectSession = useCallback(
    (targetId: string) => {
      pruneSession(targetId);
      void disconnectSession(targetId);
    },
    [disconnectSession, pruneSession],
  );

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
  const experimentSlots = slotList.map((s) => ({
    sessionId: s.sessionId,
    label: labelFor(s.sessionId),
    snapshots: s.snapshots,
  }));
  const { bySession: handoffBySession } = useCaptureHandoffMirror(workspaceSessionIds);

  const handleSelectTab = useCallback(
    (targetId: string) => {
      if (targetId === selectedSessionId) return;
      const fromId = selectedSessionId;
      if (fromId) {
        const leaving = editBySession[fromId] ?? emptyEditState();
        const pendingForLeaving = fromId === sessionId && pendingReconcile;
        if (hasUnsyncedLocalMirror(leaving, pendingForLeaving)) {
          const label = labelFor(fromId);
          const ok = window.confirm(
            `Advisory: session "${label}" has unsynced local entity mirror edits ` +
              `(local overlay or pending reconcile). Switching tabs does not sync or discard edits. ` +
              `session_id remains authoritative on the bridge.\n\nSwitch anyway?`,
          );
          if (!ok) return;
        }
      }
      void selectTab(targetId);
    },
    [
      selectedSessionId,
      editBySession,
      sessionId,
      pendingReconcile,
      labelFor,
      selectTab,
    ],
  );

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100">
      <GovernanceChrome connected={connected} multiSession={connectedCount >= 2} />
      <RuntimeWorkstationShell
        header={
          <header>
            <h1 className="text-lg font-semibold text-slate-100">
              RT Sandbox — Runtime Workstation
            </h1>
            <p className="text-sm text-slate-400">
              Multi-session loopback prototype (max 3) — world editing, Cesium mirror,
              pull telemetry; not SA replay authority.
            </p>
          </header>
        }
        sessionRail={
          <>
            <BridgeConnectionBar
              connected={connected}
              busy={busy}
              onConnect={() => void connectNewSession()}
              onDisconnect={() => {
                if (selectedSessionId) handleDisconnectSession(selectedSessionId);
              }}
            />
            <SessionTabBar
              slots={slotList}
              orderedSessionIds={workspaceSessionIds}
              selectedSessionId={selectedSessionId}
              editingSessionId={editingSessionId}
              atCapacity={atCapacity}
              busy={busy}
              handoffBySession={handoffBySession}
              labelFor={labelFor}
              onRename={renameSession}
              onSelect={handleSelectTab}
              onReorder={reorderSessions}
              onNew={() => void connectNewSession()}
              onClose={handleDisconnectSession}
            />
            {connected && (
              <RefreshControls
                pullHz={pullHz}
                autoRefresh={autoRefresh}
                pulling={pulling}
                onPullHzChange={setPullHz}
                onAutoRefreshChange={setAutoRefresh}
                onRefresh={() => void doPull()}
              />
            )}
            {backgroundSlots.length > 0 && (
              <BackgroundDiagnostics
                slots={backgroundSlots}
                handoffBySession={handoffBySession}
                orderedSessionIds={workspaceSessionIds}
                editingSessionId={editingSessionId}
                terrainLayersOn={terrainLayersOn}
                pollPaused={!backgroundDiagOpen}
                onOpenChange={setBackgroundDiagOpen}
                labelFor={labelFor}
              />
            )}
          </>
        }
        workflowStrip={
          <SessionWorkflowStrip
            connected={connected}
            sessionState={sessionState}
            simPaused={simPaused}
            editingAllowed={editingEnabled}
            lastError={lastError}
            connectedCount={connectedCount}
            editingSessionId={editingSessionId}
          />
        }
        worldColumn={
          connected && sessionId ? (
            <>
              <EntityPalette
                selectedType={selectedType}
                onSelectType={setSelectedType}
                entityCountsByType={mergedEntityCounts}
                editingEnabled={editingEnabled}
              />
              <WorldEditingGrid
                entities={entities}
                selectedEntityId={selectedEntityId}
                selectedType={selectedType}
                editingEnabled={editingEnabled}
                worldSummary={mergedWorldSummary}
                showTerrainContour={terrainLayersOn}
                showContourLines={terrainLayers.showContourOverlays}
                onSelectEntity={handleSelectEntity}
                onSpawn={handleSpawn}
                onMove={handleMove}
                onDelete={handleDelete}
              />
              <EditingCognitionStrip
                lastCommand={lastCommand}
                pendingReconcile={pendingReconcile}
                mirrorSnapshot={snapshots.entity_pose_mirror}
              />
              <EditHistoryPanel history={editHistory} />
            </>
          ) : undefined
        }
        vizColumn={
          connected && sessionId ? (
            <CesiumRuntimePanel
              sessionId={sessionId}
              orderedSessionIds={workspaceSessionIds}
              editingSessionId={editingSessionId}
              entities={entities}
              selectedEntityId={selectedEntityId}
              selectedType={selectedType}
              worldSummary={mergedWorldSummary}
              mirrorSnapshot={snapshots.entity_pose_mirror}
              pendingReconcile={pendingReconcile}
              editingEnabled={editingEnabled}
              lastCommand={lastCommand}
              onSelectEntity={handleSelectEntity}
              onSpawn={handleSpawn}
              onMove={handleMove}
              onDelete={handleDelete}
              onTerrainLayersChange={setTerrainLayers}
            />
          ) : (
            <ConnectPlaceholder />
          )
        }
        mirrorsColumn={
          connected ? (
            <div className="space-y-4">
              <TacticalManualPanel
                sessionId={sessionId}
                editingEnabled={editingEnabled}
                entities={entities}
                selectedEntityId={selectedEntityId}
                mode={tactical.mode}
                state={tactical.state}
                busy={tactical.busy}
                error={tactical.error}
                targetPickActive={tactical.targetPickActive}
                onModeChange={(m) => void tactical.setMode(m)}
                onUseSelectedInterceptor={() => {
                  if (selectedEntityId) {
                    void tactical.selectRole("interceptor", selectedEntityId);
                  }
                }}
                onStartTargetPick={() =>
                  tactical.setTargetPickActive(!tactical.targetPickActive)
                }
                onAssign={() => void tactical.assign()}
                onClear={() => void tactical.clear()}
              />
              {tactical.mode === "assisted" && (
                <TacticalAssistedPanel
                  sessionId={sessionId}
                  editingEnabled={editingEnabled}
                  entities={entities}
                  state={tactical.state}
                  recommendation={tactical.recommendation}
                  busy={tactical.busy}
                  error={tactical.error}
                  onRequestRecommendation={() => void tactical.requestRec()}
                  onApprove={() => void tactical.approveRec()}
                  onReject={() => void tactical.rejectRec()}
                />
              )}
              {tactical.mode === "autonomous" && (
                <TacticalAutonomousPanel
                  sessionId={sessionId}
                  editingEnabled={editingEnabled}
                  entities={entities}
                  state={tactical.state}
                  busy={tactical.busy}
                  error={tactical.error}
                  onPause={() => void tactical.pauseLoop()}
                  onResume={() => void tactical.resumeLoop()}
                  onReturnToManual={() => void tactical.returnToManual()}
                />
              )}
              <RuntimeCognitionHub
                sessionId={sessionId}
                terrainLayers={terrainLayers}
                terrainLayersEnabled={terrainLayersOn}
                experimentCompareActive={experimentCompareActive}
                experimentAnalyticsActive={experimentAnalyticsActive}
                experimentContinuityReviewActive={experimentContinuityReviewActive}
                experimentF5Active={experimentF5Active}
                snapshots={{
                  world_summary: snapshots.world_summary,
                  session_health: snapshots.session_health,
                  entity_pose_mirror: snapshots.entity_pose_mirror,
                }}
              />
              <div className="grid gap-4 lg:grid-cols-2">
                <SessionLifecyclePanel
                  snapshot={snapshots.lifecycle_state}
                  hideCognition={hidePanelCognition}
                />
                <SessionHealthPanel
                  snapshot={snapshots.session_health}
                  hideCognition={hidePanelCognition}
                />
                <WorldSummaryPanel
                  snapshot={snapshots.world_summary}
                  hideCognition={hidePanelCognition}
                />
                <ClockMirrorPanel
                  snapshot={snapshots.clock_mirror}
                  hideCognition={hidePanelCognition}
                />
                <EntityPoseMirrorPanel
                  snapshot={snapshots.entity_pose_mirror}
                  hideCognition={hidePanelCognition}
                />
              </div>
            </div>
          ) : (
            <MirrorsIdleCard />
          )
        }
        pipelineFooter={
          <div className="space-y-4">
            <CaptureHandoffWorkflowPanel
              connected={connected}
              sessionState={sessionState}
              sessionId={sessionId}
              handoffBySession={handoffBySession}
              workspaceSessionIds={workspaceSessionIds}
            />
            <ExperimentWorkbenchPanel
              connected={connected}
              slots={experimentSlots}
              activeSessionId={sessionId}
              handoffBySession={handoffBySession}
              terrainLayersEnabled={terrainLayersOn}
              compareModeActive={experimentCompareActive}
              onCompareModeChange={setExperimentCompareActive}
              analyticsActive={experimentAnalyticsActive}
              onAnalyticsActiveChange={setExperimentAnalyticsActive}
              continuityReviewActive={experimentContinuityReviewActive}
              onContinuityReviewActiveChange={setExperimentContinuityReviewActive}
              f5Active={experimentF5Active}
              onF5ActiveChange={setExperimentF5Active}
            />
          </div>
        }
        diagnostics={
          <CollapsibleUiDiagnostics
            defaultOpen={!connected}
            sessionId={sessionId}
            subscriptionId={subscriptionId}
            lastPullUtc={lastPullUtc}
            drainedCount={drainedCount}
            lastError={lastError}
            sessionState={sessionState}
            lastCommand={lastCommand?.type}
            pendingReconcile={pendingReconcile}
          />
        }
      />
    </div>
  );
}
