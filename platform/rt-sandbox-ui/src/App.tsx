import { useCallback, useEffect, useMemo, useState } from "react";
import { GovernanceChrome } from "@/components/GovernanceChrome";
import { anyTerrainLayerEnabled } from "@/cesium/terrainLayers";
import {
  defaultVisibilityFromRegistry,
  toTerrainLayerVisibility,
  type VisualLayerVisibility,
} from "@/cesium/visualLayerRegistry";
import {
  DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE,
  type CesiumTerrainProviderMode,
} from "@/cesium/terrainProviderConfig";
import { hasUnsyncedLocalMirror } from "@/editing/sessionMirrorDirty";
import type { AdvisoryExperimentRollup } from "@/handoff/advisoryTypes";
import { useCaptureHandoffMirror } from "@/hooks/useCaptureHandoffMirror";
import { useCaptureControls } from "@/hooks/useCaptureControls";
import { useRtSessionWorkspace } from "@/hooks/useRtSessionWorkspace";
import {
  emptyEditState,
  useSessionEntityEditing,
} from "@/hooks/useSessionEntityEditing";
import { useSessionDisplayNames } from "@/hooks/useSessionDisplayNames";
import { useSessionTabOrder } from "@/hooks/useSessionTabOrder";
import {
  resolveSelectedDefenderId,
  resolveSelectedTargetId,
  useRuntimeControls,
} from "@/hooks/useRuntimeControls";
import { useTacticalState } from "@/hooks/useTacticalState";
import { ENTITY_RAIL_SPAWN_XY } from "@/entity/entityControlStates";
import { AppWorkstationSlots } from "@/workstation/AppWorkstationSlots";
import { scenarioControlStates } from "@/scenario/scenarioControlStates";
import { defaultPose } from "@/world/entityCatalog";
import { sessionStateFromSnapshots } from "@/telemetry/channelIndex";
import type { EntityType } from "@/world/entityCatalog";
import {
  clearSessionLayerVisibility,
  readSessionLayerVisibility,
  writeSessionLayerVisibility,
} from "@/workstation/sessionLayerVisibilityStore";
import {
  readStoredSessionRuntimeProfile,
  writeStoredSessionRuntimeProfile,
  type SessionRuntimeProfile,
} from "@/runtime/sessionRuntimeProfile";
import { useLiveRuntimePreflight } from "@/hooks/useLiveRuntimePreflight";

export default function App() {
  const [backgroundDiagOpen, setBackgroundDiagOpen] = useState(false);
  const [sessionRuntimeProfile, setSessionRuntimeProfile] =
    useState<SessionRuntimeProfile>(readStoredSessionRuntimeProfile);
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
    refreshSessionAfterApply,
    connectNewSession,
    activeRequestedRuntimeProfile,
    disconnectSession,
    selectTab,
    backgroundSlots,
  } = useRtSessionWorkspace({ pauseBackgroundPoll: !backgroundDiagOpen });

  const preflightProfile = connected
    ? activeRequestedRuntimeProfile
    : sessionRuntimeProfile;
  const {
    livePreflight,
    preflightLoading,
    preflightError,
  } = useLiveRuntimePreflight(preflightProfile);

  const [selectedType, setSelectedType] = useState<EntityType>("drone");
  const [layerVisibility, setLayerVisibility] = useState<VisualLayerVisibility>(() =>
    defaultVisibilityFromRegistry(),
  );
  const [terrainProviderMode, setTerrainProviderMode] =
    useState<CesiumTerrainProviderMode>(DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE);

  const handleSessionRuntimeProfileChange = useCallback(
    (profile: SessionRuntimeProfile) => {
      setSessionRuntimeProfile(profile);
      writeStoredSessionRuntimeProfile(profile);
    },
    [],
  );

  const handleLayerVisibilityChange = useCallback(
    (next: VisualLayerVisibility) => {
      setLayerVisibility(next);
      if (sessionId) writeSessionLayerVisibility(sessionId, next);
    },
    [sessionId],
  );

  useEffect(() => {
    if (!sessionId) return;
    const saved = readSessionLayerVisibility(sessionId);
    setLayerVisibility(saved ?? defaultVisibilityFromRegistry());
  }, [sessionId]);

  const terrainLayers = toTerrainLayerVisibility(layerVisibility);
  const terrainLayersOn = anyTerrainLayerEnabled(terrainLayers);
  const [experimentCompareActive, setExperimentCompareActive] = useState(false);
  const [experimentAnalyticsActive, setExperimentAnalyticsActive] = useState(false);
  const [experimentContinuityReviewActive, setExperimentContinuityReviewActive] =
    useState(false);
  const [experimentF5Active, setExperimentF5Active] = useState(false);
  const [experimentRollup, setExperimentRollup] = useState<AdvisoryExperimentRollup | null>(
    null,
  );

  const sessionState = sessionStateFromSnapshots(snapshots);
  const worldSummary = snapshots.world_summary?.payload as
    | Record<string, unknown>
    | undefined;

  const slotList = [...slots.values()].filter((s) => s.connected);
  const connectedSessionIds = slotList.map((s) => s.sessionId);
  const { orderedSessionIds: workspaceSessionIds, reorderSessions, pruneSession } =
    useSessionTabOrder(connectedSessionIds);

  const {
    editBySession,
    editHistory,
    selectedEntityId,
    entities,
    mergedWorldSummary,
    mergedEntityCounts,
    editingEnabled,
    lastCommand,
    pendingReconcile,
    handleSpawn,
    handleMove,
    handleDelete,
    handleApplyScenario,
    applyRuntimeStatus,
    commandBusy,
    patchSessionEdit,
  } = useSessionEntityEditing({
    sessionId,
    selectedSessionId,
    editingSessionId,
    connected,
    sessionState,
    entityPoseMirror: snapshots.entity_pose_mirror,
    worldSummary,
    lastPullUtc,
    workspaceSessionIds,
    selectedType,
    doPull,
    refreshAfterApply: refreshSessionAfterApply,
    setLastError,
  });

  const tactical = useTacticalState(
    sessionId,
    snapshots.tactical_state,
    snapshots.tactical_recommendation,
    editingEnabled,
  );

  const selectedDefenderId = useMemo(
    () =>
      resolveSelectedDefenderId(
        selectedEntityId,
        entities,
        tactical.state?.selected_interceptor_id,
      ),
    [selectedEntityId, entities, tactical.state?.selected_interceptor_id],
  );

  const selectedTargetId = useMemo(
    () =>
      resolveSelectedTargetId(
        selectedEntityId,
        entities,
        tactical.state?.selected_target_id,
      ),
    [selectedEntityId, entities, tactical.state?.selected_target_id],
  );

  const runtime = useRuntimeControls({
    sessionId,
    editingEnabled,
    selectedDefenderId,
    selectedTargetId,
    requestedRuntimeProfile: activeRequestedRuntimeProfile,
    doPull,
    setLastError,
  });

  const capture = useCaptureControls({
    sessionId,
    editingEnabled,
    doPull,
    setLastError,
  });

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

  const simPaused = snapshots.clock_mirror?.payload?.paused === true;
  const hidePanelCognition = connected;

  const handleDisconnectSession = useCallback(
    (targetId: string) => {
      clearSessionLayerVisibility(targetId);
      pruneSession(targetId);
      void disconnectSession(targetId);
    },
    [disconnectSession, pruneSession],
  );

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
      if (fromId) {
        writeSessionLayerVisibility(fromId, layerVisibility);
      }
      const restored =
        readSessionLayerVisibility(targetId) ?? defaultVisibilityFromRegistry();
      setLayerVisibility(restored);
      void selectTab(targetId);
    },
    [
      selectedSessionId,
      editBySession,
      sessionId,
      pendingReconcile,
      labelFor,
      selectTab,
      layerVisibility,
    ],
  );

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100">
      <GovernanceChrome connected={connected} multiSession={connectedCount >= 2} />
      <AppWorkstationSlots
        connected={connected}
        connectedCount={connectedCount}
        sessionId={sessionId}
        subscriptionId={subscriptionId}
        selectedSessionId={selectedSessionId}
        editingSessionId={editingSessionId}
        atCapacity={atCapacity}
        busy={busy}
        pulling={pulling}
        autoRefresh={autoRefresh}
        pullHz={pullHz}
        lastPullUtc={lastPullUtc}
        drainedCount={drainedCount}
        lastError={lastError}
        sessionState={sessionState}
        simPaused={simPaused}
        slotList={slotList}
        workspaceSessionIds={workspaceSessionIds}
        handoffBySession={handoffBySession}
        backgroundSlots={backgroundSlots}
        backgroundDiagOpen={backgroundDiagOpen}
        onBackgroundDiagOpenChange={setBackgroundDiagOpen}
        labelFor={labelFor}
        renameSession={renameSession}
        onSelectTab={handleSelectTab}
        reorderSessions={reorderSessions}
        sessionRuntimeProfile={sessionRuntimeProfile}
        onSessionRuntimeProfileChange={handleSessionRuntimeProfileChange}
        activeRequestedRuntimeProfile={activeRequestedRuntimeProfile}
        livePreflight={livePreflight}
        preflightLoading={preflightLoading}
        preflightError={preflightError}
        onConnectNewSession={() => void connectNewSession(sessionRuntimeProfile)}
        onDisconnectSelected={() => {
          if (selectedSessionId) handleDisconnectSession(selectedSessionId);
        }}
        onCloseSession={handleDisconnectSession}
        onPullHzChange={setPullHz}
        onAutoRefreshChange={setAutoRefresh}
        onRefresh={() => void doPull()}
        layerVisibility={layerVisibility}
        terrainLayers={terrainLayers}
        terrainLayersOn={terrainLayersOn}
        terrainProviderMode={terrainProviderMode}
        onTerrainProviderModeChange={setTerrainProviderMode}
        onLayerVisibilityChange={handleLayerVisibilityChange}
        entities={entities}
        selectedEntityId={selectedEntityId}
        selectedType={selectedType}
        onSelectType={setSelectedType}
        mergedEntityCounts={mergedEntityCounts}
        mergedWorldSummary={mergedWorldSummary}
        editingEnabled={editingEnabled}
        editHistory={editHistory}
        lastCommand={lastCommand}
        pendingReconcile={pendingReconcile}
        onSelectEntity={handleSelectEntity}
        onSpawn={handleSpawn}
        onSpawnAttacker={() =>
          handleSpawn(defaultPose("drone", ENTITY_RAIL_SPAWN_XY.x, ENTITY_RAIL_SPAWN_XY.y), "drone")
        }
        onSpawnDefenderEntity={() =>
          handleSpawn(
            defaultPose("interceptor", ENTITY_RAIL_SPAWN_XY.x, ENTITY_RAIL_SPAWN_XY.y),
            "interceptor",
          )
        }
        onDeleteSelected={() => {
          if (selectedEntityId) handleDelete(selectedEntityId);
        }}
        entityControlsDisabled={!editingEnabled}
        entityDeleteDisabled={!editingEnabled || !selectedEntityId}
        onMove={handleMove}
        onDelete={handleDelete}
        onApplyToRuntime={() => void handleApplyScenario()}
        applyToRuntimeDisabled={
          !sessionId ||
          !editingEnabled ||
          !scenarioControlStates(sessionState).applyScenario ||
          entities.length === 0 ||
          commandBusy
        }
        applyScenarioDisabled={
          !sessionId ||
          !editingEnabled ||
          !scenarioControlStates(sessionState).applyScenario ||
          entities.length === 0 ||
          commandBusy
        }
        applyRuntimeStatus={applyRuntimeStatus}
        snapshots={snapshots}
        experimentCompareActive={experimentCompareActive}
        onExperimentCompareActiveChange={setExperimentCompareActive}
        experimentAnalyticsActive={experimentAnalyticsActive}
        onExperimentAnalyticsActiveChange={setExperimentAnalyticsActive}
        experimentContinuityReviewActive={experimentContinuityReviewActive}
        onExperimentContinuityReviewActiveChange={setExperimentContinuityReviewActive}
        experimentF5Active={experimentF5Active}
        onExperimentF5ActiveChange={setExperimentF5Active}
        experimentRollup={experimentRollup}
        onExperimentRollupChange={setExperimentRollup}
        experimentSlots={experimentSlots}
        tactical={tactical}
        runtimeBusy={runtime.busy}
        captureBusy={capture.busy}
        captureSummary={capture.summary}
        selectedDefenderId={selectedDefenderId}
        selectedTargetId={selectedTargetId}
        onPauseSim={() => void runtime.pauseSim()}
        onResumeSim={() => void runtime.resumeSim()}
        onResetSession={() => void runtime.resetSession()}
        onStopSession={() => void runtime.stopSession()}
        onSpawnDefender={() => void runtime.spawnDefender()}
        onStartCapture={() => void capture.startCapture()}
        onStopCapture={() => void capture.stopCapture()}
        onAssignTarget={() => void runtime.assignTarget()}
        onCancelAssignment={() => void runtime.cancelAssignment()}
        hidePanelCognition={hidePanelCognition}
      />
    </div>
  );
}
