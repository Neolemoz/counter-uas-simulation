import { useCallback, useEffect, useState } from "react";
import { GovernanceChrome } from "@/components/GovernanceChrome";
import { anyTerrainLayerEnabled } from "@/cesium/terrainLayers";
import {
  defaultVisibilityFromRegistry,
  toTerrainLayerVisibility,
  type VisualLayerVisibility,
} from "@/cesium/visualLayerRegistry";
import { hasUnsyncedLocalMirror } from "@/editing/sessionMirrorDirty";
import type { AdvisoryExperimentRollup } from "@/handoff/advisoryTypes";
import { useCaptureHandoffMirror } from "@/hooks/useCaptureHandoffMirror";
import { useRtSessionWorkspace } from "@/hooks/useRtSessionWorkspace";
import {
  emptyEditState,
  useSessionEntityEditing,
} from "@/hooks/useSessionEntityEditing";
import { useSessionDisplayNames } from "@/hooks/useSessionDisplayNames";
import { useSessionTabOrder } from "@/hooks/useSessionTabOrder";
import { useTacticalState } from "@/hooks/useTacticalState";
import { sessionStateFromSnapshots } from "@/telemetry/channelIndex";
import type { EntityType } from "@/world/entityCatalog";
import {
  clearSessionLayerVisibility,
  readSessionLayerVisibility,
  writeSessionLayerVisibility,
} from "@/workstation/sessionLayerVisibilityStore";
import { AppWorkstationSlots } from "@/workstation/AppWorkstationSlots";

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

  const [selectedType, setSelectedType] = useState<EntityType>("drone");
  const [layerVisibility, setLayerVisibility] = useState<VisualLayerVisibility>(() =>
    defaultVisibilityFromRegistry(),
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
    setLastError,
  });

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
        onConnectNewSession={() => void connectNewSession()}
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
        onMove={handleMove}
        onDelete={handleDelete}
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
        hidePanelCognition={hidePanelCognition}
      />
    </div>
  );
}
