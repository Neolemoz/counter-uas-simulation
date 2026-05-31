import { useState } from "react";
import type { CaptureHandoffRow } from "@/bridge/types";
import { CaptureHandoffWorkflowPanel } from "@/components/CaptureHandoffWorkflowPanel";
import { TacticalAssistedPanel } from "@/components/TacticalAssistedPanel";
import { TacticalAutonomousPanel } from "@/components/TacticalAutonomousPanel";
import { TacticalManualPanel } from "@/components/TacticalManualPanel";
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
  DEFAULT_DEFENSE_ZONE_CONFIG,
  type DefenseZoneConfig,
} from "@/cesium/defenseZoneConfig";
import {
  DEFAULT_RADAR_DOME_CONFIG,
  type RadarDomeConfig,
} from "@/cesium/sensorDomeLayer";
import {
  DEFAULT_SENSOR_DOME_ZONE_MODE,
  type SensorDomeZoneMode,
} from "@/cesium/terrainLayers";
import type { TerrainLayerVisibility } from "@/cesium/terrainLayers";
import type { VisualLayerVisibility } from "@/cesium/visualLayerRegistry";
import { CesiumRuntimePanel } from "@/components/CesiumRuntimePanel";
import { WorldEditingGrid } from "@/components/WorldEditingGrid";
import type { EditHistoryEntry, EditCommandType } from "@/editing/editHistory";
import type { UiEntity } from "@/editing/localEntityMirror";
import { ExperimentWorkbenchPanel } from "@/experiment/ExperimentWorkbenchPanel";
import type { AdvisoryExperimentRollup } from "@/handoff/advisoryTypes";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import type { useTacticalState } from "@/hooks/useTacticalState";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { TelemetryChannel } from "@/telemetry/constants";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import { BackgroundDiagnostics } from "@/workstation/BackgroundDiagnostics";
import { BackgroundDiagnosticsCompact } from "@/workstation/BackgroundDiagnosticsCompact";
import { ConnectPlaceholder } from "@/workstation/ConnectPlaceholder";
import { MirrorsIdleCard } from "@/workstation/MirrorsIdleCard";
import { RuntimeCognitionHub } from "@/workstation/RuntimeCognitionHub";
import { RuntimeWorkstationShell } from "@/workstation/RuntimeWorkstationShell";
import { SessionTabBar } from "@/workstation/SessionTabBar";
import { SessionWorkflowStrip } from "@/workstation/SessionWorkflowStrip";

type Tactical = ReturnType<typeof useTacticalState>;

export type AppWorkstationSlotsProps = {
  connected: boolean;
  connectedCount: number;
  sessionId: string | null;
  subscriptionId: string | null;
  selectedSessionId: string | null;
  editingSessionId: string | null;
  atCapacity: boolean;
  busy: boolean;
  pulling: boolean;
  autoRefresh: boolean;
  pullHz: number;
  lastPullUtc: string | null;
  drainedCount: number;
  lastError: string | null;
  sessionState: string;
  simPaused: boolean;
  slotList: SessionSlot[];
  workspaceSessionIds: string[];
  handoffBySession: Map<string, CaptureHandoffRow[]>;
  backgroundSlots: SessionSlot[];
  backgroundDiagOpen: boolean;
  onBackgroundDiagOpenChange: (open: boolean) => void;
  labelFor: (sessionId: string) => string;
  renameSession: (sessionId: string) => void;
  onSelectTab: (sessionId: string) => void;
  reorderSessions: (orderedIds: string[]) => void;
  onConnectNewSession: () => void;
  onDisconnectSelected: () => void;
  onCloseSession: (sessionId: string) => void;
  onPullHzChange: (hz: number) => void;
  onAutoRefreshChange: (enabled: boolean) => void;
  onRefresh: () => void;
  layerVisibility: VisualLayerVisibility;
  terrainLayers: TerrainLayerVisibility;
  terrainLayersOn: boolean;
  onLayerVisibilityChange: (next: VisualLayerVisibility) => void;
  entities: UiEntity[];
  selectedEntityId: string | null;
  selectedType: EntityType;
  onSelectType: (type: EntityType) => void;
  mergedEntityCounts: Record<string, number>;
  mergedWorldSummary: Record<string, unknown>;
  editingEnabled: boolean;
  editHistory: EditHistoryEntry[];
  lastCommand:
    | {
        type: EditCommandType;
        ok: boolean;
        errorCode?: string;
        message?: string;
      }
    | undefined;
  pendingReconcile: boolean;
  onSelectEntity: (id: string | null) => void;
  onSpawn: (pose: Pose, entityType?: EntityType) => void;
  onMove: (entityId: string, pose: Pose) => void;
  onDelete: (entityId: string) => void;
  snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>;
  experimentCompareActive: boolean;
  onExperimentCompareActiveChange: (active: boolean) => void;
  experimentAnalyticsActive: boolean;
  onExperimentAnalyticsActiveChange: (active: boolean) => void;
  experimentContinuityReviewActive: boolean;
  onExperimentContinuityReviewActiveChange: (active: boolean) => void;
  experimentF5Active: boolean;
  onExperimentF5ActiveChange: (active: boolean) => void;
  experimentRollup: AdvisoryExperimentRollup | null;
  onExperimentRollupChange: (rollup: AdvisoryExperimentRollup | null) => void;
  experimentSlots: {
    sessionId: string;
    label: string;
    snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>;
  }[];
  tactical: Tactical;
  hidePanelCognition: boolean;
};

export function AppWorkstationSlots(props: AppWorkstationSlotsProps) {
  const {
    connected,
    connectedCount,
    sessionId,
    subscriptionId,
    selectedSessionId,
    editingSessionId,
    atCapacity,
    busy,
    pulling,
    autoRefresh,
    pullHz,
    lastPullUtc,
    drainedCount,
    lastError,
    sessionState,
    simPaused,
    slotList,
    workspaceSessionIds,
    handoffBySession,
    backgroundSlots,
    backgroundDiagOpen,
    onBackgroundDiagOpenChange,
    labelFor,
    renameSession,
    onSelectTab,
    reorderSessions,
    onConnectNewSession,
    onDisconnectSelected,
    onCloseSession,
    onPullHzChange,
    onAutoRefreshChange,
    onRefresh,
    layerVisibility,
    terrainLayers,
    terrainLayersOn,
    onLayerVisibilityChange,
    entities,
    selectedEntityId,
    selectedType,
    onSelectType,
    mergedEntityCounts,
    mergedWorldSummary,
    editingEnabled,
    editHistory,
    lastCommand,
    pendingReconcile,
    onSelectEntity,
    onSpawn,
    onMove,
    onDelete,
    snapshots,
    experimentCompareActive,
    onExperimentCompareActiveChange,
    experimentAnalyticsActive,
    onExperimentAnalyticsActiveChange,
    experimentContinuityReviewActive,
    onExperimentContinuityReviewActiveChange,
    experimentF5Active,
    onExperimentF5ActiveChange,
    experimentRollup,
    onExperimentRollupChange,
    experimentSlots,
    tactical,
    hidePanelCognition,
  } = props;

  const [radarDomeConfig, setRadarDomeConfig] = useState<RadarDomeConfig>(
    DEFAULT_RADAR_DOME_CONFIG,
  );
  const [defenseZoneConfig, setDefenseZoneConfig] = useState<DefenseZoneConfig>(
    DEFAULT_DEFENSE_ZONE_CONFIG,
  );
  const [radarDomeSelectedOnly, setRadarDomeSelectedOnly] = useState(false);
  const [defenseZoneSelectedOnly, setDefenseZoneSelectedOnly] = useState(false);
  const [radarDomeVisible, setRadarDomeVisible] = useState(true);
  const [radarVolumeVisible, setRadarVolumeVisible] = useState(true);
  const [defenseZoneVisible, setDefenseZoneVisible] = useState(true);
  const [radarDomeLabelsVisible, setRadarDomeLabelsVisible] = useState(true);
  const [sensorDomeZoneMode, setSensorDomeZoneMode] = useState<SensorDomeZoneMode>(
    DEFAULT_SENSOR_DOME_ZONE_MODE,
  );
  const selectedEntity = entities.find((entity) => entity.entity_id === selectedEntityId);
  const sensorDomeOptions = {
    show: radarDomeVisible,
    showRing: radarDomeVisible,
    showVolume: radarVolumeVisible,
    selectedEntityId:
      selectedEntity?.entity_type === "radar" ? selectedEntityId : null,
    selectedOnly: radarDomeSelectedOnly,
    showLabels: radarDomeLabelsVisible,
    radii: radarDomeConfig,
  };
  const radarPreviewControlState = {
    layerEnabled: terrainLayers.showSensorDomes,
    showVolume: radarVolumeVisible,
    showRing: radarDomeVisible,
    selectedOnly: radarDomeSelectedOnly,
    showLabels: radarDomeLabelsVisible,
  };
  const radarPreviewControlHandlers = {
    onShowVolumeChange: setRadarVolumeVisible,
    onShowRingChange: setRadarDomeVisible,
    onSelectedOnlyChange: setRadarDomeSelectedOnly,
    onShowLabelsChange: setRadarDomeLabelsVisible,
  };
  const defenseZoneOptions = {
    show: defenseZoneVisible,
    selectedEntityId:
      selectedEntity?.entity_type === "waypoint_marker" ? selectedEntityId : null,
    selectedOnly: defenseZoneSelectedOnly,
    showLabels: defenseZoneConfig.showLabels,
    config: defenseZoneConfig,
  };

  return (
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
            onConnect={onConnectNewSession}
            onDisconnect={onDisconnectSelected}
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
            onSelect={onSelectTab}
            onReorder={reorderSessions}
            onNew={onConnectNewSession}
            onClose={onCloseSession}
          />
          {connected && (
            <RefreshControls
              pullHz={pullHz}
              autoRefresh={autoRefresh}
              pulling={pulling}
              onPullHzChange={onPullHzChange}
              onAutoRefreshChange={onAutoRefreshChange}
              onRefresh={onRefresh}
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
      cognitionColumn={
        connected && sessionId ? (
          <RuntimeCognitionHub
            sessionId={sessionId}
            orderedSessionIds={workspaceSessionIds}
            layerVisibility={layerVisibility}
            terrainLayers={terrainLayers}
            terrainLayersEnabled={terrainLayersOn}
            entities={entities}
            selectedEntityId={selectedEntityId}
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
        ) : undefined
      }
      globeFooter={
        backgroundSlots.length > 0 ? (
          <div className="space-y-2">
            <BackgroundDiagnosticsCompact
              slots={backgroundSlots}
              orderedSessionIds={workspaceSessionIds}
              pollPaused={!backgroundDiagOpen}
              onExpandDetails={() => onBackgroundDiagOpenChange(true)}
              labelFor={labelFor}
            />
            <BackgroundDiagnostics
              slots={backgroundSlots}
              handoffBySession={handoffBySession}
              orderedSessionIds={workspaceSessionIds}
              editingSessionId={editingSessionId}
              terrainLayersOn={terrainLayersOn}
              open={backgroundDiagOpen}
              pollPaused={!backgroundDiagOpen}
              onOpenChange={onBackgroundDiagOpenChange}
              labelFor={labelFor}
            />
          </div>
        ) : undefined
      }
      worldColumn={
        connected && sessionId ? (
          <div className="flex flex-col gap-4">
            <EntityPalette
              selectedType={selectedType}
              onSelectType={onSelectType}
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
              radarDomeConfig={radarDomeConfig}
              defenseZoneConfig={defenseZoneConfig}
              radarDomeSelectedOnly={radarDomeSelectedOnly}
              defenseZoneSelectedOnly={defenseZoneSelectedOnly}
              radarDomeVisible={radarDomeVisible}
              radarVolumeVisible={radarVolumeVisible}
              defenseZoneVisible={defenseZoneVisible}
              radarDomeLabelsVisible={radarDomeLabelsVisible}
              sensorDomeZoneMode={sensorDomeZoneMode}
              sensorDomeLayerEnabled={terrainLayers.showSensorDomes}
              onRadarDomeConfigChange={setRadarDomeConfig}
              onDefenseZoneConfigChange={setDefenseZoneConfig}
              onRadarDomeSelectedOnlyChange={setRadarDomeSelectedOnly}
              onDefenseZoneSelectedOnlyChange={setDefenseZoneSelectedOnly}
              onRadarDomeVisibleChange={setRadarDomeVisible}
              onRadarVolumeVisibleChange={setRadarVolumeVisible}
              onDefenseZoneVisibleChange={setDefenseZoneVisible}
              onRadarDomeLabelsVisibleChange={setRadarDomeLabelsVisible}
              onSensorDomeZoneModeChange={setSensorDomeZoneMode}
              onSelectEntity={onSelectEntity}
              onSpawn={onSpawn}
              onMove={onMove}
              onDelete={onDelete}
            />
            <EditingCognitionStrip
              lastCommand={lastCommand}
              pendingReconcile={pendingReconcile}
              mirrorSnapshot={snapshots.entity_pose_mirror}
            />
            <EditHistoryPanel history={editHistory} />
          </div>
        ) : undefined
      }
      vizColumn={
        connected && sessionId ? (
          <CesiumRuntimePanel
            sessionId={sessionId}
            orderedSessionIds={workspaceSessionIds}
            connectedCount={connectedCount}
            editingSessionId={editingSessionId}
            entities={entities}
            selectedEntityId={selectedEntityId}
            selectedType={selectedType}
            worldSummary={mergedWorldSummary}
            mirrorSnapshot={snapshots.entity_pose_mirror}
            pendingReconcile={pendingReconcile}
            editingEnabled={editingEnabled}
            lastCommand={lastCommand}
            onSelectEntity={onSelectEntity}
            onSpawn={onSpawn}
            onMove={onMove}
            onDelete={onDelete}
            layerVisibility={layerVisibility}
            sensorDomeOptions={sensorDomeOptions}
            defenseZoneOptions={defenseZoneOptions}
            sensorDomeZoneMode={sensorDomeZoneMode}
            radarPreviewControls={
              selectedEntity?.entity_type === "radar"
                ? {
                    state: radarPreviewControlState,
                    handlers: radarPreviewControlHandlers,
                  }
                : null
            }
            onLayerVisibilityChange={onLayerVisibilityChange}
            tacticalState={tactical.state}
            slotList={slotList}
          />
        ) : (
          <ConnectPlaceholder />
        )
      }
      tacticalColumn={
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
          </div>
        ) : (
          <MirrorsIdleCard />
        )
      }
      captureFooter={
        <CaptureHandoffWorkflowPanel
          connected={connected}
          sessionState={sessionState}
          sessionId={sessionId}
          handoffBySession={handoffBySession}
          workspaceSessionIds={workspaceSessionIds}
          experimentRollup={experimentRollup}
        />
      }
      experimentFooter={
        <ExperimentWorkbenchPanel
          connected={connected}
          slots={experimentSlots}
          activeSessionId={sessionId}
          handoffBySession={handoffBySession}
          onHandoffEligibilityRollupChange={onExperimentRollupChange}
          terrainLayersEnabled={terrainLayersOn}
          compareModeActive={experimentCompareActive}
          onCompareModeChange={onExperimentCompareActiveChange}
          analyticsActive={experimentAnalyticsActive}
          onAnalyticsActiveChange={onExperimentAnalyticsActiveChange}
          continuityReviewActive={experimentContinuityReviewActive}
          onContinuityReviewActiveChange={onExperimentContinuityReviewActiveChange}
          f5Active={experimentF5Active}
          onF5ActiveChange={onExperimentF5ActiveChange}
        />
      }
      diagnostics={
        <div className="space-y-4">
          <div className="grid gap-4 lg:grid-cols-2 xl:grid-cols-3">
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
        </div>
      }
    />
  );
}
