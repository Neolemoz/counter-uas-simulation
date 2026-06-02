import { useCallback, useMemo, useState } from "react";
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
  CaptureControlBar,
  CollapsibleUiDiagnostics,
  EntityControlBar,
  LifecycleControlBar,
  RefreshControls,
  RuntimeControlBar,
  ScenarioControlBar,
} from "@/components/RefreshControls";
import { CaptureSummaryStrip } from "@/components/CaptureSummaryStrip";
import { AdapterStatusPanel } from "@/components/AdapterStatusPanel";
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
import { ScenarioEvaluationPanel } from "@/components/ScenarioEvaluationPanel";
import { WorldEditingGrid } from "@/components/WorldEditingGrid";
import type { ApplyRuntimeStatus } from "@/components/WorldEditorApplyStatus";
import type { EditHistoryEntry, EditCommandType } from "@/editing/editHistory";
import type { UiEntity } from "@/editing/localEntityMirror";
import { ExperimentWorkbenchPanel } from "@/experiment/ExperimentWorkbenchPanel";
import type { AdvisoryExperimentRollup } from "@/handoff/advisoryTypes";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";
import type { useTacticalState } from "@/hooks/useTacticalState";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { LiveCaptureSummary } from "@/telemetry/captureSummary";
import type { TelemetryChannel } from "@/telemetry/constants";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import {
  DEFAULT_PLANNING_COVERAGE_OPTIONS,
  EMPTY_PLANNING_POLYGON,
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
  addPlanningRadarSite,
  addPlanningVertex,
  canFinishPlanningPolygon,
  cancelPlanningDrawing,
  clearPlanningPolygon,
  clearPlanningRadarSites,
  deletePlanningRadarSite,
  estimatePlanningCoverage,
  finishPlanningPolygon,
  planningToolAllowsDrawing,
  planningToolAllowsRadarPlacement,
  planningToolUsesCesiumClick,
  selectPlanningRadarSite,
  updatePlanningRadarPreset,
  type PlanningCoverageEstimate,
  type PlanningCoverageLayerOptions,
  type PlanningPolygonState,
  type PlanningRadarPresetId,
  type PlanningRadarState,
  type PlanningTool,
  type PlanningVertex,
} from "@/cesium/planningDrawing";
import { BackgroundDiagnostics } from "@/workstation/BackgroundDiagnostics";
import { BackgroundDiagnosticsCompact } from "@/workstation/BackgroundDiagnosticsCompact";
import { ConnectPlaceholder } from "@/workstation/ConnectPlaceholder";
import { MirrorsIdleCard } from "@/workstation/MirrorsIdleCard";
import { RuntimeCognitionHub } from "@/workstation/RuntimeCognitionHub";
import { RuntimeWorkstationShell } from "@/workstation/RuntimeWorkstationShell";
import { SessionTabBar } from "@/workstation/SessionTabBar";
import { SessionWorkflowStrip } from "@/workstation/SessionWorkflowStrip";

type Tactical = ReturnType<typeof useTacticalState>;

export type RuntimeWorkspaceMode = "grid" | "planning";

export const DEFAULT_RUNTIME_WORKSPACE_MODE: RuntimeWorkspaceMode = "grid";

export function editingEnabledForWorkspaceMode(
  _mode: RuntimeWorkspaceMode,
  editingEnabled: boolean,
): boolean {
  return editingEnabled;
}

export function workspaceModeShowsPlanningPlaceholder(
  mode: RuntimeWorkspaceMode,
): boolean {
  return mode === "planning";
}

export function RuntimeWorkspaceModeSelector({
  mode,
  onModeChange,
}: {
  mode: RuntimeWorkspaceMode;
  onModeChange: (mode: RuntimeWorkspaceMode) => void;
}) {
  const options: { mode: RuntimeWorkspaceMode; label: string }[] = [
    { mode: "grid", label: "Grid Mode" },
    { mode: "planning", label: "Planning Mode" },
  ];
  return (
    <div
      className="rounded border border-slate-800 bg-slate-950/50 p-2"
      data-testid="runtime-workspace-mode-selector"
    >
      <div className="grid grid-cols-2 gap-1">
        {options.map((option) => (
          <button
            key={option.mode}
            type="button"
            onClick={() => onModeChange(option.mode)}
            aria-pressed={mode === option.mode}
            className={`rounded border px-2.5 py-1.5 text-xs font-semibold transition-colors ${
              mode === option.mode
                ? "border-cyan-600/70 bg-cyan-950/60 text-cyan-100"
                : "border-slate-700 bg-slate-950 text-slate-400 hover:border-slate-600 hover:text-slate-200"
            }`}
          >
            {option.label}
          </button>
        ))}
      </div>
    </div>
  );
}

function formatAreaM2(areaM2: number): string {
  if (areaM2 >= 1_000_000) return `${(areaM2 / 1_000_000).toFixed(2)} km2`;
  return `${Math.round(areaM2).toLocaleString()} m2`;
}

export function PlanningModePanel({
  tool,
  polygon,
  radars,
  coverage,
  coverageOptions,
  onToolChange,
  onFinishPolygon,
  onCancelDrawing,
  onClearPolygon,
  onSelectRadarSite,
  onDeleteRadarSite,
  onRadarPresetChange,
  onClearRadarSites,
  onCoverageOptionsChange,
  onResetCoverageState,
}: {
  tool: PlanningTool;
  polygon: PlanningPolygonState;
  radars: PlanningRadarState;
  coverage: PlanningCoverageEstimate;
  coverageOptions: PlanningCoverageLayerOptions;
  onToolChange: (tool: PlanningTool) => void;
  onFinishPolygon: () => void;
  onCancelDrawing: () => void;
  onClearPolygon: () => void;
  onSelectRadarSite: (siteId: string | null) => void;
  onDeleteRadarSite: (siteId: string) => void;
  onRadarPresetChange: (siteId: string, presetId: PlanningRadarPresetId) => void;
  onClearRadarSites: () => void;
  onCoverageOptionsChange: (options: PlanningCoverageLayerOptions) => void;
  onResetCoverageState: () => void;
}) {
  const canFinish = canFinishPlanningPolygon(polygon);
  const hasDraft = polygon.draftVertices.length > 0;
  const hasCompleted = (polygon.completedVertices?.length ?? 0) > 0;
  const selectedRadar =
    radars.sites.find((site) => site.id === radars.selectedSiteId) ?? null;

  const toolOptions: { tool: PlanningTool; label: string }[] = [
    { tool: "select", label: "Select" },
    { tool: "draw_defense_area", label: "Draw Defense Area" },
    { tool: "place_radar_site", label: "Place Radar Site" },
  ];

  return (
    <div className="space-y-3" data-testid="planning-mode-placeholder">
      <div
        className="rounded border border-cyan-800/60 bg-cyan-950/20 p-3"
        data-testid="planning-toolbar-placeholder"
      >
        <h3 className="text-xs font-semibold uppercase tracking-wide text-cyan-100">
          Planning toolbar
        </h3>
        <p className="mt-2 text-xs leading-relaxed text-slate-300">
          Planning Mode is UI-local and explanatory only. Planning artifacts are not
          runtime authority, are not validated sensing, and cause no simulation behavior
          change.
        </p>
        <div className="mt-3 grid grid-cols-1 gap-2">
          {toolOptions.map((option) => (
            <button
              key={option.tool}
              type="button"
              aria-pressed={tool === option.tool}
              onClick={() => onToolChange(option.tool)}
              className={`rounded border px-2.5 py-1.5 text-xs font-semibold ${
                tool === option.tool
                  ? "border-cyan-600/70 bg-cyan-950/70 text-cyan-100"
                  : "border-slate-700 bg-slate-950 text-slate-300 hover:border-slate-600"
              }`}
            >
              {option.label}
            </button>
          ))}
        </div>
        <div className="mt-3 grid gap-2 text-xs">
          <button
            type="button"
            disabled={!canFinish}
            onClick={onFinishPolygon}
            className="rounded border border-emerald-700 bg-emerald-950/45 px-3 py-1.5 font-medium text-emerald-100 disabled:cursor-not-allowed disabled:opacity-40"
          >
            Finish Polygon
          </button>
          <button
            type="button"
            disabled={!hasDraft}
            onClick={onCancelDrawing}
            className="rounded border border-amber-700 bg-amber-950/35 px-3 py-1.5 font-medium text-amber-100 disabled:cursor-not-allowed disabled:opacity-40"
          >
            Cancel Drawing
          </button>
          <button
            type="button"
            disabled={!hasDraft && !hasCompleted}
            onClick={onClearPolygon}
            className="rounded border border-slate-700 bg-slate-950 px-3 py-1.5 font-medium text-slate-200 disabled:cursor-not-allowed disabled:opacity-40"
          >
            Clear Polygon
          </button>
        </div>
      </div>
      <div
        className="rounded border border-slate-800 bg-slate-950/45 p-3"
        data-testid="planning-panel-placeholder"
      >
        <h3 className="text-xs font-semibold uppercase tracking-wide text-slate-200">
          Planning panel
        </h3>
        <p className="mt-2 text-xs leading-relaxed text-slate-400">
          UI-local planning only: no runtime authority, no bridge commands, no
          apply_scenario path, no validated sensing, and no simulation behavior change.
        </p>
        <div className="mt-3 grid grid-cols-2 gap-2 text-xs">
          <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1 text-slate-300">
            Draft vertices {polygon.draftVertices.length}
          </span>
          <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1 text-slate-300">
            Polygon {hasCompleted ? "complete" : "not set"}
          </span>
          <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1 text-slate-300">
            Radar sites {radars.sites.length}
          </span>
          <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1 text-slate-300">
            Selected {selectedRadar ? selectedRadar.radar_type : "none"}
          </span>
        </div>
        <div className="mt-3 space-y-2" data-testid="planning-radar-site-list">
          {radars.sites.length === 0 ? (
            <p className="text-xs text-slate-500">No planning radar sites.</p>
          ) : (
            radars.sites.map((site) => (
              <button
                key={site.id}
                type="button"
                aria-pressed={site.id === radars.selectedSiteId}
                onClick={() => onSelectRadarSite(site.id)}
                className={`w-full rounded border px-2 py-1.5 text-left text-xs ${
                  site.id === radars.selectedSiteId
                    ? "border-yellow-500/70 bg-yellow-950/30 text-yellow-100"
                    : "border-slate-800 bg-slate-950 text-slate-300 hover:border-slate-700"
                }`}
              >
                {site.radar_type} ({site.detection_range_m}m)
              </button>
            ))
          )}
        </div>
        <div
          className="mt-3 rounded border border-slate-800 bg-slate-950/60 p-2 text-xs"
          data-testid="planning-visual-legend"
        >
          <h4 className="font-semibold uppercase tracking-wide text-slate-300">
            Visual legend
          </h4>
          <div className="mt-2 grid grid-cols-1 gap-1 text-slate-300">
            <span><span className="text-cyan-300">cyan area</span> defense area</span>
            <span><span className="text-green-300">green ring</span> radar range</span>
            <span><span className="text-green-400">green cells</span> covered cells</span>
            <span><span className="text-red-300">red cells</span> uncovered cells</span>
            <span><span className="text-red-400">red markers</span> blind spot hints</span>
          </div>
        </div>
        <div
          className="mt-3 rounded border border-slate-800 bg-slate-950/60 p-2 text-xs"
          data-testid="planning-coverage-status"
        >
          <div className="flex flex-wrap gap-2">
            <label className="inline-flex items-center gap-2 text-slate-300">
              <input
                type="checkbox"
                checked={coverageOptions.showCoverage}
                onChange={(event) =>
                  onCoverageOptionsChange({
                    ...coverageOptions,
                    showCoverage: event.currentTarget.checked,
                  })
                }
              />
              show coverage
            </label>
            <label className="inline-flex items-center gap-2 text-slate-300">
              <input
                type="checkbox"
                checked={coverageOptions.showBlindSpots}
                onChange={(event) =>
                  onCoverageOptionsChange({
                    ...coverageOptions,
                    showBlindSpots: event.currentTarget.checked,
                  })
                }
              />
              show blind spots
            </label>
          </div>
          <p className="mt-2 text-slate-400">
            Coverage is heuristic, a visual estimate, not validated sensing, and
            not runtime authority.
          </p>
          <div className="mt-2 grid grid-cols-2 gap-2">
            <span>Radar count {coverage.radarCount}</span>
            <span>Coverage {coverage.coveragePercent.toFixed(1)}%</span>
            <span>Polygon {formatAreaM2(coverage.totalPolygonAreaM2)}</span>
            <span>Covered {formatAreaM2(coverage.estimatedCoveredAreaM2)}</span>
            <span>Uncovered {formatAreaM2(coverage.estimatedUncoveredAreaM2)}</span>
            <span>Blind hints {coverage.blindSpotHints.length}</span>
          </div>
        </div>
        <div className="mt-3 grid gap-2 text-xs" data-testid="planning-radar-editor">
          <label className="grid gap-1 text-slate-300">
            Radar preset
            <select
              value={
                PLANNING_RADAR_PRESETS.find(
                  (preset) =>
                    preset.radar_type === selectedRadar?.radar_type &&
                    preset.detection_range_m === selectedRadar?.detection_range_m,
                )?.id ?? "medium"
              }
              disabled={!selectedRadar}
              onChange={(event) => {
                if (selectedRadar) {
                  onRadarPresetChange(
                    selectedRadar.id,
                    event.currentTarget.value as PlanningRadarPresetId,
                  );
                }
              }}
              className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-slate-100 disabled:opacity-40"
            >
              {PLANNING_RADAR_PRESETS.map((preset) => (
                <option key={preset.id} value={preset.id}>
                  {preset.label} ({preset.detection_range_m}m)
                </option>
              ))}
            </select>
          </label>
          <button
            type="button"
            disabled={!selectedRadar}
            onClick={() => {
              if (selectedRadar) onDeleteRadarSite(selectedRadar.id);
            }}
            className="rounded border border-rose-800 bg-rose-950/35 px-3 py-1.5 font-medium text-rose-100 disabled:cursor-not-allowed disabled:opacity-40"
          >
            Delete Radar Site
          </button>
          <button
            type="button"
            disabled={radars.sites.length === 0}
            onClick={onClearRadarSites}
            className="rounded border border-slate-700 bg-slate-950 px-3 py-1.5 font-medium text-slate-200 disabled:cursor-not-allowed disabled:opacity-40"
          >
            Clear Radar Sites
          </button>
          <button
            type="button"
            onClick={onResetCoverageState}
            className="rounded border border-slate-700 bg-slate-950 px-3 py-1.5 font-medium text-slate-200"
          >
            Reset Coverage View
          </button>

        </div>
      </div>
    </div>
  );
}

export const PlanningModePlaceholder = PlanningModePanel;

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
  sessionRuntimeProfile: SessionRuntimeProfile;
  onSessionRuntimeProfileChange: (profile: SessionRuntimeProfile) => void;
  activeRequestedRuntimeProfile: SessionRuntimeProfile;
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
  onSpawnAttacker: () => void;
  onSpawnDefenderEntity: () => void;
  onDeleteSelected: () => void;
  entityControlsDisabled: boolean;
  entityDeleteDisabled: boolean;
  onMove: (entityId: string, pose: Pose) => void;
  onDelete: (entityId: string) => void;
  onApplyToRuntime: () => void;
  applyToRuntimeDisabled: boolean;
  applyScenarioDisabled: boolean;
  applyRuntimeStatus: ApplyRuntimeStatus;
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
  runtimeBusy: boolean;
  captureBusy: boolean;
  captureSummary: LiveCaptureSummary;
  selectedDefenderId: string | null;
  selectedTargetId: string | null;
  onPauseSim: () => void;
  onResumeSim: () => void;
  onResetSession: () => void;
  onStopSession: () => void;
  onSpawnDefender: () => void;
  onStartCapture: () => void;
  onStopCapture: () => void;
  onAssignTarget: () => void;
  onCancelAssignment: () => void;
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
    sessionRuntimeProfile,
    onSessionRuntimeProfileChange,
    activeRequestedRuntimeProfile,
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
    onSpawnAttacker,
    onSpawnDefenderEntity,
    onDeleteSelected,
    entityControlsDisabled,
    entityDeleteDisabled,
    onMove,
    onDelete,
    onApplyToRuntime,
    applyToRuntimeDisabled,
    applyScenarioDisabled,
    applyRuntimeStatus,
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
    runtimeBusy,
    captureBusy,
    captureSummary,
    selectedDefenderId,
    selectedTargetId,
    onPauseSim,
    onResumeSim,
    onResetSession,
    onStopSession,
    onSpawnDefender,
    onStartCapture,
    onStopCapture,
    onAssignTarget,
    onCancelAssignment,
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
  const [workspaceMode, setWorkspaceMode] = useState<RuntimeWorkspaceMode>(
    DEFAULT_RUNTIME_WORKSPACE_MODE,
  );
  const [planningTool, setPlanningTool] = useState<PlanningTool>("select");
  const [planningPolygon, setPlanningPolygon] = useState<PlanningPolygonState>(
    EMPTY_PLANNING_POLYGON,
  );
  const [planningRadars, setPlanningRadars] = useState<PlanningRadarState>(
    EMPTY_PLANNING_RADARS,
  );
  const [planningCoverageOptions, setPlanningCoverageOptions] =
    useState<PlanningCoverageLayerOptions>(DEFAULT_PLANNING_COVERAGE_OPTIONS);
  const planningCoverage = useMemo(
    () => estimatePlanningCoverage(planningPolygon, planningRadars),
    [planningPolygon, planningRadars],
  );
  const planningModeActive = workspaceModeShowsPlanningPlaceholder(workspaceMode);
  const planningDrawingEnabled = planningToolAllowsDrawing(
    planningModeActive,
    planningTool,
  );
  const planningRadarPlacementEnabled = planningToolAllowsRadarPlacement(
    planningModeActive,
    planningTool,
  );
  const planningCesiumClickEnabled = planningToolUsesCesiumClick(
    planningModeActive,
    planningTool,
  );
  const effectiveEditingEnabled = editingEnabledForWorkspaceMode(
    workspaceMode,
    editingEnabled,
  );
  const cesiumEntityEditingEnabled = effectiveEditingEnabled && !planningCesiumClickEnabled;
  const handleWorkspaceModeChange = (mode: RuntimeWorkspaceMode) => {
    setWorkspaceMode(mode);
    if (mode === "grid") setPlanningTool("select");
  };
  const handlePlanningMapClick = useCallback(
    (vertex: PlanningVertex) => {
      if (planningDrawingEnabled) {
        setPlanningPolygon((current) => addPlanningVertex(current, vertex));
        return;
      }
      if (planningRadarPlacementEnabled) {
        setPlanningRadars((current) => addPlanningRadarSite(current, vertex));
      }
    },
    [planningDrawingEnabled, planningRadarPlacementEnabled],
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
            busy={busy || runtimeBusy || captureBusy}
            atCapacity={atCapacity}
            sessionRuntimeProfile={sessionRuntimeProfile}
            onSessionRuntimeProfileChange={onSessionRuntimeProfileChange}
            activeRequestedRuntimeProfile={activeRequestedRuntimeProfile}
            onConnect={onConnectNewSession}
            onDisconnect={onDisconnectSelected}
          />
          {connected && (
            <LifecycleControlBar
              connected={connected}
              editingEnabled={editingEnabled}
              busy={busy || runtimeBusy}
              sessionState={sessionState}
              onPause={onPauseSim}
              onResume={onResumeSim}
              onReset={onResetSession}
              onStopSession={onStopSession}
            />
          )}
          {connected && (
            <ScenarioControlBar
              connected={connected}
              applyDisabled={applyScenarioDisabled}
              entityCount={entities.length}
              onApplyScenario={onApplyToRuntime}
            />
          )}
          {connected && (
            <EntityControlBar
              connected={connected}
              controlsDisabled={entityControlsDisabled}
              deleteDisabled={entityDeleteDisabled}
              selectedEntityId={selectedEntityId}
              onSpawnAttacker={onSpawnAttacker}
              onSpawnDefender={onSpawnDefenderEntity}
              onDeleteSelected={onDeleteSelected}
            />
          )}
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
          <div className="flex flex-col gap-1">
            <RuntimeControlBar
              connected={connected}
              editingEnabled={editingEnabled}
              busy={busy || runtimeBusy}
              onSpawnDefender={onSpawnDefender}
            />
            <CaptureControlBar
              connected={connected}
              editingEnabled={editingEnabled}
              busy={busy || captureBusy}
              captureActive={captureSummary.captureActive}
              onStartCapture={onStartCapture}
              onStopCapture={onStopCapture}
            />
            {connected && sessionId && (
              <CaptureSummaryStrip summary={captureSummary} />
            )}
          </div>
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
          requestedRuntimeProfile={activeRequestedRuntimeProfile}
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
            lastPullUtc={lastPullUtc}
            pullHz={pullHz}
            requestedRuntimeProfile={activeRequestedRuntimeProfile}
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
            <RuntimeWorkspaceModeSelector
              mode={workspaceMode}
              onModeChange={handleWorkspaceModeChange}
            />
            {!workspaceModeShowsPlanningPlaceholder(workspaceMode) ? (
              <>
                <EntityPalette
                  selectedType={selectedType}
                  onSelectType={onSelectType}
                  entityCountsByType={mergedEntityCounts}
                  editingEnabled={effectiveEditingEnabled}
                />
                <WorldEditingGrid
                  entities={entities}
                  selectedEntityId={selectedEntityId}
                  selectedType={selectedType}
                  editingEnabled={effectiveEditingEnabled}
                  worldSummary={mergedWorldSummary}
                  mirrorSnapshot={snapshots.entity_pose_mirror}
                  captureActive={captureSummary.captureActive}
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
                  onApplyToRuntime={onApplyToRuntime}
                  applyToRuntimeDisabled={applyToRuntimeDisabled}
                  applyRuntimeStatus={applyRuntimeStatus}
                />
                <ScenarioEvaluationPanel
                  entities={entities}
                  sessionId={sessionId}
                  disabled={!effectiveEditingEnabled}
                />
                <EditingCognitionStrip
                  lastCommand={lastCommand}
                  pendingReconcile={pendingReconcile}
                  mirrorSnapshot={snapshots.entity_pose_mirror}
                />
                <EditHistoryPanel history={editHistory} />
              </>
            ) : (
              <PlanningModePanel
                tool={planningTool}
                polygon={planningPolygon}
                radars={planningRadars}
                coverage={planningCoverage}
                coverageOptions={planningCoverageOptions}
                onToolChange={setPlanningTool}
                onFinishPolygon={() =>
                  setPlanningPolygon((current) => finishPlanningPolygon(current))
                }
                onCancelDrawing={() =>
                  setPlanningPolygon((current) => cancelPlanningDrawing(current))
                }
                onClearPolygon={() => setPlanningPolygon(clearPlanningPolygon())}
                onSelectRadarSite={(siteId) =>
                  setPlanningRadars((current) =>
                    selectPlanningRadarSite(current, siteId),
                  )
                }
                onDeleteRadarSite={(siteId) =>
                  setPlanningRadars((current) => deletePlanningRadarSite(current, siteId))
                }
                onRadarPresetChange={(siteId, presetId) =>
                  setPlanningRadars((current) =>
                    updatePlanningRadarPreset(current, siteId, presetId),
                  )
                }
                onClearRadarSites={() =>
                  setPlanningRadars((current) => clearPlanningRadarSites(current))
                }
                onCoverageOptionsChange={setPlanningCoverageOptions}
                onResetCoverageState={() =>
                  setPlanningCoverageOptions(DEFAULT_PLANNING_COVERAGE_OPTIONS)
                }
              />
            )}
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
            editingEnabled={cesiumEntityEditingEnabled}
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
            planningDrawing={{
              enabled: planningCesiumClickEnabled,
              polygon: planningPolygon,
              radars: planningRadars,
              coverageOptions: planningCoverageOptions,
              onMapClick: handlePlanningMapClick,
            }}
            onLayerVisibilityChange={onLayerVisibilityChange}
            tacticalState={tactical.state}
            tacticalRecommendation={tactical.recommendation}
            slotList={slotList}
          />
        ) : (
          <ConnectPlaceholder
            sessionRuntimeProfile={sessionRuntimeProfile}
            onSessionRuntimeProfileChange={onSessionRuntimeProfileChange}
          />
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
              selectedDefenderId={selectedDefenderId}
              selectedTargetId={selectedTargetId}
              mode={tactical.mode}
              state={tactical.state}
              busy={tactical.busy || runtimeBusy}
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
              onAssignTarget={onAssignTarget}
              onCancelAssignment={onCancelAssignment}
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
            <AdapterStatusPanel
              sessionHealth={snapshots.session_health}
              worldSummary={snapshots.world_summary}
              lastPullUtc={lastPullUtc}
              pullHz={pullHz}
              requestedRuntimeProfile={activeRequestedRuntimeProfile}
              pendingRuntimeProfile={connected ? null : sessionRuntimeProfile}
            />
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
              sessionId={sessionId}
              captureSummary={captureSummary}
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
