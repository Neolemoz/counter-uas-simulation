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
import type { CameraLocationTarget, CameraPreset } from "@/cesium/cameraHelpers";
import {
  CESIUM_TERRAIN_PROVIDER_OPTIONS,
  TERRAIN_PROVIDER_VISUAL_ONLY_COPY,
  terrainProviderModeLabel,
} from "@/cesium/terrainProviderConfig";
import type { TerrainLayerVisibility } from "@/cesium/terrainLayers";
import { analyzePlanningCoverage, type PlanningCoverageAnalysis } from "@/cesium/planningCoverageAnalysis";
import type { CesiumTerrainProviderMode } from "@/cesium/terrainProviderConfig";
import type { VisualLayerVisibility } from "@/cesium/visualLayerRegistry";
import { CesiumRuntimePanel } from "@/components/CesiumRuntimePanel";
import { ScenarioEvaluationPanel } from "@/components/ScenarioEvaluationPanel";
import { WorldEditingGrid } from "@/components/WorldEditingGrid";
import type { ApplyRuntimeStatus } from "@/components/WorldEditorApplyStatus";
import type { EditHistoryEntry, EditCommandType } from "@/editing/editHistory";
import type { UiEntity } from "@/editing/localEntityMirror";
import { ExperimentWorkbenchPanel } from "@/experiment/ExperimentWorkbenchPanel";
import type { AdvisoryExperimentRollup } from "@/handoff/advisoryTypes";
import { IntelligenceAdvisoryPanel } from "@/intelligence/IntelligenceAdvisoryPanel";
import { SelectedTargetAdvisoryCard } from "@/intelligence/SelectedTargetAdvisoryCard";
import { ThreatEvaluationWorkbench } from "@/intelligence/workbench/ThreatEvaluationWorkbench";
import { INTELLIGENCE_ADVISORY_BANNER } from "@/intelligence/intelligenceGovernance";
import { getAdvisoryTransportFromSnapshot, getSelectedEntityAdvisory } from "@/intelligence/intelligenceSelectors";
import { SelectedTrackSensorWorkbench } from "@/tracks/workbench/SelectedTrackSensorWorkbench";
import { LiveTraceabilityWorkbenchSurface } from "@/traceability/workbench/LiveTraceabilityWorkbenchSurface";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";
import type { useTacticalState } from "@/hooks/useTacticalState";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { LiveCaptureSummary } from "@/telemetry/captureSummary";
import type { TelemetryChannel } from "@/telemetry/constants";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import { unifiedWorldCopy } from "@/world/bounds";
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
import {
  DEFAULT_PLANNING_LOCATION_PRESET_ID,
  PLANNING_LOCATION_GOVERNANCE_COPY,
  PLANNING_LOCATION_PRESETS,
  planningLocationPreset,
  validatedPlanningCoordinates,
  type PlanningLocationPresetId,
} from "@/cesium/planningLocations";
import {
  PLANNING_EXTENT_GOVERNANCE_COPY,
  UNIFIED_PLANNING_WORLD,
  unifiedPlanningWorld,
} from "@/cesium/planningWorld";
import { isInsideWorldBounds } from "@/cesium/planningExtentLayer";
import {
  DEFAULT_PLANNING_MEASUREMENT_STATE,
  PLANNING_MEASUREMENT_GOVERNANCE_COPY,
  PLANNING_RADIUS_OPTIONS_M,
  addPlanningMeasurementPoint,
  clearPlanningMeasurements,
  planningMeasurementSummary,
  setPlanningRadiusMeters,
  type PlanningMeasurementState,
  type PlanningRadiusMeters,
} from "@/cesium/planningMeasurements";
import {
  PLANNING_COGNITION_GOVERNANCE_COPY,
  buildPlanningSummary,
  buildPlanningWarnings,
  operationalRingSummary,
  validateUnifiedPlanningWorld,
} from "@/cesium/planningCognition";
import {
  buildPlanningMcSnapshot,
  planningGeometryFingerprint,
} from "@/layout/planningMcSnapshot";
import {
  capturePlanningLayoutCompareSlot,
  clearPlanningLayoutCompareSlots,
  nextAvailablePlanningLayoutCompareSlotLabel,
  parsePlanningMcSnapshotJson,
  removePlanningLayoutCompareSlot,
  setPlanningLayoutCompareSlot,
  type PlanningLayoutCompareSlotLabel,
  type PlanningLayoutCompareSlotV1,
} from "@/layout/planningLayoutComparison";
import {
  buildPlanningLayoutComparePanelDerivation,
  PlanningLayoutComparePanel,
} from "@/workstation/PlanningLayoutComparePanel";
import {
  buildPlanningMcPackage,
  copyPlanningMcPackage,
  downloadPlanningMcPackage,
  type PlanningMcPackageV1,
} from "@/layout/planningMcPackage";
import {
  buildMockPlanningMcResultRef,
  buildPlanningResultLink,
  buildPlanningResultLinkPreview,
  parsePlanningMcResultRefJson,
  type PlanningMcResultRefV1,
  type PlanningResultLinkV1,
} from "@/layout/planningMcResultLink";
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
    { mode: "grid", label: "Core Grid (local)" },
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
  coverageAnalysis,
  planningMeasurements,
  planningMcPackage,
  planningMcPackageStale,
  planningResultLink,
  planningResultLinkPreview,
  planningResultImportText,
  planningResultImportError,
  terrainProviderMode,
  locationPresetId,
  customLatitude,
  customLongitude,
  onTerrainProviderModeChange,
  onLocationPresetChange,
  onCustomLatitudeChange,
  onCustomLongitudeChange,
  onApplyLocation,
  onCameraPreset,
  onPlanningWorldCameraFit,
  onPlanningRadiusChange,
  onClearPlanningMeasurements,
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
  onGeneratePlanningMcPackage,
  onCopyPlanningMcPackage,
  onDownloadPlanningMcPackage,
  onPlanningResultImportTextChange,
  onImportPlanningResultMetadata,
  onImportMockPlanningResultRef,
  onClearPlanningResultImport,
  planningLayoutCompareSlots,
  planningLayoutCompareAnalytics,
  planningLayoutCompareCaptureDisabled,
  planningLayoutCompareImportSlotLabel,
  planningLayoutCompareImportText,
  planningLayoutCompareImportError,
  onCapturePlanningLayoutCompare,
  onRemovePlanningLayoutCompareSlot,
  onClearPlanningLayoutCompareSlots,
  onPlanningLayoutCompareImportSlotLabelChange,
  onPlanningLayoutCompareImportTextChange,
  onImportPlanningLayoutCompareSnapshot,
}: {
  tool: PlanningTool;
  polygon: PlanningPolygonState;
  radars: PlanningRadarState;
  coverage: PlanningCoverageEstimate;
  coverageOptions: PlanningCoverageLayerOptions;
  coverageAnalysis: PlanningCoverageAnalysis;
  planningMeasurements: PlanningMeasurementState;
  planningMcPackage: PlanningMcPackageV1 | null;
  planningMcPackageStale: boolean;
  planningResultLink: PlanningResultLinkV1 | null;
  planningResultLinkPreview: ReturnType<typeof buildPlanningResultLinkPreview>;
  planningResultImportText: string;
  planningResultImportError: string | null;
  terrainProviderMode: CesiumTerrainProviderMode;
  locationPresetId: PlanningLocationPresetId;
  customLatitude: string;
  customLongitude: string;
  onTerrainProviderModeChange: (mode: CesiumTerrainProviderMode) => void;
  onLocationPresetChange: (presetId: PlanningLocationPresetId) => void;
  onCustomLatitudeChange: (value: string) => void;
  onCustomLongitudeChange: (value: string) => void;
  onApplyLocation: () => void;
  onCameraPreset: (preset: CameraPreset) => void;
  onPlanningWorldCameraFit: () => void;
  onPlanningRadiusChange: (radiusMeters: PlanningRadiusMeters) => void;
  onClearPlanningMeasurements: () => void;
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
  onGeneratePlanningMcPackage: () => void;
  onCopyPlanningMcPackage: () => void;
  onDownloadPlanningMcPackage: () => void;
  onPlanningResultImportTextChange: (value: string) => void;
  onImportPlanningResultMetadata: () => void;
  onImportMockPlanningResultRef: () => void;
  onClearPlanningResultImport: () => void;
  planningLayoutCompareSlots: PlanningLayoutCompareSlotV1[];
  planningLayoutCompareAnalytics: ReturnType<typeof buildPlanningLayoutComparePanelDerivation>;
  planningLayoutCompareCaptureDisabled: boolean;
  planningLayoutCompareImportSlotLabel: PlanningLayoutCompareSlotLabel;
  planningLayoutCompareImportText: string;
  planningLayoutCompareImportError: string | null;
  onCapturePlanningLayoutCompare: () => void;
  onRemovePlanningLayoutCompareSlot: (slotLabel: PlanningLayoutCompareSlotV1["slot_label"]) => void;
  onClearPlanningLayoutCompareSlots: () => void;
  onPlanningLayoutCompareImportSlotLabelChange: (slotLabel: PlanningLayoutCompareSlotLabel) => void;
  onPlanningLayoutCompareImportTextChange: (value: string) => void;
  onImportPlanningLayoutCompareSnapshot: () => void;
}) {
  const canFinish = canFinishPlanningPolygon(polygon);
  const hasDraft = polygon.draftVertices.length > 0;
  const hasCompleted = (polygon.completedVertices?.length ?? 0) > 0;
  const selectedRadar =
    radars.sites.find((site) => site.id === radars.selectedSiteId) ?? null;
  const overlapPercent = coverageAnalysis.overlapPercent;
  const redundancyPercent = coverageAnalysis.redundancyPercent;

  const terrainProviderOn = terrainProviderMode === "cesium_world_terrain";
  const selectedLocationPreset = planningLocationPreset(locationPresetId);
  const customCoordinatesValid =
    validatedPlanningCoordinates(customLatitude, customLongitude) !== null;
  const cameraPresets: { preset: CameraPreset; label: string }[] = [
    { preset: "terrainOverview", label: "Overview" },
    { preset: "ridgeLine", label: "Ridge" },
    { preset: "valleyFloor", label: "Valley" },
    { preset: "sensorContext", label: "Sensor Context" },
  ];

  const toolOptions: { tool: PlanningTool; label: string }[] = [
    { tool: "select", label: "Select" },
    { tool: "draw_defense_area", label: "Draw Defense Area" },
    { tool: "place_radar_site", label: "Place Radar Site" },
    { tool: "measure_distance", label: "Measure" },
  ];

  const planningCoordinates = [
    ...(polygon.completedVertices ?? []),
    ...polygon.draftVertices,
    ...radars.sites.map((site) => site.position),
  ];
  const worldInvalidCoordinateCount = planningCoordinates.filter(
    (vertex) => !isInsideWorldBounds(vertex),
  ).length;
  const operationalRings = operationalRingSummary();
  const measurementSummary = planningMeasurementSummary(planningMeasurements);
  const planningSummary = buildPlanningSummary({
    polygon,
    radars,
    measurements: planningMeasurements,
  });
  const planningWarnings = buildPlanningWarnings({
    polygon,
    radars,
    measurements: planningMeasurements,
    activeTool: tool,
  });
  const worldValidation = validateUnifiedPlanningWorld();

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
        <div className="mt-3 rounded border border-cyan-900/60 bg-cyan-950/15 p-2 text-xs" data-testid="planning-extent-controls">
          <div className="flex flex-wrap items-center justify-between gap-2">
            <div>
              <p className="font-semibold uppercase tracking-wide text-slate-300">Planning World</p>
              <p className="mt-1 text-slate-500">
                {UNIFIED_PLANNING_WORLD.planning_extent_label} — {unifiedWorldCopy()}
              </p>
            </div>
            <button
              type="button"
              onClick={onPlanningWorldCameraFit}
              className="rounded border border-cyan-700/60 bg-cyan-950/45 px-2.5 py-1.5 font-semibold text-cyan-100"
            >
              Fit Unified World
            </button>
          </div>
          <p className="mt-2 text-[11px] leading-relaxed text-cyan-100/80">
            {PLANNING_EXTENT_GOVERNANCE_COPY}
          </p>
          {worldInvalidCoordinateCount > 0 && (
            <p className="mt-2 rounded border border-amber-700/60 bg-amber-950/35 px-2 py-1 text-[11px] text-amber-100" data-testid="planning-world-guardrail">
              World-invalid coordinates: {worldInvalidCoordinateCount} planning coordinate
              {worldInvalidCoordinateCount === 1 ? "" : "s"} outside {unifiedWorldCopy()}.
              Re-draw inside world bounds.
            </p>
          )}
        </div>
        <div className="mt-3 rounded border border-amber-900/60 bg-amber-950/15 p-2 text-xs" data-testid="planning-measurement-controls">
          <div className="flex flex-wrap items-center justify-between gap-2">
            <div>
              <p className="font-semibold uppercase tracking-wide text-slate-300">Measurement tools</p>
              <p className="mt-1 text-slate-500">Select Measure, then click two Planning-only map points for distance and bearing readout.</p>
            </div>
            <button
              type="button"
              onClick={onClearPlanningMeasurements}
              className="rounded border border-amber-700/60 bg-amber-950/45 px-2.5 py-1.5 font-semibold text-amber-100"
            >
              Clear Measurements
            </button>
          </div>
          <label className="mt-2 grid gap-1 text-slate-400">
            Radius Display
            <select
              value={planningMeasurements.radiusMeters}
              onChange={(event) =>
                onPlanningRadiusChange(Number(event.currentTarget.value) as PlanningRadiusMeters)
              }
              className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-slate-100"
            >
              {PLANNING_RADIUS_OPTIONS_M.map((radiusMeters) => (
                <option key={radiusMeters} value={radiusMeters}>
                  {radiusMeters / 1000} km radius
                </option>
              ))}
            </select>
          </label>
          <div className="mt-2 grid gap-1 text-slate-300" data-testid="planning-measurement-readout">
            <span>Distance m {measurementSummary.distance_m === null ? "Select two points" : Math.round(measurementSummary.distance_m)}</span>
            <span>Distance km {measurementSummary.distance_km === null ? "Select two points" : measurementSummary.distance_km.toFixed(2)}</span>
            <span>Bearing {measurementSummary.bearing ?? "Select two points"}</span>
            <span>Start {measurementSummary.start_readout ?? "No point selected"}</span>
            <span>End {measurementSummary.end_readout ?? "No point selected"}</span>
            <span>Radius {measurementSummary.radius_label}</span>
          </div>
          <p className="mt-2 text-[11px] leading-relaxed text-amber-100/80">
            {PLANNING_MEASUREMENT_GOVERNANCE_COPY}
          </p>
        </div>
        <div className="mt-3 rounded border border-slate-800 bg-slate-950/55 p-2 text-xs" data-testid="planning-terrain-controls">
          <div className="flex flex-wrap items-center justify-between gap-2">
            <div>
              <p className="font-semibold uppercase tracking-wide text-slate-300">Terrain controls</p>
              <p className="mt-1 text-slate-500">
                Source {terrainProviderModeLabel(terrainProviderMode)} · status {terrainProviderOn ? "on" : "off"}
              </p>
            </div>
            <button
              type="button"
              aria-pressed={terrainProviderOn}
              onClick={() =>
                onTerrainProviderModeChange(
                  terrainProviderOn ? "ellipsoid" : "cesium_world_terrain",
                )
              }
              className="rounded border border-slate-700 bg-slate-950 px-2.5 py-1.5 font-semibold text-slate-200"
            >
              Terrain {terrainProviderOn ? "On" : "Off"}
            </button>
          </div>
          <label className="mt-2 grid gap-1 text-slate-400">
            Terrain Source
            <select
              value={terrainProviderMode}
              onChange={(event) =>
                onTerrainProviderModeChange(
                  event.currentTarget.value as CesiumTerrainProviderMode,
                )
              }
              className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-slate-100"
            >
              {CESIUM_TERRAIN_PROVIDER_OPTIONS.map((option) => (
                <option key={option.mode} value={option.mode}>
                  {option.label}
                </option>
              ))}
            </select>
          </label>
        </div>
        <div className="mt-3 rounded border border-slate-800 bg-slate-950/55 p-2 text-xs" data-testid="planning-location-controls">
          <div className="flex flex-wrap items-center justify-between gap-2">
            <div>
              <p className="font-semibold uppercase tracking-wide text-slate-300">Location presets</p>
              <p className="mt-1 text-slate-500">
                Current {selectedLocationPreset.label}
              </p>
            </div>
            <button
              type="button"
              onClick={onApplyLocation}
              disabled={locationPresetId === "custom" && !customCoordinatesValid}
              className="rounded border border-slate-700 bg-slate-950 px-2.5 py-1.5 font-semibold text-slate-200 disabled:cursor-not-allowed disabled:opacity-40"
            >
              Jump Camera
            </button>
          </div>
          <label className="mt-2 grid gap-1 text-slate-400">
            Location Preset
            <select
              value={locationPresetId}
              onChange={(event) =>
                onLocationPresetChange(event.currentTarget.value as PlanningLocationPresetId)
              }
              className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-slate-100"
            >
              {PLANNING_LOCATION_PRESETS.map((preset) => (
                <option key={preset.id} value={preset.id}>
                  {preset.label}
                </option>
              ))}
            </select>
          </label>
          <div className="mt-2 grid grid-cols-2 gap-2">
            <label className="grid gap-1 text-slate-400">
              Latitude
              <input
                type="text"
                value={customLatitude}
                onChange={(event) => onCustomLatitudeChange(event.currentTarget.value)}
                className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-slate-100"
              />
            </label>
            <label className="grid gap-1 text-slate-400">
              Longitude
              <input
                type="text"
                value={customLongitude}
                onChange={(event) => onCustomLongitudeChange(event.currentTarget.value)}
                className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-slate-100"
              />
            </label>
          </div>
          <p className={`mt-2 text-[11px] `}>
            {locationPresetId === "custom" && !customCoordinatesValid
              ? "Custom coordinates must use latitude -90..90 and longitude -180..180."
              : PLANNING_LOCATION_GOVERNANCE_COPY}
          </p>
        </div>
        <div className="mt-3 rounded border border-slate-800 bg-slate-950/55 p-2 text-xs" data-testid="planning-camera-presets">
          <p className="font-semibold uppercase tracking-wide text-slate-300">Camera presets</p>
          <div className="mt-2 grid grid-cols-2 gap-2">
            {cameraPresets.map((preset) => (
              <button
                key={preset.preset}
                type="button"
                onClick={() => onCameraPreset(preset.preset)}
                className="rounded border border-slate-700 bg-slate-950 px-2.5 py-1.5 font-semibold text-slate-300 hover:border-cyan-700 hover:text-cyan-100"
              >
                {preset.label}
              </button>
            ))}
          </div>
        </div>
        <div className="mt-3 rounded border border-amber-900/60 bg-amber-950/15 p-2 text-[11px] leading-relaxed text-amber-100/85" data-testid="planning-terrain-legend">
          {TERRAIN_PROVIDER_VISUAL_ONLY_COPY} {PLANNING_LOCATION_GOVERNANCE_COPY} Terrain and location presets do not affect planning metrics. Planning polygon, radar sites, coverage overlay, and blind spot markers remain display overlays in both terrain modes.
        </div>
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
        <div
          className="mt-3 rounded border border-cyan-900/60 bg-cyan-950/20 p-2 text-xs"
          data-testid="planning-cognition-panel"
        >
          <h4 className="font-semibold uppercase tracking-wide text-cyan-100">
            Planning cognition
          </h4>
          <p className="mt-1 text-[11px] leading-relaxed text-slate-400">
            {PLANNING_COGNITION_GOVERNANCE_COPY}
          </p>
          <div className="mt-2 grid grid-cols-2 gap-2 text-slate-300" data-testid="planning-summary">
            <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1">
              World {planningSummary.world_label}
            </span>
            <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1">
              Half-extent ±{planningSummary.world_half_extent_m.toLocaleString()}m
            </span>
            <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1">
              Polygons {planningSummary.polygon_count}
            </span>
            <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1">
              Radar sites {planningSummary.radar_count}
            </span>
            <span className="rounded border border-slate-800 bg-slate-950 px-2 py-1 col-span-2">
              Measurements {planningSummary.measurement_count}
            </span>
          </div>
          <div
            className="mt-2 grid grid-cols-2 gap-2 text-[11px] text-slate-400"
            data-testid="planning-operational-rings"
          >
            <span className="rounded border border-slate-800/80 bg-slate-950/80 px-2 py-1">
              City radius {operationalRings.city_radius_m.toLocaleString()} m
            </span>
            <span className="rounded border border-slate-800/80 bg-slate-950/80 px-2 py-1">
              Defense radius {operationalRings.defense_radius_m.toLocaleString()} m
            </span>
            <span className="rounded border border-slate-800/80 bg-slate-950/80 px-2 py-1">
              Warning radius {operationalRings.warning_radius_m.toLocaleString()} m
            </span>
            <span className="rounded border border-slate-800/80 bg-slate-950/80 px-2 py-1">
              Spawn band {operationalRings.spawn_band_inner_m.toLocaleString()}–
              {operationalRings.spawn_band_outer_m.toLocaleString()} m
            </span>
          </div>
          <p className="mt-2 text-[11px] text-slate-500" data-testid="planning-world-guidance">
            {worldValidation.guidance}
          </p>
          {planningWarnings.length > 0 && (
            <ul
              className="mt-2 space-y-1 text-[11px] leading-relaxed text-amber-100/90"
              data-testid="planning-warnings"
            >
              {planningWarnings.map((warning) => (
                <li
                  key={warning.warning_id}
                  className="rounded border border-amber-800/50 bg-amber-950/25 px-2 py-1"
                  data-testid={`planning-warning-${warning.warning_id}`}
                >
                  {warning.message}
                </li>
              ))}
            </ul>
          )}
        </div>
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
          <div className="mt-3 rounded border border-slate-800 bg-slate-950/70 p-2" data-testid="planning-analytics-v2">
            <p className="text-[11px] font-semibold uppercase tracking-wide text-slate-300">
              Planning analytics
            </p>
            <p className="mt-1 text-[11px] text-slate-500">Planning heuristic only.</p>
            <div className="mt-2 grid grid-cols-3 gap-2 text-slate-300">
              <span>Coverage % {coverage.coveragePercent.toFixed(1)}%</span>
              <span>Overlap % {overlapPercent.toFixed(1)}%</span>
              <span>Redundancy % {redundancyPercent.toFixed(1)}%</span>
            </div>
          </div>
          <div className="mt-3 rounded border border-slate-800 bg-slate-950/70 p-2" data-testid="planning-advisory-v2">
            <p className="text-[11px] font-semibold uppercase tracking-wide text-slate-300">
              Planning advisory
            </p>
            <p className="mt-1 text-[11px] text-slate-500">Advisory planning heuristic only.</p>
            <div className="mt-2 grid gap-1 text-slate-300">
              <span>Blind Spot Summary {coverageAnalysis.blindSpotV2.summary}</span>
              <span>Suggested Radar {coverageAnalysis.radarRecommendation ? coverageAnalysis.radarRecommendation.recommendedPresetLabel : "No additional planning radar suggested."}</span>
              <span>Suggested Position {coverageAnalysis.radarRecommendation ? `${Math.round(coverageAnalysis.radarRecommendation.approximatePlacement.x)}, ${Math.round(coverageAnalysis.radarRecommendation.approximatePlacement.y)}` : "None"}</span>
              <span>Reason {coverageAnalysis.radarRecommendation ? coverageAnalysis.radarRecommendation.reason : "Sampled planning cells are covered."}</span>
            </div>
          </div>
          <div className="mt-3 rounded border border-violet-900/60 bg-violet-950/20 p-2" data-testid="planning-mc-package-preview">
            <p className="text-[11px] font-semibold uppercase tracking-wide text-violet-200">
              Planning MC package preview
            </p>
            <p className="mt-1 text-[11px] text-slate-500">
              Read-only package preview; no MC execution, no job creation, no runtime or bridge changes.
            </p>
            {planningMcPackageStale && (
              <p className="mt-2 rounded border border-amber-700/60 bg-amber-950/35 px-2 py-1 text-[11px] text-amber-100" role="alert">
                Package may be stale. Regenerate.
              </p>
            )}
            {planningMcPackage ? (
              <div className="mt-2 grid gap-1 text-slate-300">
                <span>planning_snapshot_id {planningMcPackage.planning_snapshot_id}</span>
                <span>planning_geometry_id {planningMcPackage.planning_geometry_id}</span>
                <span>planning_extent {planningMcPackage.planning_extent.planning_extent_label} ({planningMcPackage.planning_extent.planning_extent_radius_m.toLocaleString()}m)</span>
                <span>Radar count {planningMcPackage.planning_summary.radar_count}</span>
                <span>Coverage summary {planningMcPackage.planning_summary.coverage_summary.coverage_percent.toFixed(1)}%</span>
                <span>Overlap summary {planningMcPackage.planning_summary.overlap_summary.overlap_percent.toFixed(1)}%</span>
                <span>Redundancy summary {planningMcPackage.planning_summary.redundancy_summary.redundancy_percent.toFixed(1)}%</span>
                <span>Suggested MC settings {planningMcPackage.mc_preparation.scenario_label} · {planningMcPackage.mc_preparation.suggested_run_count} runs · seed {planningMcPackage.mc_preparation.suggested_seed_base}</span>
              </div>
            ) : (
              <p className="mt-2 text-[11px] text-slate-500">No Planning MC package generated.</p>
            )}
            <div className="mt-2 flex flex-wrap gap-2">
              <button
                type="button"
                onClick={onGeneratePlanningMcPackage}
                className="rounded border border-violet-700/60 bg-violet-950/45 px-2 py-1 text-[10px] font-semibold uppercase text-violet-100"
              >
                Generate package
              </button>
              <button
                type="button"
                disabled={!planningMcPackage}
                onClick={onCopyPlanningMcPackage}
                className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 disabled:opacity-40"
              >
                Copy package JSON
              </button>
              <button
                type="button"
                disabled={!planningMcPackage}
                onClick={onDownloadPlanningMcPackage}
                className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 disabled:opacity-40"
              >
                Download package JSON
              </button>
            </div>
          </div>
          <div
            className="mt-3 rounded border border-emerald-900/60 bg-emerald-950/20 p-2"
            data-testid="planning-mc-result-link-preview"
          >
            <p className="text-[11px] font-semibold uppercase tracking-wide text-emerald-200">
              Planning MC result linkage
            </p>
            <p className="mt-1 text-[11px] text-slate-500">
              Read-only metadata import and linkage preview; no MC execution, no filesystem reads.
            </p>
            <div className="mt-2 grid gap-1 text-slate-300">
              <span>Linkage status {planningResultLinkPreview.statusLabel}</span>
              <span>Result link {planningResultLink ? `${planningResultLink.schema_version} · ${planningResultLink.status}` : "unlinked"}</span>
              {planningResultLinkPreview.mcResultId ? (
                <span>mc_result_id {planningResultLinkPreview.mcResultId}</span>
              ) : null}
              {planningResultLinkPreview.mcRunLabel ? (
                <span>mc_run_label {planningResultLinkPreview.mcRunLabel}</span>
              ) : null}
              {planningResultLinkPreview.importedUtc ? (
                <span>imported_utc {planningResultLinkPreview.importedUtc}</span>
              ) : null}
              {planningResultLinkPreview.successRate !== null ? (
                <span>success_rate {(planningResultLinkPreview.successRate * 100).toFixed(1)}%</span>
              ) : null}
              {planningResultLinkPreview.missDistanceP95 !== null ? (
                <span>miss_distance_p95 {planningResultLinkPreview.missDistanceP95.toFixed(1)} m</span>
              ) : null}
              {planningResultLinkPreview.interceptTimeMean !== null ? (
                <span>intercept_time_mean {planningResultLinkPreview.interceptTimeMean.toFixed(1)} s</span>
              ) : null}
            </div>
            {planningResultImportError ? (
              <p className="mt-2 rounded border border-rose-700/60 bg-rose-950/35 px-2 py-1 text-[11px] text-rose-100" role="alert">
                {planningResultImportError}
              </p>
            ) : null}
            <label className="mt-2 grid gap-1 text-[11px] text-slate-400">
              Paste MC result metadata JSON
              <textarea
                value={planningResultImportText}
                onChange={(event) => onPlanningResultImportTextChange(event.currentTarget.value)}
                disabled={!planningMcPackage}
                rows={3}
                className="rounded border border-slate-800 bg-slate-950 px-2 py-1 font-mono text-[10px] text-slate-200 disabled:opacity-40"
                placeholder='{"schema_version":"rt_planning_mc_result_ref_v1",...}'
              />
            </label>
            <div className="mt-2 flex flex-wrap gap-2">
              <button
                type="button"
                disabled={!planningMcPackage || planningResultImportText.trim().length === 0}
                onClick={onImportPlanningResultMetadata}
                className="rounded border border-emerald-700/60 bg-emerald-950/45 px-2 py-1 text-[10px] font-semibold uppercase text-emerald-100 disabled:opacity-40"
              >
                Import metadata
              </button>
              <button
                type="button"
                disabled={!planningMcPackage}
                onClick={onImportMockPlanningResultRef}
                className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 disabled:opacity-40"
              >
                Import mock result ref
              </button>
              <button
                type="button"
                disabled={!planningResultLink?.result_ref}
                onClick={onClearPlanningResultImport}
                className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 disabled:opacity-40"
              >
                Clear import
              </button>
            </div>
          </div>
          <PlanningLayoutComparePanel
            slots={planningLayoutCompareSlots}
            analytics={planningLayoutCompareAnalytics}
            captureDisabled={planningLayoutCompareCaptureDisabled}
            importSlotLabel={planningLayoutCompareImportSlotLabel}
            importText={planningLayoutCompareImportText}
            importError={planningLayoutCompareImportError}
            onImportSlotLabelChange={onPlanningLayoutCompareImportSlotLabelChange}
            onImportTextChange={onPlanningLayoutCompareImportTextChange}
            onImportSnapshot={onImportPlanningLayoutCompareSnapshot}
            onCaptureCurrentLayout={onCapturePlanningLayoutCompare}
            onRemoveSlot={onRemovePlanningLayoutCompareSlot}
            onClearAllSlots={onClearPlanningLayoutCompareSlots}
          />
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
  livePreflight?: import("@/runtime/livePreflight").LivePreflightResult | null;
  preflightLoading?: boolean;
  preflightError?: string | null;
  onConnectNewSession: () => void;
  onDisconnectSelected: () => void;
  onCloseSession: (sessionId: string) => void;
  onPullHzChange: (hz: number) => void;
  onAutoRefreshChange: (enabled: boolean) => void;
  onRefresh: () => void;
  layerVisibility: VisualLayerVisibility;
  terrainLayers: TerrainLayerVisibility;
  terrainLayersOn: boolean;
  terrainProviderMode: CesiumTerrainProviderMode;
  onTerrainProviderModeChange: (mode: CesiumTerrainProviderMode) => void;
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

function ThreatEvaluationWorkbenchSurface({
  advisory,
  stale,
  staleReason,
}: {
  advisory: ReturnType<typeof getSelectedEntityAdvisory>;
  stale: boolean;
  staleReason: string | null;
}) {
  if (!advisory) {
    return (
      <section
        className="rounded border border-slate-800 bg-slate-950/50 p-3 text-xs"
        data-testid="threat-evaluation-workbench-empty"
      >
        <p className="text-[10px] text-amber-100/80">{INTELLIGENCE_ADVISORY_BANNER}</p>
        <p className="mt-2 font-semibold uppercase tracking-wide text-slate-300">
          Threat evaluation workbench
        </p>
        <p className="mt-2 text-slate-500">
          Select an attacker to inspect threat evaluation details.
        </p>
      </section>
    );
  }

  return (
    <ThreatEvaluationWorkbench
      advisory={advisory}
      stale={stale}
      staleReason={staleReason}
    />
  );
}

export function IntelligenceAdvisoryWorkstationSurfaces({
  snapshots,
  selectedEntityId,
}: {
  snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>;
  selectedEntityId: string | null;
}) {
  const intelligenceAdvisoryTransport = getAdvisoryTransportFromSnapshot(
    snapshots.intelligence_advisory,
  );
  const selectedIntelligenceAdvisory = getSelectedEntityAdvisory(
    intelligenceAdvisoryTransport,
    selectedEntityId,
  );
  const selectedWorkbenchAdvisory = getSelectedEntityAdvisory(
    intelligenceAdvisoryTransport,
    selectedEntityId,
    { includeStale: true },
  );
  if (!intelligenceAdvisoryTransport) return null;

  return (
    <>
      <IntelligenceAdvisoryPanel transport={intelligenceAdvisoryTransport} />
      {selectedIntelligenceAdvisory && (
        <SelectedTargetAdvisoryCard
          transport={intelligenceAdvisoryTransport}
          selectedAttackerId={selectedEntityId}
        />
      )}
      <ThreatEvaluationWorkbenchSurface
        advisory={selectedWorkbenchAdvisory}
        stale={intelligenceAdvisoryTransport.stale}
        staleReason={intelligenceAdvisoryTransport.stale_reason}
      />
    </>
  );
}

export function TrackSensorWorkbenchWorkstationSurface({
  selectedTrackId,
}: {
  selectedTrackId: string | null;
}) {
  return (
    <section data-testid="track-sensor-workstation-surface">
      <SelectedTrackSensorWorkbench selectedTrackId={selectedTrackId} />
    </section>
  );
}

export function TrackTraceabilityWorkstationSurface({
  selectedTrackId,
  entityPoseMirror,
  intelligenceAdvisory,
}: {
  selectedTrackId: string | null;
  entityPoseMirror?: ChannelSnapshot | null;
  intelligenceAdvisory?: ReturnType<typeof getAdvisoryTransportFromSnapshot>;
}) {
  const liveEntityPoseMirror =
    entityPoseMirror?.channel === "entity_pose_mirror"
      ? (entityPoseMirror as unknown as ChannelSnapshot<"entity_pose_mirror">)
      : null;

  return (
    <section data-testid="track-traceability-workstation-surface">
      <LiveTraceabilityWorkbenchSurface
        selectedEntityId={selectedTrackId}
        entityPoseMirror={liveEntityPoseMirror}
        intelligenceAdvisory={intelligenceAdvisory}
        mirrorFreshness={liveEntityPoseMirror ? "fresh" : "unknown"}
      />
    </section>
  );
}

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
    livePreflight = null,
    preflightLoading = false,
    preflightError = null,
    onConnectNewSession,
    onDisconnectSelected,
    onCloseSession,
    onPullHzChange,
    onAutoRefreshChange,
    onRefresh,
    layerVisibility,
    terrainLayers,
    terrainLayersOn,
    terrainProviderMode,
    onTerrainProviderModeChange,
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

  const intelligenceAdvisoryTransport = getAdvisoryTransportFromSnapshot(
    snapshots.intelligence_advisory,
  );
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
  const [planningCameraPresetRequest, setPlanningCameraPresetRequest] =
    useState<{ id: number; preset: CameraPreset } | null>(null);
  const [planningWorldFitCameraRequest, setPlanningWorldFitCameraRequest] =
    useState<{ id: number } | null>(null);
  const [planningLocationRequest, setPlanningLocationRequest] =
    useState<{ id: number; location: CameraLocationTarget } | null>(null);
  const [planningLocationPresetId, setPlanningLocationPresetId] =
    useState<PlanningLocationPresetId>(DEFAULT_PLANNING_LOCATION_PRESET_ID);
  const planningExtent = unifiedPlanningWorld();
  const [planningMeasurements, setPlanningMeasurements] =
    useState<PlanningMeasurementState>(DEFAULT_PLANNING_MEASUREMENT_STATE);
  const [customPlanningLatitude, setCustomPlanningLatitude] = useState(
    String(planningLocationPreset(DEFAULT_PLANNING_LOCATION_PRESET_ID).latitudeDeg),
  );
  const [customPlanningLongitude, setCustomPlanningLongitude] = useState(
    String(planningLocationPreset(DEFAULT_PLANNING_LOCATION_PRESET_ID).longitudeDeg),
  );
  const planningCoverage = useMemo(
    () => estimatePlanningCoverage(planningPolygon, planningRadars),
    [planningPolygon, planningRadars],
  );
  const planningCoverageAnalysis = useMemo(
    () => analyzePlanningCoverage(planningPolygon, planningRadars, undefined, { radarPresets: PLANNING_RADAR_PRESETS }),
    [planningPolygon, planningRadars],
  );
  const [planningMcPackage, setPlanningMcPackage] =
    useState<PlanningMcPackageV1 | null>(null);
  const [planningResultRef, setPlanningResultRef] =
    useState<PlanningMcResultRefV1 | null>(null);
  const [planningResultImportText, setPlanningResultImportText] = useState("");
  const [planningResultImportError, setPlanningResultImportError] = useState<string | null>(
    null,
  );
  const [planningLayoutCompareSlots, setPlanningLayoutCompareSlots] = useState<
    PlanningLayoutCompareSlotV1[]
  >([]);
  const [planningLayoutCompareImportSlotLabel, setPlanningLayoutCompareImportSlotLabel] =
    useState<PlanningLayoutCompareSlotLabel>("A");
  const [planningLayoutCompareImportText, setPlanningLayoutCompareImportText] = useState("");
  const [planningLayoutCompareImportError, setPlanningLayoutCompareImportError] = useState<
    string | null
  >(null);
  const currentPlanningGeometryId = useMemo(
    () => planningGeometryFingerprint(planningPolygon, planningRadars),
    [planningPolygon, planningRadars],
  );
  const planningMcPackageStale =
    planningMcPackage !== null &&
    (planningMcPackage.planning_geometry_id !== currentPlanningGeometryId ||
      planningMcPackage.planning_extent.planning_extent_id !== planningExtent.planning_extent_id);
  const planningResultLink = useMemo(
    () =>
      planningMcPackage
        ? buildPlanningResultLink(
            planningMcPackage,
            planningResultRef,
            currentPlanningGeometryId,
          )
        : null,
    [planningMcPackage, planningResultRef, currentPlanningGeometryId],
  );
  const planningResultLinkPreview = useMemo(
    () => buildPlanningResultLinkPreview(planningResultLink),
    [planningResultLink],
  );
  const planningLayoutCompareAnalytics = useMemo(
    () => buildPlanningLayoutComparePanelDerivation(planningLayoutCompareSlots),
    [planningLayoutCompareSlots],
  );
  const planningLayoutCompareCaptureDisabled =
    planningLayoutCompareSlots.length >= 3;
  const planningModeActive = workspaceModeShowsPlanningPlaceholder(workspaceMode);
  const planningDrawingEnabled = planningToolAllowsDrawing(
    planningModeActive,
    planningTool,
  );
  const planningRadarPlacementEnabled = planningToolAllowsRadarPlacement(
    planningModeActive,
    planningTool,
  );
  const planningMeasurementEnabled = planningModeActive && planningTool === "measure_distance";
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
  const handlePlanningCameraPreset = useCallback((preset: CameraPreset) => {
    setPlanningCameraPresetRequest((current) => ({
      id: (current?.id ?? 0) + 1,
      preset,
    }));
  }, []);

  const handlePlanningWorldCameraFit = useCallback(() => {
    setPlanningWorldFitCameraRequest((current) => ({
      id: (current?.id ?? 0) + 1,
    }));
  }, []);

  const handlePlanningRadiusChange = useCallback((radiusMeters: PlanningRadiusMeters) => {
    setPlanningMeasurements((current) => setPlanningRadiusMeters(current, radiusMeters));
  }, []);

  const handleClearPlanningMeasurements = useCallback(() => {
    setPlanningMeasurements((current) => clearPlanningMeasurements(current));
  }, []);

  const handlePlanningLocationPresetChange = useCallback(
    (presetId: PlanningLocationPresetId) => {
      setPlanningLocationPresetId(presetId);
      if (presetId !== "custom") {
        const preset = planningLocationPreset(presetId);
        setCustomPlanningLatitude(String(preset.latitudeDeg));
        setCustomPlanningLongitude(String(preset.longitudeDeg));
      }
    },
    [],
  );

  const handlePlanningLocationJump = useCallback(() => {
    const preset = planningLocationPreset(planningLocationPresetId);
    const custom = validatedPlanningCoordinates(
      customPlanningLatitude,
      customPlanningLongitude,
    );
    const location =
      planningLocationPresetId === "custom"
        ? custom
        : {
            latitudeDeg: preset.latitudeDeg,
            longitudeDeg: preset.longitudeDeg,
          };
    if (!location) return;
    setPlanningLocationRequest((current) => ({
      id: (current?.id ?? 0) + 1,
      location: { ...location, label: preset.label },
    }));
  }, [planningLocationPresetId, customPlanningLatitude, customPlanningLongitude]);

  const handleGeneratePlanningMcPackage = useCallback(() => {
    const snapshot = buildPlanningMcSnapshot(
      planningPolygon,
      planningRadars,
      planningCoverageAnalysis,
      {
        terrainMode: terrainProviderMode,
        selectedLocationPreset: planningLocationPresetId,
        planningExtent,
      },
    );
    setPlanningMcPackage(buildPlanningMcPackage(snapshot));
    setPlanningResultRef(null);
    setPlanningResultImportText("");
    setPlanningResultImportError(null);
  }, [
    planningPolygon,
    planningRadars,
    planningCoverageAnalysis,
    terrainProviderMode,
    planningLocationPresetId,
    planningExtent,
  ]);

  const handleCopyPlanningMcPackage = useCallback(() => {
    if (planningMcPackage) void copyPlanningMcPackage(planningMcPackage);
  }, [planningMcPackage]);

  const handleDownloadPlanningMcPackage = useCallback(() => {
    if (planningMcPackage) downloadPlanningMcPackage(planningMcPackage);
  }, [planningMcPackage]);

  const handlePlanningResultImportTextChange = useCallback((value: string) => {
    setPlanningResultImportText(value);
    setPlanningResultImportError(null);
  }, []);

  const handleImportPlanningResultMetadata = useCallback(() => {
    if (!planningMcPackage) return;
    const parsed = parsePlanningMcResultRefJson(planningResultImportText);
    if (!parsed.ok) {
      setPlanningResultImportError(parsed.error);
      return;
    }
    setPlanningResultRef(parsed.ref);
    setPlanningResultImportError(null);
  }, [planningMcPackage, planningResultImportText]);

  const handleImportMockPlanningResultRef = useCallback(() => {
    if (!planningMcPackage) return;
    setPlanningResultRef(
      buildMockPlanningMcResultRef(planningMcPackage, {
        importedUtc: "2026-06-04T12:00:00Z",
        summary: {
          success_rate: 0.82,
          miss_distance_p95: 42.5,
          intercept_time_mean: 18.3,
        },
      }),
    );
    setPlanningResultImportError(null);
  }, [planningMcPackage]);

  const handleClearPlanningResultImport = useCallback(() => {
    setPlanningResultRef(null);
    setPlanningResultImportText("");
    setPlanningResultImportError(null);
  }, []);

  const handleCapturePlanningLayoutCompare = useCallback(() => {
    const snapshot = buildPlanningMcSnapshot(
      planningPolygon,
      planningRadars,
      planningCoverageAnalysis,
      {
        terrainMode: terrainProviderMode,
        selectedLocationPreset: planningLocationPresetId,
        planningExtent,
      },
    );
    setPlanningLayoutCompareSlots((current) =>
      capturePlanningLayoutCompareSlot(
        current,
        snapshot,
        new Date().toISOString().replace(/\.\d{3}Z$/, "Z"),
      ).slots,
    );
  }, [
    planningPolygon,
    planningRadars,
    planningCoverageAnalysis,
    terrainProviderMode,
    planningLocationPresetId,
    planningExtent,
  ]);

  const handleRemovePlanningLayoutCompareSlot = useCallback(
    (slotLabel: PlanningLayoutCompareSlotV1["slot_label"]) => {
      setPlanningLayoutCompareSlots((current) =>
        removePlanningLayoutCompareSlot(current, slotLabel),
      );
    },
    [],
  );

  const handleClearPlanningLayoutCompareSlots = useCallback(() => {
    setPlanningLayoutCompareSlots(clearPlanningLayoutCompareSlots());
    setPlanningLayoutCompareImportError(null);
  }, []);

  const handlePlanningLayoutCompareImportTextChange = useCallback((value: string) => {
    setPlanningLayoutCompareImportText(value);
    setPlanningLayoutCompareImportError(null);
  }, []);

  const handlePlanningLayoutCompareImportSlotLabelChange = useCallback(
    (slotLabel: PlanningLayoutCompareSlotLabel) => {
      setPlanningLayoutCompareImportSlotLabel(slotLabel);
      setPlanningLayoutCompareImportError(null);
    },
    [],
  );

  const handleImportPlanningLayoutCompareSnapshot = useCallback(() => {
    const parsed = parsePlanningMcSnapshotJson(planningLayoutCompareImportText);
    if (!parsed.ok) {
      setPlanningLayoutCompareImportError(parsed.error);
      return;
    }
    const captureUtc = new Date().toISOString().replace(/\.\d{3}Z$/, "Z");
    const updatedSlots = setPlanningLayoutCompareSlot(
      planningLayoutCompareSlots,
      planningLayoutCompareImportSlotLabel,
      parsed.snapshot,
      captureUtc,
    );
    setPlanningLayoutCompareSlots(updatedSlots);
    const nextLabel = nextAvailablePlanningLayoutCompareSlotLabel(updatedSlots);
    if (nextLabel) {
      setPlanningLayoutCompareImportSlotLabel(nextLabel);
    }
    setPlanningLayoutCompareImportError(null);
  }, [
    planningLayoutCompareImportSlotLabel,
    planningLayoutCompareImportText,
    planningLayoutCompareSlots,
  ]);

  const handlePlanningMapClick = useCallback(
    (vertex: PlanningVertex) => {
      if (planningDrawingEnabled) {
        setPlanningPolygon((current) => addPlanningVertex(current, vertex));
        return;
      }
      if (planningRadarPlacementEnabled) {
        setPlanningRadars((current) => addPlanningRadarSite(current, vertex));
        return;
      }
      if (planningMeasurementEnabled) {
        setPlanningMeasurements((current) => addPlanningMeasurementPoint(current, vertex));
      }
    },
    [planningDrawingEnabled, planningRadarPlacementEnabled, planningMeasurementEnabled],
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
              requestedRuntimeProfile={activeRequestedRuntimeProfile}
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
          sessionHealth={snapshots.session_health}
          livePreflightOk={livePreflight?.ok ?? null}
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
                coverageAnalysis={planningCoverageAnalysis}
                planningMeasurements={planningMeasurements}
                planningMcPackage={planningMcPackage}
                planningMcPackageStale={planningMcPackageStale}
                planningResultLink={planningResultLink}
                planningResultLinkPreview={planningResultLinkPreview}
                planningResultImportText={planningResultImportText}
                planningResultImportError={planningResultImportError}
                terrainProviderMode={terrainProviderMode}
                locationPresetId={planningLocationPresetId}
                customLatitude={customPlanningLatitude}
                customLongitude={customPlanningLongitude}
                onTerrainProviderModeChange={onTerrainProviderModeChange}
                onLocationPresetChange={handlePlanningLocationPresetChange}
                onCustomLatitudeChange={setCustomPlanningLatitude}
                onCustomLongitudeChange={setCustomPlanningLongitude}
                onApplyLocation={handlePlanningLocationJump}
                onCameraPreset={handlePlanningCameraPreset}
                onPlanningWorldCameraFit={handlePlanningWorldCameraFit}
                onPlanningRadiusChange={handlePlanningRadiusChange}
                onClearPlanningMeasurements={handleClearPlanningMeasurements}
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
                onGeneratePlanningMcPackage={handleGeneratePlanningMcPackage}
                onCopyPlanningMcPackage={handleCopyPlanningMcPackage}
                onDownloadPlanningMcPackage={handleDownloadPlanningMcPackage}
                onPlanningResultImportTextChange={handlePlanningResultImportTextChange}
                onImportPlanningResultMetadata={handleImportPlanningResultMetadata}
                onImportMockPlanningResultRef={handleImportMockPlanningResultRef}
                onClearPlanningResultImport={handleClearPlanningResultImport}
                planningLayoutCompareSlots={planningLayoutCompareSlots}
                planningLayoutCompareAnalytics={planningLayoutCompareAnalytics}
                planningLayoutCompareCaptureDisabled={planningLayoutCompareCaptureDisabled}
                planningLayoutCompareImportSlotLabel={planningLayoutCompareImportSlotLabel}
                planningLayoutCompareImportText={planningLayoutCompareImportText}
                planningLayoutCompareImportError={planningLayoutCompareImportError}
                onCapturePlanningLayoutCompare={handleCapturePlanningLayoutCompare}
                onRemovePlanningLayoutCompareSlot={handleRemovePlanningLayoutCompareSlot}
                onClearPlanningLayoutCompareSlots={handleClearPlanningLayoutCompareSlots}
                onPlanningLayoutCompareImportSlotLabelChange={
                  handlePlanningLayoutCompareImportSlotLabelChange
                }
                onPlanningLayoutCompareImportTextChange={
                  handlePlanningLayoutCompareImportTextChange
                }
                onImportPlanningLayoutCompareSnapshot={handleImportPlanningLayoutCompareSnapshot}
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
            terrainProviderMode={terrainProviderMode}
            onTerrainProviderModeChange={onTerrainProviderModeChange}
            planningCameraPresetRequest={planningCameraPresetRequest}
            planningWorldFitCameraRequest={planningWorldFitCameraRequest}
            planningLocationRequest={planningLocationRequest}
            planningDrawing={{
              enabled: planningCesiumClickEnabled,
              planningExtent,
              measurements: planningMeasurements,
              polygon: planningPolygon,
              radars: planningRadars,
              coverageOptions: planningCoverageOptions,
              onMapClick: handlePlanningMapClick,
            }}
            onLayerVisibilityChange={onLayerVisibilityChange}
            tacticalState={tactical.state}
            tacticalRecommendation={tactical.recommendation}
            intelligenceAdvisory={intelligenceAdvisoryTransport}
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
            <IntelligenceAdvisoryWorkstationSurfaces
              snapshots={snapshots}
              selectedEntityId={selectedEntityId}
            />
            <TrackSensorWorkbenchWorkstationSurface
              selectedTrackId={selectedEntityId}
            />
            <TrackTraceabilityWorkstationSurface
              selectedTrackId={selectedEntityId}
              entityPoseMirror={snapshots.entity_pose_mirror}
              intelligenceAdvisory={getAdvisoryTransportFromSnapshot(
                snapshots.intelligence_advisory,
              )}
            />
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
              entityPoseMirror={snapshots.entity_pose_mirror}
              connected={connected}
              editingEnabled={editingEnabled}
              livePreflightOk={livePreflight?.ok ?? null}
              lastPullUtc={lastPullUtc}
              pullHz={pullHz}
              requestedRuntimeProfile={activeRequestedRuntimeProfile}
              pendingRuntimeProfile={connected ? null : sessionRuntimeProfile}
              livePreflight={livePreflight}
              preflightLoading={preflightLoading}
              preflightError={preflightError}
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
