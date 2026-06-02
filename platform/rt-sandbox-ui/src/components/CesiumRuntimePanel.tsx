import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Globe2 } from "lucide-react";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { PanelShell } from "@/components/GovernanceChrome";
import { CesiumEditingCognitionStrip } from "@/components/CesiumEditingCognitionStrip";
import { FidelityTruthCognitionStrip } from "@/components/FidelityTruthCognitionStrip";
import { CesiumRuntimeView } from "@/cesium/CesiumRuntimeView";
import { isViewerUsable } from "@/cesium/cesiumEditing";
import {
  flyToBounds,
  flyToEntity,
  flyToFitEntities,
  flyToCrestLine,
  flyToRidgeLine,
  flyToSensorContext,
  flyToTerrainOverview,
  flyToTightBounds,
  flyToValleyFloor,
  setFollowEntity,
} from "@/cesium/cameraHelpers";
import type { DefenseZoneRenderOptions } from "@/cesium/defenseZoneConfig";
import type { SensorDomeRenderOptions } from "@/cesium/sensorDomeLayer";
import type { SensorDomeZoneMode } from "@/cesium/terrainLayers";
import { anyTerrainLayerEnabled } from "@/cesium/terrainLayers";
import {
  anyVisibilityOverlayEnabled,
  CANONICAL_VISUAL_LAYER_REGISTRY,
  toggleLayerVisibility,
  toTerrainLayerVisibility,
  type VisualLayerVisibility,
} from "@/cesium/visualLayerRegistry";
import { VisualLayerToggleRail } from "@/components/VisualLayerToggleRail";
import { sessionLayerVisibilityMemoryLine } from "@/workstation/sessionLayerVisibilityStore";
import { SessionComparisonCognitionStrip } from "@/workstation/SessionComparisonCognitionStrip";
import {
  BANNER_FIDELITY_TRUTH,
  BANNER_REALISM_F4,
  BANNER_TACTICAL_COMPARE,
  BANNER_TACTICAL_RANKING_CUES,
  BANNER_TACTICAL_THREAT_CORRIDOR,
  BANNER_TACTICAL_TRAJECTORY,
  BANNER_TERRAIN,
  BANNER_VISIBILITY_V3,
} from "@/governance/banners";
import { TerrainCognitionStrip } from "@/components/TerrainCognitionStrip";
import {
  RadarDomeMapQuickControls,
  type RadarDomePreviewControlHandlers,
  type RadarDomePreviewControlState,
} from "@/components/RadarDomePreviewControls";
import {
  extractFidelityContext,
  isFidelityCouplingOn,
} from "@/fidelity/fidelityCognition";
import {
  shortSessionId,
  sessionAccentColor,
} from "@/workstation/sessionVisualIdentity";
import { cesiumViewSummary } from "@/cesium/cognition";
import {
  deriveTacticalCompareDeltaLabels,
  formatTacticalCompareSummary,
} from "@/cesium/tacticalCompareDelta";
import { hasTacticalCompareGeometry } from "@/cesium/tacticalCompareOverlay";
import {
  formatTacticalRankingSummary,
  resolveTacticalRankingCues,
} from "@/cesium/tacticalRankingCueLayer";
import { feedbackEntityRows } from "@/sync/cognition";
import type { MirrorEntity } from "@/cesium/entityMarkers";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import type { Viewer } from "cesium";
import type {
  TacticalRecommendationPayload,
  TacticalStatePayload,
} from "@/bridge/tacticalCommands";
import { useTacticalCompareBaseline } from "@/hooks/useTacticalCompareBaseline";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import type {
  PlanningCoverageLayerOptions,
  PlanningPolygonState,
  PlanningRadarState,
  PlanningVertex,
} from "@/cesium/planningDrawing";
import { resolveTacticalCompareContext } from "@/workstation/tacticalCompareContext";

const CESIUM_TOOL_BTN =
  "rounded border border-cyan-700/60 bg-cyan-950/40 px-3 py-1.5 text-xs font-medium text-cyan-100 transition-colors duration-150 hover:bg-cyan-900/50 disabled:opacity-40";
const CESIUM_MENU_BTN =
  "cursor-pointer rounded border border-slate-700 bg-slate-900 px-3 py-1.5 text-slate-200 transition-colors duration-150 hover:bg-slate-800";

export function CesiumRuntimePanel({
  sessionId,
  orderedSessionIds,
  connectedCount = 1,
  editingSessionId,
  entities,
  selectedEntityId,
  selectedType,
  worldSummary,
  mirrorSnapshot,
  pendingReconcile,
  editingEnabled,
  lastCommand,
  onSelectEntity,
  onSpawn,
  onMove,
  onDelete,
  layerVisibility,
  sensorDomeOptions,
  defenseZoneOptions,
  sensorDomeZoneMode,
  radarPreviewControls = null,
  planningDrawing,
  onLayerVisibilityChange,
  tacticalState = null,
  tacticalRecommendation = null,
  slotList = [],
}: {
  sessionId: string | null;
  orderedSessionIds: readonly string[];
  connectedCount?: number;
  editingSessionId?: string | null;
  entities: MirrorEntity[];
  selectedEntityId: string | null;
  selectedType: EntityType;
  worldSummary: Record<string, unknown> | undefined;
  mirrorSnapshot: ChannelSnapshot | undefined;
  pendingReconcile: boolean;
  editingEnabled: boolean;
  lastCommand?: {
    type: import("@/editing/editHistory").EditCommandType;
    ok: boolean;
    errorCode?: string;
    message?: string;
  };
  onSelectEntity: (id: string | null) => void;
  onSpawn: (pose: Pose) => void;
  onMove: (entityId: string, pose: Pose) => void;
  onDelete: (entityId: string) => void;
  layerVisibility: VisualLayerVisibility;
  sensorDomeOptions?: SensorDomeRenderOptions;
  defenseZoneOptions?: DefenseZoneRenderOptions;
  sensorDomeZoneMode?: SensorDomeZoneMode;
  radarPreviewControls?: {
    state: RadarDomePreviewControlState;
    handlers: RadarDomePreviewControlHandlers;
  } | null;
  planningDrawing?: {
    enabled: boolean;
    polygon: PlanningPolygonState;
    radars: PlanningRadarState;
    coverageOptions: PlanningCoverageLayerOptions;
    onMapClick: (vertex: PlanningVertex) => void;
  };
  onLayerVisibilityChange: (layers: VisualLayerVisibility) => void;
  tacticalState?: TacticalStatePayload | null;
  tacticalRecommendation?: TacticalRecommendationPayload | null;
  slotList?: readonly SessionSlot[];
}) {
  const compareBaseline = useTacticalCompareBaseline(sessionId, tacticalState);
  const tacticalCompare = useMemo(
    () =>
      resolveTacticalCompareContext({
        currentState: tacticalState,
        previousState: compareBaseline,
        activeSessionId: sessionId,
        orderedSessionIds,
        slots: slotList,
        activeEntities: entities,
      }),
    [
      tacticalState,
      compareBaseline,
      sessionId,
      orderedSessionIds,
      slotList,
      entities,
    ],
  );
  const terrainLayers = toTerrainLayerVisibility(layerVisibility);
  const [followSelected, setFollowSelected] = useState(false);
  const [viewer, setViewer] = useState<Viewer | null>(null);
  const viewerRef = useRef<Viewer | null>(null);

  const terrainLayersOn = anyTerrainLayerEnabled(terrainLayers);
  const tacticalThreatCorridorOn = layerVisibility.showTacticalThreatCorridor;
  const tacticalRankingCuesOn = layerVisibility.showTacticalRankingCues;
  const tacticalCompareOn = layerVisibility.showTacticalCompareOverlay;
  const tacticalTrajectoryOn =
    layerVisibility.showTacticalPredictedPath ||
    layerVisibility.showTacticalInterceptPoint ||
    layerVisibility.showTacticalTimingLabels ||
    layerVisibility.showTacticalSelectionEmphasis;
  const tacticalRankingSummary = useMemo(() => {
    if (!tacticalRankingCuesOn) return null;
    return formatTacticalRankingSummary(
      resolveTacticalRankingCues(tacticalState, tacticalRecommendation),
    );
  }, [tacticalRankingCuesOn, tacticalState, tacticalRecommendation]);
  const tacticalCompareSummary = useMemo(() => {
    if (!tacticalCompareOn || !tacticalCompare) return null;
    return formatTacticalCompareSummary({
      source: tacticalCompare.source,
      compareSessionId: tacticalCompare.compareSessionId,
      deltas: deriveTacticalCompareDeltaLabels(
        tacticalState,
        tacticalCompare.compareState,
      ),
      hasCompareGeometry: hasTacticalCompareGeometry(
        tacticalCompare.compareState,
        tacticalCompare.compareEntities,
      ),
    });
  }, [tacticalCompareOn, tacticalCompare, tacticalState]);
  const fidelityContext = extractFidelityContext(worldSummary);
  const fidelityOn = isFidelityCouplingOn(fidelityContext);
  const selectedEntity =
    entities.find((e) => e.entity_id === selectedEntityId) ?? null;
  const layerMemoryLine = sessionLayerVisibilityMemoryLine(sessionId, true);

  const summary = cesiumViewSummary({
    pendingReconcile,
    entityCount: entities.length,
    editingEnabled,
  });

  const syncHealth =
    typeof worldSummary?.sync_health === "string"
      ? worldSummary.sync_health
      : undefined;
  const telemetryHealth =
    typeof mirrorSnapshot?.payload?.telemetry_health === "string"
      ? mirrorSnapshot.payload.telemetry_health
      : undefined;

  const driftRows = feedbackEntityRows(worldSummary);
  const perEntityDriftM = Object.fromEntries(
    driftRows
      .filter((r) => r.driftM !== null)
      .map((r) => [r.entityId, r.driftM as number]),
  );

  let commandGhost: {
    entityId: string;
    pose: { x: number; y: number; z: number };
  } | null = null;
  if (selectedEntityId && worldSummary?.feedback_entities) {
    const row = (worldSummary.feedback_entities as unknown[]).find(
      (item) =>
        item &&
        typeof item === "object" &&
        (item as Record<string, unknown>).entity_id === selectedEntityId,
    ) as Record<string, unknown> | undefined;
    const drift = row?.drift_m;
    const cmd = row?.command_pose as Record<string, unknown> | undefined;
    if (typeof drift === "number" && drift > 0.05 && cmd) {
      commandGhost = {
        entityId: selectedEntityId,
        pose: {
          x: Number(cmd.x ?? 0),
          y: Number(cmd.y ?? 0),
          z: Number(cmd.z ?? 0),
        },
      };
    }
  }

  const sessionAccent =
    sessionId && orderedSessionIds.length > 0
      ? sessionAccentColor(sessionId, orderedSessionIds)
      : undefined;
  const isEditingSession =
    sessionId != null && editingSessionId != null && sessionId === editingSessionId;

  const handleResetCamera = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToBounds(currentViewer);
  }, []);

  const handleFocusSelected = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer) || !selectedEntityId) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToEntity(currentViewer, selectedEntityId);
  }, [selectedEntityId]);

  const handleFitEntities = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToFitEntities(currentViewer, entities);
  }, [entities]);

  const handleTightBounds = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToTightBounds(currentViewer);
  }, []);

  const handleTerrainOverview = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToTerrainOverview(currentViewer);
  }, []);

  const handleRidgeLine = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToRidgeLine(currentViewer);
  }, []);

  const handleCrestLine = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToCrestLine(currentViewer);
  }, []);

  const handleValleyFloor = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToValleyFloor(currentViewer);
  }, []);

  const handleSensorContext = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer) || !selectedEntityId) return;
    setFollowSelected(false);
    setFollowEntity(currentViewer, null);
    flyToSensorContext(
      currentViewer,
      selectedEntityId,
      entities,
      terrainLayers.showTerrainMesh,
    );
  }, [selectedEntityId, entities, terrainLayers.showTerrainMesh]);

  const handleFollowToggle = useCallback(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    const next = !followSelected;
    setFollowSelected(next);
    if (next && selectedEntityId) {
      setFollowEntity(currentViewer, selectedEntityId);
    } else {
      setFollowEntity(currentViewer, null);
    }
  }, [followSelected, selectedEntityId]);

  const handleViewerReady = useCallback((nextViewer: Viewer | null) => {
    const usableViewer = isViewerUsable(nextViewer) ? nextViewer : null;
    viewerRef.current = usableViewer;
    setViewer(usableViewer);
  }, []);

  useEffect(() => {
    const currentViewer = viewerRef.current;
    if (!isViewerUsable(currentViewer)) return;
    if (followSelected && selectedEntityId) {
      setFollowEntity(currentViewer, selectedEntityId);
    } else if (!followSelected) {
      setFollowEntity(currentViewer, null);
    }
  }, [viewer, followSelected, selectedEntityId]);

  useEffect(() => {
    if (!selectedEntityId) {
      setFollowSelected(false);
      setFollowEntity(viewerRef.current, null);
    }
  }, [selectedEntityId, viewer]);

  const mapStatus = pendingReconcile
    ? "reconcile pending"
    : !editingEnabled
      ? "editing blocked"
      : null;

  return (
    <PanelShell title="Cesium runtime view" icon={Globe2} variant="primary">
      <div className="mb-4 flex flex-wrap items-center justify-between gap-3">
        <div className="flex flex-wrap items-center gap-2 text-xs">
          <span className="rounded-full border border-cyan-500/40 bg-cyan-950/40 px-2.5 py-1 font-medium text-cyan-100">
            {summary.entityCountLabel}
          </span>
          {mapStatus && (
            <span
              className={`rounded-full border px-2.5 py-1 ${
                pendingReconcile
                  ? "border-amber-600/50 bg-amber-950/40 text-amber-200"
                  : "border-slate-700 bg-slate-950/60 text-slate-400"
              }`}
            >
              {mapStatus}
            </span>
          )}
          {isEditingSession && orderedSessionIds.length > 1 && (
            <span className="rounded-full border border-amber-700/50 bg-amber-950/40 px-2.5 py-1 text-amber-200">
              editing lock
            </span>
          )}
        </div>
        <details className="relative text-xs">
          <summary className="cursor-pointer rounded border border-slate-700 bg-slate-950/60 px-2.5 py-1 text-slate-300 hover:bg-slate-900">
            Context
          </summary>
          <div className="absolute right-0 z-10 mt-2 w-80 rounded border border-slate-800 bg-slate-950 p-3 shadow-xl shadow-black/30">
            <div className="space-y-2 text-slate-400">
              {sessionId && (
                <p className="font-mono text-slate-300" title={sessionId}>
                  session {shortSessionId(sessionId)}
                </p>
              )}
              <p className="italic text-amber-200/90">{summary.caveat}</p>
              <p>{summary.dualSurfaceNote}</p>
              <p>{summary.mirrorNote}</p>
              {summary.reconcileNote && <p className="text-amber-300">{summary.reconcileNote}</p>}
              {orderedSessionIds.length > 1 && (
                <p className="text-slate-500">active globe — comparison surfaces are explanatory only</p>
              )}
              {terrainLayersOn && <p className="text-[10px] text-amber-100/80">{BANNER_TERRAIN}</p>}
              {(terrainLayers.showContourOverlays || terrainLayers.showVegetationMarkers) && (
                <p className="text-[10px] text-amber-100/80">{BANNER_REALISM_F4}</p>
              )}
              {fidelityOn && <p className="text-[10px] text-amber-100/80">{BANNER_FIDELITY_TRUTH}</p>}
              {anyVisibilityOverlayEnabled(layerVisibility) && (
                <p className="text-[10px] text-amber-100/80">{BANNER_VISIBILITY_V3}</p>
              )}
              {tacticalTrajectoryOn && (
                <p className="text-[10px] text-amber-100/80">{BANNER_TACTICAL_TRAJECTORY}</p>
              )}
              {tacticalThreatCorridorOn && (
                <p className="text-[10px] text-amber-100/80">{BANNER_TACTICAL_THREAT_CORRIDOR}</p>
              )}
              {tacticalRankingCuesOn && (
                <p className="text-[10px] text-amber-100/80">{BANNER_TACTICAL_RANKING_CUES}</p>
              )}
              {tacticalCompareOn && (
                <p className="text-[10px] text-amber-100/80">{BANNER_TACTICAL_COMPARE}</p>
              )}
            </div>
          </div>
        </details>
      </div>

      {orderedSessionIds.length > 1 && (
        <details className="mb-3 rounded border border-slate-800 bg-slate-950/40 text-xs">
          <summary className="cursor-pointer px-3 py-2 font-semibold text-slate-300">
            Session comparison context
          </summary>
          <div className="border-t border-slate-800 p-3">
            <SessionComparisonCognitionStrip
              activeSessionId={sessionId}
              orderedSessionIds={orderedSessionIds}
              comparisonGhostsEnabled={layerVisibility.showComparisonGhosts}
              sessionContrastEnabled={layerVisibility.showSessionContrast}
              compareEmphasisEnabled={layerVisibility.showCompareEmphasisV4}
            />
            {tacticalCompareSummary && (
              <p
                className="mt-2 rounded border border-slate-700/80 bg-slate-900/50 px-2 py-1.5 text-[10px] text-slate-400"
                data-testid="tactical-compare-cognition"
              >
                {tacticalCompareSummary}
              </p>
            )}
          </div>
        </details>
      )}

      <div className="mb-3 flex flex-wrap items-start gap-2">
        <button
          type="button"
          className={CESIUM_TOOL_BTN}
          disabled={!viewer || !selectedEntityId}
          onClick={handleFocusSelected}
        >
          Focus
        </button>
        <button
          type="button"
          className={CESIUM_TOOL_BTN}
          disabled={!viewer || entities.length === 0}
          onClick={handleFitEntities}
        >
          Fit
        </button>
        <button
          type="button"
          className={CESIUM_TOOL_BTN}
          disabled={!viewer}
          onClick={handleTerrainOverview}
        >
          Terrain
        </button>
        <details className="relative text-xs">
          <summary className={CESIUM_MENU_BTN}>
            More
          </summary>
          <div className="absolute right-0 z-10 mt-2 grid w-44 gap-1 rounded border border-slate-800 bg-slate-950 p-2 shadow-xl shadow-black/30">
            <button type="button" className="rounded px-2 py-1 text-left text-slate-300 hover:bg-slate-900 disabled:opacity-40" disabled={!viewer} onClick={handleResetCamera}>Reset</button>
            <button type="button" className="rounded px-2 py-1 text-left text-slate-300 hover:bg-slate-900 disabled:opacity-40" disabled={!viewer} onClick={handleTightBounds}>Tight bounds</button>
            <button type="button" className="rounded px-2 py-1 text-left text-slate-300 hover:bg-slate-900 disabled:opacity-40" disabled={!viewer} onClick={handleRidgeLine}>Ridge</button>
            <button type="button" className="rounded px-2 py-1 text-left text-slate-300 hover:bg-slate-900 disabled:opacity-40" disabled={!viewer} onClick={handleCrestLine}>Crest</button>
            <button type="button" className="rounded px-2 py-1 text-left text-slate-300 hover:bg-slate-900 disabled:opacity-40" disabled={!viewer} onClick={handleValleyFloor}>Valley</button>
            <button type="button" className="rounded px-2 py-1 text-left text-slate-300 hover:bg-slate-900 disabled:opacity-40" disabled={!viewer || !selectedEntityId || selectedEntity?.entity_type !== "radar"} onClick={handleSensorContext}>Sensor</button>
            <button type="button" className={`rounded px-2 py-1 text-left ${followSelected ? "bg-cyan-950/60 text-cyan-100" : "text-slate-300 hover:bg-slate-900"} disabled:opacity-40`} disabled={!viewer || !selectedEntityId} onClick={handleFollowToggle}>Follow {followSelected ? "on" : "off"}</button>
            <button type="button" className="rounded px-2 py-1 text-left text-red-200 hover:bg-red-950/50 disabled:opacity-40" disabled={!editingEnabled || !selectedEntityId} onClick={() => selectedEntityId && onDelete(selectedEntityId)}>Delete selected</button>
          </div>
        </details>
        <details className="relative text-xs">
          <summary className={CESIUM_TOOL_BTN}>Layers</summary>
          <div className="absolute right-0 z-10 mt-1 w-52 rounded border border-slate-800 bg-slate-950 p-2 shadow-xl shadow-black/30">
            <VisualLayerToggleRail
              visibility={layerVisibility}
              memoryLine={layerMemoryLine}
              showAdvisoryFooter={
                layerVisibility.showLayerBudgetSummary || layerVisibility.showDensityWarnings
              }
              onToggle={(layerId) =>
                onLayerVisibilityChange(
                  toggleLayerVisibility(
                    layerVisibility,
                    layerId,
                    CANONICAL_VISUAL_LAYER_REGISTRY,
                  ),
                )
              }
            />
          </div>
        </details>
      </div>

      {tacticalRankingSummary && (
        <p
          className="mb-2 rounded border border-indigo-800/50 bg-indigo-950/35 px-2.5 py-1.5 text-[10px] text-indigo-100/90"
          data-testid="tactical-ranking-summary"
        >
          {tacticalRankingSummary}
        </p>
      )}
      {tacticalCompareSummary && (
        <p
          className="mb-2 rounded border border-slate-700/80 bg-slate-900/50 px-2.5 py-1.5 text-[10px] text-slate-400"
          data-testid="tactical-compare-summary"
        >
          {tacticalCompareSummary}
        </p>
      )}

      <div className="relative">
        <CesiumRuntimeView
          sessionId={sessionId}
          sessionAccentCss={sessionAccent}
          markerEmphasis={connectedCount >= 2 ? "muted" : "full"}
          entities={entities}
          selectedEntityId={selectedEntityId}
          layerVisibility={layerVisibility}
          terrainLayers={terrainLayers}
          syncHealth={syncHealth}
          telemetryHealth={telemetryHealth}
          perEntityDriftM={perEntityDriftM}
          commandGhost={commandGhost}
          editingEnabled={editingEnabled}
          selectedType={selectedType}
          worldSummary={worldSummary}
          sensorDomeOptions={sensorDomeOptions}
          defenseZoneOptions={defenseZoneOptions}
          sensorDomeZoneMode={sensorDomeZoneMode}
          tacticalState={tacticalState}
          tacticalRecommendation={tacticalRecommendation}
          tacticalCompare={tacticalCompare}
          mirrorSnapshot={mirrorSnapshot}
          planningDrawing={planningDrawing}
          onViewerReady={handleViewerReady}
          onSelectEntity={onSelectEntity}
          onSpawn={onSpawn}
          onMove={onMove}
        />
        {radarPreviewControls && selectedEntity?.entity_type === "radar" && (
          <div className="pointer-events-none absolute left-3 top-3 z-10 max-w-[calc(100%-1.5rem)]">
            <RadarDomeMapQuickControls
              state={radarPreviewControls.state}
              handlers={radarPreviewControls.handlers}
            />
          </div>
        )}
      </div>

      <details className="mt-3 rounded border border-slate-800 bg-slate-950/35 text-xs">
        <summary className="cursor-pointer px-3 py-2 font-semibold text-slate-300">
          Map cognition
        </summary>
        <div className="space-y-3 border-t border-slate-800 p-3">
          <TerrainCognitionStrip
            selectedEntity={selectedEntity}
            entities={entities}
            layersEnabled={terrainLayersOn}
            fidelityContext={fidelityContext}
          />
          <FidelityTruthCognitionStrip
            fidelityContext={fidelityContext}
            worldSummary={worldSummary}
            selectedEntity={selectedEntity}
            entities={entities}
            showLosDivergence={terrainLayersOn}
          />
          <CesiumEditingCognitionStrip
            lastCommand={lastCommand}
            pendingReconcile={pendingReconcile}
            mirrorSnapshot={mirrorSnapshot}
            worldSummary={worldSummary}
            editingEnabled={editingEnabled}
          />
        </div>
      </details>
    </PanelShell>
  );
}
