import { useCallback, useEffect, useRef, useState } from "react";
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
  BANNER_TERRAIN,
  BANNER_VISIBILITY_V3,
} from "@/governance/banners";
import { TerrainCognitionStrip } from "@/components/TerrainCognitionStrip";
import {
  extractFidelityContext,
  isFidelityCouplingOn,
} from "@/fidelity/fidelityCognition";
import {
  shortSessionId,
  sessionAccentBgClass,
  sessionAccentColor,
} from "@/workstation/sessionVisualIdentity";
import { cesiumViewSummary } from "@/cesium/cognition";
import { feedbackEntityRows } from "@/sync/cognition";
import type { MirrorEntity } from "@/cesium/entityMarkers";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import type { Viewer } from "cesium";

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
  onLayerVisibilityChange,
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
  onLayerVisibilityChange: (layers: VisualLayerVisibility) => void;
}) {
  const terrainLayers = toTerrainLayerVisibility(layerVisibility);
  const [followSelected, setFollowSelected] = useState(false);
  const [viewer, setViewer] = useState<Viewer | null>(null);
  const viewerRef = useRef<Viewer | null>(null);

  const terrainLayersOn = anyTerrainLayerEnabled(terrainLayers);
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

  return (
    <PanelShell title="Cesium runtime view">
      <p className="mb-2 text-xs italic text-amber-200/90">{summary.caveat}</p>
      <p className="mb-2 text-xs text-slate-400">{summary.dualSurfaceNote}</p>
      <p className="mb-2 text-xs text-slate-400">{summary.mirrorNote}</p>
      {summary.reconcileNote && (
        <p className="mb-2 text-xs text-amber-300">{summary.reconcileNote}</p>
      )}
      <p className="mb-3 text-xs text-slate-500">{summary.entityCountLabel}</p>
      {terrainLayersOn && (
        <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_TERRAIN}</p>
      )}
      {(terrainLayers.showContourOverlays || terrainLayers.showVegetationMarkers) && (
        <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_REALISM_F4}</p>
      )}
      {fidelityOn && (
        <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_FIDELITY_TRUTH}</p>
      )}
      {anyVisibilityOverlayEnabled(layerVisibility) && (
        <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_VISIBILITY_V3}</p>
      )}

      {sessionId && (
        <div
          data-testid="cesium-session-chrome"
          className="mb-3 flex flex-wrap items-center gap-2 rounded border border-l-4 bg-slate-950/60 px-3 py-2 text-xs"
          style={
            sessionAccent
              ? { borderLeftColor: sessionAccent, borderColor: "rgb(51 65 85)" }
              : { borderColor: "rgb(51 65 85)" }
          }
          title={sessionId}
        >
          <span
            className={`inline-block h-3 w-3 rounded-sm ${sessionAccentBgClass(sessionId, orderedSessionIds)}`}
            aria-hidden
          />
          <span className="text-slate-300">
            Viewing session <span className="font-mono text-amber-200/90">{shortSessionId(sessionId)}</span>
          </span>
          {isEditingSession && (
            <span className="rounded border border-amber-700/60 bg-amber-950/50 px-1.5 py-0.5 text-amber-200">
              editing lock
            </span>
          )}
          {orderedSessionIds.length > 1 && (
            <span className="text-slate-500">active globe — comparison surfaces are explanatory only</span>
          )}
        </div>
      )}

      {orderedSessionIds.length > 1 && (
        <div className="mb-3">
          <SessionComparisonCognitionStrip
            activeSessionId={sessionId}
            orderedSessionIds={orderedSessionIds}
            comparisonGhostsEnabled={layerVisibility.showComparisonGhosts}
            sessionContrastEnabled={layerVisibility.showSessionContrast}
            compareEmphasisEnabled={layerVisibility.showCompareEmphasisV4}
          />
        </div>
      )}

      <div className="mb-3 flex flex-wrap gap-2">
        <VisualLayerToggleRail
          visibility={layerVisibility}
          memoryLine={layerMemoryLine}
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
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer}
          onClick={handleResetCamera}
        >
          Reset bounds
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer}
          onClick={handleTightBounds}
        >
          Tight bounds
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer || entities.length === 0}
          onClick={handleFitEntities}
        >
          Fit entities
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer || !selectedEntityId}
          onClick={handleFocusSelected}
        >
          Focus selected
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer}
          onClick={handleTerrainOverview}
        >
          Terrain overview
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer}
          onClick={handleRidgeLine}
        >
          Ridge line
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer}
          onClick={handleCrestLine}
        >
          Crest line
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer}
          onClick={handleValleyFloor}
        >
          Valley floor
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40"
          disabled={!viewer || !selectedEntityId || selectedEntity?.entity_type !== "radar"}
          onClick={handleSensorContext}
        >
          Sensor context
        </button>
        <button
          type="button"
          className={`rounded border px-2 py-1 text-xs ${
            followSelected
              ? "border-sky-500 bg-sky-900/50 text-sky-100"
              : "border-slate-600 bg-slate-800 text-slate-200 hover:bg-slate-700"
          } disabled:opacity-40`}
          disabled={!viewer || !selectedEntityId}
          onClick={handleFollowToggle}
        >
          Follow selected: {followSelected ? "on" : "off"}
        </button>
        <button
          type="button"
          className="rounded border border-red-800 bg-red-950/50 px-2 py-1 text-xs text-red-200 hover:bg-red-900/50 disabled:opacity-40"
          disabled={!editingEnabled || !selectedEntityId}
          onClick={() => selectedEntityId && onDelete(selectedEntityId)}
        >
          Delete selected
        </button>
      </div>

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
        onViewerReady={handleViewerReady}
        onSelectEntity={onSelectEntity}
        onSpawn={onSpawn}
        onMove={onMove}
      />

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
    </PanelShell>
  );
}
