import { useEffect, useMemo, useRef, useState } from "react";
import {
  ImageryLayer,
  UrlTemplateImageryProvider,
  Viewer,
} from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import { syncBoundsLayer } from "./boundsLayer";
import { attachCesiumEditingHandlers, isViewerUsable } from "./cesiumEditing";
import { setInitialGroundedCamera } from "./cameraHelpers";
import {
  clearEntityMarkers,
  syncEntityMarkers,
  type MarkerEmphasis,
  type MirrorEntity,
} from "./entityMarkers";
import { clearHorizonHintLayer, syncHorizonHintLayer } from "./horizonHintLayer";
import { clearLosSegmentLayer, syncLosSegmentLayer } from "./losSegmentLayer";
import {
  shouldUseLegacyLosPath,
  syncStackedLosPresentation,
} from "./stackedLosPresentation";
import {
  clearAllTerrainLayers,
  shouldApplyTerrainGrounding,
  syncTerrainLayers,
  type SensorDomeZoneMode,
  type TerrainLayerVisibility,
} from "./terrainLayers";
import type { DefenseZoneRenderOptions } from "./defenseZoneConfig";
import type { SensorDomeRenderOptions } from "./sensorDomeLayer";
import {
  clearTacticalCompareOverlay,
  syncTacticalCompareOverlay,
} from "./tacticalCompareOverlay";
import {
  clearTacticalCorridorLayer,
  syncTacticalCorridorLayer,
} from "./tacticalCorridorLayer";
import {
  clearTacticalRankingCueLayer,
  syncTacticalRankingCueLayer,
} from "./tacticalRankingCueLayer";
import {
  clearTacticalTrajectoryLayer,
  syncTacticalTrajectoryLayer,
} from "./tacticalTrajectoryLayer";
import {
  clearTacticalSelectionEmphasisLayer,
  syncTacticalSelectionEmphasisLayer,
} from "./tacticalSelectionEmphasisLayer";
import type { TacticalCompareContext } from "@/workstation/tacticalCompareContext";
import type { VisualLayerVisibility } from "./visualLayerRegistry";
import { clearVisibilityOverlayV4, syncVisibilityOverlayV4 } from "./visibilityOverlayV4";
import type {
  TacticalRecommendationPayload,
  TacticalStatePayload,
} from "@/bridge/tacticalCommands";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { entitiesFromSnapshot } from "@/telemetry/channelIndex";
import { parseEntityRuntimeTelemetryMap } from "@/telemetry/entityMirrorFields";
import {
  attachPlanningDrawingHandlers,
  syncPlanningDefenseAreaLayer,
  type PlanningCoverageLayerOptions,
  type PlanningPolygonState,
  type PlanningRadarState,
  type PlanningVertex,
} from "./planningDrawing";
import {
  createCesiumTerrainProvider,
  DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE,
  type CesiumTerrainProviderMode,
} from "./terrainProviderConfig";

export interface CesiumRuntimeViewProps {
  sessionId: string | null;
  sessionAccentCss?: string;
  markerEmphasis?: MarkerEmphasis;
  entities: MirrorEntity[];
  selectedEntityId: string | null;
  layerVisibility: VisualLayerVisibility;
  terrainLayers: TerrainLayerVisibility;
  syncHealth?: string;
  telemetryHealth?: string;
  perEntityDriftM?: Record<string, number>;
  commandGhost?: { entityId: string; pose: { x: number; y: number; z: number } } | null;
  editingEnabled: boolean;
  selectedType: EntityType;
  worldSummary: Record<string, unknown> | undefined;
  sensorDomeOptions?: SensorDomeRenderOptions;
  defenseZoneOptions?: DefenseZoneRenderOptions;
  sensorDomeZoneMode?: SensorDomeZoneMode;
  tacticalState?: TacticalStatePayload | null;
  tacticalRecommendation?: TacticalRecommendationPayload | null;
  tacticalCompare?: TacticalCompareContext | null;
  mirrorSnapshot?: ChannelSnapshot;
  planningDrawing?: {
    enabled: boolean;
    polygon: PlanningPolygonState;
    radars: PlanningRadarState;
    coverageOptions: PlanningCoverageLayerOptions;
    onMapClick: (vertex: PlanningVertex) => void;
  };
  terrainProviderMode?: CesiumTerrainProviderMode;
  onViewerReady?: (viewer: Viewer | null) => void;
  onSelectEntity: (id: string | null) => void;
  onSpawn: (pose: Pose) => void;
  onMove: (entityId: string, pose: Pose) => void;
}

function createTopoBaseLayer(): ImageryLayer {
  const layer = new ImageryLayer(
    new UrlTemplateImageryProvider({
      url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
      subdomains: ["a", "b", "c"],
      maximumLevel: 17,
      credit: "OpenTopoMap, SRTM, OpenStreetMap contributors",
    }),
  );
  layer.alpha = 0.92;
  layer.brightness = 0.78;
  layer.contrast = 1.08;
  layer.saturation = 0.72;
  return layer;
}

function addHillshadeOverlay(viewer: Viewer): void {
  const hillshade = new ImageryLayer(
    new UrlTemplateImageryProvider({
      url: "https://services.arcgisonline.com/ArcGIS/rest/services/Elevation/World_Hillshade/MapServer/tile/{z}/{y}/{x}",
      maximumLevel: 16,
      credit: "Esri World Hillshade",
    }),
  );
  hillshade.alpha = 0.22;
  hillshade.brightness = 0.9;
  hillshade.contrast = 1.18;
  hillshade.saturation = 0.15;
  viewer.imageryLayers.add(hillshade, 1);
}

function createViewer(
  container: HTMLDivElement,
  terrainProviderMode: CesiumTerrainProviderMode,
): Viewer {
  const viewer = new Viewer(container, {
    animation: false,
    timeline: false,
    geocoder: false,
    homeButton: false,
    sceneModePicker: false,
    baseLayerPicker: false,
    navigationHelpButton: false,
    fullscreenButton: false,
    infoBox: false,
    selectionIndicator: false,
    baseLayer: createTopoBaseLayer(),
    terrain: createCesiumTerrainProvider(terrainProviderMode),
  });
  addHillshadeOverlay(viewer);
  return viewer;
}

function tuneZoomInteraction(viewer: Viewer): void {
  const controller = viewer.scene.screenSpaceCameraController;
  controller.inertiaZoom = 0.28;
  controller.zoomFactor = 1.35;
  controller.minimumZoomDistance = 120;
  controller.maximumZoomDistance = 28000;
}

export function CesiumRuntimeView({
  sessionId,
  sessionAccentCss,
  markerEmphasis = "full",
  entities,
  selectedEntityId,
  layerVisibility,
  terrainLayers,
  syncHealth,
  telemetryHealth,
  perEntityDriftM,
  commandGhost,
  editingEnabled,
  selectedType,
  worldSummary,
  sensorDomeOptions,
  defenseZoneOptions,
  sensorDomeZoneMode = "both",
  tacticalState = null,
  tacticalRecommendation = null,
  tacticalCompare = null,
  mirrorSnapshot,
  planningDrawing,
  terrainProviderMode = DEFAULT_CESIUM_TERRAIN_PROVIDER_MODE,
  onViewerReady,
  onSelectEntity,
  onSpawn,
  onMove,
}: CesiumRuntimeViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<Viewer | null>(null);
  const [dragOverride, setDragOverride] = useState<{
    entityId: string;
    pose: { x: number; y: number; z: number };
  } | null>(null);
  const [hoveredEntityId, setHoveredEntityId] = useState<string | null>(null);
  const runtimeTelemetryByEntityId = useMemo(
    () => parseEntityRuntimeTelemetryMap(entitiesFromSnapshot(mirrorSnapshot)),
    [mirrorSnapshot],
  );

  const callbacksRef = useRef({
    onSelectEntity,
    onSpawn,
    onMove,
    worldSummary,
    selectedType,
    editingEnabled,
    entities,
  });
  callbacksRef.current = {
    onSelectEntity,
    onSpawn,
    onMove,
    worldSummary,
    selectedType,
    editingEnabled,
    entities,
  };

  useEffect(() => {
    if (!containerRef.current || !sessionId) return;

    const viewer = createViewer(containerRef.current, terrainProviderMode);
    tuneZoomInteraction(viewer);
    viewerRef.current = viewer;
    onViewerReady?.(viewer);

    if (isViewerUsable(viewer)) {
      setInitialGroundedCamera(viewer);
    }

    return () => {
      if (viewerRef.current === viewer) {
        viewerRef.current = null;
      }
      onViewerReady?.(null);
      setDragOverride(null);
      setHoveredEntityId(null);
      if (isViewerUsable(viewer)) {
        clearEntityMarkers(viewer);
        clearAllTerrainLayers(viewer);
        clearHorizonHintLayer(viewer);
        clearVisibilityOverlayV4(viewer);
        clearTacticalTrajectoryLayer(viewer);
        clearTacticalCorridorLayer(viewer);
        clearTacticalRankingCueLayer(viewer);
        clearTacticalCompareOverlay(viewer);
        clearTacticalSelectionEmphasisLayer(viewer);
        syncPlanningDefenseAreaLayer(viewer, null);
        viewer.trackedEntity = undefined;
      }
      if (!viewer.isDestroyed()) {
        viewer.destroy();
      }
    };
  }, [sessionId, terrainProviderMode, onViewerReady]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !sessionId) return;

    const detach = attachCesiumEditingHandlers(viewer, {
      get editingEnabled() {
        return callbacksRef.current.editingEnabled;
      },
      get selectedType() {
        return callbacksRef.current.selectedType;
      },
      get worldSummary() {
        return callbacksRef.current.worldSummary;
      },
      onSelectEntity: (id) => callbacksRef.current.onSelectEntity(id),
      onSpawn: (pose) => callbacksRef.current.onSpawn(pose),
      onMove: (entityId, pose) => callbacksRef.current.onMove(entityId, pose),
      onDragStart: (entityId) => {
        const ent = callbacksRef.current.entities.find((e) => e.entity_id === entityId);
        const pose = ent?.pose ?? { x: 0, y: 0, z: 10 };
        setDragOverride({
          entityId,
          pose: {
            x: Number(pose.x ?? 0),
            y: Number(pose.y ?? 0),
            z: Number(pose.z ?? 10),
          },
        });
      },
      onDragMove: (entityId, pose) => {
        setDragOverride({
          entityId,
          pose: { x: pose.x, y: pose.y, z: pose.z },
        });
      },
      onDragEnd: () => setDragOverride(null),
      onHoverEntity: (entityId) => setHoveredEntityId(entityId),
    });

    return detach;
  }, [sessionId]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !sessionId) return;

    const detach = attachPlanningDrawingHandlers(viewer, {
      enabled: planningDrawing?.enabled === true,
      onAddVertex: (vertex) => planningDrawing?.onMapClick(vertex),
    });

    return detach;
  }, [sessionId, planningDrawing?.enabled, planningDrawing?.onMapClick]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !sessionId) return;
    syncBoundsLayer(viewer, layerVisibility.showBounds, {
      showVertical: layerVisibility.showVerticalBounds,
      showCornerLabels: layerVisibility.showBounds,
    });
  }, [sessionId, layerVisibility.showBounds, layerVisibility.showVerticalBounds]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !sessionId) return;
    syncTerrainLayers(
      viewer,
      entities,
      terrainLayers,
      sensorDomeOptions,
      defenseZoneOptions,
      sensorDomeZoneMode,
    );
    const selected =
      entities.find((e) => e.entity_id === selectedEntityId) ?? null;
    syncHorizonHintLayer(viewer, layerVisibility.showHorizonHint);
    syncStackedLosPresentation(viewer, selected, entities, layerVisibility, terrainLayers);
    syncVisibilityOverlayV4(viewer, selected, entities, layerVisibility, terrainLayers);
    if (shouldUseLegacyLosPath(layerVisibility, terrainLayers, selected)) {
      syncLosSegmentLayer(
        viewer,
        selected,
        entities,
        true,
        terrainLayers.showTerrainMesh,
        false,
      );
    } else if (!layerVisibility.showStackedLos) {
      clearLosSegmentLayer(viewer);
    }
    syncTacticalTrajectoryLayer(viewer, {
      showPath: layerVisibility.showTacticalPredictedPath,
      showInterceptPoint: layerVisibility.showTacticalInterceptPoint,
      showTimingLabels: layerVisibility.showTacticalTimingLabels,
      tacticalState,
      tacticalRecommendation,
      entities,
      applyTerrainDisplay: shouldApplyTerrainGrounding(terrainLayers),
      stale: tacticalState?.tactical_health?.stale === true,
    });
    syncTacticalRankingCueLayer(viewer, {
      enabled: layerVisibility.showTacticalRankingCues,
      tacticalState,
      tacticalRecommendation,
      entities,
      selectedEntityId,
      applyTerrainDisplay: shouldApplyTerrainGrounding(terrainLayers),
      stale: tacticalState?.tactical_health?.stale === true,
    });
    syncTacticalCorridorLayer(viewer, {
      enabled: layerVisibility.showTacticalThreatCorridor,
      tacticalState,
      entities,
      applyTerrainDisplay: shouldApplyTerrainGrounding(terrainLayers),
      stale: tacticalState?.tactical_health?.stale === true,
    });
    const compareEntities =
      tacticalCompare?.compareEntities.length
        ? tacticalCompare.compareEntities
        : entities;
    syncTacticalCompareOverlay(viewer, {
      enabled: layerVisibility.showTacticalCompareOverlay,
      currentState: tacticalState,
      compareState: tacticalCompare?.compareState ?? null,
      entities: compareEntities,
      applyTerrainDisplay: shouldApplyTerrainGrounding(terrainLayers),
      stale: tacticalState?.tactical_health?.stale === true,
      compareSource: tacticalCompare?.source,
    });
    syncTacticalSelectionEmphasisLayer(viewer, {
      enabled: layerVisibility.showTacticalSelectionEmphasis,
      tacticalState,
      entities,
      selectedEntityId,
      applyTerrainDisplay: shouldApplyTerrainGrounding(terrainLayers),
      stale: tacticalState?.tactical_health?.stale === true,
    });
    syncPlanningDefenseAreaLayer(
      viewer,
      planningDrawing?.polygon,
      planningDrawing?.radars,
      planningDrawing?.coverageOptions,
    );
    syncEntityMarkers(viewer, entities, {
      selectedEntityId,
      hoveredEntityId,
      showLabels: layerVisibility.showLabels,
      syncHealth,
      telemetryHealth,
      perEntityDriftM,
      commandGhost,
      dragOverride,
      sessionAccentCss,
      applyTerrainDisplay: shouldApplyTerrainGrounding(terrainLayers),
      markerEmphasis,
      runtimeTelemetryByEntityId,
    });
  }, [
    sessionId,
    entities,
    selectedEntityId,
    layerVisibility,
    terrainLayers,
    sensorDomeOptions,
    defenseZoneOptions,
    sensorDomeZoneMode,
    syncHealth,
    telemetryHealth,
    perEntityDriftM,
    commandGhost,
    dragOverride,
    hoveredEntityId,
    sessionAccentCss,
    markerEmphasis,
    tacticalState,
    tacticalRecommendation,
    tacticalCompare,
    planningDrawing?.polygon,
    planningDrawing?.radars,
    planningDrawing?.coverageOptions,
    runtimeTelemetryByEntityId,
  ]);

  if (!sessionId) {
    return (
      <div className="flex h-[520px] min-h-[420px] items-center justify-center rounded border border-dashed border-slate-700 bg-slate-950/50 text-sm text-slate-500">
        Connect a session to open Cesium runtime view.
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className={`h-[520px] min-h-[420px] w-full overflow-hidden rounded border border-slate-800 ${
        editingEnabled ? "cursor-crosshair" : "cursor-not-allowed opacity-90"
      }`}
      data-testid="cesium-runtime-container"
    />
  );
}
