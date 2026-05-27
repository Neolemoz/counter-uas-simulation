import { useEffect, useRef, useState } from "react";
import {
  Cartesian3,
  EllipsoidTerrainProvider,
  ImageryLayer,
  OpenStreetMapImageryProvider,
  Terrain,
  Viewer,
} from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import { syncBoundsLayer, boundsCenterCartesian } from "./boundsLayer";
import { attachCesiumEditingHandlers } from "./cesiumEditing";
import { isViewerUsable } from "./cesiumEditing";
import { DEFAULT_CAMERA_HEIGHT_M } from "./constants";
import { flyOnSessionSwitch } from "./cameraHelpers";
import {
  clearEntityMarkers,
  syncEntityMarkers,
  type MirrorEntity,
} from "./entityMarkers";
import { syncLosSegmentLayer } from "./losSegmentLayer";
import {
  anyTerrainLayerEnabled,
  clearAllTerrainLayers,
  syncTerrainLayers,
  type TerrainLayerVisibility,
} from "./terrainLayers";

export interface CesiumRuntimeViewProps {
  sessionId: string | null;
  sessionAccentCss?: string;
  entities: MirrorEntity[];
  selectedEntityId: string | null;
  showBounds: boolean;
  showVerticalBounds: boolean;
  showLabels: boolean;
  terrainLayers: TerrainLayerVisibility;
  syncHealth?: string;
  telemetryHealth?: string;
  perEntityDriftM?: Record<string, number>;
  commandGhost?: { entityId: string; pose: { x: number; y: number; z: number } } | null;
  editingEnabled: boolean;
  selectedType: EntityType;
  worldSummary: Record<string, unknown> | undefined;
  onViewerReady?: (viewer: Viewer | null) => void;
  onSelectEntity: (id: string | null) => void;
  onSpawn: (pose: Pose) => void;
  onMove: (entityId: string, pose: Pose) => void;
}

function createViewer(container: HTMLDivElement): Viewer {
  return new Viewer(container, {
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
    baseLayer: new ImageryLayer(
      new OpenStreetMapImageryProvider({
        url: "https://tile.openstreetmap.org/",
      }),
    ),
    terrain: new Terrain(
      Promise.resolve(new EllipsoidTerrainProvider()),
    ),
  });
}

export function CesiumRuntimeView({
  sessionId,
  sessionAccentCss,
  entities,
  selectedEntityId,
  showBounds,
  showVerticalBounds,
  showLabels,
  terrainLayers,
  syncHealth,
  telemetryHealth,
  perEntityDriftM,
  commandGhost,
  editingEnabled,
  selectedType,
  worldSummary,
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

  const callbacksRef = useRef({
    onSelectEntity,
    onSpawn,
    onMove,
    worldSummary,
    selectedType,
    editingEnabled,
  });
  callbacksRef.current = {
    onSelectEntity,
    onSpawn,
    onMove,
    worldSummary,
    selectedType,
    editingEnabled,
  };

  useEffect(() => {
    if (!containerRef.current || !sessionId) return;

    const viewer = createViewer(containerRef.current);
    viewerRef.current = viewer;
    onViewerReady?.(viewer);

    if (isViewerUsable(viewer) && viewer.scene.globe?.ellipsoid) {
      const center = boundsCenterCartesian();
      const carto = viewer.scene.globe.ellipsoid.cartesianToCartographic(center);
      viewer.camera.setView({
        destination: Cartesian3.fromRadians(
          carto.longitude,
          carto.latitude,
          carto.height + DEFAULT_CAMERA_HEIGHT_M,
        ),
      });
      window.setTimeout(() => flyOnSessionSwitch(viewer, []), 0);
    }

    return () => {
      if (viewerRef.current === viewer) {
        viewerRef.current = null;
      }
      onViewerReady?.(null);
      setDragOverride(null);
      if (isViewerUsable(viewer)) {
        clearEntityMarkers(viewer);
        clearAllTerrainLayers(viewer);
        viewer.trackedEntity = undefined;
      }
      if (!viewer.isDestroyed()) {
        viewer.destroy();
      }
    };
  }, [sessionId, onViewerReady]);

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
        const ent = entities.find((e) => e.entity_id === entityId);
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
    });

    return detach;
  }, [sessionId, entities]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !sessionId) return;
    syncBoundsLayer(viewer, showBounds, {
      showVertical: showVerticalBounds,
      showCornerLabels: showBounds,
    });
  }, [sessionId, showBounds, showVerticalBounds]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !sessionId) return;
    syncTerrainLayers(viewer, entities, terrainLayers);
    const selected =
      entities.find((e) => e.entity_id === selectedEntityId) ?? null;
    syncLosSegmentLayer(
      viewer,
      selected,
      entities,
      anyTerrainLayerEnabled(terrainLayers) && selected != null,
      terrainLayers.showTerrainMesh,
    );
    syncEntityMarkers(viewer, entities, {
      selectedEntityId,
      showLabels,
      syncHealth,
      telemetryHealth,
      perEntityDriftM,
      commandGhost,
      dragOverride,
      sessionAccentCss,
      applyTerrainDisplay: terrainLayers.showTerrainMesh,
    });
  }, [
    sessionId,
    entities,
    selectedEntityId,
    showLabels,
    terrainLayers,
    syncHealth,
    telemetryHealth,
    perEntityDriftM,
    commandGhost,
    dragOverride,
    sessionAccentCss,
  ]);

  if (!sessionId) {
    return (
      <div className="flex h-80 items-center justify-center rounded border border-dashed border-slate-600 bg-slate-950/50 text-sm text-slate-500">
        Connect a session to open Cesium runtime view.
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className={`h-80 w-full overflow-hidden rounded border border-slate-700 ${
        editingEnabled ? "cursor-crosshair" : "cursor-not-allowed opacity-90"
      }`}
      data-testid="cesium-runtime-container"
    />
  );
}
