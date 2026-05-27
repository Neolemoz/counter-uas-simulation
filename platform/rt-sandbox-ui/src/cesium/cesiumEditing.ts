import {
  Cartesian2,
  ScreenSpaceEventHandler,
  ScreenSpaceEventType,
  type Viewer,
} from "cesium";
import type { EntityType } from "@/world/entityCatalog";
import { defaultPose } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";
import { canSpawn, clampPose } from "@/world/bounds";
import { cartographicToWorld } from "./coordinates";
import { parseRtEntityId } from "./entityId";

export interface CesiumEditingOptions {
  editingEnabled: boolean;
  selectedType: EntityType;
  worldSummary: Record<string, unknown> | undefined;
  onSelectEntity: (id: string | null) => void;
  onSpawn: (pose: Pose) => void;
  onMove: (entityId: string, pose: Pose) => void;
  onDragStart?: (entityId: string) => void;
  onDragMove?: (entityId: string, pose: Pose) => void;
  onDragEnd?: () => void;
}

export function isViewerUsable(viewer: Viewer | null | undefined): viewer is Viewer {
  if (!viewer) return false;
  if (typeof viewer.isDestroyed === "function" && viewer.isDestroyed()) return false;
  return Boolean(viewer.scene && viewer.camera && viewer.entities);
}

export function restoreCamera(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  const controller = viewer.scene.screenSpaceCameraController;
  if (!controller) return;
  controller.enableRotate = true;
  controller.enableTranslate = true;
  controller.enableZoom = true;
  controller.enableTilt = true;
  controller.enableLook = true;
}

function disableCamera(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  const controller = viewer.scene.screenSpaceCameraController;
  if (!controller) return;
  controller.enableRotate = false;
  controller.enableTranslate = false;
  controller.enableZoom = false;
  controller.enableTilt = false;
  controller.enableLook = false;
}

function pickRtEntityId(viewer: Viewer | null | undefined, position: Cartesian2): string | null {
  if (!isViewerUsable(viewer)) return null;
  const picked = viewer.scene.pick(position);
  if (!picked) return null;
  const id =
    typeof picked.id === "string"
      ? picked.id
      : picked.id && typeof picked.id === "object" && "id" in picked.id
        ? String((picked.id as { id?: string }).id ?? "")
        : "";
  return parseRtEntityId(id);
}

function pickWorldEnu(viewer: Viewer | null | undefined, position: Cartesian2): Pose | null {
  if (!isViewerUsable(viewer)) return null;
  if (!viewer.scene.globe?.ellipsoid) return null;
  const ray = viewer.camera.getPickRay(position);
  if (!ray) return null;
  const cartesian =
    viewer.scene.globe.pick(ray, viewer.scene) ??
    viewer.camera.pickEllipsoid(position, viewer.scene.globe.ellipsoid);
  if (!cartesian) return null;
  const carto = viewer.scene.globe.ellipsoid.cartesianToCartographic(cartesian);
  const world = cartographicToWorld({
    longitude: carto.longitude,
    latitude: carto.latitude,
    height: carto.height,
  });
  return clampPose({ x: world.x, y: world.y, z: world.z, yaw_deg: 0 });
}

export function attachCesiumEditingHandlers(
  viewer: Viewer | null | undefined,
  options: CesiumEditingOptions,
): () => void {
  if (!isViewerUsable(viewer) || !viewer.scene.canvas) return () => undefined;
  const handler = new ScreenSpaceEventHandler(viewer.scene.canvas);
  let draggingEntityId: string | null = null;
  let dragZ = 10;
  let pointerDownEntity: string | null = null;
  let dragMoved = false;

  handler.setInputAction((movement: { position: Cartesian2 }) => {
    if (!isViewerUsable(viewer)) return;
    if (!options.editingEnabled) return;
    const entityId = pickRtEntityId(viewer, movement.position);
    if (entityId) {
      dragMoved = false;
      pointerDownEntity = entityId;
      draggingEntityId = entityId;
      options.onSelectEntity(entityId);
      disableCamera(viewer);
      options.onDragStart?.(entityId);
      const ent = viewer.entities.getById(`rt-entity-${entityId}`);
      if (ent?.position) {
        const pos = ent.position.getValue(viewer.clock.currentTime);
        if (pos) {
          const carto = viewer.scene.globe.ellipsoid.cartesianToCartographic(pos);
          dragZ = cartographicToWorld({
            longitude: carto.longitude,
            latitude: carto.latitude,
            height: carto.height,
          }).z;
        }
      }
    }
  }, ScreenSpaceEventType.LEFT_DOWN);

  handler.setInputAction((movement: { position: Cartesian2 }) => {
    if (!isViewerUsable(viewer)) return;
    if (!options.editingEnabled) return;
    if (dragMoved) return;
    const entityId = pickRtEntityId(viewer, movement.position);
    if (entityId) {
      options.onSelectEntity(entityId);
      return;
    }
    const pose = pickWorldEnu(viewer, movement.position);
    if (!pose) return;
    const check = canSpawn(options.worldSummary, options.selectedType);
    if (!check.ok) return;
    options.onSpawn(clampPose(defaultPose(options.selectedType, pose.x, pose.y)));
    options.onSelectEntity(null);
  }, ScreenSpaceEventType.LEFT_CLICK);

  handler.setInputAction((movement: { endPosition: Cartesian2 }) => {
    if (!isViewerUsable(viewer)) return;
    if (!draggingEntityId || !options.editingEnabled) return;
    const pose = pickWorldEnu(viewer, movement.endPosition);
    if (pose) {
      dragMoved = true;
      options.onDragMove?.(draggingEntityId, clampPose({ ...pose, z: dragZ, yaw_deg: 0 }));
    }
  }, ScreenSpaceEventType.MOUSE_MOVE);

  handler.setInputAction((movement: { position: Cartesian2 }) => {
    if (!isViewerUsable(viewer)) {
      draggingEntityId = null;
      pointerDownEntity = null;
      dragMoved = false;
      return;
    }
    if (!options.editingEnabled) {
      draggingEntityId = null;
      pointerDownEntity = null;
      restoreCamera(viewer);
      return;
    }
    if (draggingEntityId && pointerDownEntity) {
      const entityId = draggingEntityId;
      const moved = dragMoved;
      draggingEntityId = null;
      pointerDownEntity = null;
      restoreCamera(viewer);
      options.onDragEnd?.();
      const pose = pickWorldEnu(viewer, movement.position);
      if (pose && moved) {
        options.onMove(entityId, clampPose({ ...pose, z: dragZ, yaw_deg: 0 }));
      }
      dragMoved = false;
    }
  }, ScreenSpaceEventType.LEFT_UP);

  return () => {
    restoreCamera(viewer);
    if (!handler.isDestroyed()) {
      handler.destroy();
    }
    draggingEntityId = null;
    pointerDownEntity = null;
  };
}

export { pickRtEntityId, pickWorldEnu };
