import {
  BoundingSphere,
  Cartesian3,
  HeadingPitchRange,
  Math as CesiumMath,
  Viewer,
} from "cesium";
import { boundsCenterCartesian } from "./boundsLayer";
import { DEFAULT_CAMERA_HEIGHT_M } from "./constants";
import {
  applyTerrainDisplayOffset,
  NOMINAL_SENSOR_DOME_RADIUS_M,
  primaryRidgePolyline,
  valleyFloorPolyline,
} from "./rtFictionalTerrain";
import {
  TERRAIN_OVERVIEW_CAMERA_HEIGHT_M,
  TIGHT_BOUNDS_CAMERA_HEIGHT_M,
} from "./visualStyle";
import { toCesiumEntityId } from "./entityId";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";

export type CameraPreset =
  | "bounds"
  | "tightBounds"
  | "entities"
  | "terrainOverview"
  | "ridgeLine"
  | "crestLine"
  | "valleyFloor"
  | "sensorContext";

function flyToHeight(
  viewer: Viewer,
  center: Cartesian3,
  heightM: number,
  duration: number,
): void {
  if (!viewer.scene.globe?.ellipsoid) return;
  const carto = viewer.scene.globe.ellipsoid.cartesianToCartographic(center);
  viewer.trackedEntity = undefined;
  viewer.camera.flyTo({
    destination: Cartesian3.fromRadians(
      carto.longitude,
      carto.latitude,
      carto.height + heightM,
    ),
    duration,
  });
}

export function flyToBounds(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyToHeight(viewer, boundsCenterCartesian(), DEFAULT_CAMERA_HEIGHT_M, 0.8);
}

export function flyToTightBounds(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyToHeight(viewer, boundsCenterCartesian(), TIGHT_BOUNDS_CAMERA_HEIGHT_M, 0.8);
}

export function flyToEntity(viewer: Viewer | null | undefined, entityId: string): void {
  if (!isViewerUsable(viewer) || !viewer.scene.globe?.ellipsoid) return;
  const entity = viewer.entities.getById(toCesiumEntityId(entityId));
  if (!entity?.position) return;
  const position =
    entity.position.getValue(viewer.clock.currentTime) ?? boundsCenterCartesian();
  const carto = viewer.scene.globe.ellipsoid.cartesianToCartographic(position);
  viewer.trackedEntity = undefined;
  viewer.camera.flyTo({
    destination: Cartesian3.fromRadians(
      carto.longitude,
      carto.latitude,
      carto.height + 400,
    ),
    duration: 0.6,
  });
}

export function flyToFitEntities(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[],
): void {
  if (!isViewerUsable(viewer)) return;
  const positions: Cartesian3[] = [];
  for (const ent of entities) {
    if (!ent.entity_id) continue;
    positions.push(
      worldToCartesian(
        Number(ent.pose.x ?? 0),
        Number(ent.pose.y ?? 0),
        Number(ent.pose.z ?? 0),
      ),
    );
  }
  if (positions.length === 0) {
    flyToBounds(viewer);
    return;
  }
  const sphere = BoundingSphere.fromPoints(positions);
  viewer.trackedEntity = undefined;
  viewer.camera.flyToBoundingSphere(sphere, {
    duration: 0.8,
    offset: new HeadingPitchRange(0, CesiumMath.toRadians(-45), sphere.radius * 2.5),
  });
}

export function flyToTerrainOverview(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyToHeight(viewer, boundsCenterCartesian(), TERRAIN_OVERVIEW_CAMERA_HEIGHT_M, 1.0);
}

function flyAlongPolyline(viewer: Viewer, polyline: [number, number, number][]): void {
  if (polyline.length < 2) {
    flyToTerrainOverview(viewer);
    return;
  }
  const positions = polyline.map(([x, y, z]) => worldToCartesian(x, y, z));
  const sphere = BoundingSphere.fromPoints(positions);
  viewer.trackedEntity = undefined;
  viewer.camera.flyToBoundingSphere(sphere, {
    duration: 1.0,
    offset: new HeadingPitchRange(0, CesiumMath.toRadians(-35), sphere.radius * 3.5),
  });
}

export function flyToRidgeLine(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyAlongPolyline(viewer, primaryRidgePolyline());
}

export function flyToCrestLine(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyAlongPolyline(viewer, primaryRidgePolyline());
}

export function flyToValleyFloor(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  const valley = valleyFloorPolyline();
  if (valley.length < 2) {
    flyToTerrainOverview(viewer);
    return;
  }
  flyAlongPolyline(viewer, valley);
}

export function flyToSensorContext(
  viewer: Viewer | null | undefined,
  radarEntityId: string,
  entities: MirrorEntity[],
  applyTerrainDisplay = true,
): void {
  if (!isViewerUsable(viewer)) return;
  const radar = entities.find((e) => e.entity_id === radarEntityId);
  if (!radar) {
    flyToEntity(viewer, radarEntityId);
    return;
  }
  const rx = Number(radar.pose.x ?? 0);
  const ry = Number(radar.pose.y ?? 0);
  const rz = Number(radar.pose.z ?? 10);
  const z = applyTerrainDisplay ? applyTerrainDisplayOffset(rx, ry, rz) : rz;
  const center = worldToCartesian(rx, ry, z);
  const r = NOMINAL_SENSOR_DOME_RADIUS_M;
  const ring: Cartesian3[] = [];
  for (let i = 0; i <= 8; i++) {
    const a = (i / 8) * Math.PI * 2;
    ring.push(worldToCartesian(rx + Math.cos(a) * r, ry + Math.sin(a) * r, z));
  }
  const sphere = BoundingSphere.fromPoints([center, ...ring]);
  viewer.trackedEntity = undefined;
  viewer.camera.flyToBoundingSphere(sphere, {
    duration: 0.9,
    offset: new HeadingPitchRange(0, CesiumMath.toRadians(-40), sphere.radius * 2.2),
  });
}

export function flyToPreset(
  viewer: Viewer | null | undefined,
  preset: CameraPreset,
  entities: MirrorEntity[] = [],
  options?: { selectedEntityId?: string | null; applyTerrainDisplay?: boolean },
): void {
  switch (preset) {
    case "tightBounds":
      flyToTightBounds(viewer);
      break;
    case "entities":
      flyToFitEntities(viewer, entities);
      break;
    case "terrainOverview":
      flyToTerrainOverview(viewer);
      break;
    case "ridgeLine":
      flyToRidgeLine(viewer);
      break;
    case "crestLine":
      flyToCrestLine(viewer);
      break;
    case "valleyFloor":
      flyToValleyFloor(viewer);
      break;
    case "sensorContext":
      if (options?.selectedEntityId) {
        flyToSensorContext(
          viewer,
          options.selectedEntityId,
          entities,
          options.applyTerrainDisplay ?? true,
        );
      } else {
        flyToTerrainOverview(viewer);
      }
      break;
    case "bounds":
    default:
      flyToBounds(viewer);
      break;
  }
}

/** Smooth framing when switching tabs / creating viewer (PLAT-RT-V1). */
export function flyOnSessionSwitch(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[] = [],
): void {
  if (!isViewerUsable(viewer)) return;
  if (entities.length > 0) {
    flyToFitEntities(viewer, entities);
  } else {
    flyToBounds(viewer);
  }
}

export function setFollowEntity(
  viewer: Viewer | null | undefined,
  entityId: string | null,
): void {
  if (!isViewerUsable(viewer)) return;
  if (!entityId) {
    viewer.trackedEntity = undefined;
    return;
  }
  const entity = viewer.entities.getById(toCesiumEntityId(entityId));
  viewer.trackedEntity = entity ?? undefined;
}

export function cameraHeightM(viewer: Viewer | null | undefined): number {
  if (!isViewerUsable(viewer) || !viewer.scene.globe?.ellipsoid) {
    return DEFAULT_CAMERA_HEIGHT_M;
  }
  const carto = viewer.scene.globe.ellipsoid.cartesianToCartographic(
    viewer.camera.position,
  );
  return carto.height;
}
