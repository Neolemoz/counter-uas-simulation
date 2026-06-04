import {
  BoundingSphere,
  Cartesian3,
  HeadingPitchRange,
  Viewer,
} from "cesium";
import { boundsCenterCartesian } from "./boundsLayer";
import { DEFAULT_CAMERA_HEIGHT_M } from "./constants";
import {
  NOMINAL_SENSOR_DOME_RADIUS_M,
  primaryRidgePolyline,
  sampleTerrainHeight,
  valleyFloorPolyline,
} from "./rtFictionalTerrain";
import {
  DEFAULT_CAMERA_PITCH_RAD,
  ENTITY_FOCUS_PITCH_RAD,
  FIT_ENTITIES_PITCH_RAD,
  markerDisplayZ,
  POLYLINE_PRESET_PITCH_RAD,
  SENSOR_CONTEXT_PITCH_RAD,
  TERRAIN_OVERVIEW_PITCH_RAD,
  ZONE_SURFACE_LIFT_M,
} from "./terrainGrounding";
import {
  zoneBoundaryPositionsGrounded,
} from "./defenseZoneGeometry";
import {
  TERRAIN_OVERVIEW_CAMERA_HEIGHT_M,
  TIGHT_BOUNDS_CAMERA_HEIGHT_M,
} from "./visualStyle";
import { toCesiumEntityId } from "./entityId";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";

export interface CameraLocationTarget {
  latitudeDeg: number;
  longitudeDeg: number;
  label?: string;
}

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
  pitchRad = DEFAULT_CAMERA_PITCH_RAD,
  preserveHeading = true,
): void {
  if (!viewer.scene.globe?.ellipsoid) return;
  viewer.trackedEntity = undefined;
  const heading = preserveHeading ? viewer.camera.heading : 0;
  const offset = new HeadingPitchRange(heading, pitchRad, heightM);
  const sphere = new BoundingSphere(center, 8);
  if (duration <= 0) {
    viewer.camera.viewBoundingSphere(sphere, offset);
    return;
  }
  viewer.camera.flyToBoundingSphere(sphere, {
    duration,
    offset,
  });
}

export function flyToBounds(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyToHeight(viewer, boundsCenterCartesian(), DEFAULT_CAMERA_HEIGHT_M, 0.8);
}

/** Initial session camera — pitched toward terrain (pass #1). */
export function setInitialGroundedCamera(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyToHeight(viewer, boundsCenterCartesian(), DEFAULT_CAMERA_HEIGHT_M, 0);
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
  const sphere = new BoundingSphere(position, 12);
  viewer.trackedEntity = undefined;
  viewer.camera.flyToBoundingSphere(sphere, {
    duration: 0.6,
    offset: new HeadingPitchRange(viewer.camera.heading, ENTITY_FOCUS_PITCH_RAD, 280),
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
        markerDisplayZ(
          Number(ent.pose.x ?? 0),
          Number(ent.pose.y ?? 0),
          Number(ent.pose.z ?? 10),
          true,
        ),
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
    offset: new HeadingPitchRange(
      viewer.camera.heading,
      FIT_ENTITIES_PITCH_RAD,
      Math.max(420, sphere.radius * 2.4),
    ),
  });
}

export function flyToTerrainOverview(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  flyToHeight(
    viewer,
    boundsCenterCartesian(),
    TERRAIN_OVERVIEW_CAMERA_HEIGHT_M,
    1.0,
    TERRAIN_OVERVIEW_PITCH_RAD,
  );
}

export function flyToLocation(
  viewer: Viewer | null | undefined,
  location: CameraLocationTarget,
): void {
  if (!isViewerUsable(viewer)) return;
  const center = Cartesian3.fromDegrees(location.longitudeDeg, location.latitudeDeg, 0);
  flyToHeight(
    viewer,
    center,
    TERRAIN_OVERVIEW_CAMERA_HEIGHT_M,
    1.0,
    TERRAIN_OVERVIEW_PITCH_RAD,
    false,
  );
}

/** @deprecated Planning extent camera fit collapsed into world-fit — use flyToBounds. */
export function flyToPlanningExtent(viewer: Viewer | null | undefined): void {
  flyToBounds(viewer);
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
    offset: new HeadingPitchRange(
      viewer.camera.heading,
      POLYLINE_PRESET_PITCH_RAD,
      Math.max(680, sphere.radius * 3.2),
    ),
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
  const centerZ = applyTerrainDisplay
    ? markerDisplayZ(rx, ry, rz, true)
    : rz + sampleTerrainHeight(rx, ry);
  const center = worldToCartesian(rx, ry, centerZ);
  const r = NOMINAL_SENSOR_DOME_RADIUS_M;
  const ring = zoneBoundaryPositionsGrounded("circle", rx, ry, r, ZONE_SURFACE_LIFT_M, 24);
  const sphere = BoundingSphere.fromPoints([center, ...ring]);
  viewer.trackedEntity = undefined;
  viewer.camera.flyToBoundingSphere(sphere, {
    duration: 0.9,
    offset: new HeadingPitchRange(
      viewer.camera.heading,
      SENSOR_CONTEXT_PITCH_RAD,
      Math.max(360, sphere.radius * 2.1),
    ),
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
    return;
  }
  flyToHeight(viewer, boundsCenterCartesian(), DEFAULT_CAMERA_HEIGHT_M, 0.8);
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
