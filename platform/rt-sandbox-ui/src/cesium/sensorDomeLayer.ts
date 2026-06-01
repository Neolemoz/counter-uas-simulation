import {
  Cartesian2,
  Cartesian3,
  Color,
  Entity,
  HeadingPitchRoll,
  LabelStyle,
  PolygonHierarchy,
  Transforms,
  VerticalOrigin,
  Viewer,
} from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { isViewerUsable } from "./cesiumEditing";
import { zoneBoundaryPositionsGrounded } from "./defenseZoneGeometry";
import { worldToCartesian } from "./coordinates";
import {
  NOMINAL_SENSOR_DOME_RADIUS_M,
  sampleTerrainHeight,
} from "./rtFictionalTerrain";
import {
  groundedSurfaceZ,
  markerDisplayZ,
  radarLabelLiftM,
  radarVolumeDomeCones,
  radarVolumeSphereCenterZ,
  RADAR_VOLUME_BASE_BLEND_ALPHA,
  ZONE_SURFACE_LIFT_M,
} from "./terrainGrounding";
import { WORLD_BOUNDS } from "@/world/bounds";

const DOME_PREFIX = "rt-terrain-dome-";

const RADAR_RGB = "34, 211, 238";
const RADAR_EDGE_RGB = "186, 230, 253";
const RADAR_HALO_RGB = "15, 23, 42";

/** Readable on topo terrain while staying below defense zone emphasis. */
export const RADAR_RING_ALPHA_SELECTED = 0.78;
export const RADAR_RING_ALPHA_DEFAULT = 0.58;
export const RADAR_FILL_ALPHA_SELECTED = 0.018;
export const RADAR_FILL_ALPHA_DEFAULT = 0.009;
export const RADAR_RING_WIDTH_SELECTED = 2.75;
export const RADAR_RING_WIDTH_DEFAULT = 2.1;
/** Selected-radar 3D volume — tuned for topo readability without overpowering flat ring. */
export const RADAR_VOLUME_FILL_ALPHA = 0.095;
export const RADAR_VOLUME_OUTLINE_ALPHA = 0.58;
export const RADAR_VOLUME_OUTLINE_WIDTH = 2.25;

export interface RadarDomeConfig {
  /** Radar detection radius — single cyan ring. */
  detectionM: number;
}

export interface SensorDomeRenderOptions {
  show?: boolean;
  /** Flat cyan detection ring on globe. */
  showRing?: boolean;
  /** Selected-radar 3D hemisphere preview. */
  showVolume?: boolean;
  selectedEntityId?: string | null;
  selectedOnly?: boolean;
  showLabels?: boolean;
  radii?: RadarDomeConfig;
}

export function shouldShowRadarDetectionRing(options: SensorDomeRenderOptions = {}): boolean {
  if (options.showRing === false) return false;
  return options.show !== false;
}

export function shouldShowRadarVolumePreview(
  isSelected: boolean,
  options: SensorDomeRenderOptions = {},
): boolean {
  if (!shouldRenderRadarVolumeDome(isSelected)) return false;
  return options.showVolume !== false;
}

export function radarPreviewLayerActive(options: SensorDomeRenderOptions = {}): boolean {
  return shouldShowRadarDetectionRing(options) || options.showVolume !== false;
}

/** @deprecated Use RadarDomeConfig — kept for call-site migration. */
export type SensorDomeRadii = RadarDomeConfig;

export const DEFAULT_RADAR_DOME_CONFIG: RadarDomeConfig = {
  detectionM: 300,
};

/** @deprecated Use DEFAULT_RADAR_DOME_CONFIG. */
export const DEFAULT_SENSOR_DOME_RADII: RadarDomeConfig = DEFAULT_RADAR_DOME_CONFIG;

export function normalizedRadarDomeConfig(
  radii?: Partial<RadarDomeConfig>,
): RadarDomeConfig {
  return {
    detectionM: Math.max(
      10,
      Number(radii?.detectionM ?? DEFAULT_RADAR_DOME_CONFIG.detectionM),
    ),
  };
}

/** Migrate legacy inner/middle/outer keys from persisted UI state. */
export function coerceSensorDomeRadii(
  value: Partial<RadarDomeConfig> & {
    innerM?: number;
    middleM?: number;
    outerM?: number;
    coreM?: number;
    engageM?: number;
    warningM?: number;
  },
): RadarDomeConfig {
  const legacy = value as {
    innerM?: number;
    middleM?: number;
    outerM?: number;
    coreM?: number;
    engageM?: number;
    warningM?: number;
  };
  if (legacy.outerM != null && value.detectionM == null) {
    return normalizedRadarDomeConfig({ detectionM: legacy.outerM });
  }
  if (value.detectionM != null) {
    return normalizedRadarDomeConfig(value);
  }
  return DEFAULT_RADAR_DOME_CONFIG;
}

export function horizontalDistanceM(
  ax: number,
  ay: number,
  bx: number,
  by: number,
): number {
  const dx = ax - bx;
  const dy = ay - by;
  return Math.sqrt(dx * dx + dy * dy);
}

/** Count entities within horizontal nominal dome range (explanatory geometry). */
export function countEntitiesInNominalDome(
  radarX: number,
  radarY: number,
  entities: MirrorEntity[],
  radiusM = NOMINAL_SENSOR_DOME_RADIUS_M,
): number {
  let count = 0;
  for (const ent of entities) {
    if (ent.entity_type === "radar") continue;
    const x = Number(ent.pose.x ?? 0);
    const y = Number(ent.pose.y ?? 0);
    if (horizontalDistanceM(radarX, radarY, x, y) <= radiusM) count += 1;
  }
  return count;
}

function ringPositionsGrounded(
  x: number,
  y: number,
  radius: number,
): Cartesian3[] {
  return zoneBoundaryPositionsGrounded("circle", x, y, radius, ZONE_SURFACE_LIFT_M, 96);
}

function labelAtBearing(
  x: number,
  y: number,
  radius: number,
  azimuthDeg: number,
  radiusScale = 1.08,
): Cartesian3 {
  const az = (azimuthDeg * Math.PI) / 180;
  const wx = x + Math.cos(az) * radius * radiusScale;
  const wy = y + Math.sin(az) * radius * radiusScale;
  return worldToCartesian(
    wx,
    wy,
    groundedSurfaceZ(wx, wy, ZONE_SURFACE_LIFT_M) + radarLabelLiftM(),
  );
}

export function labelAzimuthDegForRadarDetection(): number {
  return 24;
}

export function labelAzimuthDegForRadarVertical(): number {
  return 118;
}

export function labelRadiusScaleForRadarDetection(): number {
  return 1.05;
}

export function labelRadiusScaleForRadarVertical(): number {
  return 1.14;
}

export function radarDetectionLabelText(radiusM: number): string {
  return `Detection · ${Math.round(radiusM)}m`;
}

export function radarVerticalCoverageLabelText(): string {
  return `z ${WORLD_BOUNDS.z.min}–${WORLD_BOUNDS.z.max}m`;
}

/** 3D volume preview is selected-radar only — flat rings remain for all radars. */
export function shouldRenderRadarVolumeDome(isSelected: boolean): boolean {
  return isSelected;
}

/** Whether a radar detection ring should render. */
export function shouldRenderRadarRing(
  entityType: string,
  entityId: string,
  selectedEntityId: string | null | undefined,
  selectedRadarOnly: boolean,
): boolean {
  if (entityType !== "radar" || !entityId) return false;
  if (!selectedRadarOnly) return true;
  return entityId === selectedEntityId;
}

/** Fade for radar rings when showing all radars — selected brighter, others softer but visible. */
export function radarRingFade(
  isSelected: boolean,
  selectedRadarPresent: boolean,
  showAllRadars: boolean,
): number {
  if (isSelected) return 1;
  if (!showAllRadars) return 1;
  if (selectedRadarPresent) return 0.58;
  return 0.74;
}

function addRangeLabel(
  viewer: Viewer,
  id: string,
  position: Cartesian3,
  text: string,
  edgeRgb: string,
  fade: number,
  emphasis: number,
  isSelected: boolean,
  pixelOffsetY = -4,
): void {
  const labelAlpha = Math.min(1, 0.94 * fade * emphasis);
  viewer.entities.add(
    new Entity({
      id,
      position,
      label: {
        text,
        font: isSelected ? "bold 12px sans-serif" : "11px sans-serif",
        fillColor: Color.fromCssColorString(`rgba(${edgeRgb}, ${labelAlpha})`),
        outlineColor: Color.fromCssColorString("rgba(15, 23, 42, 0.95)"),
        outlineWidth: isSelected ? 3 : 2,
        style: LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: VerticalOrigin.BOTTOM,
        pixelOffset: new Cartesian2(0, pixelOffsetY),
        showBackground: true,
        backgroundColor: Color.fromCssColorString(`rgba(15, 23, 42, ${isSelected ? 0.9 : 0.74})`),
        backgroundPadding: new Cartesian2(6, 4),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
  );
}

function addRing(
  viewer: Viewer,
  id: string,
  positions: Cartesian3[],
  edgeRgb: string,
  ringAlpha: number,
  ringWidth: number,
  halo?: boolean,
  haloRgb = edgeRgb,
): void {
  const edgeColor = Color.fromCssColorString(`rgba(${edgeRgb}, ${Math.min(1, ringAlpha)})`);
  if (halo && ringAlpha > 0.18) {
    viewer.entities.add(
      new Entity({
        id: `${id}-halo`,
        polyline: {
          positions,
          width: ringWidth + 3.5,
          material: Color.fromCssColorString(`rgba(${haloRgb}, ${Math.min(0.72, ringAlpha * 0.55)})`),
        },
      }),
    );
  }
  viewer.entities.add(
    new Entity({
      id,
      polyline: { positions, width: ringWidth, material: edgeColor },
    }),
  );
}

function removeDomeEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(DOME_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function entityGroundPose(
  ent: MirrorEntity,
  applyTerrainDisplay: boolean,
): { x: number; y: number; z: number; groundZ: number } {
  const x = Number(ent.pose.x ?? 0);
  const y = Number(ent.pose.y ?? 0);
  const zReg = Number(ent.pose.z ?? 10);
  const z = applyTerrainDisplay
    ? markerDisplayZ(x, y, zReg, true)
    : zReg + sampleTerrainHeight(x, y);
  const groundZ = groundedSurfaceZ(x, y, ZONE_SURFACE_LIFT_M);
  return { x, y, z, groundZ };
}

function renderRadarDetectionRing(
  viewer: Viewer,
  ent: MirrorEntity,
  radii: RadarDomeConfig,
  applyTerrainDisplay: boolean,
  options: SensorDomeRenderOptions,
  isSelected: boolean,
  fade: number,
  emphasis: number,
): void {
  const { x, y } = entityGroundPose(ent, applyTerrainDisplay);
  const radius = radii.detectionM;
  const ringAlpha =
    (isSelected ? RADAR_RING_ALPHA_SELECTED : RADAR_RING_ALPHA_DEFAULT) * fade * emphasis;
  const ringWidth = isSelected ? RADAR_RING_WIDTH_SELECTED : RADAR_RING_WIDTH_DEFAULT;
  const positions = ringPositionsGrounded(x, y, radius);
  const fillRing = positions.slice(0, -1);

  viewer.entities.add(
    new Entity({
      id: `${DOME_PREFIX}radar-fill-${ent.entity_id}`,
      polygon: {
        hierarchy: new PolygonHierarchy(fillRing),
        perPositionHeight: true,
        material: Color.fromCssColorString(
          `rgba(${RADAR_RGB}, ${(isSelected ? RADAR_FILL_ALPHA_SELECTED : RADAR_FILL_ALPHA_DEFAULT) * fade * emphasis})`,
        ),
        outline: false,
      },
    }),
  );

  addRing(
    viewer,
    `${DOME_PREFIX}radar-detection-${ent.entity_id}`,
    positions,
    RADAR_EDGE_RGB,
    ringAlpha,
    ringWidth,
    true,
    RADAR_HALO_RGB,
  );

  if (options.showLabels !== false && isSelected) {
    addRangeLabel(
      viewer,
      `${DOME_PREFIX}radar-detection-label-${ent.entity_id}`,
      labelAtBearing(
        x,
        y,
        radius,
        labelAzimuthDegForRadarDetection(),
        labelRadiusScaleForRadarDetection(),
      ),
      radarDetectionLabelText(radius),
      RADAR_EDGE_RGB,
      fade,
      emphasis,
      true,
      -6,
    );
  }
}

function renderRadarVolumeDome(
  viewer: Viewer,
  ent: MirrorEntity,
  radii: RadarDomeConfig,
  applyTerrainDisplay: boolean,
  options: SensorDomeRenderOptions,
  fade: number,
  emphasis: number,
): void {
  const { x, y } = entityGroundPose(ent, applyTerrainDisplay);
  const radius = radii.detectionM;
  const centerZ = radarVolumeSphereCenterZ(x, y, radius);
  const center = worldToCartesian(x, y, centerZ);
  const { minimumCone, maximumCone } = radarVolumeDomeCones();
  const fillAlpha = RADAR_VOLUME_FILL_ALPHA * fade * emphasis;
  const outlineAlpha = Math.min(0.92, RADAR_VOLUME_OUTLINE_ALPHA * fade * emphasis);
  const basePositions = ringPositionsGrounded(x, y, radius * 0.985);
  const baseRing = basePositions.slice(0, -1);

  viewer.entities.add(
    new Entity({
      id: `${DOME_PREFIX}radar-volume-base-${ent.entity_id}`,
      polygon: {
        hierarchy: new PolygonHierarchy(baseRing),
        perPositionHeight: true,
        material: Color.fromCssColorString(
          `rgba(${RADAR_RGB}, ${RADAR_VOLUME_BASE_BLEND_ALPHA * fade * emphasis})`,
        ),
        outline: false,
      },
    }),
  );

  viewer.entities.add(
    new Entity({
      id: `${DOME_PREFIX}radar-volume-${ent.entity_id}`,
      position: center,
      orientation: Transforms.headingPitchRollQuaternion(
        center,
        new HeadingPitchRoll(0, 0, 0),
      ),
      ellipsoid: {
        radii: new Cartesian3(radius, radius, radius),
        minimumCone,
        maximumCone,
        material: Color.fromCssColorString(`rgba(${RADAR_RGB}, ${fillAlpha})`),
        outline: true,
        outlineColor: Color.fromCssColorString(`rgba(224, 242, 254, ${outlineAlpha})`),
        outlineWidth: RADAR_VOLUME_OUTLINE_WIDTH,
        shadows: 0,
      },
    }),
  );

  if (options.showLabels !== false) {
    addRangeLabel(
      viewer,
      `${DOME_PREFIX}radar-vertical-label-${ent.entity_id}`,
      labelAtBearing(
        x,
        y,
        radius,
        labelAzimuthDegForRadarVertical(),
        labelRadiusScaleForRadarVertical(),
      ),
      radarVerticalCoverageLabelText(),
      RADAR_EDGE_RGB,
      fade,
      emphasis,
      true,
      2,
    );
  }
}

export function syncSensorDomeLayer(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[],
  show: boolean,
  applyTerrainDisplay: boolean,
  options: SensorDomeRenderOptions = {},
): void {
  if (!isViewerUsable(viewer)) return;
  removeDomeEntities(viewer);
  if (!show) return;

  const selected = entities.find((ent) => ent.entity_id === options.selectedEntityId) ?? null;
  const selectedRadarOnly =
    options.selectedOnly === true && selected?.entity_type === "radar";
  const showAllRadars = !selectedRadarOnly;
  const selectedRadarPresent = selected?.entity_type === "radar";
  const radii = normalizedRadarDomeConfig(options.radii);

  for (const ent of entities) {
    if (!ent.entity_id || ent.entity_type !== "radar") continue;

    if (
      !shouldRenderRadarRing(
        ent.entity_type,
        ent.entity_id,
        options.selectedEntityId,
        selectedRadarOnly,
      )
    ) {
      continue;
    }

    const isSelected = ent.entity_id === options.selectedEntityId;
    const emphasis = isSelected ? 1.28 : 1;
    const fade = radarRingFade(isSelected, selectedRadarPresent, showAllRadars);
    if (shouldShowRadarDetectionRing(options)) {
      renderRadarDetectionRing(
        viewer,
        ent,
        radii,
        applyTerrainDisplay,
        options,
        isSelected,
        fade,
        emphasis,
      );
    }
    if (shouldShowRadarVolumePreview(isSelected, options)) {
      renderRadarVolumeDome(
        viewer,
        ent,
        radii,
        applyTerrainDisplay,
        options,
        fade,
        emphasis,
      );
    }
  }
}

export function clearSensorDomeLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeDomeEntities(viewer);
}
