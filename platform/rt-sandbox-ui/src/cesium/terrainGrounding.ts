import { Math as CesiumMath } from "cesium";
import { sampleTerrainHeight } from "./rtFictionalTerrain";

/** Pass #2 — tilt toward terrain, less horizon in default framing. */
export const DEFAULT_CAMERA_PITCH_DEG = -68;
export const TERRAIN_OVERVIEW_PITCH_DEG = -70;
export const FIT_ENTITIES_PITCH_DEG = -62;
export const POLYLINE_PRESET_PITCH_DEG = -58;
export const SENSOR_CONTEXT_PITCH_DEG = -60;
export const ENTITY_FOCUS_PITCH_DEG = -56;

export const DEFAULT_CAMERA_PITCH_RAD = CesiumMath.toRadians(DEFAULT_CAMERA_PITCH_DEG);
export const TERRAIN_OVERVIEW_PITCH_RAD = CesiumMath.toRadians(TERRAIN_OVERVIEW_PITCH_DEG);
export const FIT_ENTITIES_PITCH_RAD = CesiumMath.toRadians(FIT_ENTITIES_PITCH_DEG);
export const POLYLINE_PRESET_PITCH_RAD = CesiumMath.toRadians(POLYLINE_PRESET_PITCH_DEG);
export const SENSOR_CONTEXT_PITCH_RAD = CesiumMath.toRadians(SENSOR_CONTEXT_PITCH_DEG);
export const ENTITY_FOCUS_PITCH_RAD = CesiumMath.toRadians(ENTITY_FOCUS_PITCH_DEG);

/** Visual-only marker AGL when terrain display is off — registry pose stays authoritative. */
export const MARKER_VISUAL_AGL_MIN_M = 0.35;
export const MARKER_VISUAL_AGL_MAX_M = 1.15;
export const MARKER_VISUAL_AGL_SCALE = 0.04;

/** Minimal lift above sampled terrain for rings / zone boundaries (m). */
export const ZONE_SURFACE_LIFT_M = 0.04;
export const MARKER_SURFACE_LIFT_M = 0.06;

/** Sink radar volume equator slightly into terrain to hide hover gap (visual only). */
export const RADAR_VOLUME_CENTER_BLEND_M = 0.012;
/** Flat base disc under selected radar volume — blends dome with terrain. */
export const RADAR_VOLUME_BASE_BLEND_ALPHA = 0.055;

/** Raw fictional terrain height at ENU x/y (m). */
export function terrainBaseZ(x: number, y: number): number {
  return sampleTerrainHeight(x, y);
}

export function markerVisualRegistryZ(registryZ: number): number {
  const scaled = registryZ * MARKER_VISUAL_AGL_SCALE + MARKER_VISUAL_AGL_MIN_M;
  return Math.max(
    MARKER_VISUAL_AGL_MIN_M,
    Math.min(MARKER_VISUAL_AGL_MAX_M, scaled),
  );
}

/** Display-only marker height — clamps to terrain; ignores registry z when terrain grounding is on. */
export function markerDisplayZ(
  x: number,
  y: number,
  registryZ: number,
  applyTerrainDisplay: boolean,
): number {
  const base = terrainBaseZ(x, y);
  if (applyTerrainDisplay) {
    return base + MARKER_SURFACE_LIFT_M;
  }
  return base + markerVisualRegistryZ(registryZ);
}

export function groundedSurfaceZ(x: number, y: number, liftM = ZONE_SURFACE_LIFT_M): number {
  return terrainBaseZ(x, y) + liftM;
}

/** Sphere center for an upper-hemisphere radar dome — equator blends into terrain. */
export function radarVolumeSphereCenterZ(x: number, y: number, _radiusM: number): number {
  return terrainBaseZ(x, y) + RADAR_VOLUME_CENTER_BLEND_M;
}

export function radarVolumeDomeCones(): {
  minimumCone: number;
  maximumCone: number;
} {
  return {
    minimumCone: 0,
    maximumCone: CesiumMath.PI_OVER_TWO,
  };
}

export function zoneLabelLiftM(suffix: "core" | "mid" | "warning"): number {
  if (suffix === "core") return 1.8;
  if (suffix === "mid") return 2.4;
  return 3.2;
}

export function radarLabelLiftM(): number {
  return 1.6;
}
