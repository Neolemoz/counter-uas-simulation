/** Mirror platform/rt-sandbox-bridge/rt_sandbox/governance.py constants. */

export const WORLD_BOUNDS = {
  x: { min: -7000, max: 7000 },
  y: { min: -7000, max: 7000 },
  z: { min: 0, max: 200 },
} as const;

/** Half-extent of the unified runtime world on each horizontal axis (m). */
export const WORLD_AXIS_HALF_EXTENT_M = WORLD_BOUNDS.x.max;

/** Camera height to frame the full ±half-extent world (display-only). */
export const WORLD_FIT_CAMERA_HEIGHT_M = Math.max(
  1200,
  Math.round(WORLD_AXIS_HALF_EXTENT_M * 1.45),
);

/** Legacy SVG core grid covers a local inset only — not the full runtime world. */
export const CORE_GRID_LOCAL_INSET_RADIUS_M = 1000;

export const CESIUM_PRIMARY_EDITING_COPY =
  "Cesium globe is the primary editing surface for the unified 7 km runtime world (±7000 m).";

export const CORE_GRID_LOCAL_COPY =
  "Legacy core grid covers a ~1 km local inset only; use the Cesium globe for full-world placement.";

export function boundsGroundLabel(): string {
  return `±${WORLD_AXIS_HALF_EXTENT_M}m`;
}

export function unifiedWorldCopy(): string {
  return `Unified runtime world ±${WORLD_AXIS_HALF_EXTENT_M} m`;
}

export const MAX_ENTITY_COUNT = 32;

export const ENTITY_TYPE_LIMITS: Record<string, number> = {
  radar: 8,
  interceptor: 8,
  drone: 8,
  waypoint_marker: 8,
};

export const EDITABLE_SESSION_STATES = new Set(["running", "paused"]);

export const COMMAND_BURST_INTERVAL_MS = 200;

export interface Pose {
  x: number;
  y: number;
  z: number;
  yaw_deg?: number;
}

export function clampPose(pose: Pose): Pose {
  return {
    x: Math.min(WORLD_BOUNDS.x.max, Math.max(WORLD_BOUNDS.x.min, pose.x)),
    y: Math.min(WORLD_BOUNDS.y.max, Math.max(WORLD_BOUNDS.y.min, pose.y)),
    z: Math.min(WORLD_BOUNDS.z.max, Math.max(WORLD_BOUNDS.z.min, pose.z)),
    ...(pose.yaw_deg !== undefined ? { yaw_deg: pose.yaw_deg } : {}),
  };
}

export function canSpawn(
  worldSummary: Record<string, unknown> | undefined,
  entityType: string,
): { ok: boolean; reason?: string } {
  if (!worldSummary) return { ok: true };
  const count = Number(worldSummary.entity_count ?? 0);
  if (count >= MAX_ENTITY_COUNT) {
    return { ok: false, reason: "Total entity cap reached (32)" };
  }
  const byType = (worldSummary.by_type as Record<string, number>) ?? {};
  const typeCount = Number(byType[entityType] ?? 0);
  const limit = ENTITY_TYPE_LIMITS[entityType] ?? 8;
  if (typeCount >= limit) {
    return { ok: false, reason: `Per-type cap reached for ${entityType} (${limit})` };
  }
  return { ok: true };
}

export function isEditingAllowed(sessionState: string): boolean {
  return EDITABLE_SESSION_STATES.has(sessionState);
}
