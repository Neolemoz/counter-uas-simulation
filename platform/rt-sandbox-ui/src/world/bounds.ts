/** Mirror platform/rt-sandbox-bridge/rt_sandbox/governance.py constants. */

export const WORLD_BOUNDS = {
  x: { min: -500, max: 500 },
  y: { min: -500, max: 500 },
  z: { min: 0, max: 200 },
} as const;

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
