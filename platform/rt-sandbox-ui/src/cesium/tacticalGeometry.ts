import type { TacticalStatePayload } from "@/bridge/tacticalCommands";

export type EnuPoint = { x: number; y: number; z: number };

export const THREAT_PATH_TELEMETRY_KEYS = [
  "threat_path_enu_m",
  "attacker_path_enu_m",
  "target_path_enu_m",
] as const;

function finiteCoord(value: unknown): number | null {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

/** Parse a single ENU point from bridge dict `{x,y,z}` or legacy `[x,y,z]` tuple. */
export function parseEnuPathPoint(item: unknown): EnuPoint | null {
  if (Array.isArray(item) && item.length >= 3) {
    const x = finiteCoord(item[0]);
    const y = finiteCoord(item[1]);
    const z = finiteCoord(item[2]);
    if (x === null || y === null || z === null) return null;
    return { x, y, z };
  }

  if (item && typeof item === "object") {
    const row = item as Record<string, unknown>;
    const x = finiteCoord(row.x);
    const y = finiteCoord(row.y);
    const z = finiteCoord(row.z);
    if (x === null || y === null || z === null) return null;
    return { x, y, z };
  }

  return null;
}

/** Normalize path telemetry to canonical `{x,y,z}` points (≥2 required). */
export function normalizeEnuPath(raw: unknown): EnuPoint[] | null {
  if (!Array.isArray(raw) || raw.length < 2) return null;

  const points: EnuPoint[] = [];
  for (const item of raw) {
    const point = parseEnuPathPoint(item);
    if (!point) return null;
    points.push(point);
  }

  return points.length >= 2 ? points : null;
}

export function parseEnuPoseRecord(
  pose: Record<string, unknown> | null | undefined,
): EnuPoint | null {
  if (!pose) return null;
  return parseEnuPathPoint(pose);
}

export function parsePredictedPathTelemetry(
  state: TacticalStatePayload | null | undefined,
): EnuPoint[] | null {
  if (!state) return null;
  const raw = (state as Record<string, unknown>).predicted_path_enu_m;
  return normalizeEnuPath(raw);
}

export function parseThreatPathTelemetry(
  state: TacticalStatePayload | null | undefined,
): EnuPoint[] | null {
  if (!state) return null;
  const rawState = state as Record<string, unknown>;
  for (const key of THREAT_PATH_TELEMETRY_KEYS) {
    const path = normalizeEnuPath(rawState[key]);
    if (path) return path;
  }
  return null;
}

/**
 * Display-only intercept pose: explicit telemetry first, else final predicted path point.
 */
export function deriveDisplayInterceptPose(
  state: TacticalStatePayload | null | undefined,
  telemetryPath: EnuPoint[] | null,
): EnuPoint | null {
  const explicit = parseEnuPoseRecord(
    (state?.last_intercept_pose as Record<string, unknown> | null | undefined) ??
      undefined,
  );
  if (explicit) return explicit;
  if (telemetryPath && telemetryPath.length > 0) {
    return telemetryPath[telemetryPath.length - 1];
  }
  return null;
}
