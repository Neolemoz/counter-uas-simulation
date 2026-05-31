import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import type { EnuPoint, TacticalTrajectoryGeometry } from "./tacticalTrajectoryLayer";
import { resolveTacticalRoleIds } from "./tacticalTrajectoryLayer";

function poseFromRecord(
  pose: Record<string, unknown> | undefined,
): EnuPoint | null {
  if (!pose) return null;
  const x = Number(pose.x);
  const y = Number(pose.y);
  const z = Number(pose.z);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return null;
  }
  return { x, y, z };
}

function entityPose(entity: MirrorEntity | undefined): EnuPoint | null {
  return poseFromRecord(entity?.pose);
}

function parseThreatPathTelemetry(
  state: TacticalStatePayload | null | undefined,
): EnuPoint[] | null {
  if (!state) return null;
  const rawState = state as Record<string, unknown>;
  for (const key of [
    "threat_path_enu_m",
    "attacker_path_enu_m",
    "target_path_enu_m",
  ]) {
    const raw = rawState[key];
    if (!Array.isArray(raw) || raw.length < 2) continue;
    const points: EnuPoint[] = [];
    for (const item of raw) {
      if (!Array.isArray(item) || item.length < 3) {
        points.length = 0;
        break;
      }
      const x = Number(item[0]);
      const y = Number(item[1]);
      const z = Number(item[2]);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
        points.length = 0;
        break;
      }
      points.push({ x, y, z });
    }
    if (points.length >= 2) return points;
  }
  return null;
}

export interface ThreatCorridorGeometry {
  corridorPoints: EnuPoint[];
  mode: "telemetry_path" | "direct_fallback";
}

export function deriveThreatCorridorGeometry(
  state: TacticalStatePayload | null | undefined,
  geometry: TacticalTrajectoryGeometry,
  entities: MirrorEntity[],
): ThreatCorridorGeometry | null {
  const { targetId } = resolveTacticalRoleIds(state);
  if (!targetId) return null;

  const target = entities.find((e) => e.entity_id === targetId);
  const attackerPose = entityPose(target);
  if (!attackerPose) return null;

  const solutionPose =
    geometry.interceptPose ??
    (geometry.pathEndPose ? geometry.pathEndPose : null);
  if (!solutionPose) return null;

  const samePoint =
    Math.hypot(
      attackerPose.x - solutionPose.x,
      attackerPose.y - solutionPose.y,
      attackerPose.z - solutionPose.z,
    ) < 1;
  if (samePoint) return null;

  const telemetryPath = parseThreatPathTelemetry(state);
  if (telemetryPath) {
    const last = telemetryPath[telemetryPath.length - 1];
    const distToSolution = Math.hypot(
      last.x - solutionPose.x,
      last.y - solutionPose.y,
      last.z - solutionPose.z,
    );
    const corridorPoints =
      distToSolution > 1
        ? [...telemetryPath, solutionPose]
        : telemetryPath;
    return { corridorPoints, mode: "telemetry_path" };
  }

  return {
    corridorPoints: [attackerPose, solutionPose],
    mode: "direct_fallback",
  };
}

/** Ribbon polygon around a polyline for translucent corridor fill. */
export function buildCorridorRibbonPolygon(
  points: EnuPoint[],
  halfWidthM: number,
): EnuPoint[] {
  if (points.length < 2 || halfWidthM <= 0) return [];

  const left: EnuPoint[] = [];
  const right: EnuPoint[] = [];

  for (let i = 0; i < points.length; i += 1) {
    const prev = points[Math.max(0, i - 1)];
    const next = points[Math.min(points.length - 1, i + 1)];
    let dx = next.x - prev.x;
    let dy = next.y - prev.y;
    const len = Math.hypot(dx, dy);
    if (len < 1e-6) {
      dx = next.x - points[i].x;
      dy = next.y - points[i].y;
    }
    const segLen = Math.hypot(dx, dy) || 1;
    const nx = (-dy / segLen) * halfWidthM;
    const ny = (dx / segLen) * halfWidthM;
    const p = points[i];
    left.push({ x: p.x + nx, y: p.y + ny, z: p.z });
    right.push({ x: p.x - nx, y: p.y - ny, z: p.z });
  }

  return [...left, ...right.reverse()];
}
