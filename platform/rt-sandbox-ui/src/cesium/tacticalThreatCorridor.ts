import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import {
  parseEnuPoseRecord,
  parseThreatPathTelemetry,
  type EnuPoint,
} from "./tacticalGeometry";
import type { TacticalTrajectoryGeometry } from "./tacticalTrajectoryLayer";
import { resolveTacticalRoleIds } from "./tacticalTrajectoryLayer";

function entityPose(entity: MirrorEntity | undefined): EnuPoint | null {
  return parseEnuPoseRecord(entity?.pose);
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
