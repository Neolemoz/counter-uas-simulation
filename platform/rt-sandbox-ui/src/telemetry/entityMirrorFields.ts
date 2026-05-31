import { compactTargetStateChip } from "@/telemetry/entityMirrorUi";

export type EntityRuntimeTelemetry = {
  entityId: string;
  entityType: string;
  position: { x: number; y: number; z: number } | null;
  headingDeg: number | null;
  speedMps: number | null;
  targetState: string | null;
  assignmentState: string | null;
  activeTargetId: string | null;
};

function finiteNumber(value: unknown): number | null {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  if (typeof value === "string" && value.trim() !== "") {
    const parsed = Number(value);
    if (Number.isFinite(parsed)) return parsed;
  }
  return null;
}

function readPosition(row: Record<string, unknown>): EntityRuntimeTelemetry["position"] {
  const source =
    row.position && typeof row.position === "object"
      ? (row.position as Record<string, unknown>)
      : row.pose && typeof row.pose === "object"
        ? (row.pose as Record<string, unknown>)
        : null;
  if (!source) return null;
  const x = finiteNumber(source.x);
  const y = finiteNumber(source.y);
  const z = finiteNumber(source.z);
  if (x === null || y === null || z === null) return null;
  return { x, y, z };
}

function readHeadingDeg(row: Record<string, unknown>): number | null {
  const direct = finiteNumber(row.heading_deg);
  if (direct !== null) return direct;
  if (row.pose && typeof row.pose === "object") {
    return finiteNumber((row.pose as Record<string, unknown>).yaw_deg);
  }
  return null;
}

function readSpeedMps(row: Record<string, unknown>): number | null {
  const direct = finiteNumber(row.speed_mps);
  if (direct !== null) return direct;
  if (row.velocity && typeof row.velocity === "object") {
    return finiteNumber((row.velocity as Record<string, unknown>).speed_mps);
  }
  return null;
}

function readStringField(row: Record<string, unknown>, key: string): string | null {
  const value = row[key];
  if (typeof value !== "string" || !value.trim()) return null;
  return value;
}

export function parseEntityRuntimeTelemetry(
  row: Record<string, unknown>,
): EntityRuntimeTelemetry {
  return {
    entityId: String(row.entity_id ?? ""),
    entityType: String(row.entity_type ?? ""),
    position: readPosition(row),
    headingDeg: readHeadingDeg(row),
    speedMps: readSpeedMps(row),
    targetState: readStringField(row, "target_state"),
    assignmentState: readStringField(row, "assignment_state"),
    activeTargetId: readStringField(row, "active_target_id"),
  };
}

export function parseEntityRuntimeTelemetryMap(
  entities: Array<Record<string, unknown>>,
): Map<string, EntityRuntimeTelemetry> {
  const map = new Map<string, EntityRuntimeTelemetry>();
  for (const row of entities) {
    const parsed = parseEntityRuntimeTelemetry(row);
    if (parsed.entityId) map.set(parsed.entityId, parsed);
  }
  return map;
}

export function findEntityRuntimeTelemetry(
  entities: Array<Record<string, unknown>>,
  entityId: string | null | undefined,
): EntityRuntimeTelemetry | null {
  if (!entityId) return null;
  const row = entities.find((entity) => String(entity.entity_id ?? "") === entityId);
  return row ? parseEntityRuntimeTelemetry(row) : null;
}

export function formatPosition(
  position: EntityRuntimeTelemetry["position"],
): string {
  if (!position) return "—";
  return `(${position.x.toFixed(1)}, ${position.y.toFixed(1)}, ${position.z.toFixed(1)})`;
}

export function formatHeadingDeg(headingDeg: number | null): string {
  return headingDeg === null ? "—" : `${headingDeg.toFixed(0)}°`;
}

export function formatSpeedMps(speedMps: number | null): string {
  return speedMps === null ? "—" : `${speedMps.toFixed(1)} m/s`;
}

export function formatStateLabel(value: string | null): string {
  return value ?? "—";
}

export function formatActiveTargetId(activeTargetId: string | null): string {
  if (!activeTargetId) return "—";
  return activeTargetId.length > 10
    ? activeTargetId.slice(0, 10)
    : activeTargetId;
}

export function compactSelectedLabelSuffix(
  telemetry: EntityRuntimeTelemetry | null | undefined,
): string {
  if (!telemetry) return "";
  const parts: string[] = [];
  if (telemetry.headingDeg !== null) {
    parts.push(`hdg ${telemetry.headingDeg.toFixed(0)}°`);
  }
  if (telemetry.speedMps !== null) {
    parts.push(`spd ${telemetry.speedMps.toFixed(1)}`);
  }
  const targetChip = compactTargetStateChip(telemetry.targetState);
  if (targetChip) {
    parts.push(targetChip);
  }
  return parts.length > 0 ? ` · ${parts.join(" ")}` : "";
}
