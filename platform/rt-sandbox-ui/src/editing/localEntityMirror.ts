import type { EditCommandType } from "./editHistory";
import type { Pose } from "@/world/bounds";

export interface UiEntity {
  entity_id: string;
  entity_type: string;
  pose: Record<string, unknown>;
}

export type LocalEntityMap = Record<string, UiEntity>;

function normalizeEntity(value: unknown): UiEntity | null {
  if (!value || typeof value !== "object") return null;
  const row = value as Record<string, unknown>;
  const entityId = String(row.entity_id ?? "");
  if (!entityId) return null;
  const pose = row.pose;
  return {
    entity_id: entityId,
    entity_type: String(row.entity_type ?? ""),
    pose: pose && typeof pose === "object" ? (pose as Record<string, unknown>) : {},
  };
}

function entitiesFromResponse(
  result: Record<string, unknown>,
): UiEntity[] {
  const entities = result.entities;
  if (!Array.isArray(entities)) return [];
  return entities
    .map((entity) => normalizeEntity(entity))
    .filter((entity): entity is UiEntity => entity !== null);
}

export function applyLocalEntityCommand(
  existing: LocalEntityMap,
  commandType: EditCommandType,
  result: Record<string, unknown>,
  meta: { entityId?: string; entityType?: string; pose?: Pose },
): LocalEntityMap {
  const next = { ...existing };
  if (commandType === "delete_entity") {
    if (meta.entityId) delete next[meta.entityId];
    return next;
  }

  for (const entity of entitiesFromResponse(result)) {
    next[entity.entity_id] = entity;
  }

  const resultEntityId =
    typeof result.entity_id === "string" ? result.entity_id : undefined;
  const entityId = meta.entityId ?? resultEntityId;
  if (entityId && meta.pose) {
    const current = next[entityId];
    next[entityId] = {
      entity_id: entityId,
      entity_type: meta.entityType ?? current?.entity_type ?? "",
      pose: { ...meta.pose },
    };
  }
  return next;
}

export function mergeTelemetryAndLocalEntities(
  telemetryEntities: UiEntity[],
  localEntities: LocalEntityMap,
  locallyDeletedIds: ReadonlySet<string>,
): UiEntity[] {
  const merged = new Map<string, UiEntity>();
  for (const entity of telemetryEntities) {
    if (!locallyDeletedIds.has(entity.entity_id)) {
      merged.set(entity.entity_id, entity);
    }
  }
  for (const entity of Object.values(localEntities)) {
    if (!locallyDeletedIds.has(entity.entity_id)) {
      merged.set(entity.entity_id, entity);
    }
  }
  return [...merged.values()];
}

export function countEntitiesByType(entities: UiEntity[]): Record<string, number> {
  const byType: Record<string, number> = {};
  for (const entity of entities) {
    byType[entity.entity_type] = (byType[entity.entity_type] ?? 0) + 1;
  }
  return byType;
}

export function pruneLocalEntities(
  localEntities: LocalEntityMap,
  telemetryEntities: UiEntity[],
): LocalEntityMap {
  const telemetryIds = new Set(telemetryEntities.map((entity) => entity.entity_id));
  const next: LocalEntityMap = {};
  for (const [entityId, entity] of Object.entries(localEntities)) {
    if (!telemetryIds.has(entityId)) {
      next[entityId] = entity;
    }
  }
  return next;
}
