import type { UiEntity } from "@/editing/localEntityMirror";
import { countEntitiesByType } from "@/editing/localEntityMirror";
import {
  clampPose,
  ENTITY_TYPE_LIMITS,
  MAX_ENTITY_COUNT,
  type Pose,
} from "@/world/bounds";
import { isEntityType, type EntityType } from "@/world/entityCatalog";

export const SCENARIO_TERRAIN_PRESET = "rt_sandbox_flat" as const;

export type ScenarioEntry = {
  entity_type: EntityType;
  pose: Pose;
};

export type ApplyScenarioPayload = {
  terrain_preset: typeof SCENARIO_TERRAIN_PRESET;
  assets: ScenarioEntry[];
  defenders: ScenarioEntry[];
  attackers: ScenarioEntry[];
};

const ASSET_TYPES = new Set<EntityType>(["radar", "waypoint_marker"]);
const DEFENDER_TYPES = new Set<EntityType>(["interceptor"]);
const ATTACKER_TYPES = new Set<EntityType>(["drone"]);

function poseFromEntity(entity: UiEntity): Pose {
  const raw = entity.pose;
  const base: Pose = {
    x: Number(raw.x ?? 0),
    y: Number(raw.y ?? 0),
    z: Number(raw.z ?? 0),
  };
  if (raw.yaw_deg !== undefined && raw.yaw_deg !== null) {
    base.yaw_deg = Number(raw.yaw_deg);
  }
  return clampPose(base);
}

function entryFromEntity(entity: UiEntity): ScenarioEntry | null {
  if (!isEntityType(entity.entity_type)) return null;
  return {
    entity_type: entity.entity_type,
    pose: poseFromEntity(entity),
  };
}

export function validateScenarioCaps(entities: UiEntity[]): { ok: boolean; reason?: string } {
  if (entities.length === 0) {
    return { ok: false, reason: "No entities to apply" };
  }
  for (const entity of entities) {
    if (!isEntityType(entity.entity_type)) {
      return { ok: false, reason: `Unknown entity type: ${entity.entity_type}` };
    }
  }
  const total = entities.length;
  if (total > MAX_ENTITY_COUNT) {
    return {
      ok: false,
      reason: `Total entity cap exceeded (${total}/${MAX_ENTITY_COUNT})`,
    };
  }
  const byType = countEntitiesByType(entities);
  for (const [entityType, count] of Object.entries(byType)) {
    const limit = ENTITY_TYPE_LIMITS[entityType] ?? 8;
    if (count > limit) {
      return {
        ok: false,
        reason: `Per-type cap exceeded for ${entityType} (${count}/${limit})`,
      };
    }
  }
  return { ok: true };
}

export function entitiesToScenarioPayload(entities: UiEntity[]): ApplyScenarioPayload {
  const assets: ScenarioEntry[] = [];
  const defenders: ScenarioEntry[] = [];
  const attackers: ScenarioEntry[] = [];

  for (const entity of entities) {
    const entry = entryFromEntity(entity);
    if (!entry) continue;
    if (ASSET_TYPES.has(entry.entity_type)) {
      assets.push(entry);
    } else if (DEFENDER_TYPES.has(entry.entity_type)) {
      defenders.push(entry);
    } else if (ATTACKER_TYPES.has(entry.entity_type)) {
      attackers.push(entry);
    }
  }

  return {
    terrain_preset: SCENARIO_TERRAIN_PRESET,
    assets,
    defenders,
    attackers,
  };
}
