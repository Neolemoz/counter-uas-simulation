import type { UiEntity } from "@/editing/localEntityMirror";
import { SCENARIO_TERRAIN_PRESET } from "@/world/scenarioPayload";
import { isEntityType } from "@/world/entityCatalog";
import type { EntityType } from "@/world/entityCatalog";
import {
  LAYOUT_SCHEMA_VERSION,
  type RtLayoutEntity,
  type RtLayoutScenarioV1,
} from "./rtLayoutMcProfile";

const TYPE_SORT_ORDER: Record<EntityType, number> = {
  radar: 0,
  interceptor: 1,
  drone: 2,
  waypoint_marker: 3,
};

function layoutEntityFromUi(entity: UiEntity): RtLayoutEntity | null {
  if (!isEntityType(entity.entity_type)) return null;
  const raw = entity.pose;
  const pose: RtLayoutEntity["pose"] = {
    x: Number(raw.x ?? 0),
    y: Number(raw.y ?? 0),
    z: Number(raw.z ?? 0),
  };
  if (raw.yaw_deg !== undefined && raw.yaw_deg !== null) {
    pose.yaw_deg = Number(raw.yaw_deg);
  }
  return { entity_type: entity.entity_type, pose };
}

function compareLayoutEntities(a: RtLayoutEntity, b: RtLayoutEntity): number {
  const typeDelta =
    TYPE_SORT_ORDER[a.entity_type as EntityType] -
    TYPE_SORT_ORDER[b.entity_type as EntityType];
  if (typeDelta !== 0) return typeDelta;
  if (a.pose.x !== b.pose.x) return a.pose.x - b.pose.x;
  if (a.pose.y !== b.pose.y) return a.pose.y - b.pose.y;
  return a.pose.z - b.pose.z;
}

export function entitiesToRtLayoutScenario(
  entities: UiEntity[],
  options: { layoutId: string },
): RtLayoutScenarioV1 {
  const layoutEntities = entities
    .map(layoutEntityFromUi)
    .filter((row): row is RtLayoutEntity => row !== null)
    .sort(compareLayoutEntities);

  return {
    schema_version: LAYOUT_SCHEMA_VERSION,
    layout_id: options.layoutId,
    terrain_preset: SCENARIO_TERRAIN_PRESET,
    entities: layoutEntities,
    source: {
      authority: "rt_ui_preview",
      origin: "platform/rt-sandbox-ui",
      ui_flow: "scenario_evaluation_panel",
    },
  };
}

export function layoutIdForSession(sessionId: string | null): string {
  if (!sessionId) return "rt_layout_draft";
  const safe = sessionId.replace(/[^a-zA-Z0-9_-]+/g, "_").slice(0, 48);
  return `rt_layout_${safe}`;
}
