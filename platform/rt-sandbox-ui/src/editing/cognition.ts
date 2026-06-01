import { ENTITY_TYPE_LIMITS, MAX_ENTITY_COUNT, WORLD_BOUNDS } from "@/world/bounds";
import type { EditCommandType } from "./editHistory";

export function describeCommandIntent(commandType: EditCommandType): string {
  const labels: Record<EditCommandType, string> = {
    spawn_entity: "Registry spawn — command authoritative",
    move_entity: "Registry reposition — command authoritative",
    delete_entity: "Registry delete — command authoritative",
    apply_scenario:
      "Apply scenario — reset runtime world and spawn current layout",
  };
  return labels[commandType];
}

export function describeMirrorLag(pending: boolean): string {
  if (pending) {
    return "Awaiting telemetry pull — explanatory mirror may lag behind registry command";
  }
  return "Mirror reconciled via pull — explanatory telemetry only; not replay authority";
}

export function editSafetySummary(
  worldSummary: Record<string, unknown> | undefined,
): {
  entityCount: string;
  byType: Record<string, string>;
  bounds: string;
} {
  const count = Number(worldSummary?.entity_count ?? 0);
  const byTypeRaw = (worldSummary?.by_type as Record<string, number>) ?? {};
  const byType: Record<string, string> = {};
  for (const [type, limit] of Object.entries(ENTITY_TYPE_LIMITS)) {
    byType[type] = `${Number(byTypeRaw[type] ?? 0)}/${limit}`;
  }
  return {
    entityCount: `${count}/${MAX_ENTITY_COUNT}`,
    byType,
    bounds: `x/y [${WORLD_BOUNDS.x.min}, ${WORLD_BOUNDS.x.max}], z [${WORLD_BOUNDS.z.min}, ${WORLD_BOUNDS.z.max}]`,
  };
}

export function formatCommandResult(
  ok: boolean,
  errorCode?: string,
  message?: string,
): string {
  if (ok) return "Command accepted by bridge registry";
  return `Command rejected: ${errorCode ?? "UNKNOWN"}${message ? ` — ${message}` : ""}`;
}
