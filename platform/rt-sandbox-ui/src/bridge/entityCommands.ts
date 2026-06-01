import { sendCommand } from "./client";
import type { BridgeCommandResponse } from "./types";
import type { EntityType } from "@/world/entityCatalog";
import type { Pose } from "@/world/bounds";

export async function spawnEntity(
  sessionId: string,
  payload: { entity_type: EntityType; pose: Pose },
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "spawn_entity",
    sessionId,
    payload: {
      entity_type: payload.entity_type,
      pose: payload.pose,
    },
  });
}

export async function spawnAttacker(
  sessionId: string,
  payload: { pose: Pose },
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "spawn_attacker",
    sessionId,
    payload: { pose: payload.pose },
  });
}

export async function moveEntity(
  sessionId: string,
  payload: { entity_id: string; pose: Pose },
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "move_entity",
    sessionId,
    payload: {
      entity_id: payload.entity_id,
      pose: payload.pose,
    },
  });
}

export async function deleteEntity(
  sessionId: string,
  entityId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "delete_entity",
    sessionId,
    payload: { entity_id: entityId },
  });
}
