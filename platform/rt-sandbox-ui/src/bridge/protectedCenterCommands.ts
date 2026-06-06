import { sendCommand } from "./client";
import type { BridgeCommandResponse } from "./types";

export async function designateProtectedCenter(
  sessionId: string,
  entityId: string,
  replace = false,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "designate_protected_center",
    sessionId,
    payload: {
      entity_id: entityId,
      replace,
    },
  });
}
