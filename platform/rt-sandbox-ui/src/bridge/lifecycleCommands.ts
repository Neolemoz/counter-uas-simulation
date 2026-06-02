import { sendCommand } from "./client";
import type { BridgeCommandResponse } from "./types";

export async function resetSession(sessionId: string): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "reset_session",
    sessionId,
    payload: {},
  });
}
