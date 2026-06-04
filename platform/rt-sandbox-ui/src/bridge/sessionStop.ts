import { sendCommand, stopSession } from "./client";
import type { BridgeCommandResponse } from "./types";
import { liveStopCommandType } from "@/runtime/liveSessionUx";
import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";

export async function stopSim(sessionId: string): Promise<BridgeCommandResponse> {
  return sendCommand({ commandType: "stop_sim", sessionId });
}

export async function stopSessionForProfile(
  sessionId: string,
  profile: SessionRuntimeProfile,
): Promise<BridgeCommandResponse> {
  if (liveStopCommandType(profile) === "stop_sim") {
    return stopSim(sessionId);
  }
  return stopSession(sessionId);
}
