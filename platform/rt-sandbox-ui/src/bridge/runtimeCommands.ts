import { sendCommand } from "./client";
import type { BridgeCommandResponse } from "./types";
import type { Pose } from "@/world/bounds";

export async function pauseSim(sessionId: string): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "pause_sim",
    sessionId,
    payload: {},
  });
}

export async function resumeSim(sessionId: string): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "resume_sim",
    sessionId,
    payload: {},
  });
}

export async function spawnDefender(
  sessionId: string,
  payload: { pose?: Pose } = {},
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "spawn_defender",
    sessionId,
    payload: payload.pose ? { pose: payload.pose } : {},
  });
}

export async function assignTarget(
  sessionId: string,
  payload: { defender_id: string; target_id: string },
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "assign_target",
    sessionId,
    payload: {
      defender_id: payload.defender_id,
      target_id: payload.target_id,
    },
  });
}

export async function cancelAssignment(
  sessionId: string,
  payload: { defender_id: string },
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "cancel_assignment",
    sessionId,
    payload: { defender_id: payload.defender_id },
  });
}
