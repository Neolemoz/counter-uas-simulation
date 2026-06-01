import { sendCommand } from "./client";
import type { BridgeCommandResponse } from "./types";
import type { ApplyScenarioPayload } from "@/world/scenarioPayload";

export async function applyScenario(
  sessionId: string,
  payload: ApplyScenarioPayload,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "apply_scenario",
    sessionId,
    payload,
  });
}
