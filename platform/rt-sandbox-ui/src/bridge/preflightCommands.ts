import { sendCommand } from "./client";
import type { BridgeCommandResponse } from "./types";
import {
  parseLivePreflight,
  type LivePreflightResult,
} from "@/runtime/livePreflight";

export type LivePreflightResponse = BridgeCommandResponse & {
  preflight?: LivePreflightResult;
};

export async function checkLiveRuntimePreflight(): Promise<LivePreflightResponse> {
  const resp = (await sendCommand({
    commandType: "check_live_runtime_preflight",
  })) as LivePreflightResponse & { preflight?: unknown };
  const parsed = parseLivePreflight(resp.preflight);
  if (parsed) {
    return { ...resp, preflight: parsed };
  }
  return resp;
}
