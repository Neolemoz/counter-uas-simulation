import {
  BRIDGE_COMMAND_URL,
  BRIDGE_PULL_URL,
  TELEMETRY_CHANNELS,
} from "@/telemetry/constants";
import type {
  BridgeCommandResponse,
  ListCaptureHandoffStatusResponse,
  PullTelemetryResponse,
  SendCommandOptions,
} from "./types";

function uuid(): string {
  return crypto.randomUUID();
}

export async function sendCommand(
  options: SendCommandOptions,
): Promise<BridgeCommandResponse> {
  const body = {
    schema: "rt_bridge_request_v1",
    command_type: options.commandType,
    command_id: uuid(),
    issued_by: options.issuedBy ?? "rt_sandbox_ui",
    authority_scope: "rt_sandbox_prototype",
    ...(options.sessionId ? { session_id: options.sessionId } : {}),
    ...(options.payload !== undefined ? { payload: options.payload } : {}),
  };

  const resp = await fetch(BRIDGE_COMMAND_URL, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });

  if (!resp.ok) {
    return {
      ok: false,
      error_code: "HTTP_ERROR",
      message: `HTTP ${resp.status}`,
    };
  }

  return (await resp.json()) as BridgeCommandResponse;
}

export async function startSession(): Promise<BridgeCommandResponse> {
  return sendCommand({ commandType: "start_session" });
}

export async function stopSession(sessionId: string): Promise<BridgeCommandResponse> {
  return sendCommand({ commandType: "stop_session", sessionId });
}

export async function subscribeTelemetry(
  sessionId: string,
  channels: readonly string[] = TELEMETRY_CHANNELS,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "subscribe_telemetry",
    sessionId,
    payload: { channels: [...channels] },
  });
}

export async function unsubscribeTelemetry(
  sessionId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "unsubscribe_telemetry",
    sessionId,
    payload: {},
  });
}

export async function pullTelemetry(options: {
  sessionId: string;
  subscriptionId: string;
  maxEvents?: number;
}): Promise<PullTelemetryResponse> {
  const params = new URLSearchParams({
    session_id: options.sessionId,
    subscription_id: options.subscriptionId,
    max_events: String(options.maxEvents ?? 10),
  });

  const resp = await fetch(`${BRIDGE_PULL_URL}?${params.toString()}`);
  if (!resp.ok) {
    return {
      ok: false,
      error_code: "HTTP_ERROR",
    };
  }
  return (await resp.json()) as PullTelemetryResponse;
}

export async function listSessions(): Promise<BridgeCommandResponse> {
  return sendCommand({ commandType: "list_sessions" });
}

export async function setEditingSession(
  sessionId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "set_editing_session",
    payload: { session_id: sessionId },
  });
}

export async function discardSession(sessionId: string): Promise<BridgeCommandResponse> {
  return sendCommand({ commandType: "discard_session", sessionId });
}

/** Read-only staging mirror (PLAT-RT-SA2) — scoped by session_id. */
export async function listCaptureHandoffStatus(
  sessionId: string,
): Promise<ListCaptureHandoffStatusResponse> {
  return sendCommand({
    commandType: "list_capture_handoff_status",
    payload: { session_id: sessionId },
  }) as Promise<ListCaptureHandoffStatusResponse>;
}

export { DIAGNOSTIC_TELEMETRY_CHANNELS } from "@/telemetry/constants";
