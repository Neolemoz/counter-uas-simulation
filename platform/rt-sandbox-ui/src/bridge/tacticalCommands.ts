import { sendCommand } from "./client";
import type { BridgeCommandResponse } from "./types";

export type TacticalMode = "manual" | "assisted" | "autonomous";

export type TacticalStatePayload = {
  schema?: string;
  tactical_mode?: string;
  selected_interceptor_id?: string | null;
  selected_target_id?: string | null;
  assigned_interceptor_id?: string | null;
  assigned_target_id?: string | null;
  pending_recommendation_id?: string | null;
  autonomous_loop_status?: "running" | "paused" | null;
  assignment_lock_active?: boolean;
  tti_s?: number | null;
  /** Explanatory time-to-solution along predicted path (seconds). */
  eta_s?: number | null;
  interceptor_speed_cap_m_s?: number | null;
  tactical_health?: {
    feasible?: boolean;
    summary?: string;
    stale?: boolean;
  };
  last_intercept_pose?: Record<string, number> | null;
  authority_label?: string;
};

export type TacticalRecommendationPayload = {
  schema?: string;
  recommendation_id?: string | null;
  recommended_interceptor_id?: string | null;
  recommended_target_id?: string | null;
  candidate_id?: string | null;
  tti_s?: number | null;
  feasibility?: { feasible?: boolean; reason?: string };
  explanation?: string;
  expires_at_utc?: string;
  tactical_health?: {
    feasible?: boolean;
    summary?: string;
    stale?: boolean;
  };
  authority_label?: string;
  governance_banner?: string;
};

export function parseTacticalState(
  response: BridgeCommandResponse,
): TacticalStatePayload | null {
  const raw = response.tactical_state;
  if (!raw || typeof raw !== "object") return null;
  return raw as TacticalStatePayload;
}

export function parseTacticalRecommendation(
  response: BridgeCommandResponse,
): TacticalRecommendationPayload | null {
  const raw = response.tactical_recommendation;
  if (!raw || typeof raw !== "object") return null;
  return raw as TacticalRecommendationPayload;
}

export async function setTacticalMode(
  sessionId: string,
  mode: TacticalMode,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "set_tactical_mode",
    sessionId,
    payload: { mode },
  });
}

export async function selectCandidate(
  sessionId: string,
  role: "interceptor" | "target",
  entityId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "select_candidate",
    sessionId,
    payload: { role, entity_id: entityId },
  });
}

export async function assignCandidate(
  sessionId: string,
  payload?: { interceptor_id?: string; target_id?: string },
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "assign_candidate",
    sessionId,
    payload: payload ?? {},
  });
}

export async function clearAssignment(
  sessionId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "clear_assignment",
    sessionId,
    payload: {},
  });
}

export async function getTacticalState(
  sessionId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "get_tactical_state",
    sessionId,
    payload: {},
  });
}

export async function requestRecommendation(
  sessionId: string,
  payload?: { interceptor_id?: string; target_id?: string },
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "request_recommendation",
    sessionId,
    payload: payload ?? {},
  });
}

export async function approveRecommendation(
  sessionId: string,
  recommendationId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "approve_recommendation",
    sessionId,
    payload: { recommendation_id: recommendationId },
  });
}

export async function rejectRecommendation(
  sessionId: string,
  recommendationId?: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "reject_recommendation",
    sessionId,
    payload: recommendationId
      ? { recommendation_id: recommendationId }
      : {},
  });
}

export async function pauseAutonomousLoop(
  sessionId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "pause_autonomous_loop",
    sessionId,
    payload: {},
  });
}

export async function resumeAutonomousLoop(
  sessionId: string,
): Promise<BridgeCommandResponse> {
  return sendCommand({
    commandType: "resume_autonomous_loop",
    sessionId,
    payload: {},
  });
}
