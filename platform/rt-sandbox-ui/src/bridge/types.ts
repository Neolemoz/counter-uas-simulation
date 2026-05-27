export interface BridgeCommandResponse {
  ok: boolean;
  error_code?: string;
  message?: string;
  session_id?: string;
  subscription_id?: string;
  channels?: string[];
  initial_events?: unknown[];
  governance_banner?: string;
  [key: string]: unknown;
}

export interface PullTelemetryResponse {
  ok: boolean;
  error_code?: string;
  session_id?: string;
  subscription_id?: string;
  events?: Array<{
    channel: string;
    session_id: string;
    timestamp_utc: string;
    payload: Record<string, unknown>;
    governance_banner?: string;
  }>;
  drained_count?: number;
  governance_banner?: string;
}

export interface SendCommandOptions {
  commandType: string;
  sessionId?: string;
  payload?: Record<string, unknown>;
  issuedBy?: string;
}

/** Read-only handoff mirror row (rt_capture_handoff_row_v1). */
export type WorkflowPhase =
  | "none"
  | "staged"
  | "normalized"
  | "review_pending"
  | "ready"
  | "deferred"
  | "rejected"
  | "prepared"
  | "committed";

export interface CaptureHandoffRow {
  schema: string;
  capture_candidate_id: string;
  approval_status: string;
  normalization_status: string;
  handoff_decision?: string;
  workflow_phase: WorkflowPhase;
  validation_ok: boolean;
  validation_errors?: string[] | null;
  has_handoff_manifest: boolean;
  has_import_record: boolean;
  last_export_event_type?: string | null;
  source_origin: string;
  lineage_note: string;
  governance_banner: string;
}

export interface ListCaptureHandoffStatusResponse extends BridgeCommandResponse {
  captures?: CaptureHandoffRow[];
  session_id?: string;
}
