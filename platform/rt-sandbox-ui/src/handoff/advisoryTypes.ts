export type AdvisoryState =
  | "capture_ready"
  | "review_complete"
  | "approval_ready"
  | "handoff_ready"
  | "import_ready"
  | "blocked";

export type ChecklistStatus = "pass" | "fail" | "warn" | "unknown";

export interface AdvisoryChecklistItem {
  id: string;
  status: ChecklistStatus;
  detail?: string;
}

export interface AdvisoryDeriveInput {
  capture_candidate_id?: string;
  candidate?: Record<string, unknown> | null;
  handoff_review?: Record<string, unknown> | null;
  export_events?: string[] | Array<{ event_type?: string }>;
  normalization_validation?: { valid?: boolean } | null;
  handoff_preconditions_errors?: string[] | null;
  handoff_blocked?: boolean;
  conversion_manifest_present?: boolean;
  handoff_manifest_present?: boolean;
  handoff_manifest?: unknown;
  import_record_present?: boolean;
  conversion_steps_advisory_pass?: boolean | null;
  lineage_lint_errors?: string[] | null;
  workflow_phase?: string | null;
  session_lifecycle_state?: string | null;
  scenario_pack_ref?: string | null;
  origin?: string | null;
  pose_attested?: boolean;
}

export interface AdvisoryStatus {
  schema: "rt_sa_workflow_advisory_status_v1";
  capture_candidate_id: string;
  advisory_state: AdvisoryState | null;
  advisory_state_label: string;
  blocked: boolean;
  block_reasons: string[];
  checklist?: AdvisoryChecklistItem[];
  upstream: {
    workflow_phase?: string | null;
    last_export_event?: string | null;
    approval_status?: string;
  };
  governance_banner: string;
  terminal?: string;
}

export const ADVISORY_GOVERNANCE_BANNER =
  "SA WORKFLOW ADVISORY — explanatory; maintainer CLIs are authority";
