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

export type BlockerGroupId =
  | "normalization"
  | "review_attestation"
  | "approval_gate"
  | "packaging"
  | "lineage"
  | "experiment_warn"
  | "terminal_block";

export type ReadinessCohortId =
  | "needs_normalize"
  | "needs_review"
  | "needs_approve"
  | "needs_prepare"
  | "ready_for_commit_advisory"
  | "blocked"
  | "terminal"
  | "error";

export type QueueBandId =
  | "P0_block"
  | "P1_error"
  | "P2_normalize"
  | "P3_review"
  | "P4_approve"
  | "P5_package"
  | "P6_import"
  | "P7_terminal";

export interface QueuePriority {
  rank: number;
  band: QueueBandId | string;
  rationale: string;
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
  lineage_warnings?: string[];
}

export type ReadinessCohortV2Id =
  | ReadinessCohortId
  | "stale_review"
  | "stale_approve"
  | "multi_blocker"
  | "experiment_handoff_warn"
  | "focus_highlight";

export type FilterPresetId =
  | "blocked_today"
  | "defer_queue"
  | "import_advisory_only"
  | "review_backlog"
  | "normalize_failures"
  | "experiment_warn_only"
  | "lineage_review"
  | "all_staged";

export type TemplatePackId = "standup_json_v2" | "standup_md_daily" | "standup_md_minimal";

export interface EnrichedAdvisoryRow {
  capture_candidate_id: string;
  status: AdvisoryStatus;
  queue_priority: QueuePriority;
  blocker_groups: BlockerGroupId[];
  readiness_cohort: ReadinessCohortId;
  readiness_cohort_v2?: ReadinessCohortV2Id;
  stale_age_hours?: number | null;
  in_focus_set?: boolean;
}

export interface HandoffRollupByStage {
  normalize?: number;
  review?: number;
  approve?: number;
  prepare?: number;
  import_advisory?: number;
}

export interface HandoffRollup {
  by_stage: HandoffRollupByStage;
  blocked_count: number;
  terminal_count: number;
}

export interface MultiCaptureCohorts {
  by_primary_lane: Partial<Record<string, number>>;
  stale_age_warn_count: number;
  note?: string;
}

export interface ExperimentHandoffRollup extends AdvisoryExperimentRollup {
  manifest_ref?: string;
  cohort_index_ref?: string;
  cohort_status?: string | null;
  review_packet_paths?: string[];
}

export interface SessionAdvisorySummary {
  readiness_cohorts: Partial<Record<ReadinessCohortId, number>>;
  blocker_groups: Partial<Record<BlockerGroupId, number>>;
  total: number;
  readiness_cohorts_v2?: Partial<Record<ReadinessCohortV2Id, number>>;
  multi_capture_cohorts?: MultiCaptureCohorts;
  handoff_rollup?: HandoffRollup;
  experiment_handoff_rollup?: ExperimentHandoffRollup | null;
  schema_version?: "f7" | "f8";
}

export type TriageGroupMode =
  | "queue_band"
  | "blocker"
  | "cohort"
  | "cohort_v2"
  | "handoff_stage"
  | "experiment";

export type TriageSortMode = "queue" | "capture_id";

export interface AdvisoryExperimentRollup {
  handoff_eligibility: string;
  warn_capture_ids: string[];
  note?: string;
}

export interface TriageRowGroup {
  key: string;
  label: string;
  rows: EnrichedAdvisoryRow[];
}

export const ADVISORY_GOVERNANCE_BANNER =
  "SA WORKFLOW ADVISORY — explanatory; maintainer CLIs are authority";
