import type { CaptureHandoffRow } from "@/bridge/types";
import type { AdvisoryDeriveInput } from "./advisoryTypes";

export function advisoryInputFromMirrorRow(
  row: CaptureHandoffRow,
  sessionLifecycleState?: string | null,
  options?: { poseAttested?: boolean },
): AdvisoryDeriveInput {
  const events: string[] = [];
  if (row.last_export_event_type) {
    events.push(row.last_export_event_type);
  }

  const input: AdvisoryDeriveInput = {
    capture_candidate_id: row.capture_candidate_id,
    session_lifecycle_state: sessionLifecycleState ?? null,
    pose_attested: options?.poseAttested,
    origin: row.source_origin,
    candidate: {
      normalization_status: row.normalization_status,
      approval_status: row.approval_status,
      origin: row.source_origin,
    },
    export_events: events,
    workflow_phase: row.workflow_phase,
    conversion_manifest_present: row.approval_status === "approved",
    handoff_manifest_present: row.has_handoff_manifest,
    import_record_present: row.has_import_record,
    conversion_steps_advisory_pass: row.has_handoff_manifest ? true : null,
    lineage_lint_errors: [],
    handoff_blocked:
      row.workflow_phase === "rejected" ||
      row.workflow_phase === "deferred" ||
      row.handoff_decision === "rejected" ||
      row.handoff_decision === "deferred",
  };

  if (row.handoff_decision) {
    input.handoff_review = { decision: row.handoff_decision };
  }

  if (row.validation_ok) {
    input.normalization_validation = { valid: true };
    input.handoff_preconditions_errors = [];
  } else if (row.validation_errors?.length) {
    input.handoff_preconditions_errors = row.validation_errors;
  }

  if (row.workflow_phase === "ready" && row.approval_status === "pending") {
    events.push("handoff_reviewed");
    input.export_events = [...new Set(events)];
  }

  if (row.approval_status === "approved") {
    events.push("capture_approved", "conversion_manifest_written");
    input.export_events = [...new Set(events)];
    input.conversion_manifest_present = true;
  }

  if (row.has_handoff_manifest) {
    events.push("handoff_import_prepared");
    input.export_events = [...new Set(events)];
  }

  if (row.has_import_record) {
    events.push("handoff_import_committed");
    input.export_events = [...new Set(events)];
  }

  return input;
}
