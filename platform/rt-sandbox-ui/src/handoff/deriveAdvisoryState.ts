import { buildAdvisoryChecklist } from "./advisoryChecklist";
import { detectLineageWarnings } from "./advisoryQueue";
import {
  ADVISORY_GOVERNANCE_BANNER,
  type AdvisoryDeriveInput,
  type AdvisoryState,
  type AdvisoryStatus,
} from "./advisoryTypes";
import { advisoryStateLabel } from "./advisoryLabels";

function attachLineageWarnings(
  status: AdvisoryStatus,
  inp: AdvisoryDeriveInput,
): AdvisoryStatus {
  const warnings = detectLineageWarnings(inp.candidate);
  if (!warnings.length) return status;
  return { ...status, lineage_warnings: warnings };
}

function eventTypes(exportEvents: AdvisoryDeriveInput["export_events"]): string[] {
  if (!exportEvents) return [];
  return exportEvents.map((ev) =>
    typeof ev === "string" ? ev : (ev.event_type ?? ""),
  ).filter(Boolean);
}

function hasEvent(events: string[], name: string): boolean {
  return events.includes(name);
}

function blockReasons(inp: AdvisoryDeriveInput, events: string[]): string[] {
  const reasons: string[] = [];
  const review = inp.handoff_review ?? {};
  const decision = review.decision;

  if (hasEvent(events, "handoff_rejected") || decision === "rejected") {
    reasons.push("handoff_rejected");
  }
  if (hasEvent(events, "handoff_import_deferred") || decision === "deferred") {
    reasons.push("handoff_import_deferred");
  }
  if (inp.handoff_blocked) {
    if (!reasons.includes("handoff_rejected") && decision === "rejected") {
      reasons.push("handoff_rejected");
    }
    if (!reasons.includes("handoff_import_deferred") && decision === "deferred") {
      reasons.push("handoff_import_deferred");
    }
  }

  const preErrors = inp.handoff_preconditions_errors;
  if (Array.isArray(preErrors) && preErrors.length > 0 && inp.handoff_blocked) {
    reasons.push(...preErrors.slice(0, 4));
  }

  const lineage = inp.lineage_lint_errors;
  if (Array.isArray(lineage) && lineage.length > 0) {
    reasons.push(...lineage.slice(0, 2).map((e) => `lineage:${e}`));
  }

  return reasons;
}

function isCaptureReady(inp: AdvisoryDeriveInput, events: string[]): boolean {
  const candidate = inp.candidate ?? {};
  const workflow = inp.workflow_phase;
  let normOk = false;

  if (candidate.normalization_status === "normalized") {
    normOk = true;
  } else if (workflow === "normalized" && hasEvent(events, "capture_normalized")) {
    normOk = true;
  } else if (hasEvent(events, "capture_normalized")) {
    normOk = true;
  } else {
    return false;
  }

  const normVal = inp.normalization_validation;
  if (normVal?.valid === false) return false;
  if (!candidate && !hasEvent(events, "capture_normalized")) return false;
  return normOk;
}

function isReviewComplete(events: string[], inp: AdvisoryDeriveInput): boolean {
  if (!hasEvent(events, "handoff_reviewed")) return false;
  const review = inp.handoff_review;
  return typeof review === "object" && review !== null && Object.keys(review).length > 0;
}

function isApprovalReady(inp: AdvisoryDeriveInput, events: string[]): boolean {
  const candidate = inp.candidate ?? {};
  if (candidate.approval_status === "approved") return false;
  if (!hasEvent(events, "handoff_reviewed")) return false;
  if (inp.handoff_review && Object.keys(inp.handoff_review).length > 0) return false;

  const preErrors = inp.handoff_preconditions_errors;
  if (preErrors?.length === 0) return true;

  const workflow = inp.workflow_phase;
  if (workflow === "ready" && candidate.normalization_status === "normalized") {
    return true;
  }
  if (preErrors === null || preErrors === undefined) {
    return workflow === "ready";
  }
  return false;
}

function isHandoffReady(inp: AdvisoryDeriveInput, events: string[]): boolean {
  const candidate = inp.candidate ?? {};
  if (candidate.approval_status !== "approved") return false;
  if (
    !inp.conversion_manifest_present &&
    !hasEvent(events, "conversion_manifest_written")
  ) {
    return false;
  }
  return (
    hasEvent(events, "capture_approved") || candidate.approval_status === "approved"
  );
}

function isImportReady(inp: AdvisoryDeriveInput, events: string[]): boolean {
  if (!hasEvent(events, "handoff_import_prepared")) return false;
  if (!inp.handoff_manifest_present && !inp.handoff_manifest) return false;
  if (inp.conversion_steps_advisory_pass === false) return false;
  if (inp.lineage_lint_errors?.length) return false;
  return true;
}

export function deriveAdvisoryState(inp: AdvisoryDeriveInput): AdvisoryStatus {
  const captureId = inp.capture_candidate_id ?? "unknown";
  const events = eventTypes(inp.export_events);
  const candidate = inp.candidate ?? {};
  const approvalStatus =
    typeof candidate.approval_status === "string"
      ? candidate.approval_status
      : "pending";

  const base: AdvisoryStatus = {
    schema: "rt_sa_workflow_advisory_status_v1",
    capture_candidate_id: captureId,
    advisory_state: null,
    advisory_state_label: advisoryStateLabel(null),
    blocked: false,
    block_reasons: [],
    checklist: buildAdvisoryChecklist(inp),
    upstream: {
      workflow_phase: inp.workflow_phase,
      last_export_event: events.length ? events[events.length - 1] : null,
      approval_status: approvalStatus,
    },
    governance_banner: ADVISORY_GOVERNANCE_BANNER,
  };

  if (
    inp.import_record_present ||
    hasEvent(events, "handoff_import_committed")
  ) {
    base.advisory_state_label = "Committed — SA lineage active";
    base.terminal = "handoff_import_committed";
    return attachLineageWarnings(base, inp);
  }

  const reasons = blockReasons(inp, events);
  if (reasons.length > 0) {
    base.advisory_state = "blocked";
    base.advisory_state_label = advisoryStateLabel("blocked");
    base.blocked = true;
    base.block_reasons = reasons;
    return attachLineageWarnings(base, inp);
  }

  let state: AdvisoryState | null = null;
  if (isImportReady(inp, events)) state = "import_ready";
  else if (isHandoffReady(inp, events)) state = "handoff_ready";
  else if (isApprovalReady(inp, events)) state = "approval_ready";
  else if (isReviewComplete(events, inp)) state = "review_complete";
  else if (isCaptureReady(inp, events)) state = "capture_ready";

  base.advisory_state = state;
  base.advisory_state_label = advisoryStateLabel(state);
  return attachLineageWarnings(base, inp);
}
