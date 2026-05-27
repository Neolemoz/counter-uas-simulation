import type { AdvisoryChecklistItem, AdvisoryDeriveInput } from "./advisoryTypes";

export const CHECKLIST_IDS = [
  "normalization",
  "validation_doc",
  "pose_cognition",
  "origin",
  "session_state",
  "scenario_pack",
  "export_audit",
  "lineage",
] as const;

export type ChecklistId = (typeof CHECKLIST_IDS)[number];

export const CHECKLIST_LABELS: Record<ChecklistId, string> = {
  normalization: "Normalization",
  validation_doc: "Validation doc",
  pose_cognition: "Pose cognition",
  origin: "Origin",
  session_state: "Session state",
  scenario_pack: "Scenario pack",
  export_audit: "Export audit",
  lineage: "Lineage",
};

const REQUIRED_EXPORT_EVENTS = [
  "capture_validated",
  "capture_normalized",
  "export_pose_normalized",
] as const;

function eventTypes(exportEvents: AdvisoryDeriveInput["export_events"]): string[] {
  if (!exportEvents) return [];
  return exportEvents
    .map((ev) => (typeof ev === "string" ? ev : (ev.event_type ?? "")))
    .filter(Boolean);
}

export function buildAdvisoryChecklist(inp: AdvisoryDeriveInput): AdvisoryChecklistItem[] {
  const candidate = inp.candidate ?? {};
  const normVal = inp.normalization_validation ?? {};
  const review = inp.handoff_review ?? {};
  const events = eventTypes(inp.export_events);
  const checklist: AdvisoryChecklistItem[] = [];

  if (candidate.normalization_status === "normalized") {
    checklist.push({ id: "normalization", status: "pass" });
  } else if (candidate && Object.keys(candidate).length > 0) {
    checklist.push({ id: "normalization", status: "fail" });
  } else {
    checklist.push({ id: "normalization", status: "unknown" });
  }

  if (normVal.valid === true) {
    checklist.push({ id: "validation_doc", status: "pass" });
  } else if (normVal && Object.keys(normVal).length > 0) {
    checklist.push({ id: "validation_doc", status: "fail" });
  } else {
    checklist.push({ id: "validation_doc", status: "unknown" });
  }

  const notes = typeof review.notes === "string" ? review.notes : "";
  if (inp.pose_attested || notes.toLowerCase().includes("pose")) {
    checklist.push({ id: "pose_cognition", status: "pass" });
  } else {
    checklist.push({
      id: "pose_cognition",
      status: "warn",
      detail: "maintainer attestation required",
    });
  }

  const origin =
    typeof candidate.origin === "string"
      ? candidate.origin
      : typeof inp.origin === "string"
        ? inp.origin
        : "";
  if (origin.includes("rt_sandbox_capture_v1")) {
    checklist.push({ id: "origin", status: "pass" });
  } else if (origin) {
    checklist.push({ id: "origin", status: "fail", detail: "unexpected origin" });
  } else {
    checklist.push({ id: "origin", status: "unknown" });
  }

  const lifecycle = inp.session_lifecycle_state?.toLowerCase() ?? "";
  if (!lifecycle) {
    checklist.push({ id: "session_state", status: "unknown" });
  } else if (lifecycle === "failed" || lifecycle === "discarded") {
    checklist.push({ id: "session_state", status: "fail", detail: lifecycle });
  } else {
    checklist.push({ id: "session_state", status: "pass" });
  }

  const packRef =
    inp.scenario_pack_ref ??
    (typeof candidate.scenario_pack_ref === "string"
      ? candidate.scenario_pack_ref
      : null);
  if (!packRef) {
    checklist.push({ id: "scenario_pack", status: "pass" });
  } else {
    checklist.push({
      id: "scenario_pack",
      status: "warn",
      detail: "pack ref present — validate via maintainer CLI",
    });
  }

  const missingExport = REQUIRED_EXPORT_EVENTS.filter((e) => !events.includes(e));
  if (events.length === 0) {
    checklist.push({ id: "export_audit", status: "unknown" });
  } else if (missingExport.length === 0) {
    checklist.push({ id: "export_audit", status: "pass" });
  } else if (
    events.includes("capture_normalized") ||
    inp.workflow_phase === "normalized" ||
    inp.workflow_phase === "ready"
  ) {
    checklist.push({
      id: "export_audit",
      status: "warn",
      detail: `missing: ${missingExport.join(", ")}`,
    });
  } else {
    checklist.push({
      id: "export_audit",
      status: "fail",
      detail: `missing: ${missingExport.join(", ")}`,
    });
  }

  const lineageErrors = inp.lineage_lint_errors;
  if (lineageErrors && lineageErrors.length > 0) {
    checklist.push({ id: "lineage", status: "fail", detail: lineageErrors[0] });
  } else {
    checklist.push({ id: "lineage", status: "pass" });
  }

  return checklist;
}
