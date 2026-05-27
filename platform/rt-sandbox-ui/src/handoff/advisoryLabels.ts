import { CHECKLIST_LABELS, type ChecklistId } from "./advisoryChecklist";
import type { AdvisoryState, ChecklistStatus } from "./advisoryTypes";

export { CHECKLIST_LABELS };

export function checklistStatusIcon(status: ChecklistStatus): string {
  switch (status) {
    case "pass":
      return "✓";
    case "fail":
      return "✗";
    case "warn":
      return "!";
    default:
      return "?";
  }
}

export function checklistChipLabel(id: string, status: ChecklistStatus): string {
  const label = CHECKLIST_LABELS[id as ChecklistId] ?? id;
  return `${label}: ${status}`;
}

export function advisoryStateLabel(state: AdvisoryState | null): string {
  switch (state) {
    case "capture_ready":
      return "Capture ready (advisory)";
    case "review_complete":
      return "Review complete (advisory)";
    case "approval_ready":
      return "Approval ready (advisory)";
    case "handoff_ready":
      return "Advisory: handoff packaging ready";
    case "import_ready":
      return "Import ready (advisory)";
    case "blocked":
      return "Blocked (advisory)";
    default:
      return "Not ready (advisory)";
  }
}

export type AdvisoryTone = "ok" | "warn" | "error" | "neutral";

export function advisoryStateTone(
  state: AdvisoryState | null,
  blocked: boolean,
): AdvisoryTone {
  if (blocked || state === "blocked") return "error";
  switch (state) {
    case "import_ready":
    case "handoff_ready":
    case "approval_ready":
      return "ok";
    case "review_complete":
    case "capture_ready":
      return "neutral";
    default:
      return "warn";
  }
}
