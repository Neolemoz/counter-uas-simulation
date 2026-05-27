import type { CaptureHandoffRow, WorkflowPhase } from "@/bridge/types";

export type HandoffChipTone = "ok" | "warn" | "error" | "neutral";

export interface HandoffChip {
  label: string;
  tone: HandoffChipTone;
}

const PHASE_PRIORITY: Record<WorkflowPhase, number> = {
  committed: 90,
  rejected: 80,
  deferred: 70,
  ready: 60,
  prepared: 50,
  review_pending: 40,
  normalized: 30,
  staged: 20,
  none: 0,
};

export function highestWorkflowPhase(rows: CaptureHandoffRow[]): WorkflowPhase {
  if (rows.length === 0) return "none";
  return rows.reduce(
    (best, row) =>
      PHASE_PRIORITY[row.workflow_phase] > PHASE_PRIORITY[best]
        ? row.workflow_phase
        : best,
    "none" as WorkflowPhase,
  );
}

export function handoffChipsForRow(row: CaptureHandoffRow): HandoffChip[] {
  const chips: HandoffChip[] = [];
  if (row.last_export_event_type) {
    chips.push({ label: row.last_export_event_type, tone: "neutral" });
  }
  switch (row.workflow_phase) {
    case "ready":
      chips.push({ label: "ready for maintainer import", tone: "ok" });
      break;
    case "committed":
      chips.push({ label: "committed to corpus", tone: "ok" });
      break;
    case "deferred":
      chips.push({ label: "deferred", tone: "warn" });
      break;
    case "rejected":
      chips.push({ label: "rejected", tone: "error" });
      break;
    case "prepared":
      chips.push({ label: "handoff prepared", tone: "neutral" });
      break;
    default:
      break;
  }
  if (!row.validation_ok && row.validation_errors?.length) {
    chips.push({ label: "validation issues", tone: "warn" });
  }
  return chips;
}

export function phaseLabel(phase: WorkflowPhase): string {
  return phase.replace(/_/g, " ");
}

export function phaseTone(phase: WorkflowPhase): HandoffChipTone {
  switch (phase) {
    case "ready":
    case "committed":
      return "ok";
    case "deferred":
    case "review_pending":
      return "warn";
    case "rejected":
      return "error";
    default:
      return "neutral";
  }
}
