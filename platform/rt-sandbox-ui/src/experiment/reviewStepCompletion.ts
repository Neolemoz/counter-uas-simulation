import type { UnifiedReviewStepId } from "./experimentUnifiedReview";
import type { ExperimentManifest } from "./experimentSchema";
import type { ReportDockPresence } from "./reviewPacketPreview";
import type { WorkbenchV2State } from "./workbenchV2State";

export type ReviewStepCompletionState =
  | "complete"
  | "imported"
  | "derived"
  | "missing"
  | "opened"
  | "skipped"
  | "n/a"
  | "active"
  | "draft"
  | "exported";

export function formatStepCompletionLabel(state: ReviewStepCompletionState): string {
  return state;
}

export function resolveStepCompletion(
  stepId: UnifiedReviewStepId,
  options: {
    v2State: WorkbenchV2State;
    manifest: ExperimentManifest;
    presence: ReportDockPresence;
    packetTabEverFocused?: boolean;
    packetDownloaded?: boolean;
  },
): ReviewStepCompletionState {
  const { v2State, manifest, presence, packetTabEverFocused, packetDownloaded } = options;
  const inCompleted = v2State.steps_completed.includes(stepId);

  switch (stepId) {
    case "select_scope":
      if (v2State.primary_manifest_ref || manifest.runs.length > 0) return "complete";
      return "missing";
    case "f1_analytics":
      if (presence.f1_analytics) return "imported";
      return "missing";
    case "f3_continuity":
      if (inCompleted || v2State.active_run_id) return "opened";
      if (v2State.review_step !== "f3_continuity" && stepPast(v2State, "f3_continuity")) {
        return "skipped";
      }
      return "missing";
    case "f5_metrics":
      if (presence.f5_metrics) return "imported";
      return "missing";
    case "f5b_fidelity":
      if (presence.f5b_fidelity) return "imported";
      if (!presence.f5_metrics && !presence.f5b_fidelity) return "n/a";
      return "missing";
    case "compare":
      if (v2State.review_step === "compare") return "active";
      if (inCompleted) return "complete";
      return "missing";
    case "export_packet":
      if (packetDownloaded) return "exported";
      if (packetTabEverFocused || inCompleted) return "draft";
      return "draft";
    default:
      return "missing";
  }
}

function stepPast(v2State: WorkbenchV2State, stepId: UnifiedReviewStepId): boolean {
  const ids = [
    "select_scope",
    "f1_analytics",
    "f3_continuity",
    "f5_metrics",
    "f5b_fidelity",
    "compare",
    "export_packet",
  ] as const;
  const current = ids.indexOf(v2State.review_step);
  const target = ids.indexOf(stepId);
  return current > target;
}

export function buildStepCompletionMap(options: {
  v2State: WorkbenchV2State;
  manifest: ExperimentManifest;
  presence: ReportDockPresence;
  packetTabEverFocused?: boolean;
  packetDownloaded?: boolean;
}): Record<UnifiedReviewStepId, ReviewStepCompletionState> {
  const steps = [
    "select_scope",
    "f1_analytics",
    "f3_continuity",
    "f5_metrics",
    "f5b_fidelity",
    "compare",
    "export_packet",
  ] as const;
  const out = {} as Record<UnifiedReviewStepId, ReviewStepCompletionState>;
  for (const id of steps) {
    out[id] = resolveStepCompletion(id, options);
  }
  return out;
}

export function slotIdToReviewStep(
  slotId: string,
): UnifiedReviewStepId | null {
  switch (slotId) {
    case "f1_analytics":
      return "f1_analytics";
    case "f3_annex":
      return "f3_continuity";
    case "f5_metrics":
      return "f5_metrics";
    case "f5b_fidelity":
      return "f5b_fidelity";
    default:
      return null;
  }
}
