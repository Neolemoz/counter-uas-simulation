import type { UnifiedReviewStepId } from "./experimentUnifiedReview";
import type { ReportDockSlotId } from "./experimentUnifiedReview";

export type ReportDockGroupId = "analytics" | "continuity" | "metrics" | "fidelity";

export const REPORT_DOCK_GROUPS: ReadonlyArray<{
  id: ReportDockGroupId;
  label: string;
  slotIds: readonly ReportDockSlotId[];
  reviewStep: UnifiedReviewStepId;
}> = [
  { id: "analytics", label: "Analytics", slotIds: ["f1_analytics"], reviewStep: "f1_analytics" },
  { id: "continuity", label: "Continuity", slotIds: ["f3_annex"], reviewStep: "f3_continuity" },
  { id: "metrics", label: "Metrics", slotIds: ["f5_metrics"], reviewStep: "f5_metrics" },
  { id: "fidelity", label: "Fidelity", slotIds: ["f5b_fidelity"], reviewStep: "f5b_fidelity" },
];

export function defaultGroupExpanded(
  groupId: ReportDockGroupId,
  reviewStep: UnifiedReviewStepId,
): boolean {
  switch (groupId) {
    case "analytics":
      return reviewStep === "f1_analytics";
    case "continuity":
      return reviewStep === "f3_continuity";
    case "metrics":
      return reviewStep === "f5_metrics";
    case "fidelity":
      return reviewStep === "f5b_fidelity";
    default:
      return false;
  }
}

export function defaultGroupExpandedMap(
  reviewStep: UnifiedReviewStepId,
): Record<ReportDockGroupId, boolean> {
  const out = {} as Record<ReportDockGroupId, boolean>;
  for (const g of REPORT_DOCK_GROUPS) {
    out[g.id] = defaultGroupExpanded(g.id, reviewStep);
  }
  return out;
}
