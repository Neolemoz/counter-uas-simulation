import { StatusBadge } from "@/workstation/StatusBadge";
import type { ReadinessCohortId } from "./advisoryTypes";

const LABELS: Record<ReadinessCohortId, string> = {
  needs_normalize: "needs normalize",
  needs_review: "needs review",
  needs_approve: "needs approve",
  needs_prepare: "needs prepare",
  ready_for_commit_advisory: "commit advisory",
  blocked: "blocked",
  terminal: "terminal",
  error: "error",
};

export function ReadinessCohortChip({ cohort }: { cohort: ReadinessCohortId }) {
  const tone =
    cohort === "ready_for_commit_advisory"
      ? "warn"
      : cohort === "blocked" || cohort === "error"
        ? "error"
        : "neutral";
  return (
    <StatusBadge
      label={LABELS[cohort]}
      tone={tone}
      title="Advisory cohort — not operational readiness"
    />
  );
}
