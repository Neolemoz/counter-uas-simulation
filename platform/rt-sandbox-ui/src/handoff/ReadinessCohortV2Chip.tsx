import type { ReadinessCohortV2Id } from "./advisoryTypes";

const LABELS: Partial<Record<ReadinessCohortV2Id, string>> = {
  stale_review: "stale review",
  stale_approve: "stale approve",
  multi_blocker: "multi blocker",
  experiment_handoff_warn: "exp handoff warn",
  focus_highlight: "focus",
};

export function ReadinessCohortV2Chip({ cohort }: { cohort: ReadinessCohortV2Id }) {
  const label = LABELS[cohort] ?? cohort.replace(/_/g, " ");
  return (
    <span
      className="rounded border border-violet-800/50 bg-violet-950/30 px-1.5 py-0.5 text-[10px] text-violet-200/90"
      title="Advisory cohort v2 — not operational readiness"
    >
      v2:{label}
    </span>
  );
}
