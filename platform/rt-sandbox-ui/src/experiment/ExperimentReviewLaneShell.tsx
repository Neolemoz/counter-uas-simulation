import { UNIFIED_REVIEW_STEPS, type UnifiedReviewStepId } from "./experimentUnifiedReview";

export function ExperimentReviewLaneShell({
  reviewStep,
}: {
  reviewStep: UnifiedReviewStepId;
}) {
  return (
    <div
      className="space-y-1 rounded border border-slate-800 bg-slate-950/40 p-2"
      data-testid="review-lane-shell"
    >
      <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
        Unified review lane (read-only)
      </p>
      <ol className="space-y-1">
        {UNIFIED_REVIEW_STEPS.map((step) => {
          const active = step.id === reviewStep;
          return (
            <li
              key={step.id}
              className={
                active
                  ? "rounded border border-cyan-800/60 bg-cyan-950/30 px-2 py-1"
                  : "rounded px-2 py-1 text-slate-500"
              }
            >
              <span className="text-xs font-medium text-slate-300">{step.label}</span>
              <p className="text-[10px] text-slate-500">{step.panelHint}</p>
            </li>
          );
        })}
      </ol>
    </div>
  );
}
