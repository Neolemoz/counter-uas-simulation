import {
  formatStepCompletionLabel,
  type ReviewStepCompletionState,
} from "./reviewStepCompletion";

export function ReviewStepCompletionBadge({
  state,
}: {
  state: ReviewStepCompletionState;
}) {
  const label = formatStepCompletionLabel(state);
  const hollow = state === "skipped" || state === "n/a";
  return (
    <span
      className={
        hollow
          ? "rounded border border-dashed border-slate-600 px-1.5 py-0.5 text-[10px] text-slate-500"
          : "rounded border border-slate-600 bg-slate-800/80 px-1.5 py-0.5 text-[10px] text-cyan-200/90"
      }
      data-testid="review-step-completion-badge"
      data-completion={state}
    >
      {label}
    </span>
  );
}
