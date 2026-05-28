import {
  REVIEW_STEP_IDS,
  UNIFIED_REVIEW_STEPS,
  type UnifiedReviewStepId,
} from "./experimentUnifiedReview";
import type { ExperimentRun } from "./experimentSchema";
import {
  advanceReviewStep,
  markStepCompleted,
  retreatReviewStep,
  setActiveRunId,
  setCompareRuns,
  setReviewStep,
  type WorkbenchV2State,
} from "./workbenchV2State";

export function ExperimentUnifiedReviewPanel({
  v2State,
  onV2StateChange,
  runs,
  onActivateStep,
  onContinuityRunId,
  onSyncComparePinned,
}: {
  v2State: WorkbenchV2State;
  onV2StateChange: (state: WorkbenchV2State) => void;
  runs: ExperimentRun[];
  onActivateStep: (step: UnifiedReviewStepId) => void;
  onContinuityRunId: (runId: string) => void;
  onSyncComparePinned: (runA: string | null, runB: string | null) => void;
}) {
  const stepIndex = REVIEW_STEP_IDS.indexOf(v2State.review_step);

  const selectStep = (step: UnifiedReviewStepId) => {
    const next = setReviewStep(v2State, step);
    onV2StateChange(next);
    onActivateStep(step);
  };

  const goNext = () => {
    const next = advanceReviewStep(v2State);
    onV2StateChange(next);
    onActivateStep(next.review_step);
  };

  const goPrev = () => {
    const next = retreatReviewStep(v2State);
    onV2StateChange(next);
    onActivateStep(next.review_step);
  };

  const openCurrent = () => {
    const next = markStepCompleted(v2State, v2State.review_step);
    onV2StateChange(next);
    onActivateStep(v2State.review_step);
  };

  const showRunSelectors =
    v2State.review_step === "f3_continuity" || v2State.review_step === "compare";

  return (
    <div
      className="space-y-2 rounded border border-slate-800 bg-slate-950/40 p-2"
      data-testid="unified-review-panel"
    >
      <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
        Unified review lane
      </p>
      <div className="flex flex-wrap gap-2">
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-0.5 text-xs text-slate-200 disabled:opacity-40"
          disabled={stepIndex <= 0}
          onClick={goPrev}
        >
          Prev
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-0.5 text-xs text-slate-200 disabled:opacity-40"
          disabled={stepIndex >= REVIEW_STEP_IDS.length - 1}
          onClick={goNext}
        >
          Next
        </button>
        <button
          type="button"
          className="rounded border border-cyan-800 bg-cyan-950/40 px-2 py-0.5 text-xs text-cyan-200"
          onClick={openCurrent}
        >
          Open step panel
        </button>
      </div>
      <ol className="space-y-1">
        {UNIFIED_REVIEW_STEPS.map((step) => {
          const active = step.id === v2State.review_step;
          const done = v2State.steps_completed.includes(step.id);
          return (
            <li key={step.id}>
              <button
                type="button"
                className={
                  active
                    ? "w-full rounded border border-cyan-800/60 bg-cyan-950/30 px-2 py-1 text-left"
                    : "w-full rounded px-2 py-1 text-left text-slate-500 hover:bg-slate-900/60"
                }
                onClick={() => selectStep(step.id)}
              >
                <span className="text-xs font-medium text-slate-300">
                  {done ? "✓ " : ""}
                  {step.label}
                </span>
                <p className="text-[10px] text-slate-500">{step.panelHint}</p>
              </button>
            </li>
          );
        })}
      </ol>
      {showRunSelectors && runs.length > 0 && (
        <div className="space-y-2 border-t border-slate-800 pt-2">
          {v2State.review_step === "f3_continuity" && (
            <label className="flex flex-col gap-1 text-xs text-slate-400">
              Focus run (manifest scope)
              <select
                className="rounded border border-slate-700 bg-slate-950 px-2 py-1 font-mono text-xs"
                value={v2State.active_run_id ?? runs[0]?.run_id ?? ""}
                onChange={(e) => {
                  const runId = e.target.value;
                  const next = setActiveRunId(v2State, runId);
                  onV2StateChange(next);
                  onContinuityRunId(runId);
                }}
              >
                {runs.map((r) => (
                  <option key={r.run_id} value={r.run_id}>
                    {r.label} ({r.run_id})
                  </option>
                ))}
              </select>
            </label>
          )}
          {v2State.review_step === "compare" && (
            <div className="grid gap-2 sm:grid-cols-2">
              <label className="flex flex-col gap-1 text-xs text-slate-400">
                Primary run
                <select
                  className="rounded border border-slate-700 bg-slate-950 px-2 py-1 font-mono text-xs"
                  value={v2State.compare_run_a ?? runs[0]?.run_id ?? ""}
                  onChange={(e) => {
                    const runA = e.target.value;
                    const runB = v2State.compare_run_b ?? runs[1]?.run_id ?? null;
                    const next = setCompareRuns(v2State, runA, runB);
                    onV2StateChange(next);
                    onSyncComparePinned(runA, runB);
                  }}
                >
                  {runs.map((r) => (
                    <option key={r.run_id} value={r.run_id}>
                      {r.label}
                    </option>
                  ))}
                </select>
              </label>
              <label className="flex flex-col gap-1 text-xs text-slate-400">
                Secondary run
                <select
                  className="rounded border border-slate-700 bg-slate-950 px-2 py-1 font-mono text-xs"
                  value={v2State.compare_run_b ?? runs[1]?.run_id ?? runs[0]?.run_id ?? ""}
                  onChange={(e) => {
                    const runB = e.target.value;
                    const runA = v2State.compare_run_a ?? runs[0]?.run_id ?? null;
                    const next = setCompareRuns(v2State, runA, runB);
                    onV2StateChange(next);
                    onSyncComparePinned(runA, runB);
                  }}
                >
                  {runs.map((r) => (
                    <option key={r.run_id} value={r.run_id}>
                      {r.label}
                    </option>
                  ))}
                </select>
              </label>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
