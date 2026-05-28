import type { ExperimentHandoffRollup } from "./advisoryTypes";

export function AdvisoryExperimentHandoffStrip({
  rollup,
}: {
  rollup: ExperimentHandoffRollup | null | undefined;
}) {
  if (!rollup) return null;
  return (
    <div className="mb-3 rounded border border-slate-700/80 bg-slate-950/40 p-2">
      <h4 className="mb-1 text-[10px] font-semibold uppercase tracking-wide text-slate-500">
        Experiment handoff rollup (warn-only)
      </h4>
      <p className="text-xs text-slate-400">
        Eligibility: {rollup.handoff_eligibility}
        {rollup.cohort_status ? ` · cohort: ${rollup.cohort_status}` : ""}
      </p>
      {rollup.warn_capture_ids?.length ? (
        <p className="mt-1 text-[10px] text-amber-200/80">
          Warn captures: {rollup.warn_capture_ids.join(", ")}
        </p>
      ) : null}
      {rollup.note && (
        <p className="mt-1 text-[10px] text-slate-500 italic">{rollup.note}</p>
      )}
      <p className="mt-1 text-[10px] text-slate-600">
        Experiment cohort ≠ readiness cohort v2
      </p>
    </div>
  );
}
