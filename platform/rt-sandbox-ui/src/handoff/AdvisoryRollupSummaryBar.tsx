import type { SessionAdvisorySummary } from "./advisoryTypes";
import { ADVISORY_GOVERNANCE_BANNER } from "./advisoryTypes";

/** Compact v2 rollup line for triage hub (session scope). */
export function AdvisoryRollupSummaryBar({
  summary,
  scopeNote,
}: {
  summary: SessionAdvisorySummary | null;
  scopeNote?: string;
}) {
  if (!summary?.readiness_cohorts_v2 && !summary?.handoff_rollup) {
    return null;
  }
  const v2 = summary.readiness_cohorts_v2 ?? {};
  const entries = Object.entries(v2).filter(([, n]) => (n ?? 0) > 0);
  const hr = summary.handoff_rollup;
  const mc = summary.multi_capture_cohorts;
  const exp = summary.experiment_handoff_rollup;

  if (!entries.length && !hr && !mc) return null;

  return (
    <div className="mb-3 rounded border border-slate-800/80 bg-slate-900/40 p-2">
      <p className="mb-1 text-[10px] text-amber-200/80">
        Rollups: session scope{scopeNote ? ` — ${scopeNote}` : ""}. Table rows may be
        preset/focus filtered. {ADVISORY_GOVERNANCE_BANNER}
      </p>
      {entries.length > 0 && (
        <div className="mb-1 flex flex-wrap gap-x-2 gap-y-0.5 text-[10px] text-slate-400">
          <span className="text-slate-500">cohorts v2:</span>
          {entries.map(([id, count]) => (
            <span key={id}>
              {id.replace(/_/g, " ")} ({count})
            </span>
          ))}
        </div>
      )}
      {hr?.by_stage && Object.keys(hr.by_stage).length > 0 && (
        <div className="mb-1 flex flex-wrap gap-x-2 text-[10px] text-slate-400">
          <span className="text-slate-500">handoff:</span>
          {Object.entries(hr.by_stage).map(([stage, count]) => (
            <span key={stage}>
              {stage} ({count})
            </span>
          ))}
        </div>
      )}
      {mc != null && mc.stale_age_warn_count > 0 && (
        <p className="text-[10px] text-amber-200/70">
          Stale age warn: {mc.stale_age_warn_count}
        </p>
      )}
      {exp?.note && (
        <p className="text-[10px] text-slate-500" title="warn-only">
          Experiment handoff: {exp.note}
        </p>
      )}
    </div>
  );
}
