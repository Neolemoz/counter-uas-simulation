import { ADVISORY_GOVERNANCE_BANNER } from "./advisoryTypes";
import type { SessionAdvisorySummary } from "./advisoryTypes";

export function AdvisoryBatchMirrorStrip({
  summary,
}: {
  summary: SessionAdvisorySummary | null;
}) {
  if (!summary || summary.total === 0) {
    return null;
  }

  const cohortEntries = Object.entries(summary.readiness_cohorts).filter(
    ([, n]) => (n ?? 0) > 0,
  );
  const groupEntries = Object.entries(summary.blocker_groups).filter(
    ([, n]) => (n ?? 0) > 0,
  );

  return (
    <div className="mb-4 rounded border border-slate-700 bg-slate-950/50 p-3">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Session advisory rollup (
        {summary.schema_version === "f8" ? "F8" : "F7"} — read-only)
      </h3>
      <p className="mb-2 text-[11px] text-amber-200/90">{ADVISORY_GOVERNANCE_BANNER}</p>
      <p className="mb-2 text-xs text-slate-500">{summary.total} staged capture(s)</p>
      {cohortEntries.length > 0 && (
        <div className="mb-2">
          <span className="text-[10px] uppercase text-slate-500">Cohorts: </span>
          {cohortEntries.map(([id, count]) => (
            <span key={id} className="mr-2 text-xs text-slate-400">
              {id.replace(/_/g, " ")} ({count})
            </span>
          ))}
        </div>
      )}
      {groupEntries.length > 0 && (
        <div>
          <span className="text-[10px] uppercase text-slate-500">Blocker groups: </span>
          {groupEntries.map(([id, count]) => (
            <span key={id} className="mr-2 text-xs text-slate-400">
              {id} ({count})
            </span>
          ))}
        </div>
      )}
    </div>
  );
}
