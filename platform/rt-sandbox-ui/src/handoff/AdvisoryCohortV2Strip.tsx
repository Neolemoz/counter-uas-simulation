import type { SessionAdvisorySummary } from "./advisoryTypes";
import { AdvisoryRollupSummaryBar } from "./AdvisoryRollupSummaryBar";

export function AdvisoryCohortV2Strip({
  summary,
}: {
  summary: SessionAdvisorySummary | null;
}) {
  if (!summary?.readiness_cohorts_v2 && !summary?.handoff_rollup) {
    return null;
  }

  return (
    <div className="mb-4 rounded border border-slate-700 bg-slate-950/50 p-3">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Advisory cohort v2 rollup (F8 — read-only)
      </h3>
      <AdvisoryRollupSummaryBar summary={summary} />
    </div>
  );
}
