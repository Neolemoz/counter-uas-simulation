import {
  INTELLIGENCE_ADVISORY_NO_SOLUTION_COPY,
  INTELLIGENCE_ADVISORY_SHORT_COPY,
} from "../intelligenceGovernance";
import { getAdvisoryReasonLabels } from "../intelligenceSelectors";
import type { RtIntelligenceAdvisoryV1 } from "../intelligenceAdvisory";

function formatTti(ttiS: number | null): string {
  return ttiS == null || !Number.isFinite(ttiS) ? "-" : `${ttiS.toFixed(1)} s`;
}

export function RecommendationExplanationPanel({
  advisory,
}: {
  advisory: RtIntelligenceAdvisoryV1;
}) {
  const recommendation = advisory.recommended_defender;
  const reasonLabels = getAdvisoryReasonLabels(advisory);

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="recommendation-explanation-panel"
    >
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <h3 className="font-semibold uppercase tracking-wide text-slate-300">
          Recommendation explanation
        </h3>
        <span className="text-[10px] text-amber-100/80">{INTELLIGENCE_ADVISORY_SHORT_COPY}</span>
      </div>
      <dl className="grid grid-cols-2 gap-2 text-slate-300">
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Recommended defender</dt>
          <dd className="font-mono text-slate-200">
            {recommendation.defender_id ?? INTELLIGENCE_ADVISORY_NO_SOLUTION_COPY}
          </dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">TTI</dt>
          <dd className="text-slate-200">{formatTti(recommendation.tti_s)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Feasible</dt>
          <dd className="text-slate-200">{recommendation.feasibility.feasible ? "Yes" : "No"}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Feasibility reason</dt>
          <dd className="text-slate-200">{recommendation.feasibility.reason}</dd>
        </div>
      </dl>
      <p className="mt-2 text-slate-400">{advisory.reasoning.explanation}</p>
      {reasonLabels.length > 0 && (
        <ul className="mt-2 flex flex-wrap gap-1" aria-label="Recommendation reason labels">
          {reasonLabels.map((label) => (
            <li
              key={label}
              className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-300"
            >
              {label}
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
