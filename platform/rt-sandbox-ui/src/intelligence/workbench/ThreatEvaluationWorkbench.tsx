import type { RtIntelligenceAdvisoryV1 } from "../intelligenceAdvisory";
import { INTELLIGENCE_ADVISORY_BANNER } from "../intelligenceGovernance";
import { ConfidenceExplanationPanel } from "./ConfidenceExplanationPanel";
import { DefenderComparisonPanel } from "./DefenderComparisonPanel";
import { RecommendationExplanationPanel } from "./RecommendationExplanationPanel";
import { ThreatBreakdownPanel } from "./ThreatBreakdownPanel";

export function ThreatEvaluationWorkbench({
  advisory,
  stale = false,
  staleReason = null,
}: {
  advisory: RtIntelligenceAdvisoryV1;
  stale?: boolean;
  staleReason?: string | null;
}) {
  const rank = advisory.threat_evaluation.threat_rank;
  const score = advisory.threat_evaluation.threat_score;
  const rankLabel = rank == null || !Number.isFinite(rank) ? "-" : `#${rank}`;
  const scoreLabel = score == null || !Number.isFinite(score) ? "-" : score.toFixed(1);

  return (
    <section
      className="space-y-3 rounded border border-slate-700/70 bg-slate-900/70 p-3"
      data-testid="threat-evaluation-workbench"
    >
      <div className="sticky top-0 z-10 border-b border-slate-800 bg-slate-900/95 pb-2">
        <p className="text-[10px] text-amber-100/80">{INTELLIGENCE_ADVISORY_BANNER}</p>
        {stale && (
          <p className="mt-1 rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-100">
            Advisory stale - explanation data preserved for review
            {staleReason ? `: ${staleReason}` : ""}.
          </p>
        )}
        <div className="mt-2 flex flex-wrap items-center justify-between gap-2">
          <h2 className="text-xs font-semibold uppercase tracking-wide text-slate-200">
            Threat evaluation workbench
          </h2>
          <dl className="flex flex-wrap gap-2 text-[10px] text-slate-300">
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">Attacker</dt>
              <dd className="font-mono text-slate-100">{advisory.identity.attacker_id}</dd>
            </div>
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">Threat rank</dt>
              <dd className="text-slate-100">{rankLabel}</dd>
            </div>
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">Threat score</dt>
              <dd className="text-slate-100">{scoreLabel}</dd>
            </div>
          </dl>
        </div>
      </div>
      <ThreatBreakdownPanel advisory={advisory} />
      <DefenderComparisonPanel advisory={advisory} />
      <RecommendationExplanationPanel advisory={advisory} />
      <ConfidenceExplanationPanel advisory={advisory} />
    </section>
  );
}
