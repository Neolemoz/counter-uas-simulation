import { advisoryUiState, type RtIntelligenceAdvisoryTransportV1, type RtIntelligenceAdvisoryV1 } from "./intelligenceAdvisory";
import {
  INTELLIGENCE_ADVISORY_BANNER,
  INTELLIGENCE_ADVISORY_NO_SOLUTION_COPY,
  INTELLIGENCE_ADVISORY_SHORT_COPY,
  INTELLIGENCE_ADVISORY_STALE_COPY,
} from "./intelligenceGovernance";
import {
  isProtectedCenterUnavailable,
  PROTECTED_CENTER_UNAVAILABLE_COPY,
} from "./protectedCenterCopy";
import {
  getAdvisoryConfidenceLabel,
  getAdvisoryReasonLabels,
  getRankedAdvisories,
} from "./intelligenceSelectors";

function formatScore(score: number | null): string {
  return score == null || !Number.isFinite(score) ? "-" : score.toFixed(1);
}

function formatRank(rank: number | null): string {
  return rank == null || !Number.isFinite(rank) ? "-" : `#${rank}`;
}

function formatTti(ttiS: number | null): string {
  return ttiS == null || !Number.isFinite(ttiS) ? "-" : `${ttiS.toFixed(1)} s`;
}

function advisoryCountLabel(count: number): string {
  return count === 1 ? "1 advisory" : `${count} advisories`;
}

function defenderLabel(advisory: RtIntelligenceAdvisoryV1): string {
  return advisory.recommended_defender.defender_id ?? INTELLIGENCE_ADVISORY_NO_SOLUTION_COPY;
}

function AdvisoryRow({ advisory }: { advisory: RtIntelligenceAdvisoryV1 }) {
  const reasonLabels = getAdvisoryReasonLabels(advisory);
  return (
    <li
      className="rounded border border-slate-800 bg-slate-950/45 p-2 text-xs text-slate-300"
      data-testid="intelligence-advisory-row"
    >
      <div className="flex flex-wrap items-center justify-between gap-2">
        <span className="font-mono text-slate-100">{advisory.identity.attacker_id}</span>
        <span className="rounded border border-indigo-800/50 bg-indigo-950/40 px-2 py-0.5 text-indigo-100">
          Threat {formatRank(advisory.threat_evaluation.threat_rank)}
        </span>
      </div>
      <dl className="mt-2 grid grid-cols-2 gap-2">
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Threat score</dt>
          <dd className="text-slate-200">{formatScore(advisory.threat_evaluation.threat_score)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Recommended defender</dt>
          <dd className="text-slate-200">{defenderLabel(advisory)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">TTI</dt>
          <dd className="text-slate-200">{formatTti(advisory.recommended_defender.tti_s)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Confidence</dt>
          <dd className="text-slate-200">{getAdvisoryConfidenceLabel(advisory)}</dd>
        </div>
      </dl>
      {reasonLabels.length > 0 && (
        <ul className="mt-2 flex flex-wrap gap-1" aria-label="Advisory reason labels">
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
    </li>
  );
}

export function IntelligenceAdvisoryPanel({
  transport,
}: {
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined;
}) {
  const state = advisoryUiState(transport);
  const advisories = getRankedAdvisories(transport);
  const count = transport?.advisories.length ?? 0;

  return (
    <section
      className="rounded border border-slate-700/70 bg-slate-900/70 p-3"
      data-testid="intelligence-advisory-panel"
    >
      <p className="mb-1 text-[10px] text-amber-100/80">{INTELLIGENCE_ADVISORY_BANNER}</p>
      <p className="mb-3 text-[10px] text-slate-500">{INTELLIGENCE_ADVISORY_SHORT_COPY}</p>

      <div className="mb-3 flex flex-wrap items-center justify-between gap-2 text-xs">
        <span className="font-semibold uppercase tracking-wide text-slate-300">
          Intelligence advisories
        </span>
        <span className="rounded border border-slate-700 bg-slate-950 px-2 py-0.5 text-slate-300">
          {advisoryCountLabel(count)}
        </span>
      </div>

      {state === "loading" && (
        <p className="text-xs text-slate-500">Waiting for advisory transport.</p>
      )}
      {state === "empty" && (
        <p className="text-xs text-slate-500">No current intelligence advisories.</p>
      )}
      {state === "stale" && (
        <p
          className="rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1.5 text-xs text-amber-100"
          data-testid={
            isProtectedCenterUnavailable(transport?.stale_reason)
              ? "intelligence-protected-center-unavailable"
              : "intelligence-advisory-stale"
          }
        >
          {isProtectedCenterUnavailable(transport?.stale_reason)
            ? PROTECTED_CENTER_UNAVAILABLE_COPY
            : `${INTELLIGENCE_ADVISORY_STALE_COPY}${
                transport?.stale_reason ? ` Reason: ${transport.stale_reason}.` : ""
              }`}
        </p>
      )}
      {state === "active" && (
        <ol className="space-y-2">
          {advisories.map((advisory) => (
            <AdvisoryRow key={advisory.identity.advisory_id} advisory={advisory} />
          ))}
        </ol>
      )}
    </section>
  );
}
