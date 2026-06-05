import { advisoryUiState, type RtIntelligenceAdvisoryTransportV1 } from "./intelligenceAdvisory";
import {
  INTELLIGENCE_ADVISORY_NO_SOLUTION_COPY,
  INTELLIGENCE_ADVISORY_SHORT_COPY,
  INTELLIGENCE_ADVISORY_STALE_COPY,
} from "./intelligenceGovernance";
import {
  getAdvisoryConfidenceLabel,
  getAdvisoryReasonLabels,
  getSelectedEntityAdvisory,
} from "./intelligenceSelectors";

function formatTti(ttiS: number | null): string {
  return ttiS == null || !Number.isFinite(ttiS) ? "-" : `${ttiS.toFixed(1)} s`;
}

export function SelectedTargetAdvisoryCard({
  transport,
  selectedAttackerId,
}: {
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined;
  selectedAttackerId: string | null | undefined;
}) {
  const state = advisoryUiState(transport);
  const advisory = getSelectedEntityAdvisory(transport, selectedAttackerId, {
    includeStale: true,
  });

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/50 p-3 text-xs"
      data-testid="selected-target-advisory-card"
    >
      <p className="mb-2 text-[10px] text-amber-100/80">{INTELLIGENCE_ADVISORY_SHORT_COPY}</p>

      {state === "loading" && (
        <p className="text-slate-500">Waiting for advisory transport.</p>
      )}
      {state === "stale" && (
        <p className="text-amber-200">
          {INTELLIGENCE_ADVISORY_STALE_COPY}
          {transport?.stale_reason ? ` Reason: ${transport.stale_reason}.` : ""}
        </p>
      )}
      {state !== "loading" && state !== "stale" && !selectedAttackerId && (
        <p className="text-slate-500">Select an attacker to view advisory details.</p>
      )}
      {state !== "loading" && state !== "stale" && selectedAttackerId && !advisory && (
        <p className="text-slate-500">
          No intelligence advisory for selected attacker {selectedAttackerId}.
        </p>
      )}
      {advisory && (
        <>
          <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
            <span className="font-mono text-slate-100">{advisory.identity.attacker_id}</span>
            <span className="text-slate-400">{getAdvisoryConfidenceLabel(advisory)}</span>
          </div>
          <dl className="grid grid-cols-2 gap-2 text-slate-300">
            <div>
              <dt className="text-[10px] uppercase text-slate-500">Recommended defender</dt>
              <dd>{advisory.recommended_defender.defender_id ?? INTELLIGENCE_ADVISORY_NO_SOLUTION_COPY}</dd>
            </div>
            <div>
              <dt className="text-[10px] uppercase text-slate-500">TTI</dt>
              <dd>{formatTti(advisory.recommended_defender.tti_s)}</dd>
            </div>
          </dl>
          <p className="mt-2 text-slate-400">{advisory.reasoning.explanation}</p>
          <ul className="mt-2 flex flex-wrap gap-1" aria-label="Selected advisory reason labels">
            {getAdvisoryReasonLabels(advisory).map((label) => (
              <li
                key={label}
                className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-300"
              >
                {label}
              </li>
            ))}
          </ul>
        </>
      )}
    </section>
  );
}
