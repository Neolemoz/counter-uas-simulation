import { formatNumber, formatTimestamp, labelFromToken } from "./formatters";
import { TRACEABILITY_LINKAGE_NOTE } from "./traceabilityGovernance";
import type { AdvisoryOrigin } from "./traceabilityWorkbenchTypes";

function OriginField({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

export function AdvisoryOriginPanel({ origin }: { origin: AdvisoryOrigin | null }) {
  const stale = origin?.advisory_freshness === "stale";
  const noRecommendation =
    origin !== null &&
    (origin.recommended_defender === null || origin.recommended_defender.trim().length === 0);
  const partialLinkage =
    origin !== null &&
    (origin.advisory_id === null || origin.attacker_id === null);

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="advisory-origin-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Advisory origin
      </h3>
      <div className="mb-3 grid gap-1 text-center text-[10px] uppercase text-slate-400 sm:grid-cols-2">
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">
          Threat Evaluation
        </div>
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">Advisory</div>
      </div>
      {origin === null ? (
        <p
          className="rounded border border-slate-800 bg-slate-950/60 px-2 py-2 text-slate-400"
          data-testid="advisory-origin-missing"
        >
          No advisory origin available for this lineage.
        </p>
      ) : (
        <>
          {stale && (
            <p
              className="mb-2 rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-100"
              data-testid="advisory-origin-stale-banner"
            >
              Stale advisory - explanation data preserved for review
              {origin.stale_reason ? `: ${origin.stale_reason}` : ""}.
            </p>
          )}
          {noRecommendation && (
            <p
              className="mb-2 rounded border border-slate-700 bg-slate-950/60 px-2 py-1 text-[10px] text-slate-300"
              data-testid="advisory-origin-no-recommendation"
            >
              No feasible defender recommendation available.
            </p>
          )}
          {partialLinkage && (
            <p
              className="mb-2 rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-100"
              data-testid="advisory-origin-partial-linkage"
            >
              Partial advisory linkage - some origin fields are unavailable.
            </p>
          )}
          <dl className="grid gap-2 sm:grid-cols-2">
            <OriginField label="advisory_id" value={origin.advisory_id ?? "-"} />
            <OriginField label="attacker_id" value={origin.attacker_id ?? "-"} />
            <OriginField
              label="recommended_defender"
              value={origin.recommended_defender ?? "-"}
            />
            <OriginField
              label="defender_rank"
              value={
                origin.defender_rank === null || !Number.isFinite(origin.defender_rank)
                  ? "-"
                  : `#${origin.defender_rank}`
              }
            />
            <OriginField label="tti_s" value={formatNumber(origin.tti_s, 1)} />
            <OriginField
              label="advisory_freshness"
              value={labelFromToken(origin.advisory_freshness)}
            />
            <OriginField label="advisory_utc" value={formatTimestamp(origin.advisory_utc)} />
            <OriginField label="stale_reason" value={origin.stale_reason ?? "-"} />
          </dl>
          <div className="mt-2">
            <p className="mb-1 text-[10px] uppercase text-slate-500">reason_codes</p>
            {origin.reason_codes.length === 0 ? (
              <p className="text-slate-500">No reason codes available.</p>
            ) : (
              <ul className="flex flex-wrap gap-1" aria-label="Advisory reason codes">
                {origin.reason_codes.map((code) => (
                  <li
                    key={code}
                    className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-300"
                  >
                    {labelFromToken(code)}
                  </li>
                ))}
              </ul>
            )}
          </div>
          <p className="mt-2 text-slate-300">
            {origin.explanation ?? "No advisory explanation available."}
          </p>
        </>
      )}
      <p className="mt-2 text-[10px] text-slate-500">{TRACEABILITY_LINKAGE_NOTE}</p>
    </section>
  );
}
