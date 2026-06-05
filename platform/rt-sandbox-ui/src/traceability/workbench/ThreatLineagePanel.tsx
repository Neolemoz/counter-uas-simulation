import { formatNumber, labelFromToken } from "./formatters";
import { TRACEABILITY_CONFIDENCE_CAVEAT } from "./traceabilityGovernance";
import type { ThreatLineage } from "./traceabilityWorkbenchTypes";

function LineageField({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

export function ThreatLineagePanel({ lineage }: { lineage: ThreatLineage | null }) {
  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="threat-lineage-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Threat lineage
      </h3>
      <div className="mb-3 grid gap-1 text-center text-[10px] uppercase text-slate-400 sm:grid-cols-2">
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">Attacker</div>
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">
          Threat Evaluation
        </div>
      </div>
      {lineage === null ? (
        <p
          className="rounded border border-slate-800 bg-slate-950/60 px-2 py-2 text-slate-400"
          data-testid="threat-lineage-missing"
        >
          No threat evaluation lineage available for this track.
        </p>
      ) : (
        <>
          <dl className="grid gap-2 sm:grid-cols-2">
            <LineageField label="attacker_id" value={lineage.attacker_id} />
            <LineageField
              label="threat_rank"
              value={
                lineage.threat_rank === null || !Number.isFinite(lineage.threat_rank)
                  ? "-"
                  : `#${lineage.threat_rank}`
              }
            />
            <LineageField label="threat_score" value={formatNumber(lineage.threat_score, 1)} />
            <LineageField
              label="heuristic_confidence_level"
              value={lineage.heuristic_confidence_level}
            />
            <LineageField
              label="heuristic_confidence_score"
              value={formatNumber(lineage.heuristic_confidence_score, 2)}
            />
          </dl>
          <div className="mt-3">
            <p className="mb-1 text-[10px] uppercase text-slate-500">threat_components</p>
            {lineage.threat_components.length === 0 ? (
              <p className="text-slate-500">No threat components available.</p>
            ) : (
              <table className="w-full text-left text-[10px] text-slate-300">
                <thead>
                  <tr className="border-b border-slate-800 text-slate-500">
                    <th className="py-1 pr-2">Component</th>
                    <th className="py-1 pr-2">Value</th>
                    <th className="py-1 pr-2">Normalized</th>
                    <th className="py-1">Weight</th>
                  </tr>
                </thead>
                <tbody>
                  {lineage.threat_components.map((row) => (
                    <tr key={row.key} className="border-b border-slate-900/80">
                      <td className="py-1 pr-2 text-slate-200">{row.label}</td>
                      <td className="py-1 pr-2 font-mono">{row.value_display}</td>
                      <td className="py-1 pr-2 font-mono">
                        {formatNumber(row.normalized, 2)}
                      </td>
                      <td className="py-1 font-mono">{formatNumber(row.weight, 0)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            )}
          </div>
          <div className="mt-2">
            <p className="mb-1 text-[10px] uppercase text-slate-500">confidence_basis</p>
            {lineage.confidence_basis.length === 0 ? (
              <p className="text-slate-500">No confidence basis available.</p>
            ) : (
              <ul className="flex flex-wrap gap-1" aria-label="Threat confidence basis">
                {lineage.confidence_basis.map((basis) => (
                  <li
                    key={basis}
                    className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-300"
                  >
                    {labelFromToken(basis)}
                  </li>
                ))}
              </ul>
            )}
          </div>
          <p className="mt-2 text-[10px] text-amber-100/80">{TRACEABILITY_CONFIDENCE_CAVEAT}</p>
        </>
      )}
    </section>
  );
}
