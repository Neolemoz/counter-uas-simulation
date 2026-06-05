import type { RtIntelligenceAdvisoryV1 } from "../intelligenceAdvisory";

const REASON_LABELS: Record<string, string> = {
  critical_target: "Critical zone",
  shortest_tti: "Shortest TTI",
  only_feasible: "Only feasible defender",
  descending_fast: "Descending fast",
  inside_warning_ring: "Inside warning ring",
  no_solution: "No feasible defender",
  feasible_pair_available: "Feasible pair available",
  tti_tie_break: "TTI tie break",
  insufficient_inputs: "Insufficient inputs",
};

function formatTti(ttiS: number | null): string {
  return ttiS == null || !Number.isFinite(ttiS) ? "-" : `${ttiS.toFixed(1)} s`;
}

function humanizeReasonCode(code: string): string {
  return code
    .split("_")
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}

function reasonLabels(codes: string[]): string[] {
  return codes.map((code) => REASON_LABELS[code] ?? humanizeReasonCode(code));
}

export function DefenderComparisonPanel({ advisory }: { advisory: RtIntelligenceAdvisoryV1 }) {
  const defenders = advisory.defender_ranking.ranked_defenders;

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="defender-comparison-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Defender comparison
      </h3>
      {defenders.length === 0 ? (
        <p className="text-slate-500">No defender ranking available for this advisory.</p>
      ) : (
        <div className="overflow-x-auto">
          <table className="w-full border-collapse text-left">
            <thead className="text-[10px] uppercase text-slate-500">
              <tr>
                <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Rank</th>
                <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Defender</th>
                <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Feasible</th>
                <th className="border-b border-slate-800 pb-1 pr-3 font-medium">TTI</th>
                <th className="border-b border-slate-800 pb-1 font-medium">Reason codes</th>
              </tr>
            </thead>
            <tbody>
              {defenders.map((defender) => {
                const labels = reasonLabels(defender.reason_codes);
                return (
                  <tr key={`${defender.rank}-${defender.defender_id}`} className="text-slate-300">
                    <td className="border-b border-slate-900 py-1.5 pr-3 text-slate-200">
                      #{defender.rank}
                    </td>
                    <td className="border-b border-slate-900 py-1.5 pr-3 font-mono">
                      {defender.defender_id}
                    </td>
                    <td className="border-b border-slate-900 py-1.5 pr-3">
                      {defender.feasible ? "Yes" : "No"}
                    </td>
                    <td className="border-b border-slate-900 py-1.5 pr-3">
                      {formatTti(defender.tti_s)}
                    </td>
                    <td className="border-b border-slate-900 py-1.5">
                      {labels.length === 0 ? (
                        <span className="text-slate-600">-</span>
                      ) : (
                        <ul className="flex flex-wrap gap-1" aria-label="Defender reason labels">
                          {labels.map((label) => (
                            <li
                              key={label}
                              className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-300"
                            >
                              {label}
                            </li>
                          ))}
                        </ul>
                      )}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
    </section>
  );
}
