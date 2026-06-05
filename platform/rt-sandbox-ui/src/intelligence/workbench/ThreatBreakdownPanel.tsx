import type { RtIntelligenceAdvisoryV1, ThreatComponentValue } from "../intelligenceAdvisory";

type ThreatComponentKey = keyof RtIntelligenceAdvisoryV1["threat_evaluation"]["threat_components"];

const COMPONENT_ROWS: Array<{
  key: ThreatComponentKey;
  label: string;
}> = [
  { key: "distance_to_protected_center", label: "Distance to protected center" },
  { key: "best_feasible_tti", label: "Best feasible TTI" },
  { key: "descent_factor", label: "Descent factor" },
  { key: "critical_zone_factor", label: "Critical zone factor" },
];

function finiteNumber(value: number | null | undefined): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function formatRawValue(component: ThreatComponentValue): string {
  const valueM = finiteNumber(component.value_m);
  if (valueM !== null) return `${valueM.toFixed(1)} m`;

  const valueS = finiteNumber(component.value_s);
  if (valueS !== null) return `${valueS.toFixed(1)} s`;

  const valueMps = finiteNumber(component.value_mps);
  if (valueMps !== null) return `${valueMps.toFixed(2)} m/s`;

  if (typeof component.active === "boolean") return component.active ? "Active" : "Inactive";

  return "-";
}

function formatNumber(value: number | null | undefined, digits = 2): string {
  const parsed = finiteNumber(value);
  return parsed === null ? "-" : parsed.toFixed(digits);
}

function contribution(component: ThreatComponentValue): number | null {
  const normalized = finiteNumber(component.normalized);
  const weight = finiteNumber(component.weight);
  if (normalized === null || weight === null) return null;
  return normalized * weight;
}

export function ThreatBreakdownPanel({ advisory }: { advisory: RtIntelligenceAdvisoryV1 }) {
  const components = advisory.threat_evaluation.threat_components;

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="threat-breakdown-panel"
    >
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <h3 className="font-semibold uppercase tracking-wide text-slate-300">
          Threat breakdown
        </h3>
        <span className="text-slate-500">
          Score {formatNumber(advisory.threat_evaluation.threat_score, 1)}
        </span>
      </div>
      <div className="overflow-x-auto">
        <table className="w-full border-collapse text-left">
          <thead className="text-[10px] uppercase text-slate-500">
            <tr>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Factor</th>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Raw value</th>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Normalized</th>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Weight</th>
              <th className="border-b border-slate-800 pb-1 font-medium">Contribution</th>
            </tr>
          </thead>
          <tbody>
            {COMPONENT_ROWS.map(({ key, label }) => {
              const component = components[key];
              return (
                <tr key={key} className="text-slate-300">
                  <td className="border-b border-slate-900 py-1.5 pr-3 text-slate-200">
                    {label}
                  </td>
                  <td className="border-b border-slate-900 py-1.5 pr-3">
                    {formatRawValue(component)}
                  </td>
                  <td className="border-b border-slate-900 py-1.5 pr-3">
                    {formatNumber(component.normalized)}
                  </td>
                  <td className="border-b border-slate-900 py-1.5 pr-3">
                    {formatNumber(component.weight, 1)}
                  </td>
                  <td className="border-b border-slate-900 py-1.5">
                    {formatNumber(contribution(component), 1)}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <p className="mt-2 text-[10px] text-slate-500">
        Contributions are UI-derived as normalized value times advisory weight.
      </p>
    </section>
  );
}
