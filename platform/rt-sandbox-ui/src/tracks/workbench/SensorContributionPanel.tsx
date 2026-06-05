import { labelFromToken } from "./formatters";
import type { SensorContributionRow } from "./trackSensorWorkbenchTypes";

const REQUIRED_ROWS = ["radar", "camera", "fused_detection", "tracker_update"] as const;

function rowForSource(
  rows: readonly SensorContributionRow[],
  source: (typeof REQUIRED_ROWS)[number],
): SensorContributionRow {
  return (
    rows.find((row) => row.source === source) ?? {
      source,
      status: "not_available",
      freshness: "unknown",
      contribution: "-",
      agreement: "-",
      notes: "No contribution record available.",
    }
  );
}

export function SensorContributionPanel({
  rows,
}: {
  rows: readonly SensorContributionRow[];
}) {
  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="sensor-contribution-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Sensor contribution
      </h3>
      <div className="overflow-x-auto">
        <table className="w-full border-collapse text-left">
          <thead className="text-[10px] uppercase text-slate-500">
            <tr>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Source</th>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Status</th>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Freshness</th>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Contribution</th>
              <th className="border-b border-slate-800 pb-1 pr-3 font-medium">Agreement</th>
              <th className="border-b border-slate-800 pb-1 font-medium">Notes</th>
            </tr>
          </thead>
          <tbody>
            {REQUIRED_ROWS.map((source) => {
              const row = rowForSource(rows, source);
              return (
                <tr key={source} className="text-slate-300">
                  <td className="border-b border-slate-900 py-1.5 pr-3 text-slate-200">
                    {labelFromToken(row.source)}
                  </td>
                  <td className="border-b border-slate-900 py-1.5 pr-3">
                    {row.status}
                  </td>
                  <td className="border-b border-slate-900 py-1.5 pr-3">
                    {row.freshness}
                  </td>
                  <td className="border-b border-slate-900 py-1.5 pr-3">
                    {row.contribution}
                  </td>
                  <td className="border-b border-slate-900 py-1.5 pr-3">
                    {row.agreement}
                  </td>
                  <td className="border-b border-slate-900 py-1.5">
                    {row.notes}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <p className="mt-2 text-[10px] text-slate-500">
        Sensor contribution is explanatory input visibility only, not sensor truth.
      </p>
    </section>
  );
}
