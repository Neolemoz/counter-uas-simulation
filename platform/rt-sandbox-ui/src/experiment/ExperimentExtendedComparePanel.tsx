import { BANNER_EXPERIMENT_F5 } from "@/governance/banners";
import type {
  ExperimentMetricsReport,
  ExperimentRun,
  PerRunExtended,
} from "./experimentSchema";
import { pairsForRuns, toggleExtendedCompareRun } from "./experimentF5UiHelpers";

function formatRollupCounts(record: Record<string, number>): string {
  return Object.entries(record)
    .map(([k, v]) => `${k}=${v}`)
    .join(", ");
}

export function ExperimentExtendedComparePanel({
  filteredRuns,
  perRunExtended,
  metricsReport,
  selectedRunIds,
  onSelectedRunIdsChange,
}: {
  filteredRuns: ExperimentRun[];
  perRunExtended: PerRunExtended[];
  metricsReport: ExperimentMetricsReport;
  selectedRunIds: string[];
  onSelectedRunIdsChange: (ids: string[]) => void;
}) {
  const pairs = pairsForRuns(
    metricsReport.compare_pairs_extended,
    selectedRunIds,
  );

  const extById = new Map(perRunExtended.map((r) => [r.run_id, r]));

  return (
    <section className="space-y-3" data-testid="experiment-extended-compare-panel">
      <p className="text-[10px] text-amber-100/80">{BANNER_EXPERIMENT_F5}</p>
      <p className="text-[10px] text-slate-500">
        Extended compare — up to 4 manifest runs; explanatory badges only
      </p>

      <div className="flex flex-wrap gap-2">
        {filteredRuns.map((run) => {
          const checked = selectedRunIds.includes(run.run_id);
          return (
            <label
              key={run.run_id}
              className="flex items-center gap-1 rounded border border-slate-700 bg-slate-900 px-2 py-1 text-[10px] text-slate-300"
            >
              <input
                type="checkbox"
                checked={checked}
                onChange={() =>
                  onSelectedRunIdsChange(
                    toggleExtendedCompareRun(selectedRunIds, run.run_id),
                  )
                }
              />
              <span className="font-mono text-sky-200/90">{run.label}</span>
            </label>
          );
        })}
      </div>

      {selectedRunIds.length < 2 && (
        <p className="text-xs text-slate-500">Select at least two runs to compare.</p>
      )}

      {pairs.length > 0 && (
        <div className="space-y-2">
          {pairs.map((pair) => (
            <div
              key={`${pair.run_id_a}-${pair.run_id_b}`}
              className="rounded border border-slate-800 bg-slate-950/50 p-2"
            >
              <p className="mb-1 font-mono text-[10px] text-slate-400">
                {pair.run_id_a} ↔ {pair.run_id_b}
              </p>
              <div className="flex flex-wrap gap-1">
                {pair.badges.map((b) => (
                  <span
                    key={b.id}
                    className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-sky-200/90"
                    title={b.detail}
                  >
                    {b.label}
                  </span>
                ))}
              </div>
            </div>
          ))}
        </div>
      )}

      <dl className="rounded border border-slate-800 bg-slate-950/50 p-2 text-[10px] text-slate-400">
        <dt className="font-medium text-slate-500">rollup counts (derived)</dt>
        <dd>
          tactical modes:{" "}
          {formatRollupCounts(metricsReport.rollup_extended.tactical_rollup.mode_counts)}
        </dd>
        <dd>
          ridges:{" "}
          {formatRollupCounts(metricsReport.rollup_extended.terrain_rollup.ridge_counts)}
        </dd>
        <dd>
          visibility:{" "}
          {formatRollupCounts(
            metricsReport.rollup_extended.visibility_rollup.los_label_counts,
          )}
        </dd>
      </dl>

      {selectedRunIds.length > 0 && (
        <ul className="space-y-1 text-[10px] text-slate-400">
          {selectedRunIds.map((id) => {
            const ext = extById.get(id);
            if (!ext) return null;
            return (
              <li key={id} className="font-mono">
                {id}: cognition: ridge={ext.nearest_ridge ?? "—"} band=
                {ext.elevation_band ?? "—"}; cognition: los=
                {ext.los_cognition_label ?? "—"}
              </li>
            );
          })}
        </ul>
      )}
    </section>
  );
}
