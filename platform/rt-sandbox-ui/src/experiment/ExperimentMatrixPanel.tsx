import { BANNER_EXPERIMENT_F5 } from "@/governance/banners";
import type {
  ComparePairAnalytics,
  ExperimentManifest,
  ExperimentMetricsReport,
  ExperimentRun,
} from "./experimentSchema";
import {
  buildMatrixGrid,
  collectMatrixAxisKeys,
  isMatrixReviewClass,
  pairsForRuns,
} from "./experimentF5UiHelpers";

function entityCountForRun(run: ExperimentRun): number | null {
  const n = run.snapshot?.world_summary?.entity_count;
  return typeof n === "number" ? n : null;
}

function badgesForCell(
  runId: string | null,
  allRuns: ExperimentRun[],
  comparePairs: ComparePairAnalytics[],
): ComparePairAnalytics["badges"] {
  if (!runId) return [];
  const neighbors = allRuns
    .map((r) => r.run_id)
    .filter((id) => id !== runId);
  const pairs = pairsForRuns(comparePairs, [runId, ...neighbors.slice(0, 1)]);
  const relevant = pairs.filter(
    (p) => p.run_id_a === runId || p.run_id_b === runId,
  );
  const out: ComparePairAnalytics["badges"] = [];
  const seen = new Set<string>();
  for (const pair of relevant) {
    for (const badge of pair.badges) {
      if (seen.has(badge.id)) continue;
      seen.add(badge.id);
      out.push(badge);
    }
  }
  return out.slice(0, 4);
}

export function ExperimentMatrixPanel({
  manifest,
  metricsReport,
  axisRow,
  axisCol,
  onAxisRowChange,
  onAxisColChange,
}: {
  manifest: ExperimentManifest;
  metricsReport: ExperimentMetricsReport;
  axisRow: string;
  axisCol: string;
  onAxisRowChange: (axis: string) => void;
  onAxisColChange: (axis: string) => void;
}) {
  const experimentClass = metricsReport.experiment_class;
  if (!isMatrixReviewClass(experimentClass)) {
    return null;
  }

  const runs = manifest.runs;
  const axisKeys = collectMatrixAxisKeys(runs);
  const useGrid = axisKeys.length >= 2 && axisRow && axisCol && axisRow !== axisCol;
  const rollup = metricsReport.rollup_extended.matrix_rollup;

  const grid =
    useGrid && axisRow && axisCol
      ? buildMatrixGrid(runs, axisRow, axisCol)
      : null;

  const runById = new Map(runs.map((r) => [r.run_id, r]));

  return (
    <section className="space-y-3" data-testid="experiment-matrix-panel">
      <p className="text-[10px] text-amber-100/80">{BANNER_EXPERIMENT_F5}</p>
      <p className="text-[10px] text-slate-500">
        Matrix review — class{" "}
        <span className="font-mono text-slate-400">{experimentClass}</span> (count-only
        rollups)
      </p>

      {axisKeys.length >= 2 && (
        <div className="flex flex-wrap gap-2 text-xs text-slate-500">
          <label>
            row axis
            <select
              className="ml-1 rounded border border-slate-700 bg-slate-950 px-1 py-0.5 text-xs"
              value={axisRow}
              onChange={(e) => onAxisRowChange(e.target.value)}
            >
              {axisKeys.map((k) => (
                <option key={k} value={k}>
                  {k}
                </option>
              ))}
            </select>
          </label>
          <label>
            col axis
            <select
              className="ml-1 rounded border border-slate-700 bg-slate-950 px-1 py-0.5 text-xs"
              value={axisCol}
              onChange={(e) => onAxisColChange(e.target.value)}
            >
              {axisKeys.map((k) => (
                <option key={k} value={k}>
                  {k}
                </option>
              ))}
            </select>
          </label>
        </div>
      )}

      {grid && grid.rows.length > 0 ? (
        <div className="overflow-x-auto">
          <table className="w-full border-collapse text-[10px] text-slate-300">
            <thead>
              <tr>
                <th className="border border-slate-800 p-1 text-left text-slate-500">
                  {axisRow} \ {axisCol}
                </th>
                {grid.cols.map((c) => (
                  <th key={c} className="border border-slate-800 p-1 font-mono">
                    {c}
                  </th>
                ))}
              </tr>
            </thead>
            <tbody>
              {grid.rows.map((row) => (
                <tr key={row}>
                  <td className="border border-slate-800 p-1 font-mono text-slate-400">
                    {row}
                  </td>
                  {grid.cols.map((col) => {
                    const cell = grid.cells.find(
                      (c) => c.rowValue === row && c.colValue === col,
                    );
                    const runId = cell?.runId ?? null;
                    const run = runId ? runById.get(runId) : undefined;
                    const count = run ? entityCountForRun(run) : null;
                    const badges = badgesForCell(
                      runId,
                      runs,
                      metricsReport.compare_pairs_extended,
                    );
                    return (
                      <td
                        key={col}
                        className="border border-slate-800 p-1 align-top"
                      >
                        {runId ? (
                          <div className="space-y-1">
                            <span className="block font-mono text-sky-200/90">
                              {run?.label ?? runId}
                            </span>
                            {count !== null && (
                              <span className="text-slate-500">entities={count}</span>
                            )}
                            {badges.length > 0 && (
                              <div className="flex flex-wrap gap-0.5">
                                {badges.map((b) => (
                                  <span
                                    key={b.id}
                                    className="rounded border border-slate-700 bg-slate-900 px-1 text-[9px] text-sky-200/80"
                                    title={b.detail}
                                  >
                                    {b.label}
                                  </span>
                                ))}
                              </div>
                            )}
                          </div>
                        ) : (
                          <span className="text-slate-600">missing</span>
                        )}
                      </td>
                    );
                  })}
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      ) : (
        <div className="flex flex-wrap gap-2">
          {runs.map((run) => {
            const count = entityCountForRun(run);
            const coords = run.matrix_coords
              ? Object.entries(run.matrix_coords)
                  .map(([k, v]) => `${k}=${v}`)
                  .join(", ")
              : run.label;
            return (
              <span
                key={run.run_id}
                className="rounded border border-slate-700 bg-slate-900 px-2 py-1 text-[10px] text-sky-200/90"
              >
                <span className="font-mono">{run.run_id}</span>
                <span className="ml-1 text-slate-500">{coords}</span>
                {count !== null && (
                  <span className="ml-1 text-slate-500">entities={count}</span>
                )}
              </span>
            );
          })}
        </div>
      )}

      <p className="text-[10px] text-slate-500">
        matrix rollup: expected={rollup.expected_cells} populated=
        {rollup.populated_cells} missing={rollup.missing_cells}
      </p>
    </section>
  );
}
