import { useMemo, useState } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import { BANNER_ANALYTICS } from "@/governance/banners";
import {
  deriveExperimentAnalytics,
  exportAnalyticsJson,
} from "./analyticsDerive";
import { formatImportError, safeParseAnalyticsReport } from "./experimentImportGuards";
import type {
  ExperimentAnalyticsReport,
  ExperimentBatchSpec,
  ExperimentManifest,
  PerRunAnalytics,
} from "./experimentSchema";

function annexCountSum(
  counts: PerRunAnalytics["annex_timeline_counts"],
): number {
  if (!counts) return 0;
  return (
    (counts.mode_switches ?? 0) +
    (counts.assignment_timeline ?? 0) +
    (counts.pause_resume_transitions ?? 0) +
    (counts.recommendation_timeline ?? 0)
  );
}

export function ExperimentAnalyticsPanel({
  manifest,
  batchSpec,
}: {
  manifest: ExperimentManifest;
  batchSpec?: ExperimentBatchSpec;
}) {
  const derived = useMemo(
    () => deriveExperimentAnalytics(manifest, batchSpec),
    [manifest, batchSpec],
  );
  const [report, setReport] = useState<ExperimentAnalyticsReport | null>(null);
  const active = report ?? derived;

  const refreshFromManifest = () => setReport(null);

  const importReport = () => {
    const text = window.prompt("Paste rt_experiment_analytics_report_v1 JSON");
    if (!text) return;
    const parsed = safeParseAnalyticsReport(text);
    if (!parsed.ok) {
      window.alert(`Invalid analytics report: ${formatImportError(parsed.error)}`);
      return;
    }
    setReport(parsed.data);
  };

  const exportReport = () => {
    const withTime = { ...active, derived_at_utc: new Date().toISOString() };
    const blob = new Blob([exportAnalyticsJson(withTime)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${manifest.experiment_id}-analytics.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <div data-testid="experiment-analytics">
    <PanelShell title="Experiment analytics">
      <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_ANALYTICS}</p>
      <div className="mb-3 flex flex-wrap gap-2">
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={refreshFromManifest}
        >
          Refresh from manifest
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={importReport}
        >
          Import report
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={exportReport}
        >
          Export report
        </button>
      </div>

      <div className="mb-3 grid gap-2 sm:grid-cols-2 lg:grid-cols-4">
        <div className="rounded border border-slate-800 p-2 text-xs text-slate-400">
          <div className="text-slate-500">runs</div>
          <div className="text-lg text-slate-200">{active.rollup.run_count}</div>
        </div>
        <div className="rounded border border-slate-800 p-2 text-xs text-slate-400">
          <div className="text-slate-500">captures</div>
          <div className="text-lg text-slate-200">{active.rollup.capture_count}</div>
        </div>
        <div className="rounded border border-slate-800 p-2 text-xs text-slate-400 sm:col-span-2">
          <div className="text-slate-500">mode distribution (counts only)</div>
          <div className="mt-1 font-mono text-slate-300">
            {Object.entries(active.rollup.mode_counts)
              .map(([k, v]) => `${k}:${v}`)
              .join(" · ") || "—"}
          </div>
        </div>
      </div>

      <div className="mb-3 flex flex-wrap gap-2">
        {active.per_run.map((row) => (
          <span
            key={row.run_id}
            className="rounded border border-slate-700 bg-slate-900 px-2 py-1 text-[10px] text-slate-400"
          >
            <span className="text-slate-300">{row.label}</span> · {row.tactical_mode ?? "—"} · ent{" "}
            {row.entity_count ?? "—"} · cap {row.has_capture ? "yes" : "no"}
            {annexCountSum(row.annex_timeline_counts) > 0
              ? ` · annex Σ${annexCountSum(row.annex_timeline_counts)}`
              : ""}
          </span>
        ))}
      </div>

      {active.compare_pairs.length > 0 && (
        <div className="overflow-x-auto">
          <p className="mb-1 text-[10px] text-slate-500">Pairwise compare (no winner column)</p>
          <table className="w-full text-[10px] text-slate-400">
            <thead>
              <tr className="text-left text-slate-500">
                <th className="pr-2">A</th>
                <th className="pr-2">B</th>
                <th>badges</th>
              </tr>
            </thead>
            <tbody>
              {active.compare_pairs.map((pair) => (
                <tr key={`${pair.run_id_a}-${pair.run_id_b}`} className="border-t border-slate-800">
                  <td className="py-1 font-mono text-slate-300">{pair.run_id_a}</td>
                  <td className="py-1 font-mono text-slate-300">{pair.run_id_b}</td>
                  <td className="py-1">
                    {pair.badges.length
                      ? pair.badges.map((b) => b.id).join(", ")
                      : "—"}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </PanelShell>
    </div>
  );
}
