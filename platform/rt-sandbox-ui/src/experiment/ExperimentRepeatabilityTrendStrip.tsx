import { BANNER_EXPERIMENT_F5 } from "@/governance/banners";
import type {
  ExperimentManifest,
  ExperimentMetricsReport,
  PerRunAnalytics,
} from "./experimentSchema";
import {
  attachF1Rows,
  consecutiveComparePairs,
  orderRepeatabilityRuns,
  repeatFingerprintAnnotation,
  trendBadgesForPair,
} from "./experimentRepeatabilityTrend";

export function ExperimentRepeatabilityTrendStrip({
  manifest,
  metricsReport,
  f1PerRun,
}: {
  manifest: ExperimentManifest;
  metricsReport: ExperimentMetricsReport;
  f1PerRun: PerRunAnalytics[];
}) {
  const ordered = attachF1Rows(
    orderRepeatabilityRuns(manifest.runs, metricsReport.per_run_extended),
    f1PerRun,
  );
  if (ordered.length === 0) return null;

  const runIds = ordered.map((o) => o.run.run_id);
  const consecutive = consecutiveComparePairs(
    runIds,
    metricsReport.compare_pairs_extended,
  );
  const fpNote = repeatFingerprintAnnotation(metricsReport.rollup_extended);

  return (
    <section className="space-y-3" data-testid="experiment-repeatability-trend-strip">
      <p className="text-[10px] text-amber-100/80">{BANNER_EXPERIMENT_F5}</p>
      <p className="text-[10px] text-slate-500">
        Repeatability sweep — ordered scan only; not time-series authority
      </p>
      <ul className="flex flex-wrap gap-2">
        {ordered.map((item, idx) => {
          const pair =
            idx > 0
              ? consecutive.find(
                  (p) =>
                    (p.run_id_a === runIds[idx - 1] && p.run_id_b === item.run.run_id) ||
                    (p.run_id_b === runIds[idx - 1] && p.run_id_a === item.run.run_id),
                )
              : undefined;
          const badges = pair ? trendBadgesForPair(pair) : [];
          return (
            <li
              key={item.run.run_id}
              className="min-w-[8rem] rounded border border-slate-700 bg-slate-950/50 px-2 py-1 text-[10px] text-slate-400"
            >
              <div className="font-medium text-slate-300">{item.run.label}</div>
              <div>
                entities {item.f1?.entity_count ?? "—"} · tti{" "}
                {item.f1?.tti_s ?? "—"} · occlusion{" "}
                {item.extended.occlusion_marker_count ?? "—"}
              </div>
              {item.extended.repeat_index != null && (
                <div className="text-slate-500">repeat_index={item.extended.repeat_index}</div>
              )}
              {badges.length > 0 && (
                <div className="mt-1 flex flex-wrap gap-0.5">
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
            </li>
          );
        })}
      </ul>
      {fpNote && <p className="font-mono text-[10px] text-slate-500">{fpNote}</p>}
    </section>
  );
}
