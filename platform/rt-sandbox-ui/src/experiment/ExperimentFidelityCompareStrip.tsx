import { BANNER_FIDELITY_TRUTH } from "@/governance/banners";
import { containsForbiddenLexicon } from "@/cesium/cognition";
import {
  FIDELITY_METRICS_GOVERNANCE_BANNER,
  type ExperimentFidelityMetricsReport,
  type ExperimentManifest,
} from "./experimentSchema";
import {
  attestationFreshnessBadge,
  driftSummary,
  fidelityCompareBadgeLabel,
  fidelityRepeatabilitySummary,
  pairBadgesForRuns,
  truthVsExplanatorySummary,
} from "./experimentFidelityUiHelpers";

export function ExperimentFidelityCompareStrip({
  manifest,
  fidelityReport,
}: {
  manifest: ExperimentManifest;
  fidelityReport: ExperimentFidelityMetricsReport;
}) {
  const rows = [...fidelityReport.per_run_fidelity].sort((a, b) =>
    a.run_id.localeCompare(b.run_id),
  );
  if (rows.length === 0) return null;

  const runLabels = new Map(manifest.runs.map((r) => [r.run_id, r.label]));
  const fpSummary = fidelityRepeatabilitySummary(
    fidelityReport.rollup_fidelity.repeatability_truth_rollup.truth_fingerprints,
  );
  const divergenceCount =
    fidelityReport.rollup_fidelity.divergence_rollup.cognition_truth_divergence_count;

  return (
    <section className="space-y-3" data-testid="experiment-fidelity-compare-strip">
      <p className="text-[10px] text-amber-100/80">{FIDELITY_METRICS_GOVERNANCE_BANNER}</p>
      <p className="text-[10px] text-slate-500">{BANNER_FIDELITY_TRUTH}</p>
      <p className="text-[10px] text-slate-500">
        Fidelity compare — truth-attested vs explanatory; read-only derived report
      </p>
      <ul className="flex flex-wrap gap-2">
        {rows.map((row, idx) => {
          const freshness = attestationFreshnessBadge(row.fidelity_attestation_status);
          const prior = idx > 0 ? rows[idx - 1] : null;
          const pairBadges =
            prior != null
              ? pairBadgesForRuns(
                  prior.run_id,
                  row.run_id,
                  fidelityReport.compare_pairs_fidelity,
                )
              : [];
          return (
            <li
              key={row.run_id}
              className="min-w-[10rem] rounded border border-slate-700 bg-slate-950/50 px-2 py-1 text-[10px] text-slate-400"
            >
              <div className="font-medium text-slate-300">
                {runLabels.get(row.run_id) ?? row.run_id}
              </div>
              <div className="mt-1 flex flex-wrap gap-0.5">
                <span
                  className={
                    freshness.tone === "ok"
                      ? "rounded border border-emerald-800/60 bg-emerald-950/40 px-1 text-[9px] text-emerald-200/90"
                      : freshness.tone === "warn"
                        ? "rounded border border-amber-800/60 bg-amber-950/40 px-1 text-[9px] text-amber-200/90"
                        : "rounded border border-slate-700 bg-slate-900 px-1 text-[9px] text-slate-400"
                  }
                >
                  {freshness.label}
                </span>
                <span className="rounded border border-slate-700 bg-slate-900 px-1 text-[9px] text-sky-200/70">
                  explanatory
                </span>
                {row.cognition_truth_divergence && (
                  <span className="rounded border border-amber-800/60 bg-amber-950/40 px-1 text-[9px] text-amber-200/90">
                    cognition_truth_divergence
                  </span>
                )}
              </div>
              <div className="mt-1 text-slate-500">{truthVsExplanatorySummary(row)}</div>
              <div className="text-slate-500">{driftSummary(row)}</div>
              {pairBadges.length > 0 && (
                <div className="mt-1 flex flex-wrap gap-0.5">
                  {pairBadges.map((b) => (
                    <span
                      key={b.id}
                      className="rounded border border-slate-700 bg-slate-900 px-1 text-[9px] text-sky-200/80"
                    >
                      {fidelityCompareBadgeLabel(b.id)}
                    </span>
                  ))}
                </div>
              )}
            </li>
          );
        })}
      </ul>
      <p className="text-[10px] text-slate-500">
        divergence count {divergenceCount}
        {fpSummary ? ` · ${fpSummary}` : ""}
      </p>
      {containsForbiddenLexicon(FIDELITY_METRICS_GOVERNANCE_BANNER) && (
        <p className="hidden" data-testid="forbidden-lexicon-flag">
          forbidden
        </p>
      )}
    </section>
  );
}
