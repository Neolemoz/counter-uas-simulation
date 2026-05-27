import type {
  ComparePairAnalytics,
  ExperimentMetricsReport,
  ExperimentRun,
  PerRunAnalytics,
  PerRunExtended,
} from "./experimentSchema";

const TREND_BADGE_IDS = new Set([
  "mode_changed",
  "tti_delta",
  "pause_resume_delta",
  "visibility_label_diff",
  "terrain_context_diff",
  "assignment_changed",
]);

export type OrderedRepeatabilityRun = {
  run: ExperimentRun;
  extended: PerRunExtended;
  f1?: PerRunAnalytics;
};

export function orderRepeatabilityRuns(
  runs: ExperimentRun[],
  perRunExtended: PerRunExtended[],
): Omit<OrderedRepeatabilityRun, "f1">[] {
  const extById = new Map(perRunExtended.map((r) => [r.run_id, r]));
  const ordered = [...runs].sort((a, b) => {
    const extA = extById.get(a.run_id);
    const extB = extById.get(b.run_id);
    const riA = extA?.repeat_index;
    const riB = extB?.repeat_index;
    const hasA = riA != null;
    const hasB = riB != null;
    if (hasA && hasB && riA !== riB) return riA - riB;
    if (hasA !== hasB) return hasA ? -1 : 1;
    const t = a.recorded_at_utc.localeCompare(b.recorded_at_utc);
    return t !== 0 ? t : a.run_id.localeCompare(b.run_id);
  });
  return ordered.map((run) => ({
    run,
    extended: extById.get(run.run_id)!,
  }));
}

export function attachF1Rows(
  ordered: Omit<OrderedRepeatabilityRun, "f1">[],
  f1PerRun: PerRunAnalytics[],
): OrderedRepeatabilityRun[] {
  const f1ById = new Map(f1PerRun.map((r) => [r.run_id, r]));
  return ordered.map((item) => ({
    ...item,
    f1: f1ById.get(item.run.run_id),
  }));
}

export function consecutiveComparePairs(
  orderedRunIds: string[],
  comparePairs: ComparePairAnalytics[],
): ComparePairAnalytics[] {
  const out: ComparePairAnalytics[] = [];
  for (let i = 0; i < orderedRunIds.length - 1; i += 1) {
    const a = orderedRunIds[i];
    const b = orderedRunIds[i + 1];
    const pair = comparePairs.find(
      (p) =>
        (p.run_id_a === a && p.run_id_b === b) || (p.run_id_a === b && p.run_id_b === a),
    );
    if (pair) out.push(pair);
  }
  return out;
}

export function trendBadgesForPair(pair: ComparePairAnalytics): ComparePairAnalytics["badges"] {
  return pair.badges.filter((b) => TREND_BADGE_IDS.has(b.id));
}

export function repeatFingerprintAnnotation(
  rollup: ExperimentMetricsReport["rollup_extended"],
): string | null {
  const multi = rollup.repeatability_rollup.fingerprints.filter((f) => f.run_count > 1);
  if (multi.length === 0) return null;
  return multi
    .map((f) => `spec_fingerprint ${f.spec_fingerprint} (${f.run_count} runs)`)
    .join("; ");
}
