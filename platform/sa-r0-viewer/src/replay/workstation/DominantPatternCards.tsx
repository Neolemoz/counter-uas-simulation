import { useSweepStore } from "../useSweepStore";

export function DominantPatternCards() {
  const sweep = useSweepStore((s) => s.sweep);
  if (!sweep) return null;

  const patterns = sweep.replay_aggregation?.dominant_patterns ?? [];
  const cohorts = sweep.replay_cohorts ?? [];

  if (!patterns.length && !cohorts.length) return null;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h3 className="mb-2 font-semibold text-slate-200">Dominant replay patterns</h3>
      <div className="flex flex-wrap gap-2">
        {patterns.map((p, i) => (
          <div
            key={`p-${i}`}
            className="max-w-full rounded border border-slate-600 bg-slate-800/80 px-2 py-1 text-xs text-slate-300"
          >
            {p}
          </div>
        ))}
        {cohorts.map((c) => (
          <div
            key={c.cohort_id}
            className="max-w-full rounded border border-violet-800/60 bg-violet-950/40 px-2 py-1 text-xs text-violet-200"
          >
            {c.label}
          </div>
        ))}
      </div>
    </section>
  );
}
