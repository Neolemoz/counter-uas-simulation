import { useSweepStore } from "../useSweepStore";

export function LosDegradationSummary() {
  const sweep = useSweepStore((s) => s.sweep);
  if (!sweep) return null;

  const los = sweep.spatial_aggregate.layers.los_degraded;
  const total = los?.counts?.reduce((a, b) => a + b, 0) ?? 0;
  const hist = sweep.replay_aggregation?.outcome_histogram?.los_degraded_count ?? [];

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-slate-200">LOS degradation summary</h2>
      <p className="text-xs text-slate-400">
        Aggregate grid cells: <strong className="text-slate-200">{total}</strong>
      </p>
      {hist.length > 0 && (
        <p className="mt-1 text-xs text-slate-400">
          Per-member counts: {hist.join(", ")} (explanatory)
        </p>
      )}
      {los?.caveat && <p className="mt-2 text-[10px] text-slate-500">{los.caveat}</p>}
    </section>
  );
}
